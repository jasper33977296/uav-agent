"""機上緩衝與回補（uav-system 的 C 層）。

**5G 斷線時資料根本沒有遺失——它就在這台 Pi 上，只是送不出去。**
代理照樣看得到飛控的一切；現在（2026-08-26 之前）那段資料被直接丟掉，
於是地面站事後看軌跡時只有一個空洞。

做法：以 1 Hz 把地面站真正入庫的那幾個欄位寫進環形緩衝；地面鏈路恢復後
在背景把缺口補傳上去。

四個刻意的設計：

* **降頻＋只留必要欄位**。完整 MAVLink 串流約 16.5 KB/s；這裡一筆約 100
  bytes、1 Hz ＝ **6 KB/分鐘**。斷一小時也才 360 KB，恢復後幾秒補完。
  留全串流會在恢復的那一刻跟活的遙測搶頻寬——**補歷史不該拖慢現在**。
* **環形有上限**（預設 30 分鐘）。滿了丟最舊的，不是無限長大：
  記憶體在飛機上是有限的，而且真的斷一小時以上時，最舊那段的價值最低。
* **只在鏈路不通時累積**。通的時候即時遙測本來就送得到，緩衝只是浪費。
* **補傳走背景、低優先**。它與即時轉發共用同一條 5G，而即時的優先。
"""
import json
import logging
import threading
import time
import urllib.error
import urllib.request
from collections import deque

log = logging.getLogger("uav-agent.backfill")

#: 取樣間隔（秒）。與地面站的 db_write_hz 對齊——**補傳的解析度沒有理由
#: 高於它本來就會存的解析度**
SAMPLE_S = 1.0
#: 環形緩衝的時間上限（秒）。30 分鐘 ＝ 1800 筆 ≈ 180 KB
BUFFER_S = 1800.0
#: 一次補傳最多幾筆。分批送：一包太大時，5G 抖一下整包就白送
BATCH = 300


class Backfill(threading.Thread):
    """斷線期間累積取樣，恢復後補傳。

    `snapshot()` 由主迴圈每拍呼叫（便宜：只讀 dict）；真正的取樣由本執行緒
    按 SAMPLE_S 節流，所以主迴圈的頻率變化不會改變資料密度。
    """

    def __init__(self, gs: str, port: int, board_uid_fn, state_fn, link_ok_fn):
        super().__init__(daemon=True)
        self.url = f"http://{gs}:{port}/api/telemetry/backfill"
        self.board_uid_fn = board_uid_fn
        self.state_fn = state_fn
        self.link_ok_fn = link_ok_fn
        self.buf: deque = deque(maxlen=int(BUFFER_S / SAMPLE_S))
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self.sampled = 0
        self.sent = 0
        self.failed = 0
        self.stayed_armed = None
        self._last_sample = 0.0

    def stop(self):
        self._stop.set()

    def stats(self) -> dict:
        return {"buffered": len(self.buf), "sampled": self.sampled,
                "sent": self.sent, "failed": self.failed}

    # ── 取樣 ───────────────────────────────────────────────────
    def _sample(self, now: float) -> None:
        s = self.state_fn() or {}
        # **沒有位置就不取樣**：0,0 是自駕儀的「不知道」哨兵，而補傳的資料
        # 會直接進 telemetry 表——填一個假位置比缺一筆嚴重得多
        fix = s.get("gps_fix")
        has_pos = fix not in (None, "NO_GPS", "NO_FIX") and (
            s.get("lat") or s.get("lon"))
        row = {
            "t": round(now, 2),
            "lat": s.get("lat") if has_pos else None,
            "lon": s.get("lon") if has_pos else None,
            "alt_rel": s.get("alt_rel_m"),
            "heading": s.get("heading"),
            "ground_speed": s.get("groundspeed_ms"),
            "battery_pct": s.get("battery_pct"),
            "battery_voltage": s.get("voltage"),
            "satellites": s.get("gps_sats"),
            "flight_mode": s.get("mode"),
            "armed": bool(s.get("armed")),
        }
        with self._lock:
            self.buf.append(row)
        self.sampled += 1
        # armed 只要在這段期間曾經是 True，就記著——D 層用它判斷
        # 「回來之後還算不算同一趟」
        if row["armed"]:
            self.stayed_armed = True
        elif self.stayed_armed is None:
            self.stayed_armed = False

    # ── 補傳 ───────────────────────────────────────────────────
    def _flush(self) -> None:
        with self._lock:
            if not self.buf:
                return
            batch = [self.buf[i] for i in range(min(BATCH, len(self.buf)))]
        body = json.dumps({"board_uid": self.board_uid_fn(),
                           "samples": batch,
                           "stayed_armed": self.stayed_armed}).encode()
        req = urllib.request.Request(
            self.url, data=body, method="POST",
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=8) as r:
                r.read()
        except Exception as e:
            self.failed += 1
            log.warning("補傳失敗（%s），保留在緩衝裡下次再送", e)
            return
        with self._lock:
            for _ in range(len(batch)):
                if self.buf:
                    self.buf.popleft()
        self.sent += len(batch)
        log.info("已補傳 %d 筆（緩衝剩 %d）", len(batch), len(self.buf))

    def run(self):
        while not self._stop.wait(0.2):
            now = time.time()
            if not self.board_uid_fn():
                continue                      # 還沒註冊，補傳沒有收件人
            link_ok = self.link_ok_fn()
            if not link_ok:
                if now - self._last_sample >= SAMPLE_S:
                    self._last_sample = now
                    self._sample(now)
                continue
            # 鏈路通了：**先把缺口補完，補完就把 stayed_armed 歸零**
            if self.buf:
                self._flush()
            elif self.stayed_armed is not None:
                self.stayed_armed = None
