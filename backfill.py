"""機上緩衝與回補（uav-system 的 C 層）。

**5G 斷線時資料根本沒有遺失——它就在這台 Pi 上，只是送不出去。**
代理照樣看得到飛控的一切；現在（2026-08-26 之前）那段資料被直接丟掉，
於是地面站事後看軌跡時只有一個空洞。

做法：以 1 Hz 把地面站真正入庫的那幾個欄位寫進環形緩衝；地面鏈路恢復後
在背景把缺口補傳上去。

五個刻意的設計：

* **降頻＋只留必要欄位**。完整 MAVLink 串流約 16.5 KB/s；這裡一筆約 100
  bytes、1 Hz ＝ **6 KB/分鐘**。斷一小時也才 360 KB，恢復後幾秒補完。
  留全串流會在恢復的那一刻跟活的遙測搶頻寬——**補歷史不該拖慢現在**。
* **環形有上限**（預設 30 分鐘）。滿了丟最舊的，不是無限長大：
  記憶體在飛機上是有限的，而且真的斷一小時以上時，最舊那段的價值最低。
* **只在鏈路不通時累積**。通的時候即時遙測本來就送得到，緩衝只是浪費。
  「通不通」看的是 `uplink_ok`（地面站說它收到我的遙測了），**不是
  「我聽不聽得到地面站」**——後者是 2026-09-08 實測抓到的錯誤來源。
* **只在 armed 時累積**（2026-09-08）。地面站的即時路徑只在
  `armed and session_id` 時才寫 telemetry 表（`main.py:152`），
  補傳卻無條件寫——同一個「飛機停在地上什麼都沒發生」的狀態，走即時路是
  不記錄、走補傳路變成記錄，**差別只在於當時鏈路有沒有斷**。實測一次
  72 秒的中斷就補進 59 筆停機坪資料。兩條路對「什麼值得記錄」要有同一個答案。
* **補傳走背景、低優先**。它與即時轉發共用同一條 5G，而即時的優先。
* **時間用單調時鐘記，送出前才換算成牆鐘**（2026-08-31）。牆鐘會被 NTP 調，
  而這台 Pi 的 RTC 沒有電池——開機時系統時間是 1970，靠 5G 上的 NTP 修正。
  取樣當下直接寫 `time.time()` 的話，冷開機那段的樣本會帶著 1970 的時間戳
  送進地面站的 telemetry 表，而且跳躍發生在同一個環形緩衝裡，**單一批次的
  時間可能橫跨 56 年且非單調**。記單調時間、flush 時再用當下的偏移換算，
  取樣當下時鐘是錯的也沒關係——只要送出時已經同步，補上去的絕對時間就是對的。
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
#: 牆鐘顯然還沒同步的門檻（2020-09-13）。小於它就是「時鐘還沒被校正過」。
#:
#: **這台 Pi 的 RTC 沒有電池**：journal 實測每次開機都是
#: `rpi-rtc: setting system clock to 1970-01-01T00:00:14`，要等 NTP 才會跳到
#: 正確時間——而 NTP 走的是 5G，也就是**失聯時校正不了**。冷開機當下
#: `board_uid` 來自飛控 UART（幾秒就有，不依賴 5G）、`gs_link_ok` 預設 False，
#: 兩個取樣條件在 NTP 同步之前就成立，所以開機那段的樣本會帶著 1970 的時間。
CLOCK_SANE_EPOCH = 1_600_000_000


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
        self._last_sample = 0.0       # 單調時鐘刻度（見 run()）
        self._clock_warned = False
        #: 地面站收下了、但判定時間戳不合理而丟掉的筆數。**機端要自己看得到**
        #: ——不然「已補傳 N 筆」會是一句謊話，而唯一的痕跡在別人的 log 裡
        self.rejected = 0
        self.dropped = 0              # 我方主動放棄的（送了也不會被接受）
        #: 因為沒解鎖而沒取樣的次數。**要看得見**——不然「斷線了卻沒補回
        #: 任何東西」會像故障，而它其實是正確行為
        self.skipped_ground = 0
        self._next_try = 0.0          # 單調刻度：失敗後不要每 0.2 秒重試

    def stop(self):
        self._stop.set()

    def stats(self) -> dict:
        return {"buffered": len(self.buf), "sampled": self.sampled,
                "sent": self.sent, "failed": self.failed,
                "rejected": self.rejected, "dropped": self.dropped,
                "skipped_ground": self.skipped_ground}

    # ── 取樣 ───────────────────────────────────────────────────
    def _sample(self, mono: float) -> None:
        s = self.state_fn() or {}
        # **沒有位置就不取樣**：0,0 是自駕儀的「不知道」哨兵，而補傳的資料
        # 會直接進 telemetry 表——填一個假位置比缺一筆嚴重得多
        fix = s.get("gps_fix")
        has_pos = fix not in (None, "NO_GPS", "NO_FIX") and (
            s.get("lat") or s.get("lon"))
        row = {
            # **單調時間，不是牆鐘**（見模組說明）。欄位名帶底線＝機上內部用，
            # 送出去的那份在 `_batch()` 換算成 `t`
            "_m": round(mono, 2),
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
    def _batch(self, rows: list) -> list:
        """把緩衝裡的單調時間換算成牆鐘。**在送出的這一刻換算**。

        取樣當下的牆鐘可能還沒被 NTP 校正（開機那段就是 1970），但送得出去
        代表 5G 通了，而 5G 通了 NTP 也就跟著同步——**用送出當下的偏移回推，
        取樣當下時鐘是錯的也沒關係**，補上去的絕對時間仍然是對的。

        不動緩衝裡的那份：送失敗時它要原封留著下次再送。
        """
        off = time.time() - time.monotonic()
        out = []
        for r in rows:
            row = dict(r)
            row["t"] = round(row.pop("_m") + off, 2)
            out.append(row)
        return out

    def _post(self, body: bytes) -> dict:
        """真的送出去，並**把回話讀回來**。

        抽成一個方法是為了讓測試擋得住 HTTP 而**不必蓋掉 `_flush`**——蓋掉的
        話批次換算與緩衝彈出這兩段就一格都測不到。

        回話要讀是因為 200 不代表資料被收下：地面站會過濾時間戳不合理的樣本
        （`rejected_implausible`），而那正是機上時鐘出事的唯一徵兆。不讀的話
        「已補傳 N 筆」就是一句謊話，證據還只在別人的 log 裡。
        """
        req = urllib.request.Request(
            self.url, data=body, method="POST",
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=8) as r:
            raw = r.read()
        try:
            return json.loads(raw or b"{}")
        except ValueError:
            return {}

    def _flush(self) -> None:
        if time.monotonic() < self._next_try:
            return                    # 上次失敗後的退避（見 `_fail`）
        # **時鐘還沒同步就先不送。** 5G 剛通到 timesyncd 真的把時間 step 過去
        # 之間有幾秒空窗，那幾秒送出去的換算基準仍然是 1970。資料留在環形裡
        # 等下一輪——環形有上限，等不到也只是丟最舊的，不會無限長大
        if time.time() < CLOCK_SANE_EPOCH:
            if not self._clock_warned:
                self._clock_warned = True
                log.warning("牆鐘還沒同步（現在是 %s）→ 暫不補傳：換算基準是錯的，"
                            "送出去會是 1970 的時間戳。資料留在緩衝裡等時鐘校正",
                            time.strftime("%Y-%m-%d %H:%M:%S"))
            return
        self._clock_warned = False
        with self._lock:
            if not self.buf:
                return
            rows = [self.buf[i] for i in range(min(BATCH, len(self.buf)))]
        batch = self._batch(rows)
        body = json.dumps({"board_uid": self.board_uid_fn(),
                           "samples": batch,
                           "stayed_armed": self.stayed_armed}).encode()
        try:
            res = self._post(body)
        except urllib.error.HTTPError as e:
            if self._unacceptable(e):
                # **整批的時間戳都被判定不合理＝再送幾次也一樣。** 留著它就是
                # 每 0.2 秒重送一批永遠不會被接受的資料，一路佔著 5G 上行，
                # 而那條線正是即時遙測在走的。丟掉並大聲說，不要靜靜地重試
                self._drop(len(batch))
                return
            return self._fail(e)
        except Exception as e:
            return self._fail(e)
        with self._lock:
            for _ in range(len(batch)):
                if self.buf:
                    self.buf.popleft()
        self.sent += len(batch)
        rej = res.get("rejected_implausible") or 0
        if rej:
            self.rejected += rej
            log.warning("地面站丟掉 %d/%d 筆時間戳不合理的樣本——**機上時鐘"
                        "可能還沒與 NTP 同步**，那段缺口沒有真的補回來",
                        rej, len(batch))
        log.info("已補傳 %d 筆（入庫 %s、重複 %s、丟棄 %s；緩衝剩 %d）",
                 len(batch), res.get("inserted", "?"),
                 res.get("skipped_duplicate", "?"), rej, len(self.buf))

    @staticmethod
    def _unacceptable(e) -> bool:
        """這個 HTTP 錯誤是「再送幾次也一樣」嗎？

        只認地面站明講的那一種（整批時間戳不合理）。**其他一律當成暫時性**
        ——猜錯的方向要選對：把暫時的當永久＝丟掉本來補得回來的資料，
        把永久的當暫時＝多重試幾次。前者不可逆，後者只是吵。

        **不比對狀態碼，只認回話裡的 `code`。** 第一版寫了 `if e.code != 400`，
        而那個端點回的是 **422**——判定因此從來不會生效，永遠不會成功的重送
        會一路退避重試下去。狀態碼是對方可以換的實作細節，`code` 才是雙方
        講好的契約；綁前者就是把自己接到一個沒有約定的東西上。
        """
        try:
            detail = json.loads(e.read() or b"{}").get("detail") or {}
        except ValueError:
            return False
        return isinstance(detail, dict) and \
            detail.get("code") == "implausible_timestamps"

    def _drop(self, n: int) -> None:
        with self._lock:
            for _ in range(n):
                if self.buf:
                    self.buf.popleft()
        self.dropped += n
        log.error("⚠ 放棄 %d 筆補傳：地面站判定整批時間戳都不合理，重送也一樣。"
                  "**這段缺口補不回來了**——檢查機上時鐘（這台 Pi 的 RTC "
                  "沒有電池，開機時間從 1970 起算）", n)

    def _fail(self, e) -> None:
        self.failed += 1
        # 失敗後退開 2 秒再試。原本每 0.2 秒重送一次，地面站不通的時候
        # 那是一個對著 5G 上行猛打的迴圈，而即時遙測正在走同一條線
        self._next_try = time.monotonic() + 2.0
        log.warning("補傳失敗（%s），保留在緩衝裡 2 秒後再送", e)

    def run(self):
        while not self._stop.wait(0.2):
            # **節流也用單調時鐘**：牆鐘往回校正 Δ 秒的話，`now - _last_sample`
            # 會變成負的，取樣就整整停擺 Δ 秒——而且是靜默的，`sampled` 不動、
            # log 不叫。這台 Pi 每次開機都會發生一次大幅度的時間跳躍
            mono = time.monotonic()
            if not self.board_uid_fn():
                continue                      # 還沒註冊，補傳沒有收件人
            link_ok = self.link_ok_fn()
            if not link_ok:
                if mono - self._last_sample >= SAMPLE_S:
                    self._last_sample = mono
                    # **沒解鎖就不補**（2026-09-08）：地面站的即時路徑在
                    # 這種狀態下本來就不寫任何東西，補傳沒有理由比它更積極。
                    # `armed is None`＝從來沒看到過飛控，那時也沒有東西好補
                    if self.state_fn().get("armed"):
                        self._sample(mono)
                    else:
                        self.skipped_ground += 1
                continue
            # 鏈路通了：**先把缺口補完，補完就把 stayed_armed 歸零**
            if self.buf:
                self._flush()
            elif self.stayed_armed is not None:
                self.stayed_armed = None
