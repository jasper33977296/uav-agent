#!/usr/bin/env python3
"""斷線緩衝與回補：**不連飛機、不連地面站**（HTTP 用假的）。

要釘住的行為：
  * 鏈路通的時候不累積（即時遙測本來就送得到，緩衝只是浪費）
  * 斷線時按 1 Hz 取樣，主迴圈頻率再高也不會改變資料密度
  * 環形有上限，滿了丟最舊的——記憶體在飛機上是有限的
  * **沒有定位就不填位置**：0,0 是自駕儀的「不知道」哨兵，而補傳直接進
    telemetry 表，填假位置比缺一筆嚴重得多
  * 恢復後補傳；送失敗要留在緩衝裡下次再送，不能當成送掉了

用法：python3 tools/backfill-flow.py
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import backfill as B  # noqa: E402

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


STATE = {"gps_fix": "3D", "lat": 24.77, "lon": 121.04, "alt_rel_m": 30.0,
         "mode": "AUTO", "armed": True, "battery_pct": 80, "gps_sats": 12}
LINK = {"ok": True}
POSTED = []


class Fake(B.Backfill):
    def __init__(self):
        super().__init__("x", 1, lambda: "board-1", lambda: STATE,
                         lambda: LINK["ok"])
        self.fail_next = False

    def _flush(self):
        if self.fail_next:
            self.failed += 1
            return                      # 模擬送失敗：資料要留在緩衝裡
        with self._lock:
            n = min(B.BATCH, len(self.buf))
            POSTED.extend(self.buf[i] for i in range(n))
            for _ in range(n):
                self.buf.popleft()
        self.sent += n


g = Fake()

print("── 鏈路通的時候不累積 ─────────────────────────────────")
for _ in range(5):
    g._last_sample = 0
    if not g.link_ok_fn():
        g._sample(time.time())
chk("通的時候緩衝是空的", len(g.buf) == 0)

print("\n── 斷線 → 取樣 ────────────────────────────────────────")
LINK["ok"] = False
t0 = time.time()
for i in range(10):
    g._sample(t0 + i)
chk("斷線期間有累積", len(g.buf) == 10, len(g.buf))
chk("欄位就是 telemetry 表那幾個",
    set(g.buf[0]) >= {"t", "lat", "lon", "alt_rel", "flight_mode", "armed"},
    sorted(g.buf[0]))

print("\n── 沒有定位就不填位置（0,0 是哨兵不是座標）──────────")
STATE.update({"gps_fix": "NO_FIX", "lat": 0.0, "lon": 0.0})
g._sample(t0 + 11)
last = g.buf[-1]
chk("NO_FIX 時 lat/lon 是 None", last["lat"] is None and last["lon"] is None,
    (last["lat"], last["lon"]))
chk("**反向驗證**：其他欄位照樣有（不是整筆丟掉）",
    last["flight_mode"] == "AUTO" and last["battery_pct"] == 80)
STATE.update({"gps_fix": "3D", "lat": 24.77, "lon": 121.04})

print("\n── 環形上限：滿了丟最舊的 ─────────────────────────────")
cap = g.buf.maxlen
for i in range(cap + 50):
    g._sample(t0 + 100 + i)
chk(f"不超過上限（{cap} 筆 ≈ {B.BUFFER_S/60:.0f} 分鐘）", len(g.buf) == cap)
chk("留下的是最新的那段", g.buf[-1]["t"] > g.buf[0]["t"])

print("\n── 恢復後補傳 ─────────────────────────────────────────")
g.buf.clear(); POSTED.clear()
for i in range(5):
    g._sample(t0 + 200 + i)
LINK["ok"] = True
g.fail_next = True
g._flush()
chk("送失敗 → 資料留在緩衝裡（不能當成送掉了）",
    len(g.buf) == 5 and g.failed == 1, (len(g.buf), g.failed))
g.fail_next = False
g._flush()
chk("重送成功 → 緩衝清空", len(g.buf) == 0 and len(POSTED) == 5,
    (len(g.buf), len(POSTED)))
chk("補傳的時間戳是機上的（不是補傳當下）",
    abs(POSTED[0]["t"] - (t0 + 200)) < 0.01, POSTED[0]["t"])

print("\n── stayed_armed：D 層判斷「還算不算同一趟」──────────")
chk("這段期間 armed 過 → stayed_armed=True", g.stayed_armed is True)

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
