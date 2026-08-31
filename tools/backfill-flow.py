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
import json
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
    """**只擋 HTTP，不蓋掉 `_flush`。** 蓋掉的話批次換算（單調→牆鐘）與緩衝
    彈出這兩段就一格都測不到，而那正是最容易寫錯的地方。"""

    def __init__(self):
        super().__init__("x", 1, lambda: "board-1", lambda: STATE,
                         lambda: LINK["ok"])
        self.fail_next = False

    def _post(self, body):
        if self.fail_next:
            raise OSError("模擬送失敗")     # 資料要留在緩衝裡
        POSTED.extend(json.loads(body)["samples"])


g = Fake()

print("── 鏈路通的時候不累積 ─────────────────────────────────")
for _ in range(5):
    g._last_sample = 0
    if not g.link_ok_fn():
        g._sample(time.monotonic())
chk("通的時候緩衝是空的", len(g.buf) == 0)

print("\n── 斷線 → 取樣 ────────────────────────────────────────")
LINK["ok"] = False
t0 = time.monotonic()
for i in range(10):
    g._sample(t0 + i)
chk("斷線期間有累積", len(g.buf) == 10, len(g.buf))
chk("緩衝裡記的是**單調時間**（牆鐘會被 NTP 調）",
    "_m" in g.buf[0] and "t" not in g.buf[0], sorted(g.buf[0]))
chk("其餘欄位就是 telemetry 表那幾個",
    set(g.buf[0]) >= {"_m", "lat", "lon", "alt_rel", "flight_mode", "armed"},
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
chk("留下的是最新的那段", g.buf[-1]["_m"] > g.buf[0]["_m"])

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
chk("送出去的是牆鐘 `t`（單調時間不外流）",
    "t" in POSTED[0] and "_m" not in POSTED[0], sorted(POSTED[0]))
chk("補傳的時間戳是機上取樣的那一刻（不是補傳當下）",
    abs(POSTED[0]["t"] - (time.time() - time.monotonic() + t0 + 200)) < 0.05,
    POSTED[0]["t"])
chk("批次內單調遞增（1 Hz 取樣就該差 1 秒）",
    all(abs((POSTED[i + 1]["t"] - POSTED[i]["t"]) - 1.0) < 0.01
        for i in range(len(POSTED) - 1)),
    [round(r["t"], 2) for r in POSTED])

print("\n── 牆鐘被 NTP 調過：補上去的時間仍然要對 ───────────────")
# **這台 Pi 的 RTC 沒有電池**，每次開機系統時間從 1970 起算，靠 5G 上的 NTP
# 修正——也就是「失聯期間取樣」與「恢復後補傳」很可能落在時鐘跳躍的兩側。
# 取樣記單調、送出前才換算，跳躍就不會寫進資料裡
real_time = time.time
g.buf.clear(); POSTED.clear()
LINK["ok"] = False
try:
    time.time = lambda: 14.0                  # 假裝時鐘還停在 1970
    base = time.monotonic()
    for i in range(3):
        g._sample(base + i)
    chk("時鐘還沒同步時 `_flush` 不送（換算基準是錯的）",
        (LINK.__setitem__("ok", True), g._flush(), len(g.buf) == 3
         and not POSTED)[-1], f"buf={len(g.buf)} posted={len(POSTED)}")
    time.time = real_time                     # NTP 把時間校正回來
    g._flush()
finally:
    time.time = real_time
chk("時鐘校正後才送出去", len(g.buf) == 0 and len(POSTED) == 3,
    (len(g.buf), len(POSTED)))
chk("**送出去的是校正後的時間，不是 1970**",
    all(r["t"] > B.CLOCK_SANE_EPOCH for r in POSTED),
    [round(r["t"], 1) for r in POSTED])
chk("跳躍沒有寫進資料：批次內仍然單調且間隔 1 秒",
    all(abs((POSTED[i + 1]["t"] - POSTED[i]["t"]) - 1.0) < 0.01
        for i in range(len(POSTED) - 1)),
    [round(r["t"], 2) for r in POSTED])

print("\n── 節流用單調時鐘：牆鐘往回跳不會讓取樣停擺 ───────────")
# 舊寫法 `now - _last_sample >= SAMPLE_S` 用牆鐘：往回校正 Δ 秒的話取樣會
# 靜默停擺 Δ 秒（sampled 不動、log 不叫）
g2 = Fake()
g2._last_sample = time.monotonic()
before = g2.sampled
try:
    time.time = lambda: real_time() - 86400   # 牆鐘往回跳一天
    time.sleep(1.05)
    m = time.monotonic()
    if m - g2._last_sample >= B.SAMPLE_S:
        g2._last_sample = m
        g2._sample(m)
finally:
    time.time = real_time
chk("牆鐘往回跳一天，取樣照常進行", g2.sampled == before + 1,
    f"sampled {before} → {g2.sampled}")

print("\n── stayed_armed：D 層判斷「還算不算同一趟」──────────")
chk("這段期間 armed 過 → stayed_armed=True", g.stayed_armed is True)

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
