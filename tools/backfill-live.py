#!/usr/bin/env python3
"""補傳的**端到端**：真的 `Backfill` → 真的地面站 → 真的入庫。

**為什麼補這一支**：現有兩支測試各自 stub 掉一邊——`tools/backfill-flow.py`
擋住 HTTP，`uav-system/scripts/test-blackout-backfill.py` 直接對 API POST。
兩邊都綠，中間那一段卻從來沒有合起來跑過。2026-08-31 對照資料庫的結果是
**C 層補傳在正式系統上從來沒有成功跑完一次**（`telemetry` 裡 `backfilled`
的列 0 筆），窗口開著但沒有東西穿過去。這支就是那一段。

釘住的重點是**冷開機那個情境**：這台 Pi 的 RTC 沒有電池，開機時系統時間是
1970，靠 5G 上的 NTP 修正——而取樣正好發生在 5G 還沒通的時候。所以這裡刻意
在「時鐘還沒同步」的狀態下取樣，恢復後才送，看地面站收不收。

**需要跑著的地面站**（docker compose up）。會建一台名為 `__backfill_live`
的測試機並在結束時刪掉。

用法：python3 tools/backfill-live.py [主機] [後端埠]
"""
import json
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from backfill import Backfill  # noqa: E402

GS = sys.argv[1] if len(sys.argv) > 1 else "localhost"
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 38000
UID = "backfilllive-board-0000"
BASE = f"http://{GS}:{PORT}/api"

STATE = {"gps_fix": "3D", "lat": 24.7712, "lon": 121.0435, "alt_rel_m": 42.0,
         "heading": 90.0, "groundspeed_ms": 8.5, "mode": "AUTO", "armed": True,
         "battery_pct": 77, "voltage": 22.4, "gps_sats": 14}
LINK = {"ok": True}
ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


def api(path, body=None, method="POST"):
    req = urllib.request.Request(
        BASE + path, data=json.dumps(body).encode() if body else None,
        method=method, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=8) as r:
        return json.loads(r.read() or b"{}")


class Watched(Backfill):
    """真的 `Backfill`，只是把地面站的回話留一份下來看。"""

    def __init__(self):
        super().__init__(GS, PORT, lambda: UID, lambda: STATE,
                         lambda: LINK["ok"])
        self.replies = []

    def _post(self, body):
        res = super()._post(body)          # 真的送、真的入庫
        self.replies.append(res)
        return res


try:
    reg = api("/agent/hello", {"board_uid": UID, "autopilot": "ardupilot",
                               "fw": "4.7.0", "agent_uid": "backfill-live",
                               "vehicle_type": 2})
except urllib.error.URLError as e:
    print(f"skip：連不上地面站 {BASE}（{e}）。**skip ≠ pass**")
    sys.exit(2)
drone_id = reg.get("drone_id")
print(f"測試機：{reg.get('name')}（{drone_id}）")

g = Watched()
N = 5
try:
    print("\n── 冷開機情境：時鐘還停在 1970 的時候取樣 ─────────────")
    # 這台 Pi 每次開機都是這樣：RTC 沒電池，系統時間從 1970 起算，而
    # board_uid 來自飛控 UART（幾秒就有）、gs_link_ok 預設 False，
    # 兩個取樣條件都在 NTP 同步之前就成立
    real_time = time.time
    LINK["ok"] = False
    base = time.monotonic()
    try:
        time.time = lambda: 14.0
        for i in range(N):
            g._sample(base + i)
        LINK["ok"] = True
        g._flush()
        chk("時鐘還沒同步時不送（換算基準是錯的）",
            len(g.buf) == N and not g.replies,
            f"buf={len(g.buf)} 送出 {len(g.replies)} 批")
    finally:
        time.time = real_time              # NTP 把時間校正回來

    print("\n── 5G 恢復、NTP 同步之後補傳 ──────────────────────────")
    g._flush()
    res = g.replies[-1] if g.replies else {}
    chk("地面站收下了", bool(res.get("ok")), json.dumps(res, ensure_ascii=False))
    chk(f"**{N} 筆全部入庫**（不是被時間戳過濾掉）", res.get("inserted") == N,
        f"inserted={res.get('inserted')}")
    chk("沒有任何一筆被判定時間戳不合理",
        res.get("rejected_implausible") == 0
        and g.stats()["rejected"] == 0,
        f"rejected={res.get('rejected_implausible')}")
    chk("緩衝清空、計數對得上",
        len(g.buf) == 0 and g.sent == N and g.failed == 0, g.stats())
    chk("地面站認出這是同一趟（stayed_armed 傳到了）",
        res.get("stayed_armed") is True, res.get("stayed_armed"))

    print("\n── 重送同一段：以先到的為準，不是長出第二份 ───────────")
    for i in range(N):
        g._sample(base + i)                # 一模一樣的時間戳
    g._flush()
    res2 = g.replies[-1]
    chk("重複的被去重（skipped_duplicate）",
        res2.get("skipped_duplicate") == N and res2.get("inserted") == 0,
        json.dumps(res2, ensure_ascii=False))

    print("\n── 時間戳真的是取樣當下，不是補傳當下 ─────────────────")
    # 全部集中在補傳那一刻＝軌跡變成一個點，那等於沒補
    # 用 _batch 重算同一批的換算結果來比對（不動緩衝）
    ts = [r["t"] for r in g._batch([{"_m": base + i} for i in range(N)])]
    span = ts[-1] - ts[0]
    chk(f"{N} 筆橫跨 {N - 1} 秒（1 Hz 取樣的間距被保住）",
        span is not None and abs(span - (N - 1)) < 0.05, f"span={span}")
finally:
    if drone_id:
        try:
            api(f"/drones/{drone_id}", method="DELETE")
            print(f"\n（已清掉測試機 {drone_id}）")
        except Exception as e:
            print(f"\n**清不掉測試機 {drone_id}**：{e}")

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
