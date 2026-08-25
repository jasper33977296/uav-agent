#!/usr/bin/env python3
"""失聯處置（選項 C）的判定表測試——**不連飛控、不送任何指令**。

**為什麼這段一定要有測試**：它是唯一一段會在沒有人看著的時候自己下指令給
飛機的程式碼，而它的輸入（模式、高度、電量、任務狀態、模式歸屬）是五維的。
讀碼讀不出五維表格裡哪一格填錯了，只有把每一格都跑一次才知道。

判定的三種回傳，對應三種完全不同的後果：
  * 字串   → **立刻送 RTL**（會真的動到飛機）
  * False → 這個狀態永不介入
  * None  → 續飛觀察中，下一拍再評估

用法：python3 tools/link-loss-table.py
"""
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# agent.py 匯入 serial/pymavlink/modem，這台開發機不見得有；判定邏輯完全用不到
for name in ("serial",):
    sys.modules.setdefault(name, types.ModuleType(name))
try:
    import pymavlink  # noqa: F401
except ImportError:
    print("skip：沒有 pymavlink，判定表測不了。**skip ≠ pass**")
    sys.exit(2)

import agent as A


class Fake(A.Agent):
    """只借判定方法，不跑 __init__（那會開序列埠與 UDP socket）。"""

    def __init__(self, **st):
        self.mon = types.SimpleNamespace(state=st, events=[])
        self.mode_owner = st.pop("_owner", None)
        self.loss_started = 0.0
        self._solo_logged = True            # 靜音，測的是判定不是播報
        self.a = types.SimpleNamespace(
            link_loss_max_solo_s=120.0, link_loss_min_batt=40)


def case(label, want_state, want_verdict, *, solo=10.0, **st):
    g = Fake(**st)
    state = g._loss_state()
    verdict = g._loss_verdict(state, solo)
    got = ("RTL：" + verdict) if isinstance(verdict, str) else \
          ("不介入" if verdict is False else "續飛觀察")
    ok = state == want_state and got.split("：")[0] == want_verdict
    print(f"{'✓' if ok else '✗'} {label:<34} → {state:<15} {got}")
    return ok


ok = True
print("── 狀態判定與處置 ─────────────────────────────────────────")
ok &= case("未 armed", "DISARMED", "不介入", armed=False, mode="STABILIZE")
ok &= case("已在 RTL", "RETURNING", "不介入", armed=True, mode="RTL")
ok &= case("已在 LAND", "RETURNING", "不介入", armed=True, mode="LAND")
ok &= case("armed 但還在地上", "ARMED_GROUND", "不介入",
           armed=True, mode="GUIDED", alt_rel_m=0.2, groundspeed_ms=0.0)
ok &= case("飛手手飛 STABILIZE", "PILOT_CONTROL", "不介入",
           armed=True, mode="STABILIZE", alt_rel_m=30, groundspeed_ms=3)
ok &= case("飛手切的 LOITER（溯源＝pilot）", "PILOT_CONTROL", "不介入",
           armed=True, mode="LOITER", alt_rel_m=30, groundspeed_ms=1,
           _owner="pilot")
ok &= case("溯源不明的 LOITER", "PILOT_CONTROL", "不介入",
           armed=True, mode="LOITER", alt_rel_m=30, groundspeed_ms=1,
           _owner=None)
ok &= case("地面站按的暫停（LOITER）", "HOLDING", "RTL",
           armed=True, mode="LOITER", alt_rel_m=30, groundspeed_ms=1,
           _owner="us")
ok &= case("GUIDED 停在空中", "HOLDING", "RTL",
           armed=True, mode="GUIDED", alt_rel_m=30, groundspeed_ms=0.2)
ok &= case("任務進行中（上限內）", "FLYING_MISSION", "續飛觀察",
           armed=True, mode="AUTO", alt_rel_m=40, groundspeed_ms=8,
           mission_seq=3, mission_total=9, mission_state=3, battery_pct=80)
ok &= case("任務進行中但電量 35%", "FLYING_MISSION", "RTL",
           armed=True, mode="AUTO", alt_rel_m=40, groundspeed_ms=8,
           mission_state=3, battery_pct=35)
ok &= case("任務進行中但單飛超時", "FLYING_MISSION", "RTL", solo=200.0,
           armed=True, mode="AUTO", alt_rel_m=40, groundspeed_ms=8,
           mission_state=3, battery_pct=80)
ok &= case("任務跑完（mission_state=5）", "HOLDING", "RTL",
           armed=True, mode="AUTO", alt_rel_m=40, groundspeed_ms=0.3,
           mission_state=5, battery_pct=80)
ok &= case("舊韌體不回報 mission_state", "FLYING_MISSION", "續飛觀察",
           armed=True, mode="AUTO", alt_rel_m=40, groundspeed_ms=8,
           mission_state=0, battery_pct=80)
ok &= case("電量未知不當成低電量", "FLYING_MISSION", "續飛觀察",
           armed=True, mode="AUTO", alt_rel_m=40, groundspeed_ms=8,
           mission_state=3, battery_pct=None)

print("\n── 反向驗證：斷言真的分得出來 ──────────────────────────")
# 同一個 LOITER，只改溯源，處置必須相反——這是整套規則最容易寫錯的一格
a = Fake(armed=True, mode="LOITER", alt_rel_m=30, groundspeed_ms=1, _owner="us")
b = Fake(armed=True, mode="LOITER", alt_rel_m=30, groundspeed_ms=1,
         _owner="pilot")
flip = (isinstance(a._loss_verdict(a._loss_state(), 10.0), str)
        and b._loss_verdict(b._loss_state(), 10.0) is False)
print(f"{'✓' if flip else '✗'} 只改模式歸屬，LOITER 的處置由 RTL 翻成不介入")
ok &= flip

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
