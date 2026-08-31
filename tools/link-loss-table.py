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
import time
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
        owner = st.pop("_owner", None)
        by_seq = st.pop("_by_seq", False)
        st.setdefault("armed", True)
        # 心跳新鮮＝不是 DISCONNECTED。要測 DISCONNECTED 就傳 _hb_age
        hb_age = st.pop("_hb_age", 0.0)
        self.mon = types.SimpleNamespace(state=st, events=[],
                                         last_hb=time.time() - hb_age)
        self.mode_owner = owner
        self.loss_started = 0.0
        self.seq = st.pop("_seq", None)   # 改航線序列進行中？
        self._solo_logged = True            # 靜音，測的是判定不是播報
        self.a = types.SimpleNamespace(
            link_loss_max_solo_s=30.0, link_loss_min_batt=15,
            # 複裁 C 的替代完成判定：**預設關**，與正式預設一致。
            # 測試若預設開著，就測不出「預設是關的」這件事本身
            mission_complete_by_seq=by_seq)


def case(label, want_state, want_verdict, *, solo=10.0, **st):
    g = Fake(**st)
    state = g.flight_state()
    verdict = g._loss_verdict(state, solo)
    got = ("RTL：" + verdict) if isinstance(verdict, str) else \
          ("不介入" if verdict is False else "續飛觀察")
    ok = state == want_state and got.split("：")[0] == want_verdict
    print(f"{'✓' if ok else '✗'} {label:<34} → {state:<15} {got}")
    return ok


AIR = dict(landed_state="in_air")

ok = True
print("── 連不上／未起飛 ─────────────────────────────────────────")
ok &= case("飛控沒心跳", "DISCONNECTED", "不介入", _hb_age=30.0, mode="AUTO")
ok &= case("未 armed、GPS 未定位", "NOT_READY", "不介入",
           armed=False, mode="STABILIZE", gps_fix="NO_FIX")
ok &= case("未 armed、GPS 已定位", "READY", "不介入",
           armed=False, mode="STABILIZE", gps_fix="3D")
ok &= case("飛控 failsafe 中（CRITICAL）", "EMERGENCY", "不介入",
           mode="AUTO", system_status=5, **AIR)

print("\n── 已在自己處理／人拿著 ───────────────────────────────")
ok &= case("已在 RTL", "RETURNING", "不介入", mode="RTL", **AIR)
ok &= case("已在 LAND", "LANDING", "不介入", mode="LAND", **AIR)
ok &= case("landed_state=landing", "LANDING", "不介入",
           mode="GUIDED", landed_state="landing")
ok &= case("飛手手飛 STABILIZE", "PILOT_CONTROL", "不介入",
           mode="STABILIZE", **AIR)
ok &= case("飛手切的 LOITER（溯源＝pilot）", "PILOT_CONTROL", "不介入",
           mode="LOITER", _owner="pilot", **AIR)
ok &= case("溯源不明的 LOITER", "PILOT_CONTROL", "不介入",
           mode="LOITER", _owner=None, **AIR)

print("\n── 地面與起飛 ─────────────────────────────────────────")
ok &= case("armed 在地上（landed_state）", "ARMED_GROUND", "不介入",
           mode="GUIDED", landed_state="on_ground")
ok &= case("armed 在地上（韌體不回報，靠高度）", "ARMED_GROUND", "不介入",
           mode="GUIDED", alt_rel_m=0.2, groundspeed_ms=0.0)
ok &= case("爬升中", "TAKING_OFF", "RTL",
           mode="GUIDED", landed_state="takeoff")

print("\n── 改航線序列中失聯（§7）───────────────────────────────")
# 序列跑到一半失聯＝飛機停在一個沒有人確認過的中間狀態，而接下來每一步都需要
# 地面站——它已經不在了。所以不管當下是哪個狀態，都立刻 RTL
ok &= case("序列進行中（在飛任務）→ 不續飛，立刻 RTL", "FLYING_MISSION", "RTL",
           solo=5.0, mode="AUTO", mission_state=3, battery_pct=90,
           _seq={"intent_id": "x", "step": "upload"}, **AIR)
ok &= case("序列進行中（暫停中）→ 立刻 RTL", "HOLDING", "RTL", solo=5.0,
           mode="GUIDED", battery_pct=90,
           _seq={"intent_id": "x", "step": "hold"}, **AIR)
# **反向驗證**：序列不在時，同樣的狀態要回到正常的續飛判斷
ok &= case("**反向驗證**：沒有序列時同一格是續飛觀察", "FLYING_MISSION",
           "續飛觀察", solo=5.0, mode="AUTO", mission_state=3, battery_pct=90,
           **AIR)
# 飛手拿著的時候，序列旗標不得讓我們去動飛機
ok &= case("序列進行中但飛手接管 → 仍然不介入", "PILOT_CONTROL", "不介入",
           solo=5.0, mode="STABILIZE",
           _seq={"intent_id": "x", "step": "hold"}, **AIR)

print("\n── 表沒列到的狀態不得下指令 ───────────────────────────")
g = Fake(mode="AUTO", landed_state="in_air")
unknown = g._loss_verdict("SOME_NEW_STATE", 10.0) is False
print(f"{'✓' if unknown else '✗'} 未列入的狀態 → 不動作（而不是掉進續飛分支）")
ok &= unknown

print("\n── 空中 ───────────────────────────────────────────────")
ok &= case("地面站按的暫停（LOITER）", "HOLDING", "RTL",
           mode="LOITER", _owner="us", **AIR)
ok &= case("GUIDED 停在空中", "HOLDING", "RTL", mode="GUIDED", **AIR)
ok &= case("任務進行中（上限內）", "FLYING_MISSION", "續飛觀察", solo=20.0,
           mode="AUTO", mission_seq=3, mission_total=9, mission_state=3,
           battery_pct=80, **AIR)
ok &= case("任務進行中但電量 12%", "FLYING_MISSION", "RTL", solo=20.0,
           mode="AUTO", mission_state=3, battery_pct=12, **AIR)
ok &= case("任務進行中但單飛超時", "FLYING_MISSION", "RTL", solo=45.0,
           mode="AUTO", mission_state=3, battery_pct=80, **AIR)
ok &= case("任務跑完（mission_state=5）", "HOLDING", "RTL",
           mode="AUTO", mission_state=5, battery_pct=80, **AIR)
ok &= case("舊韌體不回報 mission_state", "FLYING_MISSION", "續飛觀察", solo=20.0,
           mode="AUTO", mission_state=0, battery_pct=80, **AIR)
# 複裁 E（2026-08-31）：**讀不到電量＝不准續飛**。原本這格是「續飛觀察」，
# 因為 `batt is not None and ...` 在電量未知時把整項檢查跳過了——等於把
# 「不知道」當成「電量充足」，而這條規則存在的前提就是知道還剩多少電
ok &= case("電量讀不到 → 不准續飛（複裁 E）", "FLYING_MISSION", "RTL", solo=20.0,
           mode="AUTO", mission_state=3, battery_pct=None, **AIR)
# **反向驗證**：不是把電量檢查一路改成 RTL——讀得到而且夠，還是要續飛
ok &= case("**反向驗證**：電量讀得到且充足仍續飛", "FLYING_MISSION", "續飛觀察",
           solo=20.0, mode="AUTO", mission_state=3, battery_pct=80, **AIR)

print("\n── 複裁 C：seq 替代完成判定（預設關）──────────────────")
# 旗標關著時，seq 到底不得改變任何判定——**預設關**本身就是要測的東西
ok &= case("旗標關：seq 到底也不算跑完，靠時間上限收尾", "FLYING_MISSION",
           "續飛觀察", solo=20.0, mode="AUTO", mission_state=0,
           mission_seq=9, mission_total=9, battery_pct=80, **AIR)
ok &= case("旗標開：seq 到底＝跑完 → RTL", "FLYING_MISSION", "RTL", solo=20.0,
           mode="AUTO", mission_state=0, mission_seq=9, mission_total=9,
           battery_pct=80, _by_seq=True, **AIR)
ok &= case("旗標開但還沒飛到底 → 續飛觀察", "FLYING_MISSION", "續飛觀察",
           solo=20.0, mode="AUTO", mission_state=0, mission_seq=4,
           mission_total=9, battery_pct=80, _by_seq=True, **AIR)
# 有權威值就用權威值：mission_state 有回報時，替代判定不得插手
ok &= case("旗標開但韌體有回報 mission_state → 用權威值", "FLYING_MISSION",
           "續飛觀察", solo=20.0, mode="AUTO", mission_state=3, mission_seq=9,
           mission_total=9, battery_pct=80, _by_seq=True, **AIR)
ok &= case("旗標開但 total 不明（0）→ 不猜", "FLYING_MISSION", "續飛觀察",
           solo=20.0, mode="AUTO", mission_state=0, mission_seq=9,
           mission_total=0, battery_pct=80, _by_seq=True, **AIR)

print("\n── 反向驗證：斷言真的分得出來 ──────────────────────────")
# 同一個 LOITER，只改溯源，處置必須相反——這是整套規則最容易寫錯的一格
a = Fake(mode="LOITER", landed_state="in_air", _owner="us")
b = Fake(mode="LOITER", landed_state="in_air", _owner="pilot")
flip = (isinstance(a._loss_verdict(a.flight_state(), 10.0), str)
        and b._loss_verdict(b.flight_state(), 10.0) is False)
print(f"{'✓' if flip else '✗'} 只改模式歸屬，LOITER 的處置由 RTL 翻成不介入")
ok &= flip

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
