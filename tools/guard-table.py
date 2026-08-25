#!/usr/bin/env python3
"""守門判定表（狀態機文件 §2、協定 §5.2）。**不連飛機、不送指令。**

**為什麼要全表跑一次**：守門是 11 個狀態 × 7 個意圖＝77 格。讀碼讀不出哪一格
填錯，而填錯的兩個方向代價不對稱——該擋沒擋＝在飛手手上飛的時候被系統搶走
控制權；不該擋卻擋了＝操作員按了沒反應，然後去找繞道。

用法：python3 tools/guard-table.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import guard  # noqa: E402

STATES = ["DISCONNECTED", "NOT_READY", "READY", "ARMED_GROUND", "TAKING_OFF",
          "FLYING_MISSION", "HOLDING", "RETURNING", "LANDING",
          "PILOT_CONTROL", "EMERGENCY"]
ACTIONS = ["start_mission", "pause", "resume", "change_route",
           "rtl", "land", "abort"]

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


print("── 全表（○＝允許、·＝擋下）─────────────────────────────")
print(f"{'狀態':<16}" + "".join(f"{a[:6]:>8}" for a in ACTIONS))
for st in STATES:
    row = ""
    for a in ACTIONS:
        allowed, _ = guard.check(st, a)
        row += f"{'○' if allowed else '·':>8}"
    print(f"{st:<16}{row}")

print("\n── 不可退讓的幾格 ─────────────────────────────────────")
# §5.4：飛手拿著就什麼都不做。這是最危險的一種分岔
chk("PILOT_CONTROL 全部拒絕",
    all(not guard.check("PILOT_CONTROL", a)[0] for a in ACTIONS))
chk("DISCONNECTED 全部拒絕",
    all(not guard.check("DISCONNECTED", a)[0] for a in ACTIONS))
# §3：禁止在飛行中重下 start_mission（會從頭開始飛）
chk("FLYING_MISSION 不准重下 start_mission",
    not guard.check("FLYING_MISSION", "start_mission")[0])
chk("HOLDING 不准重下 start_mission",
    not guard.check("HOLDING", "start_mission")[0])
# 地上 RTL ＝一次沒有人按下的起飛（與失聯處置 ARMED_GROUND 同一個理由）
chk("地面（READY／ARMED_GROUND）不准 rtl",
    not guard.check("READY", "rtl")[0]
    and not guard.check("ARMED_GROUND", "rtl")[0])
chk("已在返航／降落時不准再下 rtl／land 以外的東西",
    not guard.check("RETURNING", "pause")[0]
    and not guard.check("LANDING", "change_route")[0])
chk("飛任務中可以改航線", guard.check("FLYING_MISSION", "change_route")[0])
chk("暫停中可以續飛", guard.check("HOLDING", "resume")[0])
chk("暫停中不准再暫停", not guard.check("HOLDING", "pause")[0])

print("\n── 白名單而不是黑名單 ─────────────────────────────────")
allowed, why = guard.check("SOME_NEW_STATE", "rtl")
chk("表上沒有的狀態 → 拒絕（不是放行）", not allowed, why)
allowed, why = guard.check("FLYING_MISSION", "do_a_barrel_roll")
chk("不認得的意圖 → 拒絕", not allowed, why)

print("\n── 拒絕的理由要說得出「那現在能做什麼」 ───────────────")
bad = []
for st in STATES:
    for a in ACTIONS:
        allowed, why = guard.check(st, a)
        if allowed:
            continue
        if not why:
            bad.append(f"{st}/{a} 沒有理由")
        elif "現在可以做" not in why and "現在什麼都不做" not in why:
            bad.append(f"{st}/{a} 理由沒說現在能做什麼：{why}")
chk("每一格拒絕都說得出現在能做什麼", not bad, bad[:3])

print("\n── 執行者分工（丙案）──────────────────────────────────")
chk("代理執行的只有 rtl／land／abort",
    guard.AGENT_ACTIONS == {"rtl", "land", "abort"}, sorted(guard.AGENT_ACTIONS))
chk("需要任務方言的都留給地面站",
    all(guard.executor(a) == "ground"
        for a in ("pause", "resume", "change_route", "start_mission")))
chk("但守門一律先過代理",
    all(guard.check("FLYING_MISSION", a)[0] is not None for a in ACTIONS))

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
