#!/usr/bin/env python3
"""守門判定表（狀態機文件 §2、協定 §5.2）。**不連飛機、不送指令。**

**為什麼要全表跑一次**：守門是 11 個狀態 × 8 個意圖＝88 格。讀碼讀不出哪一格
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
           "rtl", "land", "abort", "disarm"]

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
# 地上 abort＝送一個沒有意義的模式指令。守門的工作是只放行有意義的
chk("地面（NOT_READY／READY／ARMED_GROUND）不准 abort",
    not guard.check("NOT_READY", "abort")[0]
    and not guard.check("READY", "abort")[0]
    and not guard.check("ARMED_GROUND", "abort")[0])
# **空中上鎖＝馬達停轉、飛機直接掉下來。** 這不是「停止」，是墜毀
chk("空中一律不准 disarm",
    not any(guard.check(st, "disarm")[0]
            for st in ("FLYING_MISSION", "HOLDING", "TAKING_OFF",
                       "RETURNING", "LANDING", "PILOT_CONTROL", "EMERGENCY")))
chk("**反向驗證**：地上 armed 時要能 disarm（不是把 disarm 一律關掉）",
    guard.check("ARMED_GROUND", "disarm")[0])
chk("**反向驗證**：空中還是要能 abort（不是把 abort 一律關掉）",
    guard.check("FLYING_MISSION", "abort")[0]
    and guard.check("HOLDING", "abort")[0]
    and guard.check("RETURNING", "abort")[0])

print("\n── 複裁 A：RC 未連線不得起飛／開始任務 ────────────────")
# 地面站把 takeoff／mission_fly／arm／群組 upload 全部映射到 start_mission
# （apps/command/app/guard_client.py），所以擋住這一個意圖就同時擋住了
# 單機起飛與群組同時起飛——裁定要的正是這兩件事
GROUND_STATES = ["NOT_READY", "READY", "ARMED_GROUND"]
chk("RC 掉線時地面三個狀態都不准 start_mission",
    all(not guard.check(st, "start_mission", False)[0] for st in GROUND_STATES))
# **三態的分界就是整條規則的地基**：None 是「這台韌體沒回報」＝不知道，
# 把它當成「沒有 RC」會讓所有還沒升級／還沒驗證過的機都起飛不了（036 的教訓）
chk("**rc_link=None（不知道）不擋**——不是把沒回報當成沒有 RC",
    all(guard.check(st, "start_mission", None)[0] for st in ("READY",
                                                            "ARMED_GROUND")))
chk("**反向驗證**：RC 在線時照樣放行（不是把起飛一律關掉）",
    guard.check("READY", "start_mission", True)[0]
    and guard.check("ARMED_GROUND", "start_mission", True)[0])
# 兩參數呼叫維持原行為：地面站的 test-intent-guard.py 就是這樣叫的
chk("省略 rc_link 的舊呼叫不受影響",
    guard.check("READY", "start_mission")[0])
# RC 掉線時更需要「把飛機弄回地面」那個方向能用
chk("RC 掉線不影響 rtl／land／abort（那是把飛機弄回地面的方向）",
    all(guard.check("FLYING_MISSION", a, False)[0]
        for a in ("rtl", "land", "abort")))
chk("RC 掉線不影響 change_route／pause／resume",
    guard.check("FLYING_MISSION", "change_route", False)[0]
    and guard.check("FLYING_MISSION", "pause", False)[0]
    and guard.check("HOLDING", "resume", False)[0])
# 狀態本來就擋掉的格子，理由要維持原本那句——它比「遙控器未連線」更說得出
# 當下該做什麼（在飛任務中的人要知道的是「用改航線」，不是遙控器）
_, why_air = guard.check("FLYING_MISSION", "start_mission", False)
chk("狀態本來就擋的格子，理由不被 RC 那句蓋掉", "已經在飛任務了" in why_air,
    why_air)
_, why_rc = guard.check("READY", "start_mission", False)
chk("RC 拒絕的理由說得出現在能做什麼", "現在可以做" in why_rc, why_rc)
chk("RC 拒絕的理由說得出是遙控器的問題", "遙控器" in why_rc)

#: 「說得出下一步」的**判準，不是字串**。原本這裡寫死找「現在可以做」四個字，
#: 於是 2026-09-02 把 `NOT_READY` 的尾句改成更可行動的「先讓它就緒」時，
#: 這一格反而變紅——**測試在懲罰一個變好的訊息**。
#:
#: 那句改動的理由：`NOT_READY` 的意思就是「它現在飛不了」，照樣列出白名單裡的
#: `start_mission` 是一句誤導（它會在飛控預檢那關被擋，而使用者會照著我們的話
#: 去撞一次牆）。真正能做的事不是任何一個意圖，是**讓它就緒**。
#:
#: 所以判準改成「有沒有指出下一步」，而不是「有沒有那個字串」。
_NEXT_STEP = ("現在可以做", "現在什麼都不做", "先讓它就緒",
              "直接上傳那一份", "請先確認")

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
        elif not any(k in why for k in _NEXT_STEP):
            bad.append(f"{st}/{a} 理由沒說現在能做什麼：{why}")
chk("每一格拒絕都說得出現在能做什麼", not bad, bad[:3])

print("\n── 執行者分工（丙案）──────────────────────────────────")
chk("代理執行的只有 rtl／land／abort",
    guard.AGENT_ACTIONS == {"rtl", "land", "abort"}, sorted(guard.AGENT_ACTIONS))
chk("需要任務方言的都留給地面站",
    all(guard.executor(a) == "ground"
        for a in ("pause", "resume", "change_route", "start_mission",
                  "disarm")))
chk("但守門一律先過代理",
    all(guard.check("FLYING_MISSION", a)[0] is not None for a in ACTIONS))

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
