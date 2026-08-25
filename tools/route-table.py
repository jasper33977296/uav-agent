#!/usr/bin/env python3
"""續飛航點選法與提案的測試（機上唯一的實作）。**不連飛機、不送指令。**

測的是那三個限定（狀態機文件 §6.2）——它們少一個都會把「最近的航點」變成
危險的規則：

  1. 續飛到 NAV_TAKEOFF ＝叫一台在空中的機重新執行起飛
  2. 續飛到 NAV_LAND／NAV_RTL ＝當場降落或直接返航
  3. 用 3D 距離會選到正下方但低很多的航點＝續飛過去立刻下降

以及提案本身要說得出「會怎麼調整」（§6.3）：距離、方位、高度差、掉頭警告。
**確認畫面只問「確定嗎？」是不合格的**——那種確認框沒有資訊，人只會照按。

**這是唯一一份實作**（使用者裁定 2026-08-25：統一在機上算）。地面站不再有
備援副本——再寫一份就是第二個事實來源，而它的症狀特別陰險：同一台機，有代理
時飛去 A 點、代理掉線時飛去 B 點，沒有錯誤訊息，只有事後看軌跡覺得怪。
代價是沒有代理的機做不了飛行中改航線，那時候地面站會老實說做不了。

用法：python3 tools/route-table.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import route as CR  # noqa: E402

# 以台北一帶為基準，經度每度約 101 km、緯度每度約 111 km
LAT, LON = 25.0550, 121.5060


def wp(i, dlat_m=0.0, dlon_m=0.0, alt=30.0, command=16):
    return {"seq": i, "lat": LAT + dlat_m / 111000.0,
            "lon": LON + dlon_m / 101000.0, "alt": alt,
            "action": "waypoint", "command": command}


ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


print("── §6.2 限定一：起飛／降落／返航項不得被選中 ──────────")
wps = [
    wp(0, 0, 0, command=22),          # NAV_TAKEOFF——就在機體正下方
    wp(1, 0, 0, command=21),          # NAV_LAND——也在正下方
    wp(2, 0, 0, command=20),          # NAV_RTL
    wp(3, 300, 0, command=16),        # 真正的導航航點，300 m 外
]
r = CR.pick_resume_wp(wps, LAT, LON)
chk("最近的三個是起飛/降落/返航 → 全部跳過，選 300m 外的導航點",
    r and r["index"] == 3, r and f"index={r['index']} dist={r['distance_m']}m")

print("\n── §6.2 限定二：用水平距離，不用 3D ──────────────────")
wps = [
    wp(0, 5, 0, alt=2.0),             # 水平 5 m，但低 28 m
    wp(1, 40, 0, alt=30.0),           # 水平 40 m，同高
]
r = CR.pick_resume_wp(wps, LAT, LON)
chk("正下方但低很多的點確實比較近（水平 5m）→ 就是要選它，不是選同高的",
    r and r["index"] == 0, r and f"index={r['index']}")
# 但提案必須把「會下降」講出來，不能只是選了就算
p = CR.build_proposal(wps=wps, cur={"lat": LAT, "lon": LON, "alt_rel": 30.0,
                                    "heading": 0.0},
                      hold_alt=30.0, mission_name="A", mission_id="m1")
chk("選了低點就必須在提案裡說「會下降」",
    any("下降" in w for w in p["warnings"]), p["warnings"])

print("\n── §6.3 提案要說得出「會怎麼調整」 ────────────────────")
wps = [wp(0, -400, 0), wp(1, 800, 0)]     # 最近的在正南 400 m
p = CR.build_proposal(
    wps=wps, cur={"lat": LAT, "lon": LON, "alt_rel": 20.0, "heading": 0.0},
    hold_alt=25.0, mission_name="航線B", mission_id="m1", cur_seq=2)
chk("有續飛航點", p["resume_wp"] is not None)
chk("有距離", p["resume_wp"]["distance_m"] > 300, p["resume_wp"]["distance_m"])
chk("有方位（正南≈180°）", 175 <= p["resume_wp"]["bearing_deg"] <= 185,
    p["resume_wp"]["bearing_deg"])
chk("有懸停高度差（20→25＝+5）", p["hold"]["alt_delta_m"] == 5.0,
    p["hold"]["alt_delta_m"])
chk("機體朝北、航點在南 → 警告會先掉頭",
    any("掉頭" in w for w in p["warnings"]), p["warnings"])
chk("三步序列寫出來了", len(p["steps"]) == 3, p["steps"])
chk("步驟裡指名續飛到第幾點", "第 0 點" in p["steps"][2], p["steps"][2])

print("\n── 算不出來的時候要說算不出來，不是給個猜的答案 ────────")
p = CR.build_proposal(wps=wps, cur={"lat": None, "lon": None, "alt_rel": 20.0},
                      hold_alt=25.0, mission_name="A", mission_id="m1")
chk("沒有位置 → ok=false 並說明原因", not p["ok"] and p["blockers"],
    p["blockers"])
# **0,0 不是位置**：GPS 沒定位時自駕儀送 0/0，照收的話會從幾內亞灣外海去算
# 「最近的航點」，而且算得出一個看起來很正常的數字
p = CR.build_proposal(wps=wps, cur={"lat": 0.0, "lon": 0.0, "alt_rel": 20.0},
                      hold_alt=25.0, mission_name="A", mission_id="m1")
chk("0,0 被當成「不知道」而不是幾內亞灣", not p["ok"] and p["blockers"],
    p["blockers"])
p = CR.build_proposal(
    wps=[wp(0, 0, 0, command=21)],        # 整條航線只有一個降落項
    cur={"lat": LAT, "lon": LON, "alt_rel": 20.0}, hold_alt=None,
    mission_name="A", mission_id="m1")
chk("新航線沒有可續飛的導航點 → 擋下並說明", not p["ok"], p["blockers"])

print("\n── §5.1 提案會過期，執行前一定重算 ────────────────────")
old = CR.build_proposal(wps=[wp(0, 100, 0), wp(1, 900, 0)],
                        cur={"lat": LAT, "lon": LON, "alt_rel": 20.0},
                        hold_alt=None, mission_name="A", mission_id="m1")
# 機體飛到第二個航點附近了 → 最近點換人
moved = CR.build_proposal(wps=[wp(0, 100, 0), wp(1, 900, 0)],
                          cur={"lat": LAT + 900 / 111000.0, "lon": LON,
                               "alt_rel": 20.0},
                          hold_alt=None, mission_name="A", mission_id="m1")
d = CR.drift_reason(old, moved)
chk("續飛航點換了 → 中止重提", d and "續飛航點" in d, d)
same = CR.build_proposal(wps=[wp(0, 100, 0), wp(1, 900, 0)],
                         cur={"lat": LAT + 5 / 111000.0, "lon": LON,
                              "alt_rel": 20.0},
                         hold_alt=None, mission_name="A", mission_id="m1")
chk("**反向驗證**：只飄了幾公尺 → 不重提（不是什麼都判成過期）",
    CR.drift_reason(old, same) is None, CR.drift_reason(old, same))

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
