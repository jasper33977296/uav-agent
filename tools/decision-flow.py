#!/usr/bin/env python3
"""提案→確認→序列回報，測的是 **agent.py 自己的處理器**（不連飛機、不送指令）。

uav-system 的 `test-intent-guard.py` 測的是傳輸與 guard／route 模組；那支測試
裡的 `on_decision`／`on_progress` 是為測試寫的。**真正跑在機上的是這裡這幾個
方法**，它們有自己的邏輯：提案查表、過期比對、守門二次檢查、序列旗標與死線。
那些邏輯只在真機上跑過就等於沒測過——真機不會為了驗證而飛到 FLYING_MISSION。

用法：python3 tools/decision-flow.py
"""
import sys
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import agent as A  # noqa: E402
import route       # noqa: E402

LAT, LON = 25.0550, 121.5060
WPS = [{"seq": 0, "lat": LAT - 400 / 111000.0, "lon": LON, "alt": 30.0,
        "action": "waypoint", "command": 16},
       {"seq": 1, "lat": LAT + 800 / 111000.0, "lon": LON, "alt": 30.0,
        "action": "waypoint", "command": 16}]


class Fake(A.Agent):
    """只借處理器，不跑 __init__（那會開序列埠與 UDP socket）。"""

    def __init__(self, state="FLYING_MISSION", lat=LAT, lon=LON):
        self._state = state
        self.mon = types.SimpleNamespace(
            state={"gps_fix": "3D", "lat": lat, "lon": lon, "alt_rel_m": 30.0,
                   "heading": 0.0, "mission_seq": 2, "sysid": 1},
            last_hb=time.monotonic())
        self.outbox = A.Outbox()
        self.proposals = {}
        self.seq = None
        self.a = types.SimpleNamespace(seq_timeout_s=120.0)

    def flight_state(self):        # 手餵：總不能為了測試把真飛機弄成別的狀態
        return self._state


ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


def propose(g, iid="i1"):
    return g.on_intent({"intent_id": iid, "action": "change_route",
                        "params": {"wps": WPS, "hold_alt": 25.0,
                                   "mission_name": "測試", "mission_id": "m1"}})


print("── 提案 ───────────────────────────────────────────────")
g = Fake()
ev = propose(g)
chk("回 proposal", ev["event"] == "proposal", ev["event"])
chk("續飛航點是第 0 點（南邊 400m 那個）",
    ev["proposal"]["resume_wp"]["index"] == 0, ev["proposal"]["resume_wp"])
chk("提案留在機上等確認", "i1" in g.proposals)
chk("提案裡留著航點（確認時要用同一份重算）",
    g.proposals["i1"].get("_wps") == WPS)

print("\n── 確認 ───────────────────────────────────────────────")
ev = g.on_decision({"intent_id": "i1", "approved": True})
chk("確認 → cleared", ev["event"] == "cleared", ev.get("reason"))
chk("序列旗標開始", g.seq and g.seq["step"] == "approved", g.seq)
chk("序列旗標有死線（地面站掛掉時不能永遠留著）",
    g.seq["deadline"] > time.monotonic(), g.seq.get("deadline"))

print("\n── 序列逐步回報 ───────────────────────────────────────")
for stp in ("hold", "upload", "goto"):
    ev = g.on_progress({"intent_id": "i1", "step": stp, "ok": True})
    chk(f"{stp} → noted 且序列仍在進行",
        ev["event"] == "noted" and g.seq and g.seq["step"] == stp)
g.on_progress({"intent_id": "i1", "step": "resume", "ok": True})
chk("最後一步後序列結束", g.seq is None)
chk("提案用完就清掉（不留著等人重按）", "i1" not in g.proposals)

print("\n── 某一步失敗就結束序列 ───────────────────────────────")
g = Fake()
propose(g)
g.on_decision({"intent_id": "i1", "approved": True})
g.on_progress({"intent_id": "i1", "step": "upload", "ok": False,
               "detail": "回讀比對不符"})
chk("失敗 → 序列旗標清掉（不會卡住失聯處置）", g.seq is None)

print("\n── 序列死線：地面站中途掛掉 ───────────────────────────")
g = Fake()
propose(g)
g.on_decision({"intent_id": "i1", "approved": True})
g.seq["deadline"] = time.monotonic() - 1        # 假裝已經拖很久
g._expire_seq(time.monotonic())
chk("逾時清掉序列旗標", g.seq is None)
chk("清掉時留一筆事件送地面站（不是安靜地消失）",
    any("逾時" in n["text"] for n in g.outbox.unsent()),
    [n["text"] for n in g.outbox.unsent()][-1:])

print("\n── 確認的位階低於守門（協定 §5.2）─────────────────────")
g = Fake()
propose(g)
g._state = "PILOT_CONTROL"                  # 飛手在人看提案時拿起遙控器
ev = g.on_decision({"intent_id": "i1", "approved": True})
chk("人按了確認，代理仍然拒絕", ev["event"] == "guard_refused", ev.get("reason"))
chk("拒絕後不會留下序列旗標", g.seq is None)

print("\n── 提案過期（協定 §5.1）───────────────────────────────")
g = Fake()
propose(g)
g.mon.state["lat"] = LAT + 800 / 111000.0   # 機體飛到第二個航點附近
ev = g.on_decision({"intent_id": "i1", "approved": True})
chk("過期 → 擋下", ev["event"] == "guard_refused", ev.get("reason"))
chk("附上新提案要人重看", ev.get("proposal") is not None)
chk("新提案取代舊的（下次確認比對的是新的）",
    g.proposals["i1"]["resume_wp"]["index"] == 1)

print("\n── **反向驗證**：只飄幾公尺不該判成過期 ────────────────")
g = Fake()
propose(g)
g.mon.state["lat"] = LAT + 5 / 111000.0
ev = g.on_decision({"intent_id": "i1", "approved": True})
chk("小幅移動仍然 cleared", ev["event"] == "cleared", ev.get("reason"))

print("\n── 取消與找不到提案 ───────────────────────────────────")
g = Fake()
propose(g)
ev = g.on_decision({"intent_id": "i1", "approved": False})
chk("取消 → cancelled 且不留序列", ev["event"] == "cancelled" and g.seq is None)
ev = g.on_decision({"intent_id": "沒看過的 id", "approved": True})
chk("找不到提案 → failed 並說得出怎麼辦",
    ev["event"] == "failed" and "重新取得" in ev["reason"], ev.get("reason"))

print("\n── 乾跑：問判決但不動飛機 ─────────────────────────────")
g = Fake()
g.serial = None            # 真的送指令會在這裡爆 → 證明乾跑沒有走到送出那條路
ev = g.on_intent({"intent_id": "d1", "action": "rtl",
                  "params": {"dry_run": True}})
chk("放行但不執行", ev["event"] == "would_execute", ev.get("reason"))
chk("說得出本來會下什麼模式", ev.get("mode") == "RTL", ev.get("mode"))
g._state = "READY"
ev = g.on_intent({"intent_id": "d2", "action": "rtl",
                  "params": {"dry_run": True}})
chk("**乾跑不會繞過守門**：地上仍然拒絕",
    ev["event"] == "guard_refused", ev.get("reason"))

# **地面站執行的意圖也要吃乾跑**（039 複裁 G）。原本乾跑那道排在執行者分岔
# 之後，所以 start_mission 走的是「回 cleared」——而 `cleared` 在地面站那端
# 就是放行。失聯期間壓下來的意圖恢復後補送、每則都帶 dry_run，少了這道，
# 一個人在四十秒前按下的「開始任務」會在恢復的瞬間真的執行
g = Fake(state="READY")
ev = g.on_intent({"intent_id": "d3", "action": "start_mission",
                  "params": {"dry_run": True}})
chk("**地面站執行的意圖乾跑時不回 cleared**（cleared＝放行執行）",
    ev["event"] == "would_execute", f'{ev["event"]}｜{ev.get("reason")}')
chk("乾跑的回話說得出沒有交出去執行",
    "沒有" in (ev.get("reason") or ""), ev.get("reason"))
# **反向驗證**：不是把 start_mission 一律改成不執行
ev = g.on_intent({"intent_id": "d4", "action": "start_mission", "params": {}})
chk("**反向驗證**：沒帶 dry_run 時照樣 cleared 交給地面站",
    ev["event"] == "cleared" and ev["executor"] == "ground", ev.get("reason"))
# 改航線的乾跑仍然要算提案——補送要的正是「重算提案再要人確認」
g2 = Fake()
ev = g2.on_intent({"intent_id": "d5", "action": "change_route",
                   "params": {"wps": WPS, "hold_alt": 25.0, "dry_run": True}})
chk("乾跑的 change_route 仍然算提案（算提案不動飛機）",
    ev["event"] == "proposal", ev.get("reason"))

print("\n── 複裁 A：RC 前置條件走得通整條 on_intent ────────────")
g = Fake(state="READY")
g.mon.state["rc_link"] = False
ev = g.on_intent({"intent_id": "r1", "action": "start_mission", "params": {}})
chk("RC 掉線 → 守門擋下 start_mission",
    ev["event"] == "guard_refused" and "遙控器" in ev["reason"],
    ev.get("reason"))
# **補送擋得住的關鍵**：地面站的按鈕擋得住人，擋不住四十秒前壓下來的那則
ev = g.on_intent({"intent_id": "r2", "action": "start_mission",
                  "params": {"dry_run": True}})
chk("RC 掉線時補送的 start_mission 也擋得下",
    ev["event"] == "guard_refused", ev.get("event"))
g.mon.state["rc_link"] = True
ev = g.on_intent({"intent_id": "r3", "action": "start_mission", "params": {}})
chk("**反向驗證**：RC 接回來就放行", ev["event"] == "cleared", ev.get("reason"))
g.mon.state["rc_link"] = None
ev = g.on_intent({"intent_id": "r4", "action": "start_mission", "params": {}})
chk("**rc_link=None（韌體沒回報）不擋**", ev["event"] == "cleared",
    ev.get("reason"))

print("\n── 沒帶航點的意圖 ─────────────────────────────────────")
g = Fake()
ev = g.on_intent({"intent_id": "i9", "action": "change_route", "params": {}})
chk("沒航點 → failed（不是算一個空提案）", ev["event"] == "failed",
    ev.get("reason"))

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
