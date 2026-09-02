#!/usr/bin/env python3
"""換號的狀態機（uav-system issues/040 A3）：**唯一會重開飛控的動作**。

所以守門比其他地方都嚴，本測試把每一格釘住：

  1. **只在地面**——在天上不換（換號要重開飛控）
  2. **一個行程只試一次**——失敗留給人，不要反覆重開飛控
  3. **讀回確認才重開**（ACK 是我收到了，不是我做到了）
  4. **重開期間回報 `reassigning`**——沒有它，飛控消失會被 039 當成失聯，
     我們會為了修好身分而讓飛機以為地面站不見了
  5. 逾時要放棄並大聲說，不要無限等一個不會回來的東西
  6. 用新號碼回來 → 清掉旗標並**重新註冊**（讓地面站看到 keep）

不需要飛控也不需要地面站：用假的序列埠與假狀態。

用法：python3 tools/sysid-reassign.py
"""
import sys
import types

sys.path.insert(0, "/home/k200/uav-agent")

import agent as A  # noqa: E402

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


class FakeMav:
    def __init__(self):
        self.sets, self.reads, self.reboots = [], [], []

    def param_set_send(self, tgt, comp, name, val, typ):
        self.sets.append((tgt, name, val))

    def param_request_read_send(self, tgt, comp, name, idx):
        self.reads.append(name)

    def command_long_send(self, tgt, comp, cmd, *a):
        if cmd == 246:                      # PREFLIGHT_REBOOT_SHUTDOWN
            self.reboots.append(tgt)


def mk(state="READY", sysid=1, assigned=2, action="change", auto=True):
    ag = A.Agent.__new__(A.Agent)
    ag.mon = types.SimpleNamespace(state={"sysid": sysid, "params": {}})
    ag.serial = types.SimpleNamespace(mav=FakeMav())
    ag.a = types.SimpleNamespace(sysid_auto_apply=auto)
    ag.assigned_sysid, ag.sysid_action = assigned, action
    ag.reassigning, ag.reassign_tried, ag.hello_sent = None, False, True
    ag.flight_state = lambda: state
    return ag


print("── 1. 在天上不換（換號要重開飛控）──────────────────────")
ag = mk(state="FLYING_MISSION")
ag._maybe_reassign(100.0)
chk("飛行中不動作", ag.reassigning is None and not ag.serial.mav.sets)
chk("**沒有送出重開**", not ag.serial.mav.reboots)

print("\n── 2. 地面：送 PARAM_SET，但還不重開 ───────────────────")
ag = mk(state="READY")
ag._maybe_reassign(100.0)
chk("送出 MAV_SYS_ID=2", ag.serial.mav.sets and ag.serial.mav.sets[0][2] == 2.0,
    ag.serial.mav.sets)
chk("**還沒重開**（要先讀回確認）", not ag.serial.mav.reboots)
chk("進入 reassigning 並回報 from/to",
    ag.reassigning and (ag.reassigning["from"], ag.reassigning["to"]) == (1, 2),
    ag.reassigning)

print("\n── 3. 讀回不符時不重開 ─────────────────────────────────")
ag.mon.state["params"]["MAV_SYS_ID"] = 9      # 機端回了別的值
ag._maybe_reassign(101.0)
chk("**值不對就不重開**", not ag.serial.mav.reboots, ag.serial.mav.sets)

print("\n── 4. 讀回相符 → 重開 ──────────────────────────────────")
ag.mon.state["params"]["MAV_SYS_ID"] = 2
ag._maybe_reassign(102.0)
chk("送出重開", len(ag.serial.mav.reboots) == 1)
chk("phase 進到 rebooting", ag.reassigning["phase"] == "rebooting")

print("\n── 5. 用新號碼回來 → 清旗標並重新註冊 ──────────────────")
ag.mon.state["sysid"] = 2
ag._maybe_reassign(103.0)
chk("reassigning 清掉", ag.reassigning is None)
chk("action 變成 keep", ag.sysid_action == "keep")
chk("**重新註冊**（讓地面站看到它用對號碼了）", ag.hello_sent is False)

print("\n── 6. 逾時：放棄，不要無限等 ───────────────────────────")
ag2 = mk(state="READY")
ag2._maybe_reassign(100.0)
ag2._maybe_reassign(100.0 + A.Agent.REASSIGN_TIMEOUT_S + 1)
chk("逾時後清掉 reassigning（畫面不會永遠停在換號中）",
    ag2.reassigning is None)

print("\n── 7. 一個行程只試一次 ─────────────────────────────────")
before = len(ag2.serial.mav.sets)
ag2._maybe_reassign(200.0)
chk("**不重試**（反覆重開飛控比不換更糟）",
    len(ag2.serial.mav.sets) == before, ag2.serial.mav.sets)

print("\n── 8. 反向驗證：關掉開關就完全不動 ─────────────────────")
ag3 = mk(state="READY", auto=False)
ag3._maybe_reassign(100.0)
chk("--no-sysid-auto-apply 時不送任何東西",
    not ag3.serial.mav.sets and ag3.reassigning is None)

print("\n── 9. 反向驗證：號碼本來就對就不動 ─────────────────────")
ag4 = mk(state="READY", sysid=2, assigned=2, action="keep")
ag4._maybe_reassign(100.0)
chk("keep 時不動作", not ag4.serial.mav.sets and ag4.reassigning is None)

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
