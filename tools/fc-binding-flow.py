#!/usr/bin/env python3
"""一個代理只綁一塊飛控板（uav-system issues/076）：把每一格釘住。

  1. 第一次確認到板號就綁定、寫檔、說一聲
  2. 代理重開：**確認之前不轉發、不收指令**；確認相符才放行
  3. **不重開 Pi、直接換接飛控**（076 B）：空窗之後當成新的一段連線；
     讀到別塊板子 → 鎖住（不轉發、不收指令、不執行意圖、不量訊號、不替它
     說話、不換號），而且要上地面站
  4. 換回原本那塊 → 恢復
  5. **armed 的時候不切**：在天上切斷比認錯更危險
  6. 更換流程：解除 → 下一塊確認到的板子成為綁定，舊的進歷史
  7. 錄檔的歸屬看錄製當下（076 C）
  8. 綁定檔讀不懂 → 一律當作對不上，**不是**當作沒綁過
  9. 不符時的註冊：報綁定的板號，但不帶眼前那塊飛控的任何事實
 10. 確認中沒有回答 → 重問

不需要飛控也不需要地面站：用假的連線與手做的 MAVLink 訊息。

用法：python3 tools/fc-binding-flow.py
"""
import json
import os
import sys
import tempfile
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import agent as A  # noqa: E402
import binding  # noqa: E402
import clock  # noqa: E402
import recorder  # noqa: E402
from pymavlink.dialects.v20 import ardupilotmega as apm  # noqa: E402

clock.synced = lambda: True        # 錄製器對時前不寫檔；這裡不測對時

UID_A = "2a0020001151333139383538"
UID_B = "3b0041000c51333139383538"

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


class FakeLink:
    def __init__(self):
        self.sent = []

    def send(self, buf):
        self.sent.append(buf)


_mav = apm.MAVLink(None, srcSystem=1, srcComponent=1)


def mk(msg):
    msg.pack(_mav)
    return msg


def hb(armed=False):
    return mk(apm.MAVLink_heartbeat_message(
        2, 3, 0x80 if armed else 0, 5 if armed else 0, 4 if armed else 3, 3))


def av(uid):
    raw = bytes.fromhex(uid) if uid else b""
    return mk(apm.MAVLink_autopilot_version_message(
        0, 0x040700ff, 0, 0, 0, [0] * 8, [0] * 8, [0] * 8, 0x1209, 0x5741, 0,
        list(raw.ljust(18, b"\0"))))


def att():
    return mk(apm.MAVLink_attitude_message(0, 0, 0, 0, 0, 0, 0))


def new_agent(bfile, rdir):
    """借 Agent 的方法，不跑 __init__（那會開序列埠與 UDP socket）。"""
    g = A.Agent.__new__(A.Agent)
    g.a = types.SimpleNamespace(sysid_auto_apply=True, gs="127.0.0.1",
                                backend_port=1)
    g.mon = A.Monitor()
    g.binding = binding.Binding(bfile)
    g.outbox = A.Outbox()
    g.telem, g.cmd = FakeLink(), FakeLink()
    g.recorder = recorder.Recorder(rdir, armed_only=False, rotate_min=0)
    g.uploader = None
    g.intent = None
    g.fc_sysid = 1
    g.foreign, g.reassigning = {}, None
    g.quiet_until, g.fc_msgs, g.last_telem_rx = 0.0, 0, 0.0
    g.fc_link_ok = True
    g.caps_requested, g.caps_req_at = False, 0.0
    g.fc_session_uid = None
    g.binding_state, g.binding_why = "verifying", "還沒問到飛控板身分"
    g.last_fc_rx, g.fc_untrusted_drop = 0.0, 0
    g.hello_sent = False
    g.sysid_action, g.assigned_sysid, g.reassign_tried = None, None, False
    g.written = []
    g._write = lambda fd, data: g.written.append(data)
    g.gs_parser = apm.MAVLink(None)
    g.gs_parser.robust_parsing = True
    g.fd = 3
    return g


def notes(g, kind="fc_binding"):
    return [e for e in g.outbox._q if e.get("kind") == kind]


def gs_cmd():
    m = mk(apm.MAVLink_command_long_message(1, 1, 400, 0, 1, 0, 0, 0, 0, 0, 0))
    return m.get_msgbuf()


tmp = tempfile.mkdtemp()
BF = os.path.join(tmp, ".fc_binding.json")
RD = os.path.join(tmp, "rec")
# 換號會寫 .fc_sysid——測試不准寫真的那個檔
A.FC_SYSID_FILE = os.path.join(tmp, ".fc_sysid")

# ── 1. 第一次 ─────────────────────────────────────────────────
print("── 1. 第一次確認到板號就綁定")
g = new_agent(BF, RD)
t = 100.0
g._from_fc(hb(), t)
chk("還沒綁過時照舊放行（沒有東西要保護）", len(g.telem.sent) == 1)
g._from_fc(av(UID_A), t + 0.3)
chk("綁定檔寫出來了", json.load(open(BF)).get("board_uid") == UID_A)
chk("狀態 ok", g.binding_state == "ok", g.binding_why)
chk("第一次綁定有說一聲（info）",
    [n["level"] for n in notes(g)] == ["info"], notes(g))
chk("可以替這台機說話", g._speak_uid() == UID_A)
chk("錄檔標上板號", g.recorder.read_meta(g.recorder.name)["board_uid"] == UID_A)

# ── 2. 代理重開 ─────────────────────────────────────────────────
print("── 2. 代理重開：確認之前鎖著")
g = new_agent(BF, RD)
g._from_fc(hb(), t)
g._from_fc(att(), t + 0.1)
chk("確認之前不轉發", g.telem.sent == [] and g.fc_untrusted_drop == 2)
g.to_fc(g.fd, gs_cmd())
chk("確認之前不收地面站的指令", g.written == [])
chk("確認之前不替它說話", g._speak_uid() is None)
g._from_fc(av(UID_A), t + 0.3)
chk("確認相符 → ok", g.binding_state == "ok")
chk("確認身分的那一則有轉上去（地面站自己也要讀板號）", len(g.telem.sent) == 1)
g.to_fc(g.fd, gs_cmd())
chk("相符之後收指令", len(g.written) == 1)
chk("重開後相符不算一件事（不另外喊）", notes(g) == [])

# ── 3. 熱插拔換飛控 ─────────────────────────────────────────────
print("── 3. 不重開 Pi、直接換接飛控")
file_a = g.recorder.name
t = 200.0
g._from_fc(hb(), t)
g._from_fc(hb(), t + 4.0)              # 4 秒空窗：拔線、接新飛控、開機
chk("空窗之後當成新的一段連線", g.binding_state == "verifying", g.binding_why)
chk("新一段連線要重問板號", g.caps_requested is False)
chk("錄檔換一份（一個檔案只屬於一塊板子）", g.recorder.name != file_a)
n0 = len(g.telem.sent)
g._from_fc(att(), t + 4.1)
chk("確認之前不轉發新飛控的遙測", len(g.telem.sent) == n0)
g._from_fc(av(UID_B), t + 4.4)
chk("讀到別塊板子 → mismatch", g.binding_state == "mismatch", g.binding_why)
chk("揭露不符的那一則沒有轉上去", len(g.telem.sent) == n0)
chk("綁定檔沒有被改", json.load(open(BF))["board_uid"] == UID_A)
crit = [n for n in notes(g) if n["level"] == "critical"]
chk("要上地面站（critical）", len(crit) == 1 and UID_B in crit[0]["text"])
g.to_fc(g.fd, gs_cmd())
chk("不收地面站的指令（那是給 A 的）", len(g.written) == 1)
g._from_fc(att(), t + 5.0)
chk("之後的遙測也不轉", len(g.telem.sent) == n0)
chk("不替它說話（註冊、補傳）", g._speak_uid() is None)
r = g.on_intent({"intent_id": "x", "action": "rtl"})
chk("不執行意圖", r.get("event") == "guard_refused", r.get("reason"))
chk("錄檔照錄，但標的是實際的板號 B",
    g.recorder.read_meta(g.recorder.name)["board_uid"] == UID_B)
g.sysid_action, g.assigned_sysid = "change", 7
g.mon.state["sysid"] = 1
g._maybe_reassign(t + 6)
chk("不對別塊板子換號", g.reassigning is None and not g.reassign_tried)
g._from_fc(av(UID_B), t + 9)
chk("重問的回答不重複喊", len([n for n in notes(g) if n["level"] == "critical"]) == 1)

# ── 4. 換回來 ──────────────────────────────────────────────────
print("── 4. 換回原本那塊")
g._from_fc(hb(), t + 20)
g._from_fc(av(UID_A), t + 20.3)
chk("恢復 ok", g.binding_state == "ok")
chk("恢復要說一聲（warning）", notes(g)[-1]["level"] == "warning", notes(g)[-1])
g.to_fc(g.fd, gs_cmd())
chk("恢復收指令", len(g.written) == 2)

# ── 5. armed 不切 ──────────────────────────────────────────────
print("── 5. armed：在天上不切")
t = 300.0
g._from_fc(hb(armed=True), t)
g._from_fc(av(UID_A), t + 0.3)
chk("起飛前確認相符", g.binding_state == "ok")
g._from_fc(hb(armed=True), t + 4)       # 飛行中序列埠抖了 4 秒
chk("armed 時空窗不切段", g.binding_state == "ok")
chk("但重問一次板號", g.caps_requested is False)
n0 = len(g.telem.sent)
g._from_fc(av(UID_B), t + 4.3)          # 物理上不可能，但讀數真的變了
chk("armed 時讀到不符：照樣轉發", len(g.telem.sent) == n0 + 1)
crit = [n for n in notes(g) if n["level"] == "critical"]
chk("但要大聲說、而且說明不切", "不切斷" in crit[-1]["text"], crit[-1]["text"])
g._from_fc(hb(armed=False), t + 60)
chk("落地上鎖之後就鎖住", not g._fc_trusted())

# ── 6. 更換流程 ────────────────────────────────────────────────
print("── 6. 更換流程：解除 → 下一塊成為綁定")
b = binding.Binding(BF)
rel = b.release("原飛控 IMU 故障，換上備品")
chk("解除後檔案裡沒有綁定、但有原因", json.load(open(BF))["board_uid"] is None
    and json.load(open(BF))["released"]["reason"].startswith("原飛控"))
g = new_agent(BF, RD)
g._from_fc(hb(), 400)
chk("解除後、新板子確認前：照舊放行（沒有東西要保護）", len(g.telem.sent) == 1)
g._from_fc(av(UID_B), 400.3)
bl = json.load(open(BF))
chk("新板子成為綁定（rebind）", bl["board_uid"] == UID_B and bl["how"] == "rebind")
chk("舊的進歷史", [h["board_uid"] for h in bl["history"]] == [UID_A])
chk("更換完成要說一聲，帶新舊板號與原因",
    UID_A in notes(g)[-1]["text"] and UID_B in notes(g)[-1]["text"]
    and "IMU" in notes(g)[-1]["text"])
chk("A 曾經綁過、B 現在綁著", g.binding.ever_bound(UID_A) and g.binding.ever_bound(UID_B))
try:
    g.binding.bind(UID_A)
    chk("已綁定時不可以直接改綁", False)
except AssertionError:
    chk("已綁定時不可以直接改綁", True)

# ── 7. 錄檔歸屬 ────────────────────────────────────────────────
print("── 7. 錄檔歸屬看錄製當下")
chk("A 時期的錄檔，換完之後仍以 A 回傳", g._record_owner(file_a) == (UID_A, None),
    g._record_owner(file_a))
mism = [f for f in os.listdir(RD) if f.endswith(".tlog")
        and (g.recorder.read_meta(f) or {}).get("board_uid") == UID_B]
chk("有 B 的錄檔", mism)
open(os.path.join(RD, "legacy.tlog"), "wb").write(b"x" * 10)
uid, why = g._record_owner("legacy.tlog")
chk("沒有標記的舊檔、而代理換過飛控 → 不傳", uid is None, why)
BF2 = os.path.join(tmp, "b2.json")
g2 = new_agent(BF2, RD)
g2._from_fc(hb(), 500)
g2._from_fc(av(UID_A), 500.3)
chk("沒有標記的舊檔、代理從沒換過 → 歸給唯一綁過的那塊",
    g2._record_owner("legacy.tlog") == (UID_A, None))
meta = os.path.join(RD, "nobody.tlog" + recorder.META_SUFFIX)
open(os.path.join(RD, "nobody.tlog"), "wb").write(b"x")
json.dump({"board_uid": "ffff"}, open(meta, "w"))
uid, why = g2._record_owner("nobody.tlog")
chk("從沒綁過的板子錄的 → 不傳", uid is None, why)
json.dump({"board_uid": None}, open(meta, "w"))
uid, why = g2._record_owner("nobody.tlog")
chk("錄製期間沒確認到身分 → 不傳", uid is None, why)
# 滾動刪檔時標記一起刪
r2 = recorder.Recorder(os.path.join(tmp, "roll"), armed_only=False, keep=1,
                       rotate_min=0)
r2.set_uid(UID_A)
for _ in range(3):
    r2.feed(hb().get_msgbuf(), False)
    r2._close()
left = sorted(os.listdir(os.path.join(tmp, "roll")))
chk("滾動刪檔時身分標記一起刪（沒有落單的標記）",
    {f for f in left if f.endswith(recorder.META_SUFFIX)}
    == {f + recorder.META_SUFFIX for f in left if f.endswith(".tlog")}
    and len(left) < 6, left)

# ── 8. 綁定檔讀不懂 ─────────────────────────────────────────────
print("── 8. 綁定檔讀不懂 → 當作對不上")
BF3 = os.path.join(tmp, "bad.json")
open(BF3, "w").write("{not json")
g3 = new_agent(BF3, RD)
g3._from_fc(hb(), 600)
g3._from_fc(av(UID_A), 600.3)
chk("不是當作沒綁過（那會把眼前的板子綁上去）", g3.binding_state == "mismatch")
chk("壞掉的檔案沒有被覆寫", open(BF3).read() == "{not json")
g3.say_hello()
chk("不拿一個讀不懂的板號去註冊", g3.hello_sent is False)

# ── 9. 不符時的註冊 ─────────────────────────────────────────────
print("── 9. 不符時的註冊：報綁定的板號，不帶眼前那塊的事實")
BF4 = os.path.join(tmp, "b4.json")
json.dump({"board_uid": UID_A, "history": []}, open(BF4, "w"))
g4 = new_agent(BF4, RD)
g4._from_fc(hb(), 700)
g4._from_fc(av(UID_B), 700.3)
sent = {}


class _Resp:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False

    def read(self):
        return b'{"drone_id": "d1", "name": "A"}'


def fake_urlopen(req, timeout=None):
    sent["body"] = json.loads(req.data)
    return _Resp()


A.urllib.request.urlopen = fake_urlopen
g4._start_intent_link = lambda uid, body: None
g4.say_hello()
bd = sent.get("body") or {}
chk("報的是綁定的 A", bd.get("board_uid") == UID_A, bd.get("board_uid"))
chk("不帶 B 的廠牌、韌體、機型、號碼",
    all(bd.get(k) is None for k in ("autopilot", "fw", "vehicle_type", "sysid")), bd)
chk("帶著綁定現況（地面站據此判 quarantined、不配號）",
    (bd.get("fc_binding") or {}).get("state") == "mismatch"
    and bd["fc_binding"].get("observed") == UID_B)
pub = []
g4.intent = types.SimpleNamespace(publish=pub.append)
g4._fc_link_wire = lambda: None
g4._publish_state()
chk("狀態只送綁定現況，不送 B 的飛行狀態",
    pub and pub[-1]["state"] is None and pub[-1]["derived"] is None
    and pub[-1]["fc_binding"]["state"] == "mismatch")

# ── 10. 重問 ───────────────────────────────────────────────────
print("── 10. 確認中沒有回答 → 重問")
g5 = new_agent(BF4, RD)
g5.serial = types.SimpleNamespace(mav=types.SimpleNamespace(
    command_long_send=lambda *a: None))
A.time.sleep = lambda s: None
g5._from_fc(hb(), 800)
g5.request_board_id()
chk("問過了", g5.caps_requested)
g5._verify_retry(g5.caps_req_at + 2)
chk("2 秒內不重問", g5.caps_requested)
g5._verify_retry(g5.caps_req_at + 6)
chk("5 秒沒回答 → 重問", g5.caps_requested is False)

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
