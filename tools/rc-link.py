#!/usr/bin/env python3
"""RC 遙控器狀態的三態解析（039 複裁 A）。**不連飛控、不送指令。**

**為什麼這一段要單獨釘**：它是「機在地上失聯只告警」那格的前提。整條規則的
安全性建立在「遙控器就在旁邊、人隨時可以接管」——而讀錯這個欄位有兩個方向，
代價完全不對稱：

* 把「不知道」讀成「沒有 RC」→ 所有還沒回報這個欄位的機**都起飛不了**
  （issues/036 的同一個教訓）。
* 把「沒有 RC」讀成「有 RC」→ 守門在遙控器掉線時照樣放行起飛，而那正是
  這條規則要防的事。**MAVLink 的 `RC_CHANNELS.rssi = 255` 是「無效值」不是
  「滿格」**，用 rssi 判就會踩進這一邊——所以這裡用 SYS_STATUS 的
  present／health 兩組旗標，因為只有它把「沒有回報」與「回報說沒有」分得開。

**真機覆核尚未進行**（覆核前 `rc_link` 照送，送 None 不擋）：
  1. ArduPilot 4.7 在 RC failsafe 期間**是否真的清掉 health 的這一位元**？
  2. 沒有接收機／沒設定 RC 時，present 的這一位元是不是 0？
  3. 遙控器關機到 health 翻掉之間的延遲有多長（失聯處置的前提有多新鮮）？
把飛控接上、跑 `python3 tools/rc-link.py --live /dev/ttyAMA0` 可以直接看實際
位元；覆核完把結論寫回 doc/agent-intent-protocol.md §6。

用法：python3 tools/rc-link.py [--live <裝置> [鮑率]]
"""
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

for name in ("serial",):
    sys.modules.setdefault(name, types.ModuleType(name))
try:
    import pymavlink  # noqa: F401
except ImportError:
    print("skip：沒有 pymavlink。**skip ≠ pass**")
    sys.exit(2)

import agent as A  # noqa: E402
import guard  # noqa: E402

BIT = A.SENSOR_RC_RECEIVER


def sys_status(present, health, enabled=None):
    """一則剛好夠 Monitor.feed 用的 SYS_STATUS。

    `enabled` 預設跟著 `present`：**RC 三態刻意只看 present／health**，
    但 `sensors_unhealthy`（026 B4-b）要 present 且 enabled 且不 health，
    所以這個欄位還是得有——沒有的話 feed 會直接拋 AttributeError。
    """
    return types.SimpleNamespace(
        get_type=lambda: "SYS_STATUS", voltage_battery=12000,
        battery_remaining=80, load=250,
        onboard_control_sensors_present=present,
        onboard_control_sensors_enabled=present if enabled is None else enabled,
        onboard_control_sensors_health=health)


def feed(present, health, enabled=None):
    mon = A.Monitor()
    mon.feed(sys_status(present, health, enabled))
    return mon


ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


if "--live" in sys.argv:
    from pymavlink import mavutil
    dev = sys.argv[sys.argv.index("--live") + 1]
    baud = int(sys.argv[sys.argv.index("--live") + 2]) \
        if len(sys.argv) > sys.argv.index("--live") + 2 else 57600
    m = mavutil.mavlink_connection(dev, baud=baud)
    print(f"等 SYS_STATUS（{dev}@{baud}）…按 Ctrl-C 結束")
    while True:
        msg = m.recv_match(type="SYS_STATUS", blocking=True, timeout=10)
        if msg is None:
            print("10 秒沒收到 SYS_STATUS——串流沒開？")
            continue
        p, h = (msg.onboard_control_sensors_present,
                msg.onboard_control_sensors_health)
        print(f"present bit={bool(p & BIT)}  health bit={bool(h & BIT)}  "
              f"→ rc_link={feed(p, h).state['rc_link']!r}   "
              f"(present=0x{p:08x} health=0x{h:08x})")

print(f"MAV_SYS_STATUS_SENSOR_RC_RECEIVER = 1<<16 = {BIT}")
print("\n── 三態解析 ───────────────────────────────────────────")
chk("present 有、health 有 → True（RC 在線）",
    feed(BIT, BIT).state["rc_link"] is True)
chk("present 有、health 無 → False（RC 掉線）",
    feed(BIT, 0).state["rc_link"] is False)
chk("**present 無 → None（這台韌體沒回報，不是沒有 RC）**",
    feed(0, 0).state["rc_link"] is None)
chk("present 無但 health 有（矛盾）→ 仍是 None（不半懂地解讀）",
    feed(0, BIT).state["rc_link"] is None)
chk("其他感測器的位元不會誤觸這一格",
    feed(0xFFFF, 0xFFFF).state["rc_link"] is None, "低 16 位元全開")
chk("**反向驗證**：同時有別的感測器時仍讀得出 RC",
    feed(0xFFFF | BIT, 0xFFFF).state["rc_link"] is False)

# **`rc_link` 刻意不看 `enabled`**，與 `sensors_unhealthy` 不同。方向是安全的：
# 「present 但沒 enabled」代表接收機在但不可用，而 rc_link=False 的後果是擋下
# 起飛——擋錯了人看得到理由並可以處理，放行錯了是沒有人能接管
chk("present 有、health 有，但 enabled 沒有 → 仍是 True（只看 present/health）",
    feed(BIT, BIT, enabled=0).state["rc_link"] is True)
chk("**對照**：同一組輸入下 sensors_unhealthy 不會列 RC（它要 enabled）",
    feed(BIT, 0, enabled=0).state["sensors_unhealthy"] == [],
    feed(BIT, 0, enabled=0).state["sensors_unhealthy"])
chk("**對照**：enabled 有而 health 無時，兩邊都說 RC 有問題",
    feed(BIT, 0, enabled=BIT).state["rc_link"] is False
    and "RC 接收器" in feed(BIT, 0, enabled=BIT).state["sensors_unhealthy"])

print("\n── 變化要留痕（含「不知道 → 掉線」）───────────────────")
mon = A.Monitor()
mon.feed(sys_status(BIT, BIT))
n0 = len(mon.events)
mon.feed(sys_status(BIT, 0))
chk("在線 → 掉線 留一筆事件", len(mon.events) == n0 + 1,
    mon.events[-1][2] if mon.events else None)
mon.feed(sys_status(BIT, 0))
chk("沒變化不重複記", len(mon.events) == n0 + 1)
# **這一格是 `_change` 做不到的**：它把第一次讀到的值當成「還沒讀到」，
# 於是 None → False 會安靜地過去——而那正是最該留痕的一次
mon2 = A.Monitor()
mon2.feed(sys_status(0, 0))          # 先讀到「不知道」
n1 = len(mon2.events)
mon2.feed(sys_status(BIT, 0))        # 韌體開始回報，而且說 RC 掉了
chk("**不知道 → 掉線 也留痕**（None 是值，不是「還沒讀到」）",
    len(mon2.events) == n1 + 1, mon2.events[-1][2] if mon2.events else None)

print("\n── 分層來源：SYS_STATUS 沒有時退到 RC_CHANNELS ─────────")
import time as _t
import agent as _A

mon = _A.Monitor.__new__(_A.Monitor)
mon.events = []


def derive(sys_bit=None, chancount=None, age=0.0):
    s = {"_rc_sys_status": sys_bit}
    if chancount is not None:
        s["_rc_chancount"] = chancount
        s["_rc_chan_t"] = _t.monotonic() - age
    return mon._derive_rc(s), s.get("rc_source")


chk("SYS_STATUS 有值時它說了算（優先）",
    derive(sys_bit=True, chancount=0) == (True, "sys_status"),
    "PX4 走這條")
chk("**SYS_STATUS 沒有 → 退到 chancount**（ArduPilot 4.7 走這條）",
    derive(chancount=8) == (True, "rc_channels"))
chk("chancount=0 → False（規格：沒有可用通道時應為 0）",
    derive(chancount=0) == (False, "rc_channels"))
chk("兩個來源都沒有 → None", derive() == (None, None))

print("\n── 過期：不再聽到 ≠ RC 還在 ────────────────────────────")
chk("RC_CHANNELS 停了超過門檻 → 退回 None（不是沿用舊計數）",
    derive(chancount=8, age=_A.Monitor.RC_STALE_S + 1) == (None, None),
    "**用舊事實回答關於現在的問題，是這條規則最容易犯的錯**")
chk("**反向驗證**：門檻內仍然有效",
    derive(chancount=8, age=1.0) == (True, "rc_channels"))

print("\n── 轉態留痕（含「不知道 → 掉線」那一格）────────────────")
st = {}
mon._note_rc(st, None)                      # 第一次讀到，不算轉態
n0 = len(mon.events)
mon._note_rc(st, False)                     # 不知道 → 掉線
chk("**「不知道 → 掉線」有留痕**（最該被看見的一次轉換）",
    len(mon.events) == n0 + 1 and "離線" in mon.events[-1][2],
    mon.events[-1][2] if mon.events else "")
mon._note_rc(st, False)
chk("沒有變化就不重複記", len(mon.events) == n0 + 1)

print("\n── 接到守門：三態各自的後果 ───────────────────────────")
for label, rc, want in (("在線", True, True), ("掉線", False, False),
                        ("不知道", None, True)):
    allowed, why = guard.check("READY", "start_mission", rc)
    chk(f"RC {label} → start_mission {'放行' if want else '擋下'}",
        allowed is want, why or "")

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
print("**注意：以上全是位元邏輯的測試，不是真機覆核。**"
      " ArduPilot 4.7 實際會不會清 health 位元，要接飛控跑 --live 才算數")
sys.exit(0 if ok else 1)
