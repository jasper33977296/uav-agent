#!/usr/bin/env python3
"""dongle 換 IP／網卡時代理的反應（2026-10-20）：第一次看到不動作、變了就重綁 UDP、
意圖通道立刻重連、發 notice；路由補不上要說出來。netwatch 用替身，不碰真的網路。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import agent as A  # noqa: E402
import netwatch  # noqa: E402

fails = 0


def check(cond, what):
    global fails
    print(("  ✓ " if cond else "  ✗ ") + what)
    fails += 0 if cond else 1


class FakeLink:
    def __init__(self): self.rebinds = 0
    def rebind(self): self.rebinds += 1


class FakeIntent:
    def __init__(self): self.kicks = []
    def kick(self, why): self.kicks.append(why)


class FakeOutbox:
    def __init__(self): self.notes = []
    def note(self, kind, text, level="info", now=None): self.notes.append((kind, level, text))


class Args: gs = "10.141.2.22"


A.Agent.gs_link_ok = None
A.Agent.uplink_ok = None


ag = A.Agent.__new__(A.Agent)
ag.modem = None
ag.a, ag.telem, ag.cmd, ag.intent, ag.outbox = Args(), FakeLink(), FakeLink(), FakeIntent(), FakeOutbox()
seq = []
routes = []
netwatch.dongle_net = lambda: seq[0]
netwatch.ensure_gs_route = lambda n, gs: (routes.append((n["nic"], gs)) or (f"{gs}/32 dev {n['nic']}", None))
t = [100.0]


def step(n):
    seq[:] = [n]
    t[0] += 1.1
    ag._watch_net(t[0])


print("── 開機第一次看到")
step({"nic": "enxaaa", "ip": "10.141.2.29", "prefix": 24, "gw": "10.141.2.30"})
check(ag.telem.rebinds == 0 and not ag.intent.kicks, "第一次看到：不重綁、不踢意圖通道")
check(routes == [("enxaaa", "10.141.2.22")], "第一次看到就補到地面站的路由")
check(ag.net_changes == 0, "變動次數 0")

print("── 一秒內再看：沒變")
step({"nic": "enxaaa", "ip": "10.141.2.29", "prefix": 24, "gw": "10.141.2.30"})
check(ag.telem.rebinds == 0 and len(routes) == 1, "沒變就什麼都不做")

print("── dongle 不見")
step(None)
check(ag.outbox.notes and "斷線" in ag.outbox.notes[-1][2], "發出「5G dongle 斷線」notice")
check(ag.telem.rebinds == 0, "斷線當下不重綁（沒有網卡可綁）")

print("── 回來了：網卡名字與 IP 都變了")
step({"nic": "enxbbb", "ip": "10.141.3.7", "prefix": 24, "gw": "10.141.3.1"})
check(ag.telem.rebinds == 1 and ag.cmd.rebinds == 1, "遙測與指令兩條 UDP 都換來源埠")
check(len(ag.intent.kicks) == 1 and "10.141.3.7" in ag.intent.kicks[0], "意圖通道被踢、立刻重連")
check(routes[-1] == ("enxbbb", "10.141.2.22"), "新網卡補上到地面站的路由")
check("10.141.3.7" in ag.outbox.notes[-1][2] and "10.141.2.29" in ag.outbox.notes[-1][2],
      "notice 說出舊 IP → 新 IP（斷線期間也記得之前是哪個）")
check(ag.net_changes == 2, "變動次數 2（斷線、回來）")

print("── 路由補不上要說出來")
netwatch.ensure_gs_route = lambda n, gs: (None, "地面站不在 dongle 網段、dongle 也沒有閘道")
step({"nic": "enxccc", "ip": "10.9.9.9", "prefix": 24, "gw": None})
check(ag.net_route_err and ag.outbox.notes[-1][1] == "warning", "路由失敗 → notice 是 warning、原因寫在狀態裡")

print("── 代理沒有意圖通道（還沒註冊）時不出錯")
ag.intent = None
step({"nic": "enxddd", "ip": "10.141.2.40", "prefix": 24, "gw": "10.141.2.30"})
check(ag.telem.rebinds == 3, "照樣重綁")

print("── 有 IP、但地面站兩個方向都不通 → 30 秒後重新要 IP（每分鐘最多一次）")
calls = []
class R: stdout, stderr = "NetworkManager：usb0 重新連線", ""
A.subprocess.run = lambda cmd, **k: (calls.append(cmd) or R())
ag.intent = FakeIntent()
same = {"nic": "usb0", "ip": "192.168.187.22", "prefix": 24, "gw": "192.168.187.1"}
step(same)                                  # IP 變了：重連流程，計時歸零
ag.gs_link_ok, ag.uplink_ok = False, False
for _ in range(20):                         # 22 秒：還沒到 30 秒
    step(same)
check(not calls, "不通 22 秒：還不動手")
for _ in range(10):
    step(same)
check(len(calls) == 1 and calls[0][:2] == ["sudo", "-n"] and calls[0][-1] == "usb0",
      "不通超過 30 秒：sudo -n dongle-renew.sh usb0 一次")
check(ag.outbox.notes[-1][0] == "net" and "重新跟 dongle 要 IP" in ag.outbox.notes[-1][2], "發 net notice")
for _ in range(40):
    step(same)
check(len(calls) == 1, "一分鐘內不再重新要")
for _ in range(15):
    step(same)
check(len(calls) == 2, "滿一分鐘還不通：再要一次")
ag.uplink_ok = True                          # 單向通：是回程對映的事，不重新要 IP
n = len(calls)
for _ in range(80):
    step(same)
check(len(calls) == n, "只有一個方向不通：不動 IP（那是重綁的事）")
check(ag._net_dead_since is None, "通了就把計時歸零")

print("── dongle 說的電信 IP 跟網卡上的不一樣 → 5 秒就重新要（不等 30 秒）")
class FakeModem: wan_ip = "10.141.2.29"
ag.modem = FakeModem()
calls.clear(); ag._net_renew_t = 0.0
step({"nic": "usb0", "ip": "192.168.187.42", "prefix": 24, "gw": "192.168.187.1"})
ag.uplink_ok = ag.gs_link_ok = False
for _ in range(6):
    step({"nic": "usb0", "ip": "192.168.187.42", "prefix": 24, "gw": "192.168.187.1"})
check(len(calls) == 1, "不通約 6 秒、而且 IP 對不上電信 IP：已經重新要")
check("電信 IP 是 10.141.2.29" in ag.outbox.notes[-1][2], "notice 說出為什麼（電信 IP 對不上）")
ag.modem.wan_ip = "10.141.2.29"
calls.clear(); ag._net_renew_t = 0.0
step({"nic": "usb0", "ip": "10.141.2.29", "prefix": 24, "gw": "10.141.2.30"})
for _ in range(6):
    step({"nic": "usb0", "ip": "10.141.2.29", "prefix": 24, "gw": "10.141.2.30"})
check(not calls, "IP 對得上電信 IP、只是不通：照舊等 30 秒（不是拿錯位址）")

print("全部通過" if not fails else f"{fails} 項沒過")
sys.exit(1 if fails else 0)
