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


ag = A.Agent.__new__(A.Agent)
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

print("全部通過" if not fails else f"{fails} 項沒過")
sys.exit(1 if fails else 0)
