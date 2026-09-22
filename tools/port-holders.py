#!/usr/bin/env python3
"""還有誰開著飛控序列埠，報得出來（uav-system 059 B）。

2026-09-21 的答案要 ssh 上去 `fuser -v /dev/ttyAMA0` 才看得到。這份驗代理
自己找得到、而且報到地面站：用 pty 當序列埠，**另開一個真的行程**去開它
（就像 `mavsdk_server` 那樣），跑代理真的 `_check_port_holders`。

**看不完整的那一格**：開發機上代理讀不到 root 行程的 fd 表，這時必須說
「看不完整」，不能說「沒有別人」。Pi 上 unit 給了 CAP_SYS_PTRACE，讀得到的
話那一格就不會出現——兩種結果這份都接受，但**不准兩種都沒說**。

用法：python3 tools/port-holders.py
"""
import os
import pty
import socket
import subprocess
import sys
import tempfile
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

try:
    import pymavlink  # noqa: F401
except ImportError:
    print("skip：沒有 pymavlink。**skip ≠ pass**")
    sys.exit(2)

import agent as A  # noqa: E402
import holders  # noqa: E402

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


def fake(dev):
    return types.SimpleNamespace(
        a=types.SimpleNamespace(dev=dev), outbox=A.Outbox(),
        port_holders=[], port_holders_unreadable=0, holders_checked=0.0,
        _holders_blind_said=False, PORT_HOLDERS_S=A.Agent.PORT_HOLDERS_S)


def grab(path):
    """另一個行程開著 path——**不繼承我們的 fd**（close_fds），要自己 open。"""
    code = (f"import os,time; fd=os.open({path!r}, os.O_RDWR|os.O_NOCTTY); "
            "print('ready', flush=True); time.sleep(60)")
    p = subprocess.Popen([sys.executable, "-c", code, "--sysid", "245"],
                         stdout=subprocess.PIPE, text=True)
    assert p.stdout.readline().strip() == "ready"
    return p


check = A.Agent._check_port_holders
m, s = pty.openpty()
path = os.ttyname(s)

print("── 只有我們自己 ─────────────────────────────────────────")
f = fake(path)
check(f, s, 100.0)
chk("自己開著不算搶", f.port_holders == [], f.port_holders)
blind = [n for n in f.outbox.unsent() if "看不到" in n["text"]]
if f.port_holders_unreadable:
    chk("**讀不到別人的 fd 表時要說看不完整**", len(blind) == 1,
        f"{f.port_holders_unreadable} 個行程讀不到——此時「沒找到」≠「沒有」")
else:
    chk("讀得到全部時不喊「看不完整」", blind == [], "（有 CAP_SYS_PTRACE 或 root）")

print("\n── 有人搶 ───────────────────────────────────────────────")
p = grab(path)
check(f, s, 110.0)
pids = [h["pid"] for h in f.port_holders]
chk("找得到另一個行程", pids == [p.pid], f.port_holders)
chk("帶得出命令列（那就是答案本身）",
    f.port_holders and "--sysid 245" in f.port_holders[0]["cmd"],
    f.port_holders[0]["cmd"] if f.port_holders else None)
chk("帶得出使用者", f.port_holders and f.port_holders[0]["user"])
new = [n for n in f.outbox.unsent() if n["kind"] == "fc_port" and "別的程式開著" in n["text"]]
chk("**報到地面站，而且是危急**", len(new) == 1 and new[0]["level"] == "critical",
    new[0]["text"][:80] if new else None)
chk("說得出該怎麼辦", new and "停掉它" in new[0]["text"])
chk("**不指一條不存在的路**", new and "走代理" not in new[0]["text"],
    "機上沒有給本機程式用的 MAVLink 出口（059 C）")

n0 = len(f.outbox.unsent())
check(f, s, 112.0)
chk("5 秒內不重掃", len(f.outbox.unsent()) == n0)
check(f, s, 120.0)
chk("名單沒變不重複喊", len(f.outbox.unsent()) == n0, "名單本身一直在狀態裡")

p.kill()
p.wait()
check(f, s, 130.0)
chk("走了之後名單清空", f.port_holders == [])
gone = f.outbox.unsent()[-1]
chk("**走了也說一聲**", "已沒有別的程式" in gone["text"] and gone["level"] == "info")

print("\n── 裝置名是符號連結 ─────────────────────────────────────")
link = os.path.join(tempfile.mkdtemp(), "serial0")
os.symlink(path, link)
p = grab(path)
g = fake(link)
check(g, s, 100.0)
chk("用 /dev/serial0 這類別名也找得到", [h["pid"] for h in g.port_holders] == [p.pid],
    "/proc/*/fd 的連結目標是真名，拿別名比會永遠比不到")
p.kill()
p.wait()

print("\n── 不是 tty ──────────────────────────────────────────────")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
u = fake("udpin:0.0.0.0:14550")
check(u, sock.fileno(), 100.0)
chk("socket 跳過", u.port_holders == [] and u.outbox.unsent() == [])
sock.close()

print("\n── 成本 ─────────────────────────────────────────────────")
t = time.perf_counter()
holders.scan(path, exclude_pid=os.getpid())
dt = (time.perf_counter() - t) * 1000
chk("掃一次在 100 ms 內", dt < 100, f"{dt:.1f} ms（每 {A.Agent.PORT_HOLDERS_S:.0f} 秒一次，在主迴圈上）")

os.close(s)
os.close(m)
print("\n" + ("✓ 全部通過" if ok else "✗ 有失敗"))
sys.exit(0 if ok else 1)
