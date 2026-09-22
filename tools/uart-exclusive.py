#!/usr/bin/env python3
"""代理開著飛控序列埠時，別人開不起來（uav-system 059 A）。

2026-09-21：`get-gps.py` 帶起的 `mavsdk_server` 與代理同開 `/dev/ttyAMA0`，
兩邊各拿隨機片段，校正永遠跑不完且不報錯。修法是開埠時下 `TIOCEXCL`。

這份驗的是**行為**，不是程式形狀：真的開一個 pty、讓 `_claim_exclusive`
對它下獨佔，再以另一個 fd 去 `open()`，看核心是不是回 `EBUSY`。

**反例一起驗**：pyserial 的 `exclusive=True` 是 `flock`（建議鎖），
`mavsdk_server` 這類不 flock 的程式照樣開得起來。只驗「我們的會擋」
而不驗「那個不會擋」，證明不了為什麼不能用那個比較省事的寫法。

**要用非 root 跑**——root 無視 TIOCEXCL（這正是它的已知缺口，059 B 另案），
用 root 跑只會得到一串假的失敗。

用法：python3 tools/uart-exclusive.py
"""
import errno
import os
import pty
import socket
import sys
import tty
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

try:
    import pymavlink  # noqa: F401
except ImportError:
    print("skip：沒有 pymavlink。**skip ≠ pass**")
    sys.exit(2)

import agent as A  # noqa: E402

if os.geteuid() == 0:
    print("skip：root 無視 TIOCEXCL，這份要用一般使用者跑。**skip ≠ pass**")
    sys.exit(2)

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


def fake(dev):
    return types.SimpleNamespace(a=types.SimpleNamespace(dev=dev),
                                 port_exclusive=None)


def try_open(path):
    """像 mavsdk_server 那樣開：一般的 open()，不 flock。回 errno 或 0。"""
    try:
        fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    except OSError as e:
        return e.errno
    os.close(fd)
    return 0


def main():
    claim = A.Agent._claim_exclusive

    print("— 反例：pyserial 的 exclusive=True 擋不住 —")
    import serial
    m0, s0 = pty.openpty()
    path0 = os.ttyname(s0)
    ser = serial.Serial(path0, 57600, timeout=0, exclusive=True)
    chk("pyserial 開著 exclusive=True 時，別人照樣開得起來",
        try_open(path0) == 0,
        "它做的是 flock（建議鎖），不 flock 的程式不受影響——所以不能用它")
    ser.close()
    os.close(s0)
    os.close(m0)

    print("\n— TIOCEXCL —")
    m, s = pty.openpty()
    tty.setraw(s)          # 代理開的就是 raw；canonical 模式沒換行 read 會一直等
    path = os.ttyname(s)
    chk("下獨佔之前，別人開得起來", try_open(path) == 0, "基準——否則後面的 EBUSY 不代表什麼")

    f = fake(path)
    claim(f, s)
    chk("獨佔成功有記下來", f.port_exclusive is True)
    e = try_open(path)
    chk("別人再開拿到 EBUSY", e == errno.EBUSY, f"errno={errno.errorcode.get(e, e)}")

    # 代理手上那個 fd 照常能用：獨佔擋的是新的 open()，不是自己的讀寫
    os.write(m, b"\xfe")
    chk("自己的 fd 照常讀得到", os.read(s, 1) == b"\xfe")

    # pty 的 master 還開著＝「別人還握著這個 tty」（例如 root 的 stty）。
    # 這時關掉自己的 fd 不是最後一次釋放，核心**不會**清掉旗標
    os.close(s)
    chk("反例：只關不放，重開會被自己上次的獨佔擋住",
        try_open(path) == errno.EBUSY,
        "所以 `_open_serial_once` 關之前要先 TIOCNXCL——否則 050 的取回路徑會把自己鎖在外面")

    # 照 `_open_serial_once` 的順序：放掉 → 關 → 開 → 再主張
    os.close(m)            # 換一個乾淨的 tty，上一個的旗標已經卡住了
    m, s = pty.openpty()
    tty.setraw(s)
    path = os.ttyname(s)
    f = fake(path)
    claim(f, s)
    A.Agent._release_exclusive(f, s)
    os.close(s)
    fd2 = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    chk("先放再關，重開不會被自己擋住", fd2 >= 0, "別人還握著 tty 也一樣")
    g = fake(path)
    claim(g, fd2)
    chk("重開之後重新主張獨佔", g.port_exclusive is True
        and try_open(path) == errno.EBUSY)
    A.Agent._release_exclusive(g, None)
    chk("fd 是 None 時放掉不拋", True, "重開失敗之後 self.fd 就是 None")
    os.close(fd2)
    os.close(m)

    print("\n— 不是 tty —")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    u = fake("udpin:0.0.0.0:14550")
    claim(u, sock.fileno())
    chk("socket 跳過、記成 None 而不是 False", u.port_exclusive is None,
        "--dev udpin:（SITL）沒有獨佔可言，不該被報成「沒獨佔住」")
    sock.close()

    print("\n— 下不成功 —")
    real = A.fcntl.ioctl

    def refuse(fd, req, *a):
        raise OSError(errno.ENOTTY, "Inappropriate ioctl for device")

    A.fcntl.ioctl = refuse
    try:
        m2, s2 = pty.openpty()
        v = fake(os.ttyname(s2))
        claim(v, s2)          # 不准拋：059 裁定被搶也不拒絕啟動
        chk("不拋例外、記成 False", v.port_exclusive is False,
            "沒有代理比有一個不獨佔的代理更糟")
        os.close(s2)
        os.close(m2)
    finally:
        A.fcntl.ioctl = real

    print("\n" + ("✓ 全部通過" if ok else "✗ 有失敗"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
