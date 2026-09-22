#!/usr/bin/env python3
"""橋的兩端都要有人看：序列埠被動手腳、以及飛控心跳不見了（uav-system 050）。

2026-09-16 的故障是這樣的：`uart-probe.sh` 對代理**正開著**的
`/dev/ttyAMA0` 連下七次 `stty`，把線速從 57600 一路調到 1500000 就結束了，
沒有還原。termios 是 per-device 的，代理手上那個 fd 跟著被改走——飛控照常
在 57600 送，Pi 這端用 1.5 Mbaud 讀，只讀得到雜訊。結果是飛控整整 25 分鐘
一個字都進不來，而代理**不知道、不出聲、也沒試著救**，最後靠人工重啟才恢復。

這份驗的是那次之後補上的兩道看門狗（050 需求 1 與需求 2 的前半）：

  * **偵測**：每秒問一次核心「這個埠現在到底是多少 baud」。
    **不能問 pyserial 的 `port.baudrate`**——那是它記在 Python 這邊的設定值，
    被外部改掉之後它照樣回報 57600。要問 `termios.tcgetattr`。
    當時是 `sudo` 跑的，所以 `TIOCEXCL` 也擋不住（root 無視它）。
  * **告警**：心跳不見了要大聲說。既有的看門狗只處理「心跳在、遙測停」
    （飛控把串流速率歸零），`hb_ok` 為 False 時**以前沒有 else**。

  * **取回**：埠壞了（或飛控久無心跳）就關掉重開，而且**不阻塞主迴圈**——
    開機時「開到成功為止」是對的，執行期卡在那個迴圈裡等於地面站那一側
    也一起停擺，而地面失聯處置正是靠主迴圈在跑。重開失敗時 `fd` 留在 None，
    主迴圈照常跑，下一次退避到期再試。

**失聯期間持續向地面站出聲還沒做**（050 需求 3）——所以這份不驗那個。

用 pty 當序列埠，**不碰真的飛控**。

用法：python3 tools/fc-link-watchdog.py
"""
import os
import pty
import sys
import termios
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

try:
    import pymavlink  # noqa: F401
except ImportError:
    print("skip：沒有 pymavlink。**skip ≠ pass**")
    sys.exit(2)

import agent as A  # noqa: E402

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


def setspeed(fd, const):
    a = termios.tcgetattr(fd)
    a[4] = a[5] = const
    termios.tcsetattr(fd, termios.TCSANOW, a)


class FakePort:
    """pyserial 的 Serial 只借兩個行為：改 baudrate 會重新套用整套 termios
    （真的那個 setter 會觸發 `_reconfigure_port()`），以及清輸入緩衝。"""

    def __init__(self, fd):
        self.fd, self.reconfigured, self.flushed = fd, 0, 0

    @property
    def baudrate(self):
        return 57600

    @baudrate.setter
    def baudrate(self, b):
        self.reconfigured += 1
        setspeed(self.fd, getattr(termios, "B%d" % b))

    def reset_input_buffer(self):
        self.flushed += 1


def fake(fd):
    f = types.SimpleNamespace(
        a=types.SimpleNamespace(baud=57600, dev="/dev/pty-test"),
        serial=types.SimpleNamespace(port=FakePort(fd)),
        port_checked=0.0, port_tampered=0,
        mon=types.SimpleNamespace(last_hb=None, state={}),
        fc_link_ok=None, fc_lost_since=None, fc_lost_nagged=0.0,
        fd=fd, fc_reopen_t=0.0, fc_reopens=0, fc_write_dropped=0,
        fwd_to_fc=0, fc_parser=None, port_exclusive=None,
        outbox=A.Outbox(), _reopens_at_loss=0,
        # 類別常數借真的那幾個值——**不要讓測試自己編一套門檻**
        PORT_CHECK_S=A.Agent.PORT_CHECK_S,
        FC_HB_LOST_S=A.Agent.FC_HB_LOST_S,
        FC_LOST_NAG_S=A.Agent.FC_LOST_NAG_S,
        FC_REOPEN_S=A.Agent.FC_REOPEN_S,
        FC_REOPEN_EVERY_S=A.Agent.FC_REOPEN_EVERY_S,
    )
    # 開埠會順手主張獨佔（059 A）——**綁真的那個**，不替它編假的
    f._claim_exclusive = lambda fd: A.Agent._claim_exclusive(f, fd)
    f._release_exclusive = lambda fd: A.Agent._release_exclusive(f, fd)
    f._fc_lost_text = lambda age: A.Agent._fc_lost_text(f, age)
    f._fc_lost_level = lambda: A.Agent._fc_lost_level(f)
    return f


def wire(f):
    """SimpleNamespace 不會自動綁方法——把會互相呼叫的那幾個接起來。"""
    f._open_serial_once = lambda: A.Agent._open_serial_once(f)
    f._reopen_serial = lambda now, why: A.Agent._reopen_serial(f, now, why)
    return f


def main():
    master, slave = pty.openpty()
    setspeed(slave, termios.B57600)
    f = fake(slave)
    tamper, fclink = A.Agent._check_port_tamper, A.Agent._check_fc_link

    print("\n— 序列埠被動手腳 —")
    tamper(f, slave, 100.0)
    chk("沒人動它就安靜", f.port_tampered == 0 and f.serial.port.reconfigured == 0)

    setspeed(slave, termios.B1500000)          # 重現 uart-probe.sh 留下的狀態
    tamper(f, slave, 200.0)
    chk("抓到被改成 1500000", f.port_tampered == 1)
    chk("改回去（觸發完整 reconfigure）", f.serial.port.reconfigured == 1)
    n = f.outbox.unsent()
    chk("**報到地面站**（以前只有 log.error）",
        len(n) == 1 and n[0]["kind"] == "fc_port" and "1500000" in n[0]["text"],
        n[-1]["text"] if n else None)
    chk("清掉用錯速率讀進來的垃圾", f.serial.port.flushed == 1)
    chk("核心實際速率回到 57600",
        termios.tcgetattr(slave)[5] == termios.B57600,
        "問核心，不是問 pyserial")

    setspeed(slave, termios.B115200)
    tamper(f, slave, 200.5)
    chk("節流：1 秒內不重複問核心", f.port_tampered == 1)
    tamper(f, slave, 201.5)
    chk("節流之後照樣抓得到", f.port_tampered == 2)

    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    u = fake(sock.fileno())
    u.port_checked = 0.0
    tamper(u, sock.fileno(), 300.0)
    chk("非 tty 的裝置直接跳過", u.port_tampered == 0,
        "--dev udpin: 走同一條路徑；對 socket 問 termios 只會每秒拋一次")
    sock.close()

    print("\n— 飛控心跳 —")
    fclink(f, 300.0)
    chk("還沒看過飛控就不告警", f.fc_link_ok is None, "它可能只是比 Pi 晚上電")

    f.mon.last_hb = 300.0
    fclink(f, 301.0)
    chk("心跳新鮮＝正常", f.fc_link_ok is True)
    fclink(f, 310.0)
    chk("超過門檻＝失聯", f.fc_link_ok is False, f"門檻 {A.Agent.FC_HB_LOST_S}s")
    chk("失聯起點用最後心跳時刻", f.fc_lost_since == 300.0, "不是偵測到的那一刻")
    lost = [x for x in f.outbox.unsent() if x["kind"] == "fc_link"]
    chk("**失聯報到地面站**（050 需求 3）", len(lost) == 1 and "沒有心跳" in lost[0]["text"],
        lost[-1]["text"] if lost else None)
    chk("說得出「指令送不到飛控」", lost and "送不到飛控" in lost[0]["text"])
    chk("未解鎖時是警告", lost and lost[0]["level"] == "warning")

    f.mon.last_hb = 400.0
    fclink(f, 400.5)
    chk("心跳回來＝恢復", f.fc_link_ok is True)
    chk("恢復後清掉起點", f.fc_lost_since is None)
    back = [x for x in f.outbox.unsent() if x["kind"] == "fc_link"]
    chk("恢復也報", len(back) == 2 and "恢復" in back[-1]["text"]
        and "中斷 100 秒" in back[-1]["text"], back[-1]["text"] if back else None)

    f.mon.last_hb = 500.0
    fclink(f, 510.0)
    nagged = f.fc_lost_nagged
    fclink(f, 540.0)
    chk("還沒滿間隔不重複喊", f.fc_lost_nagged == nagged)
    fclink(f, 575.0)
    chk("長時間失聯會持續留痕", f.fc_lost_nagged == 575.0,
        f"每 {A.Agent.FC_LOST_NAG_S:.0f}s；25 分鐘的故障不該只有起點一行")
    nags = [x["text"] for x in f.outbox.unsent() if "仍然沒有心跳" in x["text"]]
    chk("**持續的那幾聲也送地面站**（不只 log）", len(nags) == 1, nags)
    f.mon.state["armed"] = True
    f.mon.last_hb = 600.0
    fclink(f, 601.0)
    fclink(f, 610.0)
    lvl = [x for x in f.outbox.unsent() if x["kind"] == "fc_link"][-1]
    chk("**解鎖中失去飛控＝危急**", lvl["level"] == "critical" and "已解鎖" in lvl["text"],
        "代理連 RTL 都送不出去")

    print("— 關掉重開 —")
    reopen, maybe = A.Agent._reopen_serial, A.Agent._maybe_reopen_serial
    opened = A.Agent._open_serial_once

    class FakeConn:
        def __init__(self, fd):
            self.port = types.SimpleNamespace(fileno=lambda: fd)
            self.closed = False

        def close(self):
            self.closed = True

    g = fake(slave)
    old_conn = FakeConn(slave)
    g.serial = old_conn
    A.mavutil.mavlink_connection = lambda *a, **k: FakeConn(slave)
    chk("開得起來就回 True", opened(g) is True)
    chk("fd 換成新的", g.fd == slave)
    chk("舊的連線有被關掉", old_conn.closed, "同一個裝置不要同時開兩份")
    chk("解析器緩衝被丟掉", g.fc_parser is not None,
        "斷點前那幾個位元組不會是完整的框架")

    def boom(*a, **k):
        raise OSError(19, "No such device")      # ENODEV＝USB 被拔

    A.mavutil.mavlink_connection = boom
    chk("開不起來回 False 而不是拋", opened(g) is False)
    chk("失敗後 fd 是 None", g.fd is None, "**不是留著壞掉的 fd**")

    print("\n— 重開的排程 —")
    A.mavutil.mavlink_connection = lambda *a, **k: FakeConn(slave)
    h = wire(fake(slave))
    h.mon.last_hb = 1000.0
    maybe(h, 1001.0)
    chk("心跳新鮮就不要動埠", h.fc_reopens == 0)

    h.mon.last_hb = None
    h.fc_reopen_t = 0.0
    maybe(h, 1001.0)
    chk("還沒看過飛控也不動", h.fc_reopens == 0, "它可能只是比 Pi 晚上電")

    h.mon.last_hb = 1000.0
    maybe(h, 1000.0 + A.Agent.FC_REOPEN_S)
    chk("無心跳滿門檻就重開", h.fc_reopens == 1,
        f"門檻 {A.Agent.FC_REOPEN_S:.0f}s，比告警的 {A.Agent.FC_HB_LOST_S:.0f}s 寬")
    maybe(h, 1000.0 + A.Agent.FC_REOPEN_S + 1)
    chk("退避內不連續重開", h.fc_reopens == 1,
        f"每 {A.Agent.FC_REOPEN_EVERY_S:.0f}s 一次；連開只會一直丟掉剛回來的框架")
    maybe(h, 1000.0 + A.Agent.FC_REOPEN_S + A.Agent.FC_REOPEN_EVERY_S + 1)
    chk("退避過了再試一次", h.fc_reopens == 2)

    k = wire(fake(slave))
    k.fd = None
    k.mon.last_hb = None
    maybe(k, 2000.0)
    chk("沒有埠就一直試著把它拿回來", k.fc_reopens == 1,
        "否則重開失敗一次就永遠沒有下一次")

    print("\n— 沒有埠的時候，上行要記帳 —")
    w = fake(slave)
    A.Agent._write(w, None, b"x")
    chk("沒有埠不拋例外", True)
    chk("丟掉的封包有記下來", w.fc_write_dropped == 1,
        "「送了但沒到」與「沒送」事後是兩件事")
    chk("沒有假裝轉發成功", w.fwd_to_fc == 0)

    print("\n— 速率常數的可讀性 —")
    chk("B1500000 印成 1500000", A._baud_name(termios.B1500000) == "1500000",
        "tcgetattr 回的是常數，直接印是個沒意義的整數")
    chk("認不得的值原樣交出", A._baud_name(99999).startswith("?("),
        "**不要假裝認得**")

    os.close(master)
    os.close(slave)
    print("\n" + ("✓ 全部通過" if ok else "✗ 有失敗"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
