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

**取回連線（重開序列埠）與失聯期間持續向地面站出聲還沒做**，
是 050 剩下的兩項——所以這份不驗那些。

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
    return types.SimpleNamespace(
        a=types.SimpleNamespace(baud=57600, dev="/dev/pty-test"),
        serial=types.SimpleNamespace(port=FakePort(fd)),
        port_checked=0.0, port_tampered=0,
        mon=types.SimpleNamespace(last_hb=None, state={}),
        fc_link_ok=None, fc_lost_since=None, fc_lost_nagged=0.0,
        # 類別常數借真的那幾個值——**不要讓測試自己編一套門檻**
        PORT_CHECK_S=A.Agent.PORT_CHECK_S,
        FC_HB_LOST_S=A.Agent.FC_HB_LOST_S,
        FC_LOST_NAG_S=A.Agent.FC_LOST_NAG_S,
    )


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
    chk("清掉用錯速率讀進來的垃圾", f.serial.port.flushed == 1)
    chk("核心實際速率回到 57600",
        termios.tcgetattr(slave)[5] == termios.B57600,
        "問核心，不是問 pyserial")

    setspeed(slave, termios.B115200)
    tamper(f, slave, 200.5)
    chk("節流：1 秒內不重複問核心", f.port_tampered == 1)
    tamper(f, slave, 201.5)
    chk("節流之後照樣抓得到", f.port_tampered == 2)

    print("\n— 飛控心跳 —")
    fclink(f, 300.0)
    chk("還沒看過飛控就不告警", f.fc_link_ok is None, "它可能只是比 Pi 晚上電")

    f.mon.last_hb = 300.0
    fclink(f, 301.0)
    chk("心跳新鮮＝正常", f.fc_link_ok is True)
    fclink(f, 310.0)
    chk("超過門檻＝失聯", f.fc_link_ok is False, f"門檻 {A.Agent.FC_HB_LOST_S}s")
    chk("失聯起點用最後心跳時刻", f.fc_lost_since == 300.0, "不是偵測到的那一刻")

    f.mon.last_hb = 400.0
    fclink(f, 400.5)
    chk("心跳回來＝恢復", f.fc_link_ok is True)
    chk("恢復後清掉起點", f.fc_lost_since is None)

    f.mon.last_hb = 500.0
    fclink(f, 510.0)
    nagged = f.fc_lost_nagged
    fclink(f, 540.0)
    chk("還沒滿間隔不重複喊", f.fc_lost_nagged == nagged)
    fclink(f, 575.0)
    chk("長時間失聯會持續留痕", f.fc_lost_nagged == 575.0,
        f"每 {A.Agent.FC_LOST_NAG_S:.0f}s；25 分鐘的故障不該只有起點一行")

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
