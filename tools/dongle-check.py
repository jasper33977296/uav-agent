#!/usr/bin/env python3
"""5G dongle 到底能不能用（在 Pi 上跑，**只用標準函式庫**：空機也跑得動）。

deploy.sh 在動機上任何東西之前先跑這支，**沒過就不部署**（使用者 2026-10-19：
一定要先確認 dongle 正常運行，沒有的話不能進行下一步）。這台機的用途就是
「經 5G 跟地面站講話、同時量 5G」，5G 不通時部署上去的是一個什麼都做不了、
而且看起來像裝好了的代理。

四格，全部要過：

1. USB：看得到 Fibocom（2cb7）**而且不在 mass-storage 模式**。模組冷開機會先以
   01fd 列舉、modeswitch 之後才變成數據機；停在 01fd＝還沒好或切換失敗
2. 網路介面：那顆 USB 裝置底下有網卡，而且拿到 IPv4
3. 地面站：**從那張網卡** ping 得到 GS_HOST。走預設路由的話，有線網路通也會過，
   那就不是在驗 5G
4. AT 埠：MODEM_PORT 屬於這顆 dongle、回得出 `AT` → `OK`，`AT+CESQ` 量得到訊號
   （量不到＝沒註冊上網路）。**代理在跑時不去搶埠**——它正開著這個埠在取樣，
   兩邊同時讀會互相吃掉回應；改看代理自己報的取樣數

    python3 dongle-check.py --gs 10.141.2.22 --port /dev/ttyUSB1 [--state /opt/uav-agent/state.json]
"""
import argparse
import glob
import json
import os
import re
import select
import subprocess
import sys
import termios
import time

FIBOCOM = "2cb7"
MASS_STORAGE = {"01fd"}


def rd(p):
    try:
        with open(p) as f:
            return f.read().strip()
    except OSError:
        return None


def usb_devices():
    out = []
    for d in glob.glob("/sys/bus/usb/devices/*"):
        if rd(f"{d}/idVendor") == FIBOCOM:
            out.append({"path": os.path.realpath(d), "pid": rd(f"{d}/idProduct"),
                        "name": rd(f"{d}/product") or "?"})
    return out


def under(dev_path, child):
    """sysfs 裡的 child 是不是長在 dev_path 這顆 USB 裝置底下"""
    try:
        return os.path.realpath(child).startswith(dev_path + "/")
    except OSError:
        return False


def ipv4_of(ifname):
    r = subprocess.run(["ip", "-4", "-o", "addr", "show", "dev", ifname],
                       capture_output=True, text=True)
    m = re.search(r"inet (\S+)", r.stdout)
    return m.group(1) if m else None


def at(port, cmd, wait=1.0):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        a = termios.tcgetattr(fd)
        a[0] = 0; a[1] = 0; a[3] = 0                       # raw
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, a)
        termios.tcflush(fd, termios.TCIOFLUSH)
        os.write(fd, (cmd + "\r").encode())
        buf, end = b"", time.monotonic() + wait
        while time.monotonic() < end:
            r, _, _ = select.select([fd], [], [], 0.1)
            if r:
                buf += os.read(fd, 4096)
                if re.search(rb"\r\n(OK|ERROR|\+CME ERROR[^\r]*)\r\n", buf):
                    break
        return buf.decode(errors="replace")
    finally:
        os.close(fd)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gs", required=True)
    ap.add_argument("--port", required=True)
    ap.add_argument("--state", default="/opt/uav-agent/state.json")
    a = ap.parse_args()
    ok = True

    def say(good, label, note):
        nonlocal ok
        ok &= bool(good)
        print(f"   {'✓' if good else '✗'} {label}｜{note}")

    # 1. USB
    devs = usb_devices()
    live = [d for d in devs if d["pid"] not in MASS_STORAGE]
    if not devs:
        say(False, "USB", "看不到 Fibocom（2cb7）——沒插、沒供電，或模組正在重開")
        return 1
    if not live:
        say(False, "USB", f"只有 mass-storage 模式（{devs[0]['pid']}）——模組還沒切成數據機；"
            "等 30 秒再試，一直這樣就是 modeswitch 沒裝或供電不足")
        return 1
    dev = live[0]
    say(True, "USB", f"{FIBOCOM}:{dev['pid']} {dev['name']}")

    # 2. 網卡
    nics = [os.path.basename(n) for n in glob.glob("/sys/class/net/*")
            if under(dev["path"], f"{n}/device")]
    nic = next((n for n in nics if ipv4_of(n)), None)
    if not nics:
        say(False, "網路介面", "這顆 dongle 底下沒有網卡（驅動沒綁上？）")
    elif not nic:
        say(False, "網路介面", f"{', '.join(nics)} 沒有 IPv4——還沒撥上網路")
    else:
        say(True, "網路介面", f"{nic} {ipv4_of(nic)}")

    # 3. 地面站，**指定從 5G 那張出去**
    if nic:
        r = subprocess.run(["ping", "-c", "3", "-W", "2", "-I", nic, a.gs],
                           capture_output=True, text=True)
        m = re.search(r"(\d+) received", r.stdout)
        rtt = re.search(r"= [\d.]+/([\d.]+)/", r.stdout)
        got = int(m.group(1)) if m else 0
        say(got > 0, "地面站", f"從 {nic} ping {a.gs}：3 收 {got}"
            + (f"，平均 {rtt.group(1)} ms" if rtt else "——5G 有 IP 但到不了地面站"))

    # 4. AT 埠
    port = os.path.realpath(a.port)
    tty = os.path.basename(port)
    if not os.path.exists(port):
        ports = sorted(os.path.join("/dev", os.path.basename(t))
                       for t in glob.glob("/sys/class/tty/ttyUSB*")
                       if under(dev["path"], f"{t}/device"))
        say(False, "AT 埠", f"{a.port} 不存在；這顆 dongle 的埠是 {ports or '（沒有）'}"
            "——在 /etc/uav-agent.env 設 MODEM_PORT")
        return 0 if ok else 1
    if not under(dev["path"], f"/sys/class/tty/{tty}/device"):
        say(False, "AT 埠", f"{a.port} 不是這顆 dongle 的埠")
        return 0 if ok else 1
    agent_on = subprocess.run(["systemctl", "is-active", "--quiet", "uav-agent"]).returncode == 0
    if agent_on:
        # 代理開著這個埠在取樣——不去搶，看它自己報的
        try:
            st = json.load(open(a.state))
            age = time.time() - os.path.getmtime(a.state)
            m = st.get("modem") or {}
        except Exception:
            st, age, m = None, None, {}
        if st is None or age is None or age > 30:
            say(False, "AT 埠", "代理在跑但 state.json 讀不到或不新鮮——不知道它量不量得到")
        elif not m:
            say(False, "AT 埠", "代理在跑但沒有在取樣（MODEM_PORT 沒設？）")
        else:
            sig = m.get("rsrp") is not None or m.get("sinr") is not None
            say(m.get("samples", 0) > 0 and sig, "AT 埠（代理在取樣）",
                f"取樣 {m.get('samples')} 次、失敗 {m.get('errors')}、"
                f"RSRP {m.get('rsrp')} SINR {m.get('sinr')}")
    else:
        try:
            r1 = at(port, "AT")
            if "OK" not in r1:
                say(False, "AT 埠", f"{a.port} 對 AT 沒回 OK（回：{r1.strip()[:60] or '無'}）")
            else:
                r2 = at(port, "AT+CESQ")
                m = re.search(r"\+CESQ:\s*([\d,\s]+)", r2)
                vals = [int(x) for x in m.group(1).split(",")] if m else []
                # 最後三個是 5G 的 ss_rsrq／ss_rsrp／ss_sinr，255＝量不到
                nr = vals[6:9] if len(vals) >= 9 else []
                lte = vals[4:6] if len(vals) >= 6 else []
                has = any(v != 255 for v in nr) or any(v != 255 for v in lte)
                say(has, "AT 埠", f"{a.port} 回 OK；CESQ {','.join(map(str, vals)) or '無'}"
                    + ("" if has else "——量不到訊號：沒註冊上網路（SIM、天線、訊號）"))
        except OSError as e:
            say(False, "AT 埠", f"開不了 {a.port}：{e}（pi 在 dialout 群組嗎？有別的程式開著嗎？）")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
