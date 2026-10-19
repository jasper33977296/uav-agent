#!/usr/bin/env python3
"""安裝資訊精靈：把一台機的 --env 檔填齊（deploy.sh 在開始安裝前呼叫）。

使用者 2026-10-19：「填 env 的操作包進一鍵部署；除非一定要使用者填，
腳本自己找得到的讓使用者確認就好。」所以：

* **只有人知道的**——帳號、密碼——用問的（密碼不回顯，問完立刻試登入）
* **找得到的**——Pi 在哪、地面站位址、dongle 的 AT 埠、飛控 sysid——先找，
  帶著「怎麼找到的」給人看，Enter 接受、或打別的值
* 找的時候**只讀**：用密碼登入去看，不裝金鑰、不改機上任何東西。
  全部確認之前，機上什麼都沒動

    python3 tools/env-wizard.py drone-e.env      # 檔案不存在就新建

結束碼：0＝檔案已填齊且人確認了；1＝人取消或填不齊。
"""
import getpass
import ipaddress
import json
import os
import re
import shlex
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIELDS = [  # (key, 說明)
    ("UAV_PI_HOST", "Pi 的位址"),
    ("UAV_PI_USER", "登入帳號（代理也以它執行）"),
    ("UAV_PI_PASS", "登入密碼"),
    ("GS_HOST", "地面站位址（遙測、記錄、對時都用它）"),
    ("MODEM_PORT", "5G dongle 的 AT 埠"),
    ("FC_SYSID", "飛控出廠的 MAVLink sysid"),
    ("FC_BAUD", "飛控序列埠鮑率"),
]
#: 樹莓派基金會的網卡 OUI
RPI_OUI = ("b8:27:eb", "dc:a6:32", "e4:5f:01", "d8:3a:dd", "2c:cf:67", "28:cd:c1", "88:a2:9e")


# ── 檔案 ──────────────────────────────────────────────────────
def load(path):
    vals, lines = {}, []
    if os.path.exists(path):
        lines = open(path).read().splitlines()
        for ln in lines:
            m = re.match(r"^([A-Z_][A-Z0-9_]*)=(.*)$", ln)
            if m:
                try:
                    v = shlex.split(m.group(2))
                    vals[m.group(1)] = v[0] if v else ""
                except ValueError:
                    vals[m.group(1)] = m.group(2)
    return vals, lines


def save(path, vals, lines):
    """只改有變的那幾行，註解與其他欄位原樣留著"""
    if not lines:
        lines = open(os.path.join(HERE, "drone.env.example")).read().splitlines()
    done = set()
    out = []
    for ln in lines:
        m = re.match(r"^([A-Z_][A-Z0-9_]*)=", ln)
        if m and m.group(1) in vals:
            out.append(f"{m.group(1)}={shlex.quote(vals[m.group(1)])}")
            done.add(m.group(1))
        else:
            out.append(ln)
    for k, _ in FIELDS:
        if k in vals and k not in done:
            out.append(f"{k}={shlex.quote(vals[k])}")
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write("\n".join(out) + "\n")
    os.chmod(path, 0o600)


# ── 問 ────────────────────────────────────────────────────────
def ask(label, suggest=None, why=None, secret=False):
    if why:
        print(f"     {why}")
    if secret:
        hint = "（Enter＝沿用已填的）" if suggest else ""
        v = getpass.getpass(f"   {label}{hint}：")
        return v or (suggest or "")
    v = input(f"   {label}" + (f" [{suggest}]" if suggest else "") + "：").strip()
    return v or (suggest or "")


def pick(label, cands, current=None):
    """cands：[(值, 說明)]。編號選、Enter 取第一個、或直接打值"""
    if current and current not in [c[0] for c in cands]:
        cands = [(current, "檔案裡原本填的")] + cands
    if not cands:
        return ask(label, current)
    for i, (v, why) in enumerate(cands, 1):
        print(f"     {i}) {v}　— {why}")
    v = input(f"   {label}（編號或直接打，Enter＝1）：").strip()
    if not v:
        return cands[0][0]
    if v.isdigit() and 1 <= int(v) <= len(cands):
        return cands[int(v) - 1][0]
    return v


# ── 找 Pi ─────────────────────────────────────────────────────
def find_pis():
    """直接相連的網段上、網卡是樹莓派 OUI 的鄰居，加上地面站學到的對端"""
    out = []
    try:
        peers = subprocess.run(["docker", "exec", "uav-command", "cat", "/state/peers.json"],
                               capture_output=True, text=True, timeout=5).stdout
        for v in (json.loads(peers).get("peers") or {}).values():
            if v.get("ip"):
                out.append((v["ip"], "⚠ 地面站學到的無人機——**已經在跑代理的機**，要更新它才選"))
    except Exception:
        pass
    links = subprocess.run(["ip", "-o", "link", "show", "up"], capture_output=True, text=True).stdout
    ifs = [m.group(1) for m in re.finditer(r"^\d+: ([^:@]+)", links, re.M)
           if not re.match(r"(lo|docker|br-|veth|virbr|tun|wg)", m.group(1))]
    for i in ifs:   # 叫醒 IPv6 鄰居表（不需要知道對方的 IPv4）
        subprocess.run(["ping", "-6", "-c", "2", "-w", "2", "-I", i, "ff02::1"],
                       capture_output=True)
    neigh = subprocess.run(["ip", "neigh"], capture_output=True, text=True).stdout
    seen = {v for v, _ in out}
    for ln in neigh.splitlines():
        m = re.match(r"(\S+) dev (\S+) lladdr (\S+)", ln)
        if not m or "FAILED" in ln:
            continue
        ip, dev, mac = m.groups()
        if not mac.lower().startswith(RPI_OUI):
            continue
        host = f"{ip}%{dev}" if ip.startswith("fe80") else ip
        if host not in seen:
            seen.add(host)
            out.append((host, f"樹莓派網卡 {mac}，在 {dev}"
                        + ("（IPv6 本地位址：不必知道它的 IPv4）" if ip.startswith("fe80") else "")))
    # **新機排前面、已在跑的排最後**：Enter＝第一個，不能讓手一滑就裝到 drone C 上
    return sorted(out, key=lambda c: (c[1].startswith("⚠"), ":" in c[0], c[0]))


# ── 連上去看（只讀）──────────────────────────────────────────
def ssh(user, host, pw, cmd, stdin=None, timeout=40):
    with tempfile.NamedTemporaryFile("w", delete=False, suffix=".sh") as f:
        f.write('#!/bin/sh\nprintf "%s\\n" "$WIZ_PW"\n')
        ap = f.name
    os.chmod(ap, 0o700)
    env = dict(os.environ, SSH_ASKPASS=ap, SSH_ASKPASS_REQUIRE="force", DISPLAY=":0", WIZ_PW=pw)
    try:
        return subprocess.run(
            ["setsid", "ssh", "-o", "ConnectTimeout=10", "-o", "StrictHostKeyChecking=accept-new",
             "-o", "NumberOfPasswordPrompts=1", f"{user}@{host}", cmd],
            input=stdin, capture_output=True, text=True, timeout=timeout, env=env)
    finally:
        os.unlink(ap)


PROBE = r'''
import glob, json, os, re, select, subprocess, termios, time
out = {}
def rd(p):
    try: return open(p).read().strip()
    except OSError: return None
dev = next((os.path.realpath(d) for d in glob.glob("/sys/bus/usb/devices/*")
            if rd(d + "/idVendor") == "2cb7" and rd(d + "/idProduct") != "01fd"), None)
out["dongle"] = bool(dev)
def under(c): return dev and os.path.realpath(c).startswith(dev + "/")
nics = [os.path.basename(n) for n in glob.glob("/sys/class/net/*") if under(n + "/device")]
out["nics"] = {}
for n in nics:
    m = re.search(r"inet (\S+)", subprocess.run(["ip","-4","-o","addr","show","dev",n],
                  capture_output=True, text=True).stdout)
    out["nics"][n] = m.group(1) if m else None
def at(port, cmd="AT", wait=1.0):
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        a = termios.tcgetattr(fd); a[0]=a[1]=a[3]=0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIOFLUSH)
        os.write(fd, (cmd + "\r").encode()); buf = b""; end = time.monotonic() + wait
        while time.monotonic() < end:
            r, _, _ = select.select([fd], [], [], 0.1)
            if r:
                buf += os.read(fd, 4096)
                if re.search(rb"\r\n(OK|ERROR)\r\n", buf): break
        return buf.decode(errors="replace")
    finally: os.close(fd)
out["at_ports"] = []
for t in sorted(glob.glob("/sys/class/tty/ttyUSB*")):
    if under(t + "/device"):
        p = "/dev/" + os.path.basename(t)
        try: ok = "OK" in at(p)
        except OSError as e: ok = str(e)
        out["at_ports"].append([p, ok])
# 飛控：**每種常見鮑率各聽 2.5 秒**（2026-10-19 第二台機的飛控是 115200，不是預設的
# 57600——照預設裝，代理會接不上飛控）。認 MAVLink 訊框用「下一個訊框剛好接在這個
# 後面」驗證：錯的鮑率讀到的是雜訊，偶爾湊出一個 0xFD 不會連著湊出好幾個
out["fc"] = None
port = "/dev/ttyAMA0"
if not os.path.exists(port):
    out["fc"] = "no_port"
else:
    res = {}
    for name, b in (("57600", termios.B57600), ("115200", termios.B115200), ("921600", termios.B921600)):
        try:
            fd = os.open(port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            out["fc"] = "busy:" + str(e); break
        try:
            a = termios.tcgetattr(fd); a[0]=a[1]=a[3]=0
            a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL; a[4] = a[5] = b
            termios.tcsetattr(fd, termios.TCSANOW, a); termios.tcflush(fd, termios.TCIFLUSH)
            buf, end = b"", time.monotonic() + 2.5
            while time.monotonic() < end:
                r, _, _ = select.select([fd], [], [], 0.2)
                if r: buf += os.read(fd, 4096)
        finally: os.close(fd)
        frames, ids, i = 0, {}, 0
        while i < len(buf) - 12:
            if buf[i] == 0xFD:
                n = 12 + buf[i+1] + (13 if buf[i+2] & 1 else 0)     # 標頭 10＋CRC 2＋簽章
                if i + n < len(buf) and buf[i+n] == 0xFD:
                    frames += 1
                    if buf[i+6] == 1:                                # compid 1＝自駕儀
                        ids[buf[i+5]] = ids.get(buf[i+5], 0) + 1
                    i += n; continue
            elif buf[i] == 0xFE:
                n = 8 + buf[i+1]
                if i + n < len(buf) and buf[i+n] == 0xFE:
                    frames += 1
                    if buf[i+4] == 1:
                        ids[buf[i+3]] = ids.get(buf[i+3], 0) + 1
                    i += n; continue
            i += 1
        res[name] = {"bytes": len(buf), "frames": frames, "sysids": ids}
    if out["fc"] is None:
        out["fc"] = res
out["agent_running"] = subprocess.run(["systemctl","is-active","--quiet","uav-agent"]).returncode == 0
print(json.dumps(out))
'''


def local_ip_in(cidr):
    """這台（地面站）在同一個網段上的位址"""
    try:
        net = ipaddress.ip_interface(cidr).network
    except ValueError:
        return None
    r = subprocess.run(["ip", "-4", "-o", "addr"], capture_output=True, text=True).stdout
    for m in re.finditer(r"inet (\S+)", r):
        if ipaddress.ip_interface(m.group(1)).ip in net:
            return str(ipaddress.ip_interface(m.group(1)).ip)
    return None


def unit_default(k):
    m = re.search(rf"^Environment={k}=(.*)$",
                  open(os.path.join(HERE, "systemd/uav-agent.service")).read(), re.M)
    return m.group(1) if m else None


def main():
    if len(sys.argv) != 2:
        print(__doc__); return 1
    path = sys.argv[1]
    vals, lines = load(path)
    print(f"\n== 安裝資訊：{path}{'' if lines else '（新檔，照 drone.env.example）'} ==")
    if all(vals.get(k) for k, _ in FIELDS):
        show(vals)
        a = input("\n   照這些開始？[Y＝開始／e＝逐欄改／n＝取消]：").strip().lower()
        if a in ("", "y"):
            return 0
        if a != "e":
            return 1

    # 1. Pi 在哪
    print("\n① Pi 的位址（找相連網段上的樹莓派…）")
    vals["UAV_PI_HOST"] = pick("Pi 的位址", find_pis(), vals.get("UAV_PI_HOST"))
    # 2. 帳號、密碼：只有人知道
    print("\n② 登入帳號與密碼（只有你知道；用來裝金鑰，之後全程用金鑰）")
    for _ in range(3):
        vals["UAV_PI_USER"] = ask("帳號", vals.get("UAV_PI_USER") or "pi")
        vals["UAV_PI_PASS"] = ask("密碼", vals.get("UAV_PI_PASS"), secret=True)
        r = ssh(vals["UAV_PI_USER"], vals["UAV_PI_HOST"], vals["UAV_PI_PASS"], "echo ok")
        if r.returncode == 0 and "ok" in r.stdout:
            print("     ✓ 登入成功")
            break
        print(f"     ✗ 登不進去：{(r.stderr.strip().splitlines() or ['?'])[-1]}")
        vals["UAV_PI_PASS"] = ""
    else:
        print("   三次都登不進去，取消（檔案沒有改）"); return 1

    # 3. 連上去看（只讀）
    print("\n③ 到機上找 dongle 與飛控（只讀，不改任何東西）…")
    r = ssh(vals["UAV_PI_USER"], vals["UAV_PI_HOST"], vals["UAV_PI_PASS"], "python3 -", PROBE, 60)
    try:
        p = json.loads(r.stdout.strip().splitlines()[-1])
    except Exception:
        print(f"     ✗ 機上檢查失敗：{r.stderr.strip()[-200:]}"); p = {}

    # 地面站位址：dongle 拿到的網段上，這台自己的位址
    gs_c = []
    for nic, cidr in (p.get("nics") or {}).items():
        if cidr and (ip := local_ip_in(cidr)):
            gs_c.append((ip, f"這台在 dongle 網段（{nic} {cidr}）上的位址"))
    if not gs_c and p.get("dongle"):
        print("     ⚠ dongle 網卡還沒有 IP（Ubuntu 要靠安裝時補的 netplan）——地面站位址用預設值，請確認")
    if (d := unit_default("GS_HOST")) and d not in [c[0] for c in gs_c]:
        gs_c.append((d, "unit 裡的共用預設值"))
    print()
    vals["GS_HOST"] = pick("地面站位址", gs_c, vals.get("GS_HOST"))

    # AT 埠：屬於 dongle、回得出 OK 的
    at_c = [(port, "對 AT 回 OK") for port, ok in p.get("at_ports") or [] if ok is True]
    if not p.get("dongle"):
        print("     ⚠ 機上看不到 Fibocom dongle（沒插、沒電、或還停在 mass-storage）")
    elif not at_c:
        print(f"     ⚠ dongle 的埠都沒回 OK：{p.get('at_ports')}")
    if (d := unit_default("MODEM_PORT")) and d not in [c[0] for c in at_c]:
        at_c.append((d, "unit 裡的共用預設值（沒驗證）"))
    print()
    vals["MODEM_PORT"] = pick("dongle 的 AT 埠", at_c, vals.get("MODEM_PORT"))

    # 飛控 sysid 與鮑率：哪個鮑率下有連續的 MAVLink 訊框
    fc = p.get("fc")
    fc_c, baud_c = [], []
    if isinstance(fc, dict):
        # 要有**自駕儀自己（compid 1）**的訊框才算：錯的鮑率偶爾也會湊出幾個「連續訊框」
        # （實測 921600 在雜訊裡湊出 10 個），但湊不出一致的自駕儀來源
        good = sorted(((b, r) for b, r in fc.items() if r["frames"] >= 3 and r["sysids"]),
                      key=lambda x: -x[1]["frames"])
        for b, r in good:
            baud_c.append((b, f"2.5 秒讀到 {r['frames']} 個連續的 MAVLink 訊框"))
            for sid, n in sorted(r["sysids"].items(), key=lambda x: -x[1]):
                if str(sid) not in [c[0] for c in fc_c]:
                    fc_c.append((str(sid), f"{b} 鮑率下自駕儀（compid 1）用這個號碼送了 {n} 個訊框"))
        if not good:
            print("     ⚠ 三種鮑率都讀不到 MAVLink：" + "、".join(
                f"{b} 收 {r['bytes']} bytes" for b, r in fc.items()) + "（飛控沒接、沒電、或接錯腳）")
    else:
        why = "沒有 /dev/ttyAMA0（UART overlay 還沒開，安裝時會補）" if fc == "no_port" else (
            "飛控埠被佔用（代理在跑？）" if isinstance(fc, str) and fc.startswith("busy") else fc)
        print(f"     ⚠ 聽不到飛控：{why}")
    for d, why in ((unit_default("FC_BAUD"), "unit 裡的共用預設值（沒驗證）"),):
        if d and d not in [c[0] for c in baud_c]:
            baud_c.append((d, why))
    if "1" not in [c[0] for c in fc_c]:
        fc_c.append(("1", "ArduPilot 出廠預設（沒驗證）"))
    print()
    vals["FC_BAUD"] = pick("飛控鮑率", baud_c, vals.get("FC_BAUD"))
    print()
    vals["FC_SYSID"] = pick("飛控 sysid", fc_c, vals.get("FC_SYSID"))

    missing = [k for k, _ in FIELDS if not vals.get(k)]
    if missing:
        print(f"\n   還有沒填的：{' '.join(missing)}——取消"); return 1
    show(vals)
    if input("\n   存檔並開始安裝？[Y/n]：").strip().lower() not in ("", "y"):
        save(path, vals, lines)
        print(f"   已存到 {path}，沒有開始安裝"); return 1
    save(path, vals, lines)
    print(f"   已存到 {path}（權限 600、gitignored）")
    return 0


def show(vals):
    print()
    for k, label in FIELDS:
        v = vals.get(k, "")
        print(f"   {k:12} = {'*' * len(v) if 'PASS' in k else v}　（{label}）")


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (KeyboardInterrupt, EOFError):
        print("\n   取消（檔案沒有改）"); sys.exit(1)
