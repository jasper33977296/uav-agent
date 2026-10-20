"""5G dongle 的網卡與 IP：每秒看一次，變了就讓代理重新連上地面站。

使用者 2026-10-20：「agent 端要允許斷線又重新連線時動態讀取 dongle IP，並主動
向地面站重新連線、繼續傳送資訊；IP 可能會不同。」

**為什麼不能只靠「送不出去就重試」**：

* dongle 每次重新列舉，網卡名字（`enx`＋MAC）與 MAC 都會變，IP 也可能變
  （drone E 10-19 一天換了三個名字）
* UDP 本身沒事——socket 綁 0.0.0.0、用 sendto，每個封包核心都重新挑來源位址。
  但**電信那頭的 NAT 對映是跟著舊的來源位址／埠**，換一個來源埠逼它重建，
  地面站從新封包重新學回程位址
* 意圖通道是 TCP：掛在舊 IP 上的連線不會自己報錯，要等 10 秒靜默逾時，
  而重連退避最長會拉到 30 秒——**知道網路變了就該立刻重連**
* **新 IP 不在地面站的網段時，封包會走錯網卡**：drone E 的 eth0 有一條 metric 0 的
  預設路由（指向一個不存在的閘道），比 dongle 的 metric 700 優先。所以補一條
  「到地面站走 dongle」的主機路由（要 CAP_NET_ADMIN，unit 有給）

認 dongle 用 **USB 廠商＋驅動**，不用網卡名字：名字會變，廠商與驅動不會。
"""
import fcntl
import glob
import ipaddress
import os
import socket
import struct
import subprocess

#: 常見 5G／LTE 模組的 USB 廠商（Fibocom、Quectel、Sierra、Huawei、ZTE、Telit、SIMCom、u-blox）
MODEM_VENDORS = {"2cb7", "2c7c", "1199", "12d1", "19d2", "1bc7", "1e0e", "1546"}
#: 數據機的網卡驅動（ECM／RNDIS／QMI／MBIM／NCM）
NET_DRIVERS = {"cdc_ether", "rndis_host", "qmi_wwan", "cdc_mbim", "cdc_ncm"}
SIOCGIFADDR, SIOCGIFNETMASK = 0x8915, 0x891B


def _rd(p):
    try:
        with open(p) as f:
            return f.read().strip()
    except OSError:
        return None


def _ifreq(nic, req):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        r = fcntl.ioctl(s.fileno(), req, struct.pack("256s", nic.encode()[:15]))
        return socket.inet_ntoa(r[20:24])
    except OSError:
        return None
    finally:
        s.close()


def _gateway(nic):
    """這張網卡上的預設閘道（/proc/net/route；不必叫 ip 指令）"""
    try:
        with open("/proc/net/route") as f:
            for ln in f.readlines()[1:]:
                p = ln.split()
                if p[0] == nic and p[1] == "00000000" and int(p[3], 16) & 2:
                    return socket.inet_ntoa(struct.pack("<L", int(p[2], 16)))
    except OSError:
        pass
    return None


def dongle_net():
    """回傳 {"nic","ip","prefix","gw","vendor"}；看不到 dongle 網卡回 None。
    有 IP 的優先（一顆模組可能有好幾張網卡）"""
    found = []
    for n in glob.glob("/sys/class/net/*"):
        nic = os.path.basename(n)
        drv = os.path.basename(os.path.realpath(f"{n}/device/driver")) if os.path.exists(f"{n}/device/driver") else None
        if drv not in NET_DRIVERS:
            continue
        # 往上找 USB 裝置的廠商
        d, vendor = os.path.realpath(f"{n}/device"), None
        while d and d != "/" and vendor is None:
            vendor = _rd(f"{d}/idVendor")
            d = os.path.dirname(d)
        if vendor not in MODEM_VENDORS:
            continue        # USB 乙太網卡之類（ax88179、r8152 不在 NET_DRIVERS；cdc_ether 的別牌網卡在這裡擋）
        ip, mask = _ifreq(nic, SIOCGIFADDR), _ifreq(nic, SIOCGIFNETMASK)
        prefix = ipaddress.IPv4Network(f"0.0.0.0/{mask}").prefixlen if mask else None
        found.append({"nic": nic, "ip": ip, "prefix": prefix, "gw": _gateway(nic), "vendor": vendor})
    found.sort(key=lambda x: x["ip"] is None)
    return found[0] if found else None


def ensure_gs_route(net, gs_host):
    """讓往地面站的封包一定走 dongle。回傳說明字串；做不到回 None 並附原因。

    地面站在 dongle 的網段裡 → `GS/32 dev NIC src IP`（蓋過其他網卡上重疊的網段）；
    不在 → `GS/32 via GW dev NIC src IP`（蓋過其他網卡上的預設路由）"""
    try:
        gs = ipaddress.IPv4Address(gs_host)
    except ValueError:
        return None, f"GS_HOST={gs_host} 不是 IPv4 位址，不補路由"
    if not net or not net.get("ip"):
        return None, "dongle 網卡沒有 IP"
    onlink = gs in ipaddress.IPv4Network(f"{net['ip']}/{net['prefix'] or 32}", strict=False)
    cmd = ["ip", "route", "replace", f"{gs}/32"]
    if not onlink:
        if not net.get("gw"):
            return None, f"地面站 {gs} 不在 dongle 網段、dongle 也沒有閘道"
        cmd += ["via", net["gw"]]
    cmd += ["dev", net["nic"], "src", net["ip"], "metric", "10"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        return None, f"{' '.join(cmd)} 失敗：{r.stderr.strip()}"
    return " ".join(cmd[3:]), None
