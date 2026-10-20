#!/usr/bin/env python3
"""飛控參數的基準檢查與設定（deploy.sh 在代理停著時呼叫；也可以單獨在機上跑）。

使用者 2026-10-20：「把 MAVLink 版本檢查、baud rate 等等的預設參數配置也加到部署
腳本中，目標是讓一台全空、裝有 dongle 的無人機一鍵部署上系統、可以監控。」

**跟 set-fc-params.py 的差別**：那支的清單是 drone C 一台的裁定（4S 電壓、4200 mAh、
第 5 格模式），換一台就不對。這支只放**每一台都該成立**的條件；跟機體有關的
（電池幾 S、多少 mAh）要人給，不給就不碰。

**名字不寫死**：用索引讀**全部**參數（PARAM_REQUEST_LIST，缺的再逐索引補），
再看這台到底有哪些名字。2026-10-20 drone E 是舊韌體，叫 WPNAV_SPEED 不叫 WP_SPD，
而 ArduPilot 對不存在的名字一個字都不回——照名字問只會白等。

規則（每條都說得出理由）：
  1. 鏈路：接 Pi 的那個序列埠要是 MAVLink2。認法：SERIALn_BAUD 等於這條線的鮑率、
     而 PROTOCOL 是 1（v1）的埠 → 改 2。**改了要重開飛控才生效**，這支會在
     未解鎖時自己重開並等它回來、再讀回核對
  2. 失聯後備：FS_GCS_ENABLE=1、FS_GCS_TIMEOUT=45（> 代理單飛上限 30）
  3. 電池（只在給了 --cells 時）：LOW＝3.5 V/cell、CRT＝3.3 V/cell、低電量 RTL、危險 Land；
     給了 --capacity 才改 BATT_CAPACITY

    python3 fc-setup.py --dev /dev/ttyAMA0 --baud 115200            # 只看
    python3 fc-setup.py --dev /dev/ttyAMA0 --baud 115200 --apply    # 寫、讀回、需要時重開飛控

最後一行是 JSON（給 deploy.sh 讀）：{"ok", "todo", "applied", "rebooted", "fw", ...}
"""
import argparse
import json
import sys
import time

from pymavlink import mavutil

M = mavutil.mavlink
BAUD_CODE = {9600: 9, 19200: 19, 38400: 38, 57600: 57, 111100: 111, 115200: 115,
             230400: 230, 256000: 256, 460800: 460, 500000: 500, 921600: 921,
             1500000: 1500, 2000000: 2000}
#: 改了要重開飛控才生效的
NEEDS_REBOOT_PREFIX = ("SERIAL",)

ap = argparse.ArgumentParser()
ap.add_argument("--dev", default="/dev/ttyAMA0")
ap.add_argument("--baud", type=int, default=57600)
ap.add_argument("--apply", action="store_true")
ap.add_argument("--cells", type=int, default=0, help="電池串數（S）；0＝不碰電池參數")
ap.add_argument("--capacity", type=int, default=0, help="電池容量 mAh；0＝不碰")
ap.add_argument("--timeout", type=float, default=90.0)
a = ap.parse_args()
result = {"ok": False, "todo": {}, "applied": {}, "rebooted": False, "notes": []}


def _crash(t, v, tb):
    """**當掉也要交出 JSON**：deploy.sh 靠最後一行判斷結果，沒有那一行就分不出
    「檢查過、沒問題」與「根本沒檢查」（2026-10-20 這樣讓部署安靜地停在一半）"""
    import traceback
    traceback.print_exception(t, v, tb, file=sys.stderr)
    result["notes"].append(f"fc-setup 當掉：{t.__name__}: {v}")
    print(json.dumps(result, ensure_ascii=False))
    sys.exit(1)


sys.excepthook = _crash


def out(code):
    print(json.dumps(result, ensure_ascii=False))
    sys.exit(code)


def connect():
    m = mavutil.mavlink_connection(a.dev, baud=a.baud, source_system=254)
    hb = m.wait_heartbeat(timeout=15)
    return m, hb


print(f"   連線 {a.dev}@{a.baud} …", flush=True)
m, hb = connect()
if hb is None:
    result["notes"].append("沒有心跳（鮑率不對？埠被別人開著？）")
    print("   ✗ 沒有心跳", flush=True)
    out(1)
tgt = (m.target_system, m.target_component)
armed = bool(getattr(hb, "base_mode", 0) & M.MAV_MODE_FLAG_SAFETY_ARMED)
# 這條線上實際跑的版本：看收到的那個訊框的第一個位元組（0xFD＝v2、0xFE＝v1）
result.update(sysid=tgt[0], armed=armed,
              mavlink=2 if hb.get_msgbuf()[:1] == b"\xfd" else 1)
print(f"   ✓ 飛控 sysid={tgt[0]}，這條線上看到的是 MAVLink{result['mavlink']}", flush=True)
if armed:
    result["notes"].append("飛控是解鎖狀態——不在天上改參數")
    print("   ✗ 解鎖中，不動", flush=True)
    out(1)


def fetch_all(m):
    """用索引讀全部參數：先要整份清單，缺的再逐索引補（索引跟名字無關、每版都一樣能用）"""
    vals, count, deadline = {}, None, time.time() + a.timeout
    by_idx = {}
    m.mav.param_request_list_send(*tgt)
    last_new = time.time()
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is not None:
            name = msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()
            name = name.rstrip("\x00")
            if msg.param_index != 65535:
                count = msg.param_count
                if msg.param_index not in by_idx:
                    last_new = time.time()
                by_idx[msg.param_index] = name
            vals[name] = float(msg.param_value)
        if count and len(by_idx) >= count:
            break
        if count and time.time() - last_new > 3:      # 停了：逐索引補缺的
            for i in [i for i in range(count) if i not in by_idx][:30]:
                m.mav.param_request_read_send(tgt[0], tgt[1], b"", i)
                time.sleep(0.02)
            last_new = time.time()
    return vals, count


print("   讀全部參數（用索引，跟韌體版本的命名無關）…", flush=True)
cur, count = fetch_all(m)
if not count or len(cur) < count * 0.98:
    result["notes"].append(f"參數沒讀齊（{len(cur)}/{count}）——不改")
    print(f"   ✗ 參數沒讀齊（{len(cur)}/{count}）", flush=True)
    out(1)
print(f"   ✓ 讀到 {len(cur)} 個參數", flush=True)
result["param_count"] = len(cur)

plan = {}          # name → (目標值, 為什麼)

# 1. 鏈路：接 Pi 的埠要是 MAVLink2
code = BAUD_CODE.get(a.baud, a.baud)
ports = [n for n in range(10) if cur.get(f"SERIAL{n}_BAUD") == code
         and cur.get(f"SERIAL{n}_PROTOCOL") in (1.0, 2.0)]
if not ports:
    result["notes"].append(f"找不到鮑率 {a.baud} 的 MAVLink 序列埠——認不出接 Pi 的是哪個")
for n in ports:
    if cur.get(f"SERIAL{n}_PROTOCOL") == 1.0:
        plan[f"SERIAL{n}_PROTOCOL"] = (2, f"SERIAL{n}（{a.baud}）是 MAVLink1。代理與地面站都送 v2；"
                                          "v1 的線上收不到簽章、擴充欄位，訊息 ID > 255 的也過不去")
result["link_ports"] = [f"SERIAL{n}" for n in ports]

# 2. 失聯後備
if "FS_GCS_ENABLE" in cur:
    plan["FS_GCS_ENABLE"] = (1, "GCS failsafe 開：代理自己掛掉時，飛控這層是唯一的後備")
if "FS_GCS_TIMEOUT" in cur:
    plan["FS_GCS_TIMEOUT"] = (45, "要大於代理的單飛上限（30 秒），否則飛控會搶在代理之前動手")

# 3. 電池（人給了才碰）
if a.cells:
    for n, v, why in (("BATT_LOW_VOLT", 3.5 * a.cells, f"3.5 V/cell × {a.cells}S"),
                      ("BATT_CRT_VOLT", 3.3 * a.cells, f"3.3 V/cell × {a.cells}S"),
                      ("BATT_FS_LOW_ACT", 2, "低電量 → RTL"),
                      ("BATT_FS_CRT_ACT", 1, "危險電量 → Land")):
        if n in cur:
            plan[n] = (round(v, 2), why)
else:
    result["notes"].append("沒給電池串數——電池門檻沒有檢查（不是當成正確）")
if a.capacity and "BATT_CAPACITY" in cur:
    plan["BATT_CAPACITY"] = (a.capacity, "剩餘電量百分比是用它算的")

same = lambda x, y: abs(x - y) <= max(1e-6, abs(y) * 1e-6)   # float32 往返
todo = {n: v for n, (v, _) in plan.items() if not same(cur[n], v)}
for n, (v, why) in plan.items():
    mark = "  " if n not in todo else "→ "
    print(f"   {mark}{n:18} {cur[n]:>8.4g} → {v:<8.4g} {'（已經是）' if n not in todo else ''}　{why}",
          flush=True)
result["todo"] = todo
result["fw_names"] = "4.7" if "WP_SPD" in cur else ("pre-4.7" if "WPNAV_SPEED" in cur else "unknown")
if not todo:
    result["ok"] = True
    print("   ✓ 飛控參數已經符合基準", flush=True)
    out(0)
if not a.apply:
    print(f"   （只看）有 {len(todo)} 個要改", flush=True)
    out(0)

for n, v in todo.items():
    m.mav.param_set_send(tgt[0], tgt[1], n.encode(), float(v), M.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.2)


def read_back(m, names, t=15):
    got, end = {}, time.time() + t
    for n in names:
        m.mav.param_request_read_send(tgt[0], tgt[1], n.encode(), -1)
        time.sleep(0.05)
    while len(got) < len(names) and time.time() < end:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is None:
            continue
        nm = (msg.param_id if isinstance(msg.param_id, str) else msg.param_id.decode()).rstrip("\x00")
        if nm in names:
            got[nm] = float(msg.param_value)
    return got


time.sleep(1.0)
back = read_back(m, list(todo))
bad = [n for n, v in todo.items() if n not in back or not same(back[n], v)]
for n, v in todo.items():
    print(f"   {'✓' if n not in bad else '✗'} {n} 讀回 {back.get(n)}（要 {v}）", flush=True)
result["applied"] = {n: back.get(n) for n in todo}
if bad:
    result["notes"].append(f"沒寫進去：{bad}")
    out(1)

# 要重開才生效的：未解鎖時自己重開、等它回來、再核對一次
if any(n.startswith(NEEDS_REBOOT_PREFIX) for n in todo):
    print("   序列埠設定要重開飛控才生效 → 重開（未解鎖）…", flush=True)
    m.mav.command_long_send(tgt[0], tgt[1], M.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                            0, 1, 0, 0, 0, 0, 0, 0)
    m.close()
    time.sleep(8)
    for _ in range(6):
        try:
            m, hb = connect()
        except Exception:
            hb = None
        if hb is not None:
            break
        time.sleep(3)
    if hb is None:
        result["notes"].append("重開飛控後 30 秒沒有心跳——去機旁看")
        out(1)
    result["rebooted"] = True
    again = read_back(m, [n for n in todo if n.startswith(NEEDS_REBOOT_PREFIX)])
    print(f"   ✓ 飛控重開回來了，序列埠設定：{again}", flush=True)
result["ok"] = True
out(0)
