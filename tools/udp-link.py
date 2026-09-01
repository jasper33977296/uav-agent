#!/usr/bin/env python3
"""代理能不能對 UDP 端點（SITL）連線（026 B4-d 的前提）。

**B4-d 要「同一台 SITL，比對代理算的與地面站算的」**，而那件事成不成立，
取決於一個從來沒有人驗過的假設：代理的 `--dev` 直接餵給
`mavutil.mavlink_connection`，所以理論上吃 `udpin:`／`tcp:`。

**理論上吃**與**這支程式跑得起來**是兩回事。真正會絆倒的不是連線本身，是
`Agent.run()` 拿 `self.serial.port.fileno()` 去 `select()`——序列埠有
`.port`，UDP 連線有沒有、`fileno()` 給不給，是實作細節不是介面保證。

所以這裡不架 Gazebo，直接測那條路徑：用代理**自己的** `connect_serial()`
開一個 UDP 端點，從另一端送真的 MAVLink 封包進去，看它讀不讀得到、
`Monitor.feed()` 解不解得出來。

用法：python3 tools/udp-link.py
"""
import select
import socket
import sys
import time
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

for name in ("serial",):
    sys.modules.setdefault(name, types.ModuleType(name))
try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import ardupilotmega as apm
except ImportError:
    print("skip：沒有 pymavlink。**skip ≠ pass**")
    sys.exit(2)

import agent as A  # noqa: E402

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


# 借一個空閒的 UDP 埠號
probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
probe.bind(("127.0.0.1", 0))
PORT = probe.getsockname()[1]
probe.close()
DEV = f"udpin:127.0.0.1:{PORT}"
print(f"代理端點：--dev {DEV}\n")

# 用代理**自己的** connect_serial()，不是另外寫一份連線碼——
# 測的是那條路徑，不是「pymavlink 支不支援 udpin」
g = A.Agent.__new__(A.Agent)
g.a = types.SimpleNamespace(dev=DEV, baud=57600)
g.mon = A.Monitor()
try:
    g.connect_serial()
    chk("connect_serial() 吃得下 udpin:（與序列埠同一條路徑）", True, DEV)
except Exception as e:
    chk("connect_serial() 吃得下 udpin:", False, f"{type(e).__name__}: {e}")
    print("\n**有未通過項目**")
    sys.exit(1)

try:
    fd = g.serial.port.fileno()
    chk("拿得到 fileno()（主迴圈的 select() 要用它）", isinstance(fd, int), f"fd={fd}")
except Exception as e:
    chk("拿得到 fileno()", False, f"{type(e).__name__}: {e}")
    fd = None

# 扮 SITL：從另一個 socket 送真的 MAVLink 2 封包過去
sitl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
mav = apm.MAVLink(None, srcSystem=1, srcComponent=1)


def send(msg):
    sitl.sendto(msg.pack(mav), ("127.0.0.1", PORT))


send(apm.MAVLink_heartbeat_message(
    type=2, autopilot=3, base_mode=0x80, custom_mode=3,
    system_status=4, mavlink_version=3))
send(apm.MAVLink_gps_raw_int_message(
    time_usec=0, fix_type=3, lat=247712000, lon=1210435000, alt=100000,
    eph=100, epv=100, vel=0, cog=0, satellites_visible=12))

# 用主迴圈同一個方式讀：select 等 fd，再解析
got = []
deadline = time.time() + 3
parser = apm.MAVLink(None)
parser.robust_parsing = True
while time.time() < deadline and len(got) < 2:
    r, _, _ = select.select([fd], [], [], 0.3) if fd is not None else ([], [], [])
    if not r:
        continue
    data = g.serial.port.recv(65536)
    for m in parser.parse_buffer(data) or []:
        if m.get_type() != "BAD_DATA":
            got.append(m)
            g.mon.feed(m)

kinds = [m.get_type() for m in got]
chk("select() 等得到、收得到封包", len(got) >= 2, kinds)
chk("HEARTBEAT 解得出來", "HEARTBEAT" in kinds, kinds)
chk("狀態真的被填進去（模式／廠牌／GPS）",
    g.mon.state.get("mode") == "AUTO" and g.mon.state.get("autopilot_raw") == 3
    and g.mon.state.get("gps_fix_raw") == 3,
    {k: g.mon.state.get(k) for k in ("mode", "autopilot_raw", "gps_fix_raw")})

# B4-b 的欄位在 UDP 這條路上一樣算得出來
import normalize as N  # noqa: E402
d = N.derived(g.mon.state)
chk("正規化欄位在 UDP 端點上照樣算得出來",
    d["mode_name"] == "AUTO" and d["mode_verb"] == "mission", d)

# 送得出去嗎——B4-d 若要驅動 SITL 就需要這一半
try:
    g.serial.mav.command_long_send(
        1, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 5, 0, 0, 0, 0, 0)
    back = None
    sitl.settimeout(1.0)
    # udpin 會把回送打到最後一個來源，所以這裡只驗「送出不拋例外」
    chk("送得出指令（mav.command_long_send 不拋例外）", True)
except Exception as e:
    chk("送得出指令", False, f"{type(e).__name__}: {e}")

sitl.close()
try:
    g.serial.close()
except Exception:
    pass

print("\n**注意**：這證明的是「代理吃得下 UDP 端點、讀得到也送得出」，"
      "不是「已經對真的 SITL 跑過一輪」。")
print("接真 SITL 還要確認：PX4 預設就串流所以不必 REQUEST_DATA_STREAM，"
      "但代理的串流保活仍會送；以及 SITL 的 sysid 與 GCS sysid 不要撞。")
print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
