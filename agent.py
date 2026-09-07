#!/usr/bin/env python3
"""機上智能代理：飛控 ←→ 地面站 的橋，外加本地監控。

    飛控 (ArduPilot, /dev/ttyAMA0 @57600, sysid 1)
      ↕ MAVLink over UART
    [ 本代理 ]
      ├── UDP → 地面站:14540   backend 資料通道（唯讀入庫）
      ├── UDP ↔ 地面站:14541   command 指令通道（雙向）
      └── 本地監控：串流保活、鏈路健康、狀態變化事件

設計取捨，都有實測依據（2026-08-24）：

* **轉發用原始位元組**（`msg.get_msgbuf()`），不重新編碼。重編會換掉 seq 與
  簽章，也可能因為我們的方言表較舊而丟失欄位。原封不動轉出去最誠實。

* **一定要驗 CRC 才轉發**。實測過不驗 CRC 的版本：46KB 流量就解出大量幽靈
  訊框（sysid 冒出 15 個不存在的值）。轉發幽靈訊框等於餵髒資料給地面站。

* **串流要保活，不是設定一次就好**。ArduPilot 預設不推遙測，而且飛控重開、
  鏈路中斷重連之後串流速率會回到 0。所以偵測到「遙測停了但心跳還在」就重送
  請求，不是只在啟動時送一次。

* **兩條 UDP 用不同 socket**。command 服務靠「收到封包的來源位址」建回程路由
  表，兩條混用同一個 socket 會讓回程路由指向錯的通道。

* 對飛控我們的身分是 **sysid 255 / comp 190**（GCS 慣用值）。地面站的 command
  服務用 254，刻意錯開——在飛控眼中是兩個不同的 GCS，不會互相蓋掉。

* **地面站的封包一律原封轉發**。地面站已於 2026-08-24 移除虛擬搖桿
  （uav-system issue 035），系統範圍收斂為航路管理，不再有連續操縱封包。
  先前為了繞過 ArduPilot 的 `SYSID_MYGCS` 檢查而做的「以代理身分重新編碼
  MANUAL_CONTROL／RC_CHANNELS_OVERRIDE」已隨之移除——沒有來源，就沒有轉譯。

  但那件事留下的前提仍然要緊，只是原本寫的理由是錯的：

  > **GCS 心跳確實存在，但發的人不是代理，是地面。**（2026-08-31 查證兩邊）
  > `uav-system` 的指令服務每秒對每台已知的機送一則 `MAV_TYPE_GCS` 心跳，
  > `source_system = GCS_SYSID = 255`（`apps/command/app/mav.py` 的 `_tick`；
  > 255 是 2026-08-25 才從 254 改的，正是為了對上機上的 `SYSID_MYGCS`）。
  > 它走的路是：**指令服務 → UDP over 5G → 本代理 → 驗 CRC → 序列埠 → 飛控**。
  > 代理是那條路上的轉發者，自己**從不呼叫 `heartbeat_send`**。
  >
  > **所以 5G 一斷，飛控那邊的 GCS 心跳是真的停了。** 原文寫的「飛控完全
  > 無感」不成立（心跳源在地面，不在機上）；而 2026-08-31 我一度更正成
  > 「飛控可能一直處在 GCS failsafe」也不對——那是過度修正，平常心跳送得到。
  > 正確的是：**平常收得到、斷線時收不到**，正好就是 `FS_GCS` 要偵測的事。
  >
  > 這讓 `FS_GCS_ENABLE` 從技術細節變成飛安取捨，uav-system 已立為 039 的
  > 待裁項：**開**＝多一層完全獨立於 5G 的防線，但 ArduPilot 的 GCS failsafe
  > 逾時遠短於我們的 30 秒單飛上限，**飛控會搶在選項 C 前面**，使用者裁定的
  > 「失聯後跑完任務再 RTL」實際上不會發生；**關**＝選項 C 照裁定生效，但
  > 代理自己掛掉時就沒有任何人了。
  >
  > **待真機覆核**：讀 `FS_GCS_ENABLE` 與 `FS_GCS_TIMEOUT`（後者決定它會
  > 搶先多久）、以及 `SYSID_MYGCS` 的現值。**在裁定之前不要動那些參數**，
  > 也不要在代理裡補一個 `heartbeat_send`——那會讓機上多一個心跳源，
  > 等於在沒有裁定的情況下把 `FS_GCS` 這條防線悄悄關掉。

  代理這一層的正確性不受影響：`_check_gs_link` 本來就自己判斷，不依賴飛控。
  而**分狀態的判斷只有代理做得到**——飛控看得見「心跳沒了」，看不見那代表
  什麼，它只能做一件粗的事。見 `_check_gs_link`。
"""
import argparse
import json
import logging
import os
import select
import socket
import sys
import urllib.error
import urllib.request
import time
from collections import defaultdict

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as apm

import guard
import normalize
import recorder
import uploader
import route
from autopilot import get_driver
from backfill import Backfill
from intent import IntentLink
from modem import ModemSampler

log = logging.getLogger("uav-agent")

AGENT_VERSION = "0.12.0"         # 0.12：訊號只在離地時記（停在地面不入庫）
                                 # 0.11：預檢原因收集（說得出是哪一項）＋ STATUSTEXT 分段重組
                                 # 0.10：失聯接管（電量兩判準）＋ PILOT_CONTROL 放行 rtl/land
                                 # 0.9：訊號記錄通道接上（研究資料原本沒在存）
                                 # 0.8：模式溯源三態（不知道 ≠ 飛手）
                                 # 0.7：回傳現況上報（record_upload）＋遺失通報
                                 # 0.6：機上錄製自動回傳
                                 # 0.5：機上原始層錄製
                                 # 0.4：驅動搬到機上（026 B4-b）、039 複裁六項
#                                 （RC 前置／逐則回執／乾跑補送／電量未知不續飛）、
#                                 補傳改單調時鐘、FS_GCS 開機自檢
# **改了行為就要改這個字串。** 0.3 → 0.4 的八個 commit 一次都沒有動它，於是
# 地面站顯示的版本與機上實際跑的程式沒有關係——`ui-spec.md` §0.2f 記著同一件
# 事的教訓：事故還原的成本有相當部分來自「系統說不出自己是哪一版」。
# 部署之後看得到版本變了，才算驗證過這次部署真的生效。


def agent_uid() -> str | None:
    """這台**伴飛電腦**的唯一序號（樹莓派燒在 SoC 上的那個）。

    **與 board_uid 回答不同的問題**：board_uid＝這是哪一架飛機（飛控板），
    agent_uid＝這是哪一台伴飛電腦。5G 模組、Wi-Fi 卡、代理版本屬於後者——
    混成一個欄位，換件時就說不清是哪一邊變了。

    把 Pi 拆到另一台飛機上，它回報的會是**那台**飛控的 board_uid，
    所以身分不會被代理帶著走。
    """
    try:
        with open("/proc/device-tree/serial-number", "rb") as f:
            return f.read().rstrip(b"\x00").decode().strip() or None
    except OSError:
        return None


# 要飛控持續送的重點訊息（逐一設間隔，比舊式 ALL 串流精準）
WANTED = [1, 24, 27, 29, 30, 33, 36, 42, 62, 65, 74, 116, 125, 147, 193,
          241, 245]   # 245＝EXTENDED_SYS_STATE（landed_state）

COPTER_MODE_ID = {v: k for k, v in {
    6: "RTL", 9: "LAND", 21: "SMART_RTL", 5: "LOITER"}.items()}
#: 已經在「自己回家／降落」的模式，失聯處置不再介入
SELF_RECOVERING = {"RTL", "LAND", "SMART_RTL", "AUTO_RTL"}
#: **人不可能用手把飛機飛在這兩個模式裡**——它們的桿位輸入不控制姿態，
#: 所以看到它們就確定是自動化在飛（我方或地面站下的），不必再溯源。
#: LOITER 不在這裡：它既是地面站的「暫停」也是飛手最常用的手飛模式，
#: 誰下的必須靠溯源分辨（見 Agent._mode_owner）。
AUTOMATED_MODES = {"AUTO", "GUIDED", "GUIDED_NOGPS"}

COPTER_MODE = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
               5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 16: "POSHOLD",
               17: "BRAKE", 18: "THROW", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
               27: "AUTO_RTL"}
GPS_FIX = {0: "NO_GPS", 1: "NO_FIX", 2: "2D", 3: "3D", 4: "DGPS",
           5: "RTK_FLOAT", 6: "RTK_FIXED"}
#: MAV_LANDED_STATE。0＝UNDEFINED 是「這台韌體不回報」，不是「不在地上」
LANDED = {1: "on_ground", 2: "in_air", 3: "takeoff", 4: "landing"}

#: `MAV_SYS_STATUS_SENSOR_RC_RECEIVER`（1 << 16）。SYS_STATUS 的 present／health
#: 兩組旗標各有這一位元，合起來才是三態（039 複裁 A）：
#:
#: * present 沒有這一位 → **這台韌體沒回報 RC**＝不知道（`None`），不擋
#: * present 有、health 沒有 → RC 掉線（`False`）
#: * 兩者都有 → RC 在線（`True`）
#:
#: **為什麼用這裡而不是 `RC_CHANNELS.rssi`**：rssi 沒有「不知道」這一態——
#: 255 在 MAVLink 是「無效值」而不是「滿格」，讀成滿格的後果是 RC 掉線時守門
#: 照樣放行起飛，而那正是這條規則要防的事。present 位元把「沒有回報」與
#: 「回報說沒有」分得開，這個分別是整條規則的地基。
#:
#: **真機覆核未做**：ArduPilot 4.7 在 RC failsafe 期間是否清掉 health 位元、
#: 以及 present 位元在沒有接收機時是否為 0，都要在真機上看過才算數。
#: 覆核前這個欄位仍會照送——送 `None` 不擋，送錯才擋，而目前的風險是前者。
SENSOR_RC_RECEIVER = 1 << 16


class Link:
    """一條往地面站的 UDP 通道。"""

    def __init__(self, name, host, port):
        self.name, self.addr = name, (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.tx = self.rx = self.tx_err = 0

    def send(self, data):
        try:
            self.sock.sendto(data, self.addr)
            self.tx += 1
            return True
        except OSError as e:
            # 網路瞬斷是常態不是異常：記錄後繼續，鏈路回來就自然復原。
            # 這裡**不能讓例外上拋**——這個迴圈死掉等於飛控與地面站同時失聯。
            self.tx_err += 1
            if self.tx_err % 100 == 1:
                log.warning("%s 送出失敗（%s，第 %d 次）", self.name, e, self.tx_err)
            return False

    def recv(self):
        out = []
        while True:
            try:
                data, _ = self.sock.recvfrom(4096)
            except (BlockingIOError, OSError):
                break
            if not data:
                break
            self.rx += 1
            out.append(data)
        return out


class Monitor:
    """本地智能層：從遙測流裡導出狀態，狀態變化就記一筆事件。

    這裡刻意只做「本地看得出來、且值得留痕」的判斷，不做飛行決策——
    飛安決策留給飛控自己的 failsafe 與地面站，代理只負責看見與轉述。
    """

    def __init__(self):
        self.state = {}
        self.last_hb = 0.0
        self.counts = defaultdict(int)
        self.events = []
        #: PREARM_CHECK 位元見過沒有（026 B4-b）。**黏性**：見過一次就信
        #: health 位元；從未見過＝韌體不回報，就緒判定退回次級訊號
        self._prearm_seen = False
        #: STATUSTEXT 分段重組用（見 `_stx_reassemble`）
        self._stx_buf = {}

    #: STATUSTEXT `text` 欄位長度：滿 50＝還有下一段
    STX_CHUNK_LEN = 50
    #: 末段掉包時殘段逾時丟棄（秒）——**寧可整句丟掉也不吐半句**，
    #: 半句預檢原因比沒有原因更糟：它看起來像完整答案
    STX_STALE_S = 3.0

    def _stx_reassemble(self, msg):
        """回完整整句；分段未收完回 None。

        MAVLink2 的 STATUSTEXT 把長訊息切成 50 字一段（同 `id`、`chunk_seq`
        遞增）。**不重組就會把 50 字以上的預檢原因記成截斷的前半句**，而後
        半段因為不再以 `PreArm:` 開頭會被整段丟掉——正好是最需要解釋的那幾
        句最長。與地面站 `mavlink_rx._stx_reassemble` 同一套規則。
        """
        now = time.monotonic()
        for k in [k for k, v in self._stx_buf.items()
                  if now - v["t"] > self.STX_STALE_S]:
            del self._stx_buf[k]
        cid = getattr(msg, "id", 0) or 0
        raw = msg.text or ""
        if cid == 0:                      # 未分段（≤50 字，或舊韌體不切段）
            return raw
        seq = getattr(msg, "chunk_seq", 0) or 0
        slot = self._stx_buf.setdefault(cid, {"parts": {}, "t": now})
        slot["parts"][seq] = raw
        slot["t"] = now
        if len(raw) < self.STX_CHUNK_LEN:  # 末段（pymavlink 去尾 NUL 後 <50）
            parts = self._stx_buf.pop(cid)["parts"]
            return "".join(parts[k] for k in sorted(parts))
        return None

    def feed(self, msg):
        t = msg.get_type()
        self.counts[t] += 1
        now = time.time()
        s = self.state

        if t == "HEARTBEAT" and msg.get_srcSystem() != 255:
            self.last_hb = now
            armed = bool(msg.base_mode & 0x80)
            mode = COPTER_MODE.get(msg.custom_mode, msg.custom_mode)
            self._change("armed", armed, "⚠ 已解鎖" if armed else "已上鎖")
            self._change("mode", mode, f"飛行模式 → {mode}")
            s["sysid"] = msg.get_srcSystem()
            s["system_status"] = msg.system_status
            # 026 B4-b：驅動要的是**原始 custom_mode**，不是我們譯過的名字。
            # 譯過的那份是 Copter 專用表（`COPTER_MODE`），而驅動同時要服務
            # PX4——把譯好的字串再餵回去等於用一家的表去解另一家的數字
            s["custom_mode"] = msg.custom_mode
            s["mav_state"] = normalize.MAV_STATE.get(msg.system_status)
            s["autopilot_raw"] = msg.autopilot
            s["vehicle_type"] = msg.type
        elif t == "SYS_STATUS":
            v = msg.voltage_battery
            s["voltage"] = None if v == 0xFFFF else round(v / 1000, 3)
            s["battery_pct"] = None if msg.battery_remaining == -1 \
                else msg.battery_remaining
            s["cpu_load_pct"] = round(msg.load / 10, 1)
            s["sensors_health"] = msg.onboard_control_sensors_health
            s["sensors_present"] = msg.onboard_control_sensors_present
            s["sensors_enabled"] = msg.onboard_control_sensors_enabled
            # 026 B4-b：就緒判定的兩個輸入。**present 且 enabled 且不 health**
            # 才算異常——沒裝的感測器不健康是廢話，關掉的也是
            pres, enab, heal = (msg.onboard_control_sensors_present,
                                msg.onboard_control_sensors_enabled,
                                msg.onboard_control_sensors_health)
            s["sensors_unhealthy"] = normalize.sensors_unhealthy(pres, enab, heal)
            s["prearm_ok"], self._prearm_seen = normalize.prearm(
                pres, enab, heal, self._prearm_seen)
            # RC 遙控器在不在（039 複裁 A）。**沒回報就是 None**：把「不知道」
            # 當成「沒有 RC」會讓所有還沒驗證過這個欄位的機都起飛不了
            s["_rc_sys_status"] = (
                bool(msg.onboard_control_sensors_health & SENSOR_RC_RECEIVER)
                if msg.onboard_control_sensors_present & SENSOR_RC_RECEIVER
                else None)
            rc = self._derive_rc(s)
            # **不走 `_change`**：它把第一次讀到的值當成「還沒讀到」而不留痕，
            # 而這裡的 `None` 是一個真正的值（不知道）。用 `_change` 的話
            # 「不知道 → RC 掉線」這個轉換會安靜地過去——那正是最該留痕的一次
            self._note_rc(s, rc)
            if s["battery_pct"] is not None:
                for th in (20, 10, 5):
                    if s["battery_pct"] <= th:
                        self._change(f"batt_below_{th}", True,
                                     f"⚠ 電量低於 {th}%（{s['battery_pct']}%）")
                        break
        elif t == "RC_CHANNELS":
            # **第二個訊號來源**（uav-system 2026-09-02 實測後新增）：
            # ArduPilot 4.7 **不設** SYS_STATUS 的 RC_RECEIVER present 位元，
            # 但它有送這則（實測 4 Hz）。`chancount` 的規格定義是「正在接收的
            # RC 通道總數；**沒有可用的 RC 通道時應為 0**」——**計數不是哨兵**，
            # 所以 0 是真值，三態因此成立。
            #
            # 同一則訊息的 `rssi` 實測是 **255**＝無效，不是滿格。
            # **一則訊息裡一個欄位可用、一個不可用**——這就是當初否決 rssi
            # 的判斷為什麼是對的，而它現在有了實測佐證。
            s["_rc_chancount"] = getattr(msg, "chancount", None)
            s["_rc_chan_t"] = time.monotonic()
            self._note_rc(s, self._derive_rc(s))
        elif t == "GPS_RAW_INT":
            s["gps_fix"] = GPS_FIX.get(msg.fix_type, msg.fix_type)
            # 就緒判定比的是數字（`fix < 3`），不是名字。存原始值而不是回頭
            # 反查譯名——反查表會與正查表漂移，而漂移的症狀是靜默的錯判
            s["gps_fix_raw"] = msg.fix_type
            s["gps_sats"] = msg.satellites_visible
            self._change("gps_fix_state", s["gps_fix"],
                         f"GPS 定位 → {s['gps_fix']}（{s['gps_sats']} 顆）")
        elif t == "GLOBAL_POSITION_INT":
            s["lat"] = round(msg.lat / 1e7, 7)
            s["lon"] = round(msg.lon / 1e7, 7)
            s["alt_rel_m"] = round(msg.relative_alt / 1000, 2)
        elif t == "VFR_HUD":
            s["groundspeed_ms"] = round(msg.groundspeed, 2)
            # 航向給改航線提案用：「續飛航點在你後方 N 度，機體會先掉頭」
            # 沒有它那句警告永遠不會出現——**而不是不會發生**
            s["heading"] = round(msg.heading % 360, 1)
        elif t == "EXTENDED_SYS_STATE":
            # landed_state 是狀態機分辨「在地上／爬升中／降落中」的第一手依據。
            # UNDEFINED（0）＝**這台韌體沒回報**，不是「不在地上」——那時才退回
            # 用高度與地速猜，而且要讓人知道用的是猜的（見 Agent.flight_state）
            s["landed_state"] = LANDED.get(msg.landed_state)
        elif t in normalize.EKF_MSG_TYPES:
            # 訊息層方言：PX4 發 ESTIMATOR_STATUS、ArduPilot 發 EKF_STATUS_REPORT
            # ——**同一件事兩個訊息名**，所需位元在 bit 1..512 同義。型別清單是
            # 從驅動的 MESSAGE_ADJUSTMENTS 推導的，不是手寫（見 normalize.py）
            s["ekf_ok"] = normalize.ekf_ready(msg.flags)
        elif t == "MISSION_CURRENT":
            # 失聯處置的「跑完再 RTL」要知道任務**什麼時候算跑完**。
            # mission_state 是後來才加進 MAVLink 的欄位：舊韌體不填（0＝UNKNOWN），
            # 那時我們就宣告「測不到完成」，改由單飛時間上限收尾——
            # **不要用 seq 到底了來猜**，最後一個航點停留多久是任務自己的事。
            s["mission_seq"] = msg.seq
            s["mission_total"] = getattr(msg, "total", 0) or 0
            s["mission_state"] = getattr(msg, "mission_state", 0) or 0
        elif t == "PARAM_VALUE":
            # **值的解讀是廠牌方言**：PX4 把整數參數的位元組原樣塞進 float32，
            # ArduPilot 寫數值本身——用驅動解，不要自己判斷（026 B4-b）
            name = msg.param_id
            if isinstance(name, (bytes, bytearray)):
                name = name.decode("ascii", "replace")
            name = name.rstrip("\x00").strip()
            drv = get_driver(s.get("autopilot_raw"))
            s.setdefault("params", {})[name] = drv.decode_param(
                msg.param_value, getattr(msg, "param_type", None))
        elif t == "AUTOPILOT_VERSION":
            # 板子身分（uav-system issues/038）。**代理原本只轉發不解**——
            # 但註冊要以 board_uid 為鍵，所以自己也得看得懂。
            uid2 = bytes(getattr(msg, "uid2", b"") or b"").rstrip(b"\x00")
            s["board_uid"] = uid2.hex() if uid2 else (
                f"{msg.uid:016x}" if getattr(msg, "uid", 0) else None)
            v = getattr(msg, "flight_sw_version", 0) or 0
            if v:
                kind = {255: "official", 128: "rc", 64: "beta",
                        192: "dev"}.get(v & 0xFF)
                s["fw"] = (f"{(v >> 24) & 0xFF}.{(v >> 16) & 0xFF}."
                           f"{(v >> 8) & 0xFF}"
                           + (f" ({kind})" if kind else f" (type {v & 0xFF})"))
        elif t == "STATUSTEXT":
            whole = self._stx_reassemble(msg)
            if whole is None:             # 還在收分段
                return
            txt = whole.strip()
            if txt:
                self.events.append((now, "statustext",
                                    f"[sev{msg.severity}] {txt}"))
                log.info("飛控訊息 [sev%d] %s", msg.severity, txt)
                # **預檢失敗的原因，飛控本來就在講**——記下來，就緒判定才
                # 說得出是哪一項。`prearm_ok is False` 只說得出「有一項沒過」。
                #
                # `PreArm:` 是被擋在解鎖之前，`Arm:` 是解鎖當下被拒——兩者都是
                # 同一個問題的答案：「為什麼現在不能飛」。
                #
                # **與地面站 `mavlink_rx._statustext` 逐字相同的收集規則**：
                # 兩份實作的輸出必須一樣（026 B4-d，`tools/driver-parity.py`）
                for pfx in ("PreArm:", "Arm:"):
                    if txt.startswith(pfx):
                        s.setdefault("prearm_msgs", {})[
                            txt[len(pfx):].strip()] = time.monotonic()
                        break

    #: `RC_CHANNELS` 多久沒來就不再拿它下判斷。**「我們不再聽到」不等於
    #: 「RC 在」**——留著一個過期的計數，等於用舊事實回答關於現在的問題。
    #: 實測真機 4 Hz，5 秒＝漏 20 則才轉成「不知道」
    RC_STALE_S = 5.0

    def _derive_rc(self, s):
        """RC 在不在。**三態**：True／False／None＝不知道。兩個來源分層：

        1. `SYS_STATUS` 的 `RC_RECEIVER` 位元（PX4 有設，權威）
        2. `RC_CHANNELS.chancount`（ArduPilot 4.7 走這條）

        都沒有 → `None`，而 **`None` 不擋**（039 複裁 A）：把「不知道」當成
        「沒有 RC」會讓所有還沒回報的機都起飛不了。
        """
        v = s.get("_rc_sys_status")
        if v is not None:
            s["rc_source"] = "sys_status"
            return v
        n, t0 = s.get("_rc_chancount"), s.get("_rc_chan_t")
        if n is not None and t0 is not None and time.monotonic() - t0 <= self.RC_STALE_S:
            s["rc_source"] = "rc_channels"
            return n > 0
        s["rc_source"] = None
        return None

    def _note_rc(self, s, rc):
        """轉態留痕。**不走 `_change`**：它把第一次讀到的值當成「還沒讀到」
        而不留痕，而這裡的 `None` 是一個真正的值（不知道）——用 `_change`
        的話「不知道 → RC 掉線」會安靜地過去，那正是最該留痕的一次。"""
        prev = s.get("rc_link", "unset")
        s["rc_link"] = rc
        if prev != "unset" and prev != rc:
            txt = ("遙控器已連線" if rc else
                   "⚠ 遙控器離線——此時不得起飛／開始任務" if rc is False
                   else "遙控器狀態不明（收不到判定所需的訊息）")
            self.events.append((time.time(), "rc_link", txt))
            log.info("狀態變化：%s（來源 %s）", txt, s.get("rc_source"))

    def _change(self, key, val, message):
        if self.state.get(key) != val:
            prev = self.state.get(key)
            self.state[key] = val
            if prev is not None:            # 第一次讀到不算「變化」
                self.events.append((time.time(), key, message))
                log.info("狀態變化：%s", message)


class Agent:
    def __init__(self, a):
        self.a = a
        self.mon = Monitor()
        self.telem = Link("telemetry(14540)", a.gs, a.telemetry_port)
        self.cmd = Link("command(14541)", a.gs, a.command_port)
        self.serial = None
        self.last_stream_req = 0.0
        self.streams_requested = False
        self.caps_requested = False
        # 飛控 GCS failsafe 的開機自檢（uav-system 039，2026-09-01 裁定「開」）。
        # **一次就好，不週期重讀**——參數不會自己變，會變是因為有人去改，
        # 那種情況下重啟一次代理不算苛求。這正是地面站拔掉 SYSID_MYGCS 探測
        # 的理由之一（每 30 秒對每台 ArduPilot 機重讀）
        self.fs_gcs_asked = 0            # 問過幾次
        self.fs_gcs_asked_at = 0.0       # 上次開口的時間
        self.fs_gcs_said = False         # 結論說過了沒有
        self.hello_sent = False
        self.drone_id = None
        self.intent = None            # 意圖通道（拿到 board_uid 才起，見 say_hello）
        #: 配號（040）。`assigned_sysid` 是地面站的決定，`sysid_action` 是
        #: keep／change。`reassigning` 非 None 時代表正在換號並重開飛控——
        #: 它會被送上去，讓地面站不要把飛控消失當成失聯（設計 §3.3）
        self.assigned_sysid = None
        self.sysid_action = None
        self.reassigning = None
        self.reassign_tried = False
        self.backfill = None          # 斷線期間的取樣緩衝（C 層）
        #: intent_id → 送出去給人看的提案。**留在機上**：確認的時候不必讓
        #: 地面站把提案送回來，也就沒有「送回來的那份跟人看到的不一樣」的空間
        self.proposals = {}
        #: 改航線序列進行中（地面站在執行）。失聯處置要看它——§7 規定
        #: 「改航線序列中 → 立刻 RTL」，因為序列跑到一半失聯＝飛機停在一個
        #: 沒有人確認過的中間狀態
        self.seq = None
        # 模式溯源（失聯處置選項 C）：LOITER 既可能是地面站按的「暫停」，
        # 也可能是飛手用遙控器切過去手飛的。兩者的處置完全相反——前者要 RTL，
        # 後者**永不介入**。分辨的唯一辦法是記住是誰要求的。
        self._solo_logged = False     # 續飛決定只播報一次，不是每拍都喊
        self.mode_req = None          # (模式名, 要求時間)：我方最近一次的模式指令
        self.mode_owner = None        # "us" / "pilot"：當下這個模式是誰造成的
        self._seen_mode = None        # 上一拍觀察到的模式，用來偵測變化
        self.last_telem_rx = 0.0
        self.last_report = 0.0
        self.fwd_to_fc = 0
        # 地面站方向的解析器。**它當初存在的理由（認出搖桿封包好改寫身分）
        # 已經隨 issues/035 消失，但這段程式沒有跟著死**——它現在做兩件仍然
        # 需要的事：① 解析＝驗 CRC，擋掉壞封包不讓它進飛控的解析器；
        # ② `_note_mode_request()` 的模式溯源，也就是「這個 LOITER 是地面站
        # 按的暫停還是飛手切的手飛」——失聯處置選項 C 最關鍵的一格靠它。
        # 清死碼時要分「為什麼存在」與「做了什麼」：照理由刪會刪掉還在保護
        # 東西的檢查。
        # robust_parsing＝壞封包不拋例外，只跳過（這條迴圈不准死）。
        self.gs_parser = apm.MAVLink(None)
        self.gs_parser.robust_parsing = True
        # 飛控方向也自己解析，**不用 mavutil 的 recv_match**：那個高階介面會做
        # per-sysid 的訊息記帳（`sysid_state[...].messages`），我們一項都沒用到，
        # 而它在 instance 訊息（BATTERY_STATUS 等）上會踩到 pymavlink 自己的
        # `_instances is None` → TypeError（2026-08-24 實測，每次都吃掉一次讀取
        # 並噴多行 traceback）。直接解析原始流：CRC 一樣有驗，少一層不需要的狀態。
        self.fc_parser = apm.MAVLink(None)
        self.fc_parser.robust_parsing = True
        self.last_gs_rx = 0.0
        self.gs_link_ok = None
        self.loss_warned = False
        self.loss_acted = False
        self.loss_started = None
        #: 接管中的話，記著從什麼時候開始、失聯當下的狀態、以及期間做過什麼。
        #: **不是一個布林值**：恢復時要交代得出來（見 `_check_gs_link`）
        self.takeover = None
        self.modem = None
        self.recorder = None      # 機上原始層錄製（014）
        self.uploader = None      # 那份錄製的自動回傳（014）

    def connect_serial(self):
        while True:
            try:
                self.serial = mavutil.mavlink_connection(
                    self.a.dev, baud=self.a.baud, source_system=255,
                    source_component=190, dialect="ardupilotmega")
                log.info("已開啟 %s @ %d", self.a.dev, self.a.baud)
                return
            except Exception as e:
                # 飛控可能比 Pi 晚上電，或 UART overlay 還沒掛好
                log.warning("開序列埠失敗（%s），3 秒後重試", e)
                time.sleep(3)

    def request_board_id(self):
        """問一次飛控板身分（AUTOPILOT_VERSION → uid2）。見 uav-system issues/038。

        **不能塞在 request_streams() 裡**（2026-08-24 實測踩到）：那個函式只在
        「心跳還在但遙測停了」時才被呼叫，而 ArduPilot 的串流速率是持久化的
        `SRx_` 參數——代理重啟後遙測立刻就有，那條路徑從頭到尾不會執行。
        結果就是板子身分只有在遙測壞掉時才會被問到。

        **要開口問才有**：ArduPilot 不主動送（實測這台機的 28 種訊息裡沒有它）。
        由代理做而不是地面站：請求要送 COMMAND_LONG，那是個通用信封（同一型別
        可以裝 arm），而 backend 的 read-only 邊界不收它。回應隨遙測一起轉發
        回去，地面站照收即可。

        一次性，不設串流間隔——板子身分不會變，訂成串流只是浪費 57600。
        """
        tgt = self.mon.state.get("sysid")
        if tgt is None or self.caps_requested:
            return
        self.caps_requested = True
        try:
            # 送三次：57600 上單包丟失是真實風險，而這是**一次性**請求，
            # 丟了就再也不會問（不像串流有下一輪）。重複請求無副作用。
            for _ in range(3):
                self.serial.mav.command_long_send(
                    tgt, 1,
                    mavutil.mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES,
                    0, 1, 0, 0, 0, 0, 0, 0)
                time.sleep(0.1)
            log.info("已請求板子身分（AUTOPILOT_VERSION，目標 sysid=%d）", tgt)
        except Exception as e:
            log.warning("請求板子身分失敗：%s", e)
            self.caps_requested = False      # 失敗就讓下一拍再試

    #: 沒回應就再問一次的間隔，以及徹底放棄的時限。**不是週期重讀**：
    #: 拿到答案或用完次數之後就永遠停手
    FS_GCS_RETRY_S = 10.0
    FS_GCS_GIVEUP_S = 25.0

    def _fs_gcs_names(self):
        """這台機的失聯 failsafe 參數叫什麼名字——**向驅動要，不要寫死**。

        同一件事兩家的名字與語意都不同：ArduPilot 是
        `FS_GCS_ENABLE`（開關）／`FS_GCS_TIMEOUT`，PX4 是
        `NAV_DLL_ACT`（**行為列舉，不是開關**）／`COM_DL_LOSS_T`。所以
        「enable」在 PX4 上的意思是「動作不等於 0」。

        未知廠牌回空表＝**沒有意見**。拿別家的參數名去問，問了只會石沉大海，
        然後被誤讀成「讀不到＝沒設定」。

        （這份名單一度寫死在本檔，因為當時把它加進驅動會破壞
        `autopilot/` 與上游的逐字相同。上游 `c585825` 加了 `gcs_failsafe_params()`
        之後就改成向驅動要——寫死的那份也就跟著刪掉，不留第二個會漂的地方。）
        """
        return get_driver(self.mon.state.get("autopilot_raw")
                          ).gcs_failsafe_params() or {}

    def _batt_names(self):
        """這台機的低電量門檻參數叫什麼——**向驅動要，不要寫死**（同 `_fs_gcs_names`）。"""
        return get_driver(self.mon.state.get("autopilot_raw")
                          ).battery_low_params() or {}

    def _battery_verdict(self):
        """接管期間：該不該因為電量回家。回 `(要回家嗎, 說得出口的理由)`。

        **兩個判準任一觸發就回家；兩個都讀不到也回家**（2026-09-04 使用者裁定）。

        為什麼要兩個：`battery_pct` 來自飛控的電流積分，而那個積分的刻度
        （`BATT_AMP_PERVLT`）在本專案**沒有人驗過**——2026-09-02 實測畫面顯示
        58%，而那顆電池的電壓已經接近見底。**只看百分比會在錯的時候放行。**
        電壓比較誠實（雖然負載下會下垂），所以兩個並聯：任一說不行就不行。

        **電壓門檻向飛控要**（`BATT_LOW_VOLT`），不自己編：編出來的數字與
        飛控自己的設定不一致時，兩層會在不同時間點動作，而沒有人說得出為什麼。
        讀不到就只剩百分比那條——而那要說出來。
        """
        s = self.mon.state
        pct, volt = s.get("battery_pct"), s.get("voltage")
        params = s.get("params") or {}
        low_v = params.get((self._batt_names() or {}).get("low_volt", ""))
        hits, known = [], []
        if pct is not None:
            known.append(f"電量 {pct}%")
            if pct < self.a.link_loss_min_batt:
                hits.append(f"電量 {pct}% 低於 {self.a.link_loss_min_batt}%")
        if volt is not None and low_v:
            known.append(f"電壓 {volt:.2f}V")
            if volt <= low_v:
                hits.append(f"電壓 {volt:.2f}V 已到飛控的低電量門檻 {low_v:.2f}V")
        if not known:
            # **不知道就回家。** 這與 `rc_link` 的 None 方向相反，而方向是由
            # 「不知道時哪一邊會出事」決定的：不知道還剩多少電就繼續飛，
            # 出事的是飛機（039 複裁 E 的同一條，這裡延伸到整個接管期間）
            return True, ("電量與電壓都讀不到"
                          + ("" if low_v else "（也讀不到飛控的電壓門檻）"))
        if hits:
            return True, "；".join(hits)
        return False, "、".join(known) + " 都還在門檻上"

    def request_fs_gcs(self, now):
        """開機時問一次飛控的失聯 failsafe 設定（039，2026-09-01 裁定）。

        **為什麼現在值得問，而 2026-08-31 不值得**：那時候讀了也不會據此做
        任何事，只為了印一行話對 57600 多送一輪請求不划算。現在有裁定值，
        這個檢查會據以做事——不變式不成立就大聲說。
        """
        s = self.mon.state
        tgt = s.get("sysid")
        if tgt is None or self.fs_gcs_said:
            return
        # 電池門檻一起要——**接管期間要用它判斷該不該回家**，而代理不該
        # 自己編一個電壓數字（見驅動的 `battery_low_params` 說明）
        names = list(self._fs_gcs_names().values()) + list(self._batt_names().values())
        if not names:
            return                               # 驅動沒有意見，不要製造意見
        got = s.get("params") or {}
        if all(k in got for k in names):
            return                               # 都到了，不必再問
        if self.fs_gcs_asked >= 2:
            return                               # 問過兩輪就不再開口
        if self.fs_gcs_asked and now - self.fs_gcs_asked_at < self.FS_GCS_RETRY_S:
            return
        self.fs_gcs_asked += 1
        self.fs_gcs_asked_at = now
        try:
            for name in names:
                if name in got:
                    continue
                # 送三次：57600 上單包丟失是真實風險，而這是一次性請求
                for _ in range(3):
                    self.serial.mav.param_request_read_send(
                        tgt, 1, name.encode("ascii"), -1)
                    time.sleep(0.05)
            log.info("已請求飛控失聯 failsafe 設定（第 %d 次）：%s",
                     self.fs_gcs_asked, "、".join(names))
        except Exception as e:
            log.warning("請求 GCS failsafe 參數失敗：%s", e)
            self.fs_gcs_asked -= 1               # 沒送出去就不算問過

    def _check_fs_gcs(self, now):
        """自檢並**只說一次**：飛控那層會不會搶在代理前面動作。

        不變式（uav-system 039，2026-09-01 使用者裁定）：

            FS_GCS_TIMEOUT  >  LINK_LOSS_MAX_SOLO_S  >  LINK_ACTION_S

        飛控那層的角色是**代理的後備**——只在代理沒有動手時才出手。逾時若
        小於單飛上限，飛控會在續飛的中途搶先，而使用者裁定的「失聯後跑完
        任務再 RTL」那一格**就永遠不會發生**。這是選項 C 的**第三條**安靜
        退化途徑（另兩條：兩個門檻撞在一起、以及這條的前身），理由與那兩條
        相同：安靜地退化成另一種策略，比策略本身選錯更難發現。

        **「讀不到」與「讀到了但不合」分開講**：前者是「不知道飛控怎麼設定」，
        後者是「知道它設錯了」——處置完全不同，混成一句話等於沒說。
        """
        if self.fs_gcs_said:
            return
        s = self.mon.state
        if s.get("sysid") is None:
            return
        names = self._fs_gcs_names()
        if not names:
            if s.get("autopilot_raw") is None:
                return                            # 還沒收到心跳，廠牌未知
            self.fs_gcs_said = True
            log.info("失聯 failsafe 自檢跳過：驅動對這個廠牌"
                     "（autopilot=%s）沒有意見——**沒有意見時不要製造意見**，"
                     "拿別家的參數名去問只會被誤讀成「讀不到＝沒設定」",
                     s.get("autopilot_raw"))
            return
        n_en, n_to = names.get("enable"), names.get("timeout_s")
        got = s.get("params") or {}
        missing = [k for k in (n_en, n_to) if k and k not in got]
        if missing:
            if not self.fs_gcs_asked_at or now - self.fs_gcs_asked_at < self.FS_GCS_GIVEUP_S:
                return                            # 還在等
            self.fs_gcs_said = True
            # **讀不到 ≠ 設錯了。** 三個可能：這個韌體沒有這個參數、我們的
            # 參數名寫錯（PX4 那組上游標明未在真機覆核過）、或請求／回應在
            # 57600 上掉了。**三者我們分不出來**，所以不要假裝分得出來
            log.warning("⚠ **讀不到飛控的失聯 failsafe 設定**（缺 %s，已問 %d 次）"
                        "——不是「設錯了」而是「不知道它怎麼設」。可能這個韌體"
                        "沒有這個參數、可能我們的參數名不對、也可能請求或回應"
                        "掉了。無法確認 %s > %.0fs 這條不變式是否成立",
                        "、".join(missing), self.fs_gcs_asked,
                        n_to, self.a.link_loss_max_solo_s)
            self.mon.events.append(
                (now, "fs_gcs", f"讀不到失聯 failsafe 設定（缺 {'、'.join(missing)}）"))
            return

        self.fs_gcs_said = True
        enable, timeout = got[n_en], got[n_to]
        solo, act = self.a.link_loss_max_solo_s, self.a.link_action_s
        problems = []
        if not enable:
            # 裁定是「開」。關著的話飛控那層後備不存在——代理掛掉就沒有任何人。
            # **「開」的判準是「不等於 0」而不是「等於 1」**：ArduPilot 的
            # FS_GCS_ENABLE 是開關，PX4 的 NAV_DLL_ACT 是行為列舉，
            # 兩家共通的只有「0＝不動作」
            problems.append(
                f"{n_en}={enable:g}＝**不動作**：飛控那層後備不存在，"
                "代理自己掛掉時沒有任何人接手")
        if timeout <= solo:
            problems.append(
                f"{n_to}={timeout:g}s **不大於**單飛上限 {solo:.0f}s："
                "飛控會在續飛中途搶先動作，「失聯後跑完任務再 RTL」那一格"
                "永遠不會發生（選項 C 退化）")
        if solo <= act:
            problems.append(
                f"單飛上限 {solo:.0f}s 不大於處置起算 {act:.0f}s：選項 C 退化成 A")
        if problems:
            log.error("⚠⚠ **失聯 failsafe 不變式不成立**"
                      "（要求 %s > %.0fs > %.0fs）：%s",
                      n_to, solo, act, "；".join(problems))
            self.mon.events.append(
                (now, "fs_gcs", "⚠ 失聯 failsafe 設定與失聯處置衝突："
                                + "；".join(problems)))
            return
        log.info("失聯 failsafe 自檢通過：%s=%g、%s=%gs > 單飛上限 %.0fs "
                 "> 處置起算 %.0fs——飛控那層是後備，只在代理沒動手時才出手",
                 n_en, enable, n_to, timeout, solo, act)

    def request_streams_once(self):
        """看到飛控就把串流請求送一次。

        **原本只在「心跳在但遙測停了」時才送**，於是代理重啟後從來不重問——
        飛控還照著**上一次**被要求的清單在送。2026-08-25 實際踩到：WANTED 加了
        EXTENDED_SYS_STATE（245），代理重啟後 landed_state 永遠是 None，看起來
        像「這台韌體不支援」，其實是我們根本沒開口要。

        與 request_board_id 同一個教訓：**「只在出問題時才做」的初始化，在
        沒出問題的那條路上就等於沒做。**
        """
        if self.streams_requested or not self.mon.state.get("sysid"):
            return
        self.streams_requested = True
        self.request_streams()

    def request_streams(self):
        """請求／重新請求遙測串流。舊式與新式兩條路都走。"""
        tgt = self.mon.state.get("sysid", 1)
        try:
            self.serial.mav.request_data_stream_send(
                tgt, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, self.a.rate, 1)
            for msgid in WANTED:
                self.serial.mav.command_long_send(
                    tgt, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                    msgid, int(1e6 / self.a.rate), 0, 0, 0, 0, 0)
                time.sleep(0.02)
            log.info("已請求串流 %d Hz（目標 sysid=%d）", self.a.rate, tgt)
        except Exception as e:
            log.warning("請求串流失敗：%s", e)
        self.last_stream_req = time.time()

    def run(self):
        self.connect_serial()
        if self.a.modem_port:
            self.modem = ModemSampler(
                self.a.modem_port, self.a.gs, self.a.backend_port,
                self._position, self.a.modem_interval,
                # 記錄通道要說得出這批樣本是誰的。**多機時不能靠「主機」**
                # ——那是排版設定不是身分（issues/040 A4）
                drone_id_fn=lambda: self.drone_id,
                # 2026-09-07 使用者裁定：**停在地面上就不要記**。
                # 即時通道照送（起飛前要看得到訊號好不好），停的是入庫那條
                airborne_fn=self.airborne)
            self.modem.start()
        # **斷線期間的資料就在這台 Pi 上，只是送不出去。** 緩衝起來，
        # 鏈路恢復後補傳（uav-system 的 C 層）
        # 機上原始層：飛控送出的每一個框架都落盤。**零額外頻寬**——這些
        # 位元組本來就要讀（轉發要用）。5G 斷線期間地面站什麼都收不到，
        # 而這裡照樣看得見（issues/014）
        # **回傳器先建**：錄製器滾動刪檔前要問它「這一份回傳了沒」，
        # 而回傳器只需要一個 lambda 去讀錄製器當下在寫哪一個檔
        if self.a.upload:
            self.uploader = uploader.Uploader(
                self.a.record_dir, self.a.gs, self.a.backend_port,
                lambda: self.mon.state.get("board_uid"),
                lambda: self.recorder.name if self.recorder else None,
                self._upload_gate,
                rate_kbs=self.a.upload_kbps, chunk_kb=self.a.upload_chunk_kb,
                settle_s=self.a.upload_settle_s)
        self.recorder = recorder.Recorder(
            self.a.record_dir, max_mb=self.a.record_max_mb,
            keep=self.a.record_keep, armed_only=not self.a.record_disarmed,
            is_uploaded=self.uploader.is_uploaded if self.uploader else None,
            on_discard=self.uploader.note_pruned if self.uploader else None)
        if self.uploader:
            self.uploader.start()
        self.backfill = Backfill(
            self.a.gs, self.a.backend_port,
            lambda: self.mon.state.get("board_uid"),
            lambda: self.mon.state,
            lambda: bool(self.gs_link_ok))
        self.backfill.start()
        fd = self.serial.port.fileno()
        log.info("轉發目標：%s:%d（遙測）／%s:%d（指令）",
                 self.a.gs, self.a.telemetry_port, self.a.gs, self.a.command_port)

        while True:
            try:
                self.tick(fd)
            except Exception:
                # 這個迴圈不准死：它死了＝飛控與地面站同時失聯，而且外面看不出來
                log.exception("主迴圈例外，吞掉續跑")
                time.sleep(0.2)

    def tick(self, fd):
        r, _, _ = select.select([fd, self.telem.sock, self.cmd.sock], [], [], 0.2)
        now = time.time()

        # ── 飛控 → 地面站 ──────────────────────────────────
        if fd in r:
            port = self.serial.port
            n = getattr(port, "in_waiting", 0)
            data = port.read(n) if n else b""
            for msg in (self.fc_parser.parse_buffer(data) or []) if data else []:
                if msg.get_type() == "BAD_DATA":
                    continue
                buf = msg.get_msgbuf()
                self.telem.send(buf)          # 唯讀通道
                self.cmd.send(buf)            # 指令通道也要看得到狀態與 ACK
                self.mon.feed(msg)
                # **餵在 mon.feed 之後**：armed 要用這一拍更新過的值，
                # 否則解鎖的第一批框架會被判成「還沒 armed」而漏錄
                self.recorder.feed(buf, bool(self.mon.state.get("armed")))
                if msg.get_type() != "HEARTBEAT":
                    self.last_telem_rx = now

        # ── 地面站 → 飛控 ──────────────────────────────────
        for link in (self.telem, self.cmd):
            for data in link.recv():
                self.last_gs_rx = now
                self.to_fc(fd, data)

        # 看到飛控（心跳帶出 sysid）就問一次板子身分。放在這裡而不是
        # request_streams()——理由見 request_board_id 的說明。
        self.request_board_id()
        self.request_fs_gcs(now)
        self._check_fs_gcs(now)
        self.request_streams_once()
        self.say_hello()
        self._maybe_reassign(now)
        self._track_mode_owner()
        self._expire_seq(now)
        self._publish_state()

        self._check_gs_link(now)

        # ── 串流保活 ────────────────────────────────────────
        # 心跳還在但遙測停了＝飛控把串流速率歸零了（重開機／重連常見）。
        # 這裡重送請求，而不是只在啟動時送一次。
        hb_ok = (now - self.mon.last_hb) < 5.0
        telem_stale = (now - self.last_telem_rx) > 5.0
        if hb_ok and telem_stale and (now - self.last_stream_req) > 10.0:
            log.warning("心跳在但遙測停了 %.0f 秒，重新請求串流",
                        now - self.last_telem_rx)
            self.request_streams()

        # ── 定期狀態列 ──────────────────────────────────────
        if now - self.last_report >= self.a.report_interval:
            self.last_report = now
            self.report()

    def to_fc(self, fd, data):
        """地面站來的封包 → 飛控。

        **仍然先解析再轉發，而不是直接把 datagram 倒進序列埠**：解析等於驗 CRC，
        擋掉壞封包不讓它進飛控的解析器；解不出來時才整包原封送（寧可送出去讓
        飛控自己判斷，也不要自作主張丟掉可能合法的指令）。
        """
        try:
            msgs = self.gs_parser.parse_buffer(data) or []
        except Exception:
            msgs = []
        if not msgs:
            self._write(fd, data)
            return
        for m in msgs:
            if m.get_type() != "BAD_DATA":
                self._note_mode_request(m)
                self._write(fd, m.get_msgbuf())

    def _note_mode_request(self, m):
        """記下**我方**（地面站或代理）要求過的飛行模式。

        地面站的指令本來就經過這裡，所以不必另外開通道問它「剛才按了什麼」——
        我們是那條路徑上的必經之地，順手看一眼就有了。
        """
        t = m.get_type()
        code = None
        if t == "SET_MODE":
            code = m.custom_mode
        elif t == "COMMAND_LONG" and m.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            code = int(m.param2)
        if code is not None:
            self.mode_req = (COPTER_MODE.get(code, code), time.time())

    def _track_mode_owner(self):
        """每次觀察到模式變化，判定這個模式是我方造成的還是飛手切的。

        **只在變化的當下判定一次**：飛手把 LOITER 切走再切回來，如果每拍都拿
        「我方曾經要求過 LOITER」來比對，就會把飛手的 LOITER 誤認成我方的暫停，
        然後在他手上飛的時候送 RTL——那正是這套規則最該避免的操作。

        判不出來（代理剛啟動、飛行中重連）時 owner 留 None，由呼叫端當
        「不確定」處理，而不是預設成我方。
        """
        mode = self.mon.state.get("mode")
        if mode == self._seen_mode:
            return
        self._seen_mode = mode
        req, ts = self.mode_req or (None, 0)
        # 10 秒：模式指令送出到心跳回報新模式之間的正常延遲遠小於此；
        # 放寬到這個量級是為了容忍 57600 上的重送與丟包
        fresh = req is not None and time.time() - ts < 10
        if fresh and req == mode:
            self.mode_owner = "us"
        elif fresh:
            # 我方剛要求了**別的**模式，而它變成這個 → 有人插手，那是飛手
            self.mode_owner = "pilot"
        else:
            # **沒有依據就說沒有依據。** 代理剛啟動、飛行中重連、或飛手在
            # 我方完全沒動作時切了模式——這三種情況我們都**不知道**是誰。
            #
            # 這一格原本直接寫死 `"pilot"`（而 docstring 早就寫著「留 None」）。
            # 後果：暫停中只要代理重啟一次，那個 LOITER 就被判成飛手的，
            # `flight_state()` 回 PILOT_CONTROL，而守門表裡那一格是 `set()`
            # ——**地面站從此一個指令都下不了，直到有人手動切一次模式**。
            # 2026-09-02 使用者實際被這個擋住。
            self.mode_owner = None

    def _write(self, fd, data):
        try:
            os.write(fd, data)
            self.fwd_to_fc += 1
        except OSError as e:
            log.warning("寫入飛控失敗：%s", e)

    def say_hello(self):
        """向地面站自報身分——**註冊是它的副作用**（uav-system issues/038）。

        為什麼由代理發起而不是人在前端註冊：代理知道的（板子 UID、廠牌、韌體、
        機型）**全部是機器問得到的，讓人打字只會打錯**；而人該維護的（機架序號、
        型號、名稱）代理問不到。所以前端只需要編輯，不需要註冊表單。

        鍵是 **board_uid 不是 sysid**：sysid 是機上可以隨時改的參數，
        2026-08-24 就發生過舊記錄佔著 sysid、新機被認領進去的事。

        **等拿到 board_uid 才送**：沒有它就沒有穩定的身分，寧可晚一點註冊，
        也不要先用一個會變的鍵建檔。
        """
        if self.hello_sent:
            return
        uid = self.mon.state.get("board_uid")
        if not uid:
            return                          # 還沒問到板子身分，下一拍再說
        body = {
            "board_uid": uid,
            "agent_uid": agent_uid(),
            "autopilot": {3: "ardupilot", 12: "px4"}.get(
                self.mon.state.get("autopilot_raw")),
            "fw": self.mon.state.get("fw"),
            "vehicle_type": self.mon.state.get("vehicle_type"),
            "agent_version": AGENT_VERSION,
            # **它現在用的號碼**（uav-system issues/040 A1）。地面站據此判斷
            # 「它跑錯號碼了沒」——不送的話那頭只能配一個號給我們，
            # 分不出「本來就用對了」與「不知道它用什麼」
            "sysid": self.mon.state.get("sysid"),
        }
        try:
            req = urllib.request.Request(
                f"http://{self.a.gs}:{self.a.backend_port}/api/agent/hello",
                data=json.dumps(body).encode(), method="POST",
                headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=5) as r:
                res = json.loads(r.read() or b"{}")
            self.hello_sent = True
            self.drone_id = res.get("drone_id")
            # 配號（040 A1／A3）。**只記下來，換號在 _maybe_reassign 裡做**——
            # 註冊的回應處理不該順手重開飛控
            self.assigned_sysid = res.get("assigned_sysid")
            self.sysid_action = res.get("sysid_action")
            if self.sysid_action == "change":
                log.warning("⚠ 地面站配給這塊板子的號碼是 %s，我們現在用 %s：%s",
                            self.assigned_sysid, self.mon.state.get("sysid"),
                            res.get("sysid_reason"))
            log.info("已向地面站自報：%s（%s，board_uid=%s）", res.get("name"),
                     "**新建檔**" if res.get("created") else "既有記錄", uid)
            self._start_intent_link(uid, body)
        except Exception as e:
            # 失敗不設旗標＝下一拍重試。地面站可能還沒起來，或 5G 正好在抖。
            log.warning("自報身分失敗（%s），稍後重試", e)

    #: 換號之後等飛控用新號碼回來的上限。超過就放棄並大聲說——**不要無限
    #: 等一個不會回來的東西**，那會讓地面站的畫面永遠停在「換號中」
    REASSIGN_TIMEOUT_S = 90.0
    #: 可以換號的狀態。**只有這三個**：換號要重開飛控，而在天上重開飛控是
    #: 不可想像的。用 flight_state() 而不是自己判 armed——全系統只有一份狀態定義
    REASSIGN_OK_STATES = ("DISARMED", "NOT_READY", "READY")

    def _maybe_reassign(self, now):
        """地面站說「這個號碼不是配給你的」時，把它換過來（issues/040 A3）。

        **這是本代理唯一會重開飛控的動作**，所以守門比其他地方都嚴：

        * **只在地面**。`REASSIGN_OK_STATES` 三格，用 `flight_state()` 判——
          全系統只有一份狀態定義，自己另外看一次 armed 會漂。
        * **一個行程只試一次**。失敗就留給人，不要反覆重開飛控。
        * **設了要讀回確認才重開**。ACK 是「我收到了」不是「我做到了」，
          而這裡做錯的代價是重開一次飛控卻沒有換到號。
        * **重開期間回報 `reassigning`**（設計 §3.3）。重開的是飛控不是 Pi，
          所以意圖通道不會斷——地面站因此預期得到那塊板號會用新號碼回來，
          **不會把舊號碼的消失當成失聯**。沒有這一格，換號動作本身就會觸發
          039 的失聯處置：我們會為了修好身分而讓飛機以為地面站不見了。
        """
        if self.reassigning:
            return self._watch_reassign(now)
        if (self.sysid_action != "change" or self.reassign_tried
                or not self.a.sysid_auto_apply):
            return
        want, have = self.assigned_sysid, self.mon.state.get("sysid")
        if want is None or have is None or want == have:
            return
        st = self.flight_state()
        if st not in self.REASSIGN_OK_STATES:
            # **不要每拍抱怨**：這台機可能整趟飛行都在天上，換號等落地再說
            if not getattr(self, "_reassign_wait_logged", False):
                self._reassign_wait_logged = True
                log.warning("要把 sysid %s 換成 %s，但現在是 %s——**等落地上鎖**"
                            "（在天上重開飛控是不可想像的）", have, want, st)
            return
        self.reassign_tried = True
        log.error("⚠ 換號：sysid %s → %s。**這會重開飛控**（機在地面且已上鎖）",
                  have, want)
        try:
            # ⚠ **寫入的值域編碼是方言，而這裡只對 ArduPilot 驗過。**
            # 讀取那一側已經走驅動（`decode_param`：PX4 把整數的位元組原樣塞進
            # float32，ArduPilot 寫數值本身），**寫入沒有對應的 `encode_param`**。
            # 不在這裡自己補一個 if：`autopilot/` 是上游的逐字複製品，
            # 加東西會讓 `tools/driver-parity.py` 從此都是紅的，而那支護欄的
            # 價值就在於它平常是綠的。**要補請補在上游**（uav-system 的
            # `libs/autopilot`），下次同步就會拿到——與 `gcs_failsafe_params()`
            # 同一條路徑。在那之前：**非 ArduPilot 機不要開自動換號**。
            self.serial.mav.param_set_send(
                have, 1, b"MAV_SYS_ID", float(want),
                mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        except Exception:
            log.exception("MAV_SYS_ID 送不出去，放棄換號")
            return
        try:      # 問回來才確認得了——PARAM_SET 之後機端不一定主動廣播
            self.serial.mav.param_request_read_send(have, 1, b"MAV_SYS_ID", -1)
        except Exception:
            pass
        self.reassigning = {"from": have, "to": want, "phase": "setting",
                            "since": now}

    def _watch_reassign(self, now):
        """換號的後半段：確認參數 → 重開 → 等它用新號碼回來。"""
        r = self.reassigning
        cur = self.mon.state.get("sysid")
        if now - r["since"] > self.REASSIGN_TIMEOUT_S:
            log.error("⚠ 換號逾時（%s → %s，卡在 %s）——**放棄並留在原狀**。"
                      "飛控可能沒吃這個參數，或重開後沒有回來；請人工確認",
                      r["from"], r["to"], r["phase"])
            self.reassigning = None
            return
        if r["phase"] == "setting":
            # **讀回確認才重開**：ACK 是我收到了，不是我做到了。
            # 每拍重問一次——57600 上單一封包丟失是真實風險
            try:
                self.serial.mav.param_request_read_send(
                    r["from"], 1, b"MAV_SYS_ID", -1)
            except Exception:
                pass
            got = (self.mon.state.get("params") or {}).get("MAV_SYS_ID")
            if got is not None and int(got) == r["to"]:
                log.warning("MAV_SYS_ID 已確認為 %s，重開飛控", r["to"])
                try:
                    self.serial.mav.command_long_send(
                        r["from"], 1,
                        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                        0, 1, 0, 0, 0, 0, 0, 0)
                except Exception:
                    log.exception("重開指令送不出去")
                    self.reassigning = None
                    return
                r["phase"], r["since"] = "rebooting", now
            return
        if r["phase"] == "rebooting" and cur == r["to"]:
            log.warning("✓ 換號完成：飛控用 sysid %s 回來了", cur)
            self.reassigning = None
            self.sysid_action = "keep"
            # **重新註冊**：讓地面站看到它現在用對號碼了（會拿到 keep）
            self.hello_sent = False

    def _start_intent_link(self, uid, ident):
        """意圖通道（協定 §2）。**等註冊完才起**：地面站以 board_uid 認機，
        沒註冊過就連上去，那頭只能記成一條無主的連線。"""
        if self.intent is not None:
            return
        self.intent = IntentLink(
            f"ws://{self.a.gs}:{self.a.backend_port}/ws/agent",
            {"agent_version": AGENT_VERSION, "board_uid": uid,
             "autopilot": ident.get("autopilot"), "fw": ident.get("fw"),
             # inputs＝判斷依據的清單（協定 §4.1）。目前只有遙測；機上加裝
             # 相機後會多一項，地面站據此知道這台機的判斷是基於什麼
             "inputs": ["telemetry"], "protocol": [1],
             # 地面站據此知道哪些意圖要自己執行、哪些丟給代理（丙案分工）
             "executes": sorted(guard.AGENT_ACTIONS),
             "vets": sorted(guard.ACTIONS),
             # 026 待決點 6：驅動跟著飛機走，機隊裡就可能同時存在不同版本。
             # **先讓差異可見**——「版本不合就降級唯讀」那類守則本批不做，
             # 因為沒有人看得到的漂移，訂再多守則也管不到。
             # `limits` 依這台機的廠牌給；廠牌還沒問到（沒收過 HEARTBEAT）時
             # 是 UnknownDriver 的空表，那是誠實的「不知道」
             "driver": {**normalize.driver_info(),
                        "limits": normalize.driver_limits(
                            self.mon.state.get("autopilot_raw"))}},
            on_intent=self.on_intent, on_decision=self.on_decision,
            on_progress=self.on_progress)
        self.intent.start()

    def on_intent(self, msg) -> dict:
        """收到意圖：**先守門，再看誰執行**（協定 §5.2、丙案分工）。

        守門一律先過這裡，因為它檢查的是**執行當下**的事實——人是在看提案時
        確認的，兩者之間飛機一直在動。所以守門的位階高於 `approved`。
        """
        action = msg.get("action")
        state = self.flight_state()
        # RC 前置條件的事實在機上（039 複裁 A）：地面站把同一條規則畫成按鈕
        # 按不下去，但那是提示，**權威在這裡**——按鈕擋得住人，擋不住補送
        ok, why = guard.check(state, action, self.mon.state.get("rc_link"))
        base = {"intent_id": msg.get("intent_id"), "action": action,
                "state": state, "executor": guard.executor(action or "")}
        if not ok:
            log.warning("守門擋下意圖 %s（狀態 %s）：%s", action, state, why)
            self.mon.events.append(
                (time.time(), "guard", f"擋下 {action}：{why}"))
            return {**base, "event": "guard_refused", "reason": why}
        if action == "change_route":
            # 算提案本來就不動飛機，乾跑與否走同一條路——而**補送的
            # change_route 需要的正是重算提案再要人確認**（協定 §2）
            return self._change_route_proposal(msg, base)
        if (msg.get("params") or {}).get("dry_run"):
            # **乾跑：問判決但不動飛機。**
            # 沒有這個模式的話，要驗證守門「放行」那幾格就一定得真的下指令——
            # 2026-08-25 我就對一台停在地上的真機送出了 RTL／LAND／LOITER，
            # 只為了確認守門會放行。機是上鎖的所以無害，但**驗證不該需要
            # 動到飛機**，而且下次可能就不是上鎖狀態了
            #
            # **這道排在執行者分岔之前**（039 複裁 G）：原本它只擋代理自己
            # 執行的那幾個，地面站執行的 `start_mission` 走上面那條回
            # `cleared`——而 `cleared` 在地面站那端**就是放行**。失聯期間
            # 壓下來的意圖恢復後補送、每則都帶 `dry_run`，少了這道，一個人
            # 在四十秒前按下的「開始任務」會在恢復的瞬間真的執行
            mode = self._INTENT_MODE.get(action)
            log.info("守門放行 %s（狀態 %s）→ 乾跑，不動飛機", action, state)
            return {**base, "event": "would_execute", "mode": mode,
                    "reason": ("守門通過；乾跑模式，沒有下達 " + mode) if mode
                              else "守門通過；乾跑模式，沒有交給地面站執行"}
        if guard.executor(action) == "ground":
            # **放行不等於做完**：地面站有完整的廠牌方言，由它執行；
            # 代理只負責說「現在這件事可以做」
            log.info("守門放行 %s（狀態 %s）→ 由地面站執行", action, state)
            return {**base, "event": "cleared",
                    "reason": "守門通過，由地面站執行"}
        return self._do_agent_action(action, base)

    def _change_route_proposal(self, msg, base):
        """改航線的提案由**機上**算（狀態機文件 §0.1、§6.2）。

        **為什麼權威在這裡**：地面站看到的位置經 5G 回來已經是過期資料——
        10 m/s 巡航下一次鏈路抖動就是數十公尺誤差，而「離當前位置最近的航點」
        正是對位置最敏感的判斷。今天這條 5G 一次斷 98 秒，那不是理論值。

        帶 `prior`（人看過的那份提案）進來時順便做過期檢查（協定 §5.1）：
        **這件事也該在機上做**，因為它問的是「機體從你確認到現在飄了多少」。
        """
        p = msg.get("params") or {}
        wps = p.get("wps") or []
        if not wps:
            return {**base, "event": "failed",
                    "reason": "改航線的意圖沒有帶航點，算不出提案"}
        s = self.mon.state
        cur = dict(self._position(), heading=s.get("heading"))
        prop = route.build_proposal(
            wps=wps, cur=cur, hold_alt=p.get("hold_alt"),
            mission_name=p.get("mission_name") or "（未命名）",
            mission_id=p.get("mission_id"),
            cur_seq=s.get("mission_seq"))
        prop["_wps"] = wps          # 確認時要用同一份航點重算
        # 提案留在機上等確認。**不必讓地面站送回來**——送回來的那份與人真正
        # 看到的那份是不是同一個，沒有辦法驗證；留在這裡就沒有這個空間
        iid = base.get("intent_id")
        if iid:
            self.proposals[iid] = prop
            if len(self.proposals) > 16:
                self.proposals.pop(next(iter(self.proposals)))
        return {**base, "event": "proposal", "proposal": prop,
                "reason": "提案由機上算（位置最新）"}

    def on_decision(self, msg) -> dict:
        """人的確認（協定 §4.5）。**確認不是授權，是對那一份提案的確認。**

        所以這裡一定重算並比對（§5.1）：人是在看提案的那一刻按的，而飛機在
        那段時間裡一直在動。續飛航點換了、或距離變化超過門檻，就中止重提。

        **守門在這裡再過一次**：狀態可能在人看提案的時候變了——例如飛手拿起
        遙控器接管。人確認的位階低於守門（§5.2）。
        """
        iid = msg.get("intent_id")
        base = {"intent_id": iid, "action": "change_route",
                "state": self.flight_state(), "executor": "ground"}
        prior = self.proposals.get(iid)
        if prior is None:
            return {**base, "event": "failed",
                    "reason": "沒有這個 intent_id 的提案（過久沒確認被汰換，"
                              "或代理重啟過）——請重新取得提案"}
        if not msg.get("approved"):
            self.proposals.pop(iid, None)
            log.info("人取消了改航線（%s）", iid)
            return {**base, "event": "cancelled", "reason": "人取消"}
        ok, why = guard.check(base["state"], "change_route",
                              self.mon.state.get("rc_link"))
        if not ok:
            self.proposals.pop(iid, None)
            log.warning("確認之後守門仍擋下：%s", why)
            return {**base, "event": "guard_refused", "reason": why}
        s = self.mon.state
        cur = dict(self._position(), heading=s.get("heading"))
        now_prop = route.build_proposal(
            wps=prior.get("_wps") or self._wps_of(iid), cur=cur,
            hold_alt=(prior.get("hold") or {}).get("alt"),
            mission_name=prior.get("mission_name"),
            mission_id=prior.get("mission_id"),
            cur_seq=s.get("mission_seq"))
        drift = route.drift_reason(prior, now_prop)
        if drift:
            self.proposals[iid] = now_prop      # 換成新的，等人重看
            log.warning("確認到達時提案已過期：%s", drift)
            return {**base, "event": "guard_refused", "proposal": now_prop,
                    "reason": f"提案已經過期：{drift}。請重看新的提案再決定"}
        # 序列開始。**設一個死線**：地面站中途掛掉的話，這個旗標不能永遠留著
        # ——它會讓失聯處置一直以為序列在跑
        self.seq = {"intent_id": iid, "step": "approved", "since": time.time(),
                    "deadline": time.time() + self.a.seq_timeout_s}
        log.info("改航線已確認（%s）→ 由地面站執行三步", iid)
        self.mon.events.append((time.time(), "intent", "改航線已確認，序列開始"))
        return {**base, "event": "cleared", "proposal": now_prop,
                "reason": "確認成立，由地面站執行三步序列"}

    def _wps_of(self, iid):
        """提案裡沒有存航點時的退路——回空表，讓 build_proposal 說算不出來。"""
        return (self.proposals.get(iid) or {}).get("_wps") or []

    def on_progress(self, msg) -> dict:
        """地面站執行序列的逐步回報（協定 §4.6）。

        **代理需要知道序列跑到哪一步**，不是為了記帳，是因為失聯處置的規則
        依賴它：§7 規定「改航線序列中 → 立刻 RTL」，因為序列跑到一半失聯＝
        飛機停在一個沒有人確認過的中間狀態。
        """
        iid, step = msg.get("intent_id"), msg.get("step")
        ok = bool(msg.get("ok", True))
        base = {"intent_id": iid, "action": "change_route", "step": step,
                "state": self.flight_state(), "executor": "ground"}
        if self.seq and self.seq.get("intent_id") == iid:
            self.seq["step"] = step
            self.seq["deadline"] = time.time() + self.a.seq_timeout_s
            if step in ("resume", "done") or not ok:
                self.seq = None
                self.proposals.pop(iid, None)
        log.info("改航線序列 %s：%s%s", step, "完成" if ok else "**失敗**",
                 f"（{msg.get('detail')}）" if msg.get("detail") else "")
        self.mon.events.append(
            (time.time(), "intent",
             f"改航線 {step} {'完成' if ok else '失敗'}"))
        return {**base, "event": "noted", "ok": ok}

    def _expire_seq(self, now):
        """序列旗標的死線。地面站中途掛掉時，這個旗標留著會讓失聯處置一直
        以為序列在跑——而**旗標卡住造成的誤判，比沒有旗標更難查**。"""
        if self.seq and now > self.seq["deadline"]:
            log.warning("改航線序列逾時未完成（%.0f 秒），清掉序列旗標",
                        now - self.seq["since"])
            self.mon.events.append(
                (now, "intent", "改航線序列逾時，旗標已清除"))
            self.seq = None

    #: 代理自己執行的意圖 → 飛控模式。只要 DO_SET_MODE，不碰任務協定
    _INTENT_MODE = {"rtl": "RTL", "land": "LAND", "abort": "LOITER"}

    def _do_agent_action(self, action, base) -> dict:
        mode = self._INTENT_MODE[action]
        tgt = self.mon.state.get("sysid", 1)
        code = COPTER_MODE_ID.get(mode)
        if code is None:
            return {**base, "event": "failed", "reason": f"不認得模式 {mode}"}
        try:
            # 送三次：57600 上單一封包丟失是真實風險，而這是保命指令。
            # DO_SET_MODE 冪等，重複下達沒有副作用（與失聯處置同一套理由）
            self.mode_req = (mode, time.time())
            for _ in range(3):
                self.serial.mav.command_long_send(
                    tgt, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    code, 0, 0, 0, 0, 0)
                time.sleep(0.1)
        except Exception as e:
            log.exception("意圖 %s 送出失敗", action)
            return {**base, "event": "failed", "reason": f"送出失敗：{e}"}
        log.info("意圖 %s → 已下達 %s（狀態 %s）", action, mode, base["state"])
        self.mon.events.append((time.time(), "intent", f"{action} → {mode}"))
        # **回 sent 而不是 done**：ACK 是「我收到了」，不是「我做到了」。
        # 真的進了哪個模式由下一則 state 說（那是讀回來的事實）
        return {**base, "event": "sent", "mode": mode,
                "reason": f"已下達 {mode}，實際模式看下一則 state"}

    def _publish_state(self):
        """把當下狀態交給意圖通道。**狀態的定義只有 flight_state() 一份**——
        失聯處置與這裡送出去的是同一個值，不會出現「代理自己做一套、
        報上去另一套」。"""
        if self.intent is None:
            return
        s = self.mon.state
        self.intent.publish({
            "state": self.flight_state(),
            "sysid": s.get("sysid"),
            "intent_id": (self.seq or {}).get("intent_id"),
            "seq_step": (self.seq or {}).get("step"),
            "mission_seq": s.get("mission_seq"),
            "mission_total": s.get("mission_total"),
            # **RC 遙控器在不在**（039 複裁 A）。放在頂層而不是 derived 裡：
            # 地面站的鏡像讀的是 `p.get("rc_link")`（agent_link.py），而
            # `derived` 是整包原封鏡像的，位置放錯畫面就永遠是「不知道」。
            # **沒讀到就送 None**——不知道不擋，權威守門在這台機上
            "rc_link": s.get("rc_link"),
            # 040 A3：正在換號。**放頂層與 rc_link 同理**——地面站的鏡像讀的是
            # 頂層欄位，放進 derived 會讓畫面永遠看不到它
            "reassigning": self.reassigning,
            # **這個 LOITER 是誰造成的**：us／pilot／None＝不知道。
            # 它決定了守門讓不讓地面站動這台機，所以**它必須看得見**——
            # 原本它只活在代理的記憶體裡，畫面上只看得到「指令被拒」
            "mode_owner": self.mode_owner,
            # 014：機上錄製的回傳現況。**放頂層而不是 derived**——`derived`
            # 是拿去與地面站自己算的做比對的（026 §9），塞一個地面站算不出來
            # 的東西進去沒有意義。
            #
            # **只送份數不夠。** 「沒有東西要傳」與「傳不動」都是「沒在傳」，
            # 但一個是完成、一個是故障，而它們要做的處置完全相反——所以
            # `blocked` 帶著「為什麼沒在傳」那句話一起上去。
            "record_upload": self.uploader.state() if self.uploader else None,
            "derived": {
                "armed": bool(s.get("armed")),
                "landed": s.get("landed_state"),
                "mode": s.get("mode"),
                # **位置走 _position()**：0,0 是自駕儀「不知道」的哨兵，
                # 不是幾內亞灣外海。直接讀 s["lat"] 會把它照實推上去，
                # 地圖上就長出一個假的點——地面站早就有這條規則
                # （mavlink_rx.py GLOBAL_POSITION_INT），機端不能自己破例
                **self._position(),
                "battery_pct": s.get("battery_pct"),
                "gps_fix": s.get("gps_fix"),
                # 地面站失聯是**代理這一側看到的事實**，與地面站自己看到的
                # 不一定同時發生——兩邊不一致本身就是要報警的事（狀態機 §6.4）
                "gs_link_ok": self.gs_link_ok,
                # 026 B4-b：驅動算出來的正規化欄位（協定 §4.2）。
                # **判斷用 mode_verb、顯示用 mode_name**；`ready` 三態。
                # 地面站認得這個廠牌時會拿自己算的來對，不一致就報警——
                # 那不是要調解的分歧，是**有一邊的驅動錯了**
                **normalize.derived(s),
            },
        })

    def _upload_gate(self):
        """機上錄製可以回傳了嗎（issues/014）。回 `(可以嗎, 為什麼不行)`。

        **只在地面傳，一解鎖立刻停。** 與補傳（`backfill.py`）同一條紀律：
        **補歷史不該拖慢現在**。差別在量級——補傳一筆 100 bytes，這裡一次
        是幾 MB；飛行中送它，等於拿正在飛的那台機的遙測，去換一份已經安全
        落地的紀錄。

        **`armed` 用最後已知值判就夠了。** 它只會被新的心跳改掉，所以
        「飛控失聯、而最後看到它是 armed」自然就會擋下上傳——不必為那一格
        另外寫規則。反過來，從來沒看到過飛控（`armed is None`）時放行是對的：
        那台機不在天上，而且也沒有遙測可以被排擠。

        `gs_link_ok is None` ＝ 還沒建立過鏈路。**不放行**：那時送出去的
        每一塊都會逾時，而逾時要等 60 秒。
        """
        if self.mon.state.get("armed"):
            return False, "armed（飛行中不傳）"
        if not self.gs_link_ok:
            return False, ("地面站鏈路未建立" if self.gs_link_ok is None
                           else "地面站失聯")
        return True, None

    def _check_gs_link(self, now):
        """地面失聯偵測與處置。

        **為什麼這件事只能由代理做**：飛控的 GCS failsafe（`FS_GCS_ENABLE`）
        盯的是 `SYSID_MYGCS` 的心跳，而那則心跳是**地面的指令服務**每秒送的
        （sysid 255），經 5G → 本代理 → 序列埠到飛控。所以 5G 一斷，飛控那邊
        的 GCS 心跳也真的停了——**它不是無感的**（模組說明有完整的查證）。

        但**它看得見的只有「心跳沒了」，看不見那代表什麼**：是 5G 抖一下、
        地面站當掉、還是操作員關機離開，飛控一律只能做同一件粗的事。
        分狀態的判斷（任務中續飛／暫停中立刻回／飛手手上不介入）只有代理
        做得到，因為只有代理同時看得到飛控的全部狀態與地面的死活。

        **若 `FS_GCS_ENABLE` 是開的，飛控會搶在這一層前面**——它的逾時遠短於
        30 秒單飛上限，那時「跑完任務再 RTL」不會發生。這是 uav-system 039
        的待裁項，不是這裡可以自己決定的事。

        三層防線裡這是第二層：
          1. 飛控自己的 RC failsafe（遙控器待命，人隨時可接管）
          2. **本層**：地面失聯 → 依當下狀態決定處置
          3. 地面站的操作員判斷

        **處置規則＝選項 C（協定 §6，2026-08-25 裁定）**，逐狀態決定：
        任務進行中續飛到跑完再 RTL；暫停中／起飛中立刻 RTL；已在返航或降落
        不介入；飛手拿著永不介入。續飛另有兩個上限（單飛時間、電量）。
        為什麼不是「一律 30 秒 RTL」：5G 抖動在本場域是常態，一次抖動就中斷
        自動任務等於這套系統不能用；反過來「一律跑完」則會讓沒人看得見的
        飛機一直飛下去。分狀態是為了讓每一格的代價都說得出口。

        設計上的節制：
        * **只在 armed 時處置**。地面上做整合測試時鏈路斷來斷去是常態，
          板凳上絕不能因此送模式指令。
        * **已在 RTL/LAND 就不介入**——飛機已經在自己回家了，再送一次只會
          打斷它，或覆蓋掉操作員/飛控剛做的決定。
        * **一次失聯只動作一次**。動作後若人用遙控器接管改了模式，我們不再
          搶回來——人優先於自動化。
        * **鏈路恢復不改變當下行為**。已經開始的 RTL 不會被取消；恢復只解除
          警戒並留痕，要不要繼續是人的決定。
        """
        if not self.last_gs_rx:
            return                        # 還沒建立過鏈路，沒有「失去」可言
        age = now - self.last_gs_rx
        ok = age < self.a.link_warn_s

        if ok != self.gs_link_ok:
            was = self.gs_link_ok
            self.gs_link_ok = ok
            if ok:
                if was is None:
                    log.info("地面站鏈路建立")      # 首次連上不是「恢復」
                else:
                    # age 這時已經被剛收到的封包更新過＝一定接近 0，拿它當
                    # 中斷長度會永遠印「中斷 0 秒」。要用失聯起點算才是實話。
                    dur = now - self.loss_started if self.loss_started else 0
                    log.warning("地面站鏈路恢復（中斷 %.0f 秒）", dur)
                    self.mon.events.append(
                        (now, "gs_link", f"地面站鏈路恢復（中斷 {dur:.0f}s）"))
                    # **接管結束要交代做過什麼。** 「我不在的時候發生了什麼」
                    # 是操作員回來第一個要問的問題，而唯一答得出來的是代理
                    if self.takeover:
                        acts = self.takeover["acts"]
                        log.warning("**接管結束**（%.0f 秒）：%s", dur,
                                    "；".join(acts) if acts else "期間沒有動作")
                        self.mon.events.append(
                            (now, "takeover",
                             f"代理接管結束（{dur:.0f}s）："
                             + ("；".join(acts) if acts else "期間沒有動作")))
                        self.takeover = None
                self.loss_warned = self.loss_acted = False   # 解除警戒
                self._solo_logged = False
                self.loss_started = None
            else:
                self.loss_started = self.last_gs_rx
                # **接管開始**（2026-09-04 使用者定義）：原本連著、而且在飛，
                # 然後意外斷線。從這一刻到連線恢復為止，代理是唯一的決策者
                # ——所以它做的每一件事都要留痕，恢復時人才看得到發生過什麼
                if self.mon.state.get("armed"):
                    st0 = self.flight_state()
                    self.takeover = {"since": now, "state": st0, "acts": []}
                    log.warning("⚠ **代理接管**（失聯時狀態 %s）"
                                "——到連線恢復為止由代理決定", st0)
                    self.mon.events.append(
                        (now, "takeover", f"代理接管（失聯時 {st0}）"))
            return

        if ok:
            return

        armed = self.mon.state.get("armed")
        mode = self.mon.state.get("mode")
        if not self.loss_warned:
            self.loss_warned = True
            log.warning("⚠ 地面站失聯 %.0f 秒（armed=%s mode=%s 狀態=%s）；"
                        "%.0f 秒後依狀態處置%s",
                        age, armed, mode, self.flight_state(),
                        self.a.link_action_s,
                        "" if self.a.link_action != "none"
                        else "（設定為 none＝只告警不動作）")
            self.mon.events.append((now, "gs_link", f"⚠ 地面站失聯（{age:.0f}s）"))

        if self.loss_acted or age < self.a.link_action_s:
            return
        if self.a.link_action == "none":
            self.loss_acted = True
            return

        state = self.flight_state()
        reason = self._loss_verdict(state, now)
        if reason is None:
            return                          # 續飛觀察中：下一拍再評估
        if reason is False:
            self.loss_acted = True          # 這個狀態下永遠不介入，不必再看
            log.warning("地面站失聯逾時，狀態 %s → 不介入", state)
            self.mon.events.append(
                (now, "gs_link", f"地面失聯但不介入（{state}）"))
            return
        self._do_link_loss_action(now, age, mode, f"{state}／{reason}")

    def flight_state(self):
        """當下的飛行狀態——**全系統只有這一個定義**。

        失聯處置用它決定要不要介入，意圖協定的 `state` 訊息也送它。兩處各寫
        一份的話，日後一定會漂移成「代理說 HOLDING、畫面說 FLYING_MISSION」，
        而那正是狀態機文件 §0.1 說「只能有一個事實來源」要防的事。

        **這還不是協定裡那台權威狀態機**：協定的狀態帶著「當前 intent 走到
        第幾步」，那要等 intent 實作。這裡是純粹從飛控遙測導出的觀察值，名稱
        與狀態機文件 §1 對齊，接上 intent 後由它補上序列步驟資訊。

        每一格都往「不要亂動」的方向偏——分不出來就當飛手拿著。
        """
        s = self.mon.state
        if time.time() - self.mon.last_hb > 5.0:
            return "DISCONNECTED"          # 沒有心跳就不得下任何指令
        # MAV_STATE 5/6＝CRITICAL/EMERGENCY：飛控自己的 failsafe 正在跑
        if s.get("system_status") in (5, 6):
            return "EMERGENCY"
        if not s.get("armed"):
            # 完整判定要 prearm_ok 與 ekf_ok（狀態機文件 §1），代理目前只解到
            # GPS。**所以這兩格是近似的**——地面狀態不影響飛行中的處置，
            # 接上 intent 前不值得為它多解一批訊息
            fix = s.get("gps_fix")
            return "READY" if fix in ("3D", "DGPS", "RTK_FLOAT", "RTK_FIXED") \
                else "NOT_READY"

        landed = self._landed()

        mode = s.get("mode")
        if mode in SELF_RECOVERING:
            return "LANDING" if mode == "LAND" else "RETURNING"
        if landed == "landing":
            return "LANDING"
        if landed == "on_ground":
            return "ARMED_GROUND"
        if landed == "takeoff":
            return "TAKING_OFF"
        if mode in AUTOMATED_MODES:
            if mode == "AUTO" and s.get("mission_state") != 5:
                return "FLYING_MISSION"     # 5＝MISSION_STATE_COMPLETE
            return "HOLDING"
        if mode == "LOITER" and self.mode_owner != "pilot":
            # **溯源不明的 LOITER 也算 HOLDING**（2026-09-02 使用者裁定：
            # 「先不知道誰做的沒關係，但起碼無人機狀態我要完全掌控」）。
            #
            # 三態的差別在這裡：`"us"`＝我方按的暫停、`None`＝不知道、
            # `"pilot"`＝**確定**是飛手插手（我方剛要求了別的模式而它變成這個）。
            # 只有第三種才鎖死——**「不知道」不該與「確定是飛手」同罰**，
            # 而那正是原本的行為：判不出來就當飛手，然後什麼都不准。
            #
            # 手飛模式（STABILIZE／ALT_HOLD／ACRO）不受影響，它們走下面那行
            # ——那些模式的桿位直接控制姿態，看到它們就確定人在飛。
            return "HOLDING"
        return "PILOT_CONTROL"              # 確定是飛手的 LOITER 與所有手飛模式

    def _mission_done_by_seq(self):
        """`mission_seq == mission_total` 的替代完成判定（039 複裁 C）。

        **預設關閉，要旗標才啟用。** 韌體不回報 `mission_state` 時，目前只能
        靠單飛時間上限收尾——那是能用但鈍的收法。用 seq 到底了來補，問題在於
        `total` 到底含不含 home（航點 0），以及最後一個航點的 seq 在停留期間
        會不會就停在那個值：兩者都是**廠牌與版本的行為**，讀規格讀不出來。
        猜錯的後果不對稱——早一格判成「跑完」＝任務還沒飛完就被 RTL 打斷。

        所以照裁定：實作放這裡、預設不啟用，**真機覆核 ArduPilot 4.7 之後**
        再打開。啟用時只在韌體真的沒回報 `mission_state` 的情況下才作數——
        有權威值就用權威值，不必猜。
        """
        if not self.a.mission_complete_by_seq:
            return None
        s = self.mon.state
        if s.get("mission_state"):
            return None                 # 韌體有回報，輪不到替代判定
        seq, total = s.get("mission_seq"), s.get("mission_total")
        if seq is None or not total:
            return None
        if seq >= total:
            return (f"任務已完成（seq {seq}/{total} 到底；"
                    "韌體不回報 mission_state，用的是替代判定）")
        return None

    #: **「操作員主動要求」與「沒有人在看，我自己決定」對「不知道」的正確
    #: 反應是相反的。** 守門那邊：溯源不明時放行，讓操作員拿得回控制權
    #: （2026-09-02 使用者裁定）。失聯處置這邊：溯源不明時**不介入**——
    #: 那正是 `mode_owner` 當初存在的理由，而且它的失敗模式最糟：
    #: 飛手正用 LOITER 手飛、5G 抖一下，我們就在他手上送 RTL。
    #:
    #: 兩者共用 `flight_state()` 的 HOLDING，差別由這裡補上。
    def _loss_verdict(self, state, now):
        """回傳：字串＝立刻 RTL 的理由；False＝這個狀態永不介入；None＝再等一拍。"""
        # 不介入的四類：已經在自己回家／降落、飛手拿著、飛控 failsafe 正在跑、
        # 還沒起飛（含連不上——連不上時下任何指令都是對著空氣喊）
        if state in ("RETURNING", "LANDING", "PILOT_CONTROL", "EMERGENCY",
                     "DISCONNECTED", "NOT_READY", "READY"):
            return False
        if state == "ARMED_GROUND":
            # **規格說 ARMED_GROUND 立刻 RTL，這裡刻意不照做**（2026-08-25）：
            # 機在地上時 RTL 會先爬到 RTL_ALT——那是一次沒有人按下的起飛，
            # 而且發生在「地面站看不到」的當下。同一時間遙控器就在旁邊待命
            # （這是本系統的操作前提），人處理比飛機自己起飛安全得多。
            # 所以這格只告警留痕。要改回規格行為必須是明確裁定，不是預設值。
            return False
        if self.seq:
            # §7：改航線序列跑到一半失聯 → 立刻 RTL。飛機此刻停在一個**沒有人
            # 確認過的中間狀態**（可能已經 hold、可能已經上傳但還沒續飛），
            # 而接下來要下的每一步都需要地面站——它已經不在了
            return f"改航線序列進行中（{self.seq.get('step')}）且地面站失聯"
        # **只有 LOITER 這一格有歧義。** GUIDED 與跑完的 AUTO 也會回 HOLDING，
        # 但那兩個模式**人不可能用手飛**（桿位不控制姿態），所以溯源不明在
        # 那裡不代表任何事——把它們一起擋掉會讓「任務跑完後失聯」變成不介入，
        # 而那正是最需要 RTL 的一格
        if (state == "HOLDING" and self.mode_owner is None
                and self.mon.state.get("mode") == "LOITER"):
            # **不知道這個 LOITER 是誰造成的 → 不自己動手。** 它可能是我方
            # 的暫停（那 RTL 是對的），也可能是飛手正拿著遙控器在飛
            # （那 RTL 是最不該做的事）。分不出來時，代價不對稱：
            # 少做一次最多是飛機停在空中等人，多做一次是從飛手手上搶飛機。
            log.warning("失聯處置：LOITER 但溯源不明 → 不介入"
                        "（可能是飛手在飛；飛控自己的 failsafe 仍是後備）")
            return False
        if state in ("HOLDING", "TAKING_OFF"):
            # 沒有任務可以跑完，停在空中等一個回不來的確認毫無意義
            return "無任務可續飛（暫停中、爬升中或任務已結束）"
        if state != "FLYING_MISSION":
            # **不對著看不懂的狀態下指令。** 走到這裡代表有人新增了狀態卻沒有
            # 補這張表；此時飛控自己的 RC failsafe 與待命的飛手仍在，
            # 亂送模式指令比不送危險
            log.error("失聯處置遇到未列入的狀態 %s → 不動作", state)
            return False
        # ── FLYING_MISSION：跑完再 RTL，但有兩個上限 ──────────────
        # **電量：兩個判準任一觸發就回家，兩個都讀不到也回家**
        # （2026-09-04 使用者裁定；039 複裁 E 的延伸）。判定與理由都在
        # `_battery_verdict` 裡——那裡也說明了為什麼不能只看百分比。
        go_home, why = self._battery_verdict()
        if go_home:
            return why
        # 用 is None 而不是 `or`：loss_started 是 epoch 秒數，`or` 會把
        # 合法的 0.0 當成未設定——測試就是這樣抓到的（單飛上限永不觸發）
        base = now if self.loss_started is None else self.loss_started
        solo = now - base
        if solo > self.a.link_loss_max_solo_s:
            return f"無人監看已 {solo:.0f} 秒（上限 {self.a.link_loss_max_solo_s:.0f}）"
        if self.mon.state.get("mission_state") == 5:
            return "任務已完成"
        seq_done = self._mission_done_by_seq()
        if seq_done:
            return seq_done
        if not self._solo_logged:
            self._solo_logged = True
            ms = self.mon.state.get("mission_state")
            log.warning("地面失聯但任務進行中（%s/%s）→ 續飛，%.0f 秒或電量 %d%% "
                        "為止%s", self.mon.state.get("mission_seq"),
                        self.mon.state.get("mission_total") or "?",
                        self.a.link_loss_max_solo_s, self.a.link_loss_min_batt,
                        "" if ms else
                        ("（韌體不回報 mission_state；已啟用 seq 替代判定）"
                         if self.a.mission_complete_by_seq else
                         "（韌體不回報 mission_state＝測不到「跑完」，"
                         "只能靠時間上限收尾）"))
            self.mon.events.append(
                (now, "gs_link", "地面失聯，任務續飛中（上限內）"))
        return None

    def _do_link_loss_action(self, now, age, mode, why):
        self.loss_acted = True
        if self.takeover is not None:
            self.takeover["acts"].append(f"{self.a.link_action.upper()}（{why}）")
        want = self.a.link_action.upper()
        code = COPTER_MODE_ID.get(want)
        if code is None:
            log.error("未知的失聯處置 %s，不動作", want)
            return
        tgt = self.mon.state.get("sysid", 1)
        # 這是我方要求的模式：記進溯源，免得等一下把自己下的 RTL 當成飛手切的
        self.mode_req = (want, time.time())
        log.error("⚠⚠ 地面站失聯 %.0f 秒且機體 armed（模式 %s，%s）→ 下達 %s",
                  age, mode, why, want)
        try:
            # 送三次：57600 的鏈路上單一封包丟失是真實風險，而這是保命指令。
            # DO_SET_MODE 是冪等的，重複下達沒有副作用。
            for _ in range(3):
                self.serial.mav.command_long_send(
                    tgt, 1, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    code, 0, 0, 0, 0, 0)
                time.sleep(0.1)
            self.mon.events.append((now, "gs_link_action",
                                    f"⚠ 地面失聯 {age:.0f}s（{why}）→ 下達 {want}"))
        except Exception:
            log.exception("失聯處置送出失敗")

    def _landed(self):
        """在地上還是在天上。`landed_state` 優先，韌體不回報就退回高度＋地速。

        **寧可判成飛行中**——判錯成「在地上」會漏掉該做的處置，判錯成
        「飛行中」最多是多做一次。訊號記錄沿用同一個方向：不知道有沒有離地
        時照錄，因為**飛行中的量測補不回來**，多錄幾筆地面資料則刪得掉。

        這裡是「在不在地上」的**唯一定義**：狀態機與訊號記錄共用它。
        兩份實作一定會漂，而漂掉的那天沒有人會發現。
        """
        s = self.mon.state
        landed = s.get("landed_state")
        if landed is not None:
            return landed
        alt, spd = s.get("alt_rel_m"), s.get("groundspeed_ms")
        return ("on_ground" if (alt is not None and alt < 1.0
                                and spd is not None and spd < 0.5)
                else "in_air")

    def airborne(self):
        """離地了沒（2026-09-07 使用者裁定：**只有飛離地面才記訊號**）。"""
        return self._landed() != "on_ground"

    def _position(self):
        """給訊號採樣綁進同一筆樣本。**沒有定位就回 None，不填 0,0**——
        GPS 未定位時 GLOBAL_POSITION_INT 會回 0/0，那不是「在幾內亞灣」，
        是「不知道」。填進去會讓地圖上長出一個假的軌跡點。"""
        s = self.mon.state
        if s.get("gps_fix") in (None, "NO_GPS", "NO_FIX"):
            return {"alt_rel": s.get("alt_rel_m")}
        return {"lat": s.get("lat"), "lon": s.get("lon"),
                "alt_rel": s.get("alt_rel_m")}

    def report(self):
        s = self.mon.state
        hb_age = time.time() - self.mon.last_hb if self.mon.last_hb else None
        # **錄製器的 flush 原本是死碼。** 檔案開著 64 KB 緩衝，代理若在飛行中
        # 被 SIGKILL／掉電，尾巴那 64 KB（約十秒）就沒了——而**那十秒正是
        # 我們留這份錄製的理由**（出事前最後那一段）。狀態列本來就每 10 秒
        # 跑一次，順手在這裡落地。
        if self.recorder:
            self.recorder.flush()
        line = {
            "fc_heartbeat_age_s": round(hb_age, 1) if hb_age else None,
            "armed": s.get("armed"), "mode": s.get("mode"),
            "battery_pct": s.get("battery_pct"), "voltage": s.get("voltage"),
            "gps": f"{s.get('gps_fix')}/{s.get('gps_sats')}sats",
            "alt_rel_m": s.get("alt_rel_m"),
            "msgs_from_fc": sum(self.mon.counts.values()),
            "tx_telemetry": self.telem.tx, "tx_command": self.cmd.tx,
            "rx_from_gs": self.telem.rx + self.cmd.rx,
            "fwd_to_fc": self.fwd_to_fc,
            "tx_errors": self.telem.tx_err + self.cmd.tx_err,
            "gs_link_ok": self.gs_link_ok,
            "state": self.flight_state(),
            "intent": self.intent.stats() if self.intent else None,
            "backfill": self.backfill.stats() if self.backfill else None,
            "record": self.recorder.stats() if self.recorder else None,
            "upload": self.uploader.stats() if self.uploader else None,
            "modem": self.modem.stats() if self.modem else None,
        }
        log.info("狀態 %s", json.dumps(line, ensure_ascii=False))
        if self.a.state_file:
            tmp = self.a.state_file + ".tmp"
            with open(tmp, "w") as f:
                json.dump({"ts": time.time(), **line, **s}, f,
                          ensure_ascii=False, indent=2)
            os.replace(tmp, self.a.state_file)      # 原子換檔，讀的人不會看到半截


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dev", default=os.environ.get("FC_DEV", "/dev/ttyAMA0"))
    p.add_argument("--baud", type=int,
                   default=int(os.environ.get("FC_BAUD", 57600)))
    p.add_argument("--gs", default=os.environ.get("GS_HOST", "10.141.2.21"))
    p.add_argument("--telemetry-port", type=int,
                   default=int(os.environ.get("GS_TELEMETRY_PORT", 14540)))
    p.add_argument("--command-port", type=int,
                   default=int(os.environ.get("GS_COMMAND_PORT", 14541)))
    p.add_argument("--rate", type=int, default=int(os.environ.get("RATE", 4)))
    p.add_argument("--backend-port", type=int,
                   default=int(os.environ.get("GS_BACKEND_PORT", 38000)),
                   help="地面站 backend HTTP 埠（link-metrics 用）")
    p.add_argument("--modem-port", default=os.environ.get("MODEM_PORT",
                                                          "/dev/ttyUSB1"),
                   help="5G 模組的 AT 埠；設為空字串可停用訊號採樣")
    p.add_argument("--modem-interval", type=float,
                   default=float(os.environ.get("MODEM_INTERVAL", 1.0)))
    # ── 地面失聯處置（飛安相關，預設值刻意保守）──────────────────
    p.add_argument("--link-warn-s", type=float,
                   default=float(os.environ.get("LINK_WARN_S", 5.0)),
                   help="多久沒收到地面站封包就視為失聯並告警")
    p.add_argument("--link-action-s", type=float,
                   default=float(os.environ.get("LINK_ACTION_S", 10.0)),
                   help="失聯持續多久後執行處置（僅在 armed 時）")
    p.add_argument("--link-action",
                   default=os.environ.get("LINK_ACTION", "rtl"),
                   choices=["rtl", "land", "smart_rtl", "none"],
                   help="失聯處置動作；none＝只告警不動作")
    # ── 選項 C 的兩個上限（協定 §6）─────────────────────────────
    # **兩個預設值都是暫定的**，要由實機續航測試定案：續航多久、返航要留多少
    # 電量餘裕，是這台機的物理性質，不是可以在鍵盤上想出來的數字。
    # 在有實測之前取保守值——保守的代價是任務被打斷，不保守的代價是掉機。
    p.add_argument("--link-loss-max-solo-s", type=float,
                   default=float(os.environ.get("LINK_LOSS_MAX_SOLO_S", 30.0)),
                   help="失聯後任務最多再自己飛多久（使用者定案 2026-08-25：30s）")
    p.add_argument("--link-loss-min-batt", type=int,
                   default=int(os.environ.get("LINK_LOSS_MIN_BATT_PCT", 15)),
                   help="失聯續飛的電量下限，低於就立刻 RTL（使用者定案：15%%）")
    # 複裁 C（2026-08-31）：韌體不回報 mission_state 時，用 seq 到底了當替代
    # 的「跑完」判定。**預設關**——`total` 含不含 home、最後一個航點停留時
    # seq 停在哪，是廠牌與版本的行為，猜錯就會在任務還沒飛完時 RTL 打斷它。
    # 真機覆核 ArduPilot 4.7 之後才開
    p.add_argument("--mission-complete-by-seq", action="store_true",
                   default=os.environ.get("MISSION_COMPLETE_BY_SEQ", "")
                   .lower() in ("1", "true", "yes"),
                   help="用 mission_seq == mission_total 當替代的任務完成判定"
                        "（複裁 C；真機覆核 ArduPilot 4.7 前不要開）")
    # 改航線序列的死線：地面站中途掛掉時清掉序列旗標。三步序列正常只要幾秒，
    # 120 秒已經是「顯然出事了」的量級
    p.add_argument("--seq-timeout-s", type=float,
                   default=float(os.environ.get("SEQ_TIMEOUT_S", 120.0)),
                   help="改航線序列多久沒進展就清掉序列旗標")
    #: 040 A3：收到「這個號碼不是配給你的」時要不要自動換。**預設開**
    #: （使用者裁定 sysid 由系統指派），但留一個關得掉的開關：**這是本代理
    #: 唯一會重開飛控的動作**，現場調校時可能不希望它自己動。
    #: 非 ArduPilot 機建議關掉——參數寫入的值域編碼還沒有跨廠牌驗過。
    p.add_argument("--no-sysid-auto-apply", dest="sysid_auto_apply",
                   action="store_false",
                   default=os.environ.get("SYSID_AUTO_APPLY", "1") != "0")
    #: 機上原始層錄製（issues/014）。**預設只在 armed 時錄**——待機時的資料量
    #: 大而價值低，與飛控自己 `LOG_DISARMED=0` 的選擇一致
    p.add_argument("--record-dir",
                   default=os.environ.get("RECORD_DIR", "/home/pi/uav-record"))
    p.add_argument("--record-max-mb", type=int,
                   default=int(os.environ.get("RECORD_MAX_MB", 512)))
    p.add_argument("--record-keep", type=int,
                   default=int(os.environ.get("RECORD_KEEP", 20)))
    p.add_argument("--record-disarmed", action="store_true",
                   default=os.environ.get("RECORD_DISARMED", "0") != "0",
                   help="連未解鎖時也錄（預設否）")
    #: 錄製的自動回傳（issues/014）。**預設開**：錄下來的東西如果要人 scp
    #: 才拿得到，它就只對記得去拿的人存在——而想起來要去拿的時機通常是
    #: 出事之後，那時卡上的滾動可能已經把它刪掉了。
    #: 危險的不是「傳」而是「什麼時候傳」，那一格由 `_upload_gate()` 守
    #: （只在地面、鏈路通、落地靜置過），所以預設開是安全的。
    p.add_argument("--no-upload", dest="upload", action="store_false",
                   default=os.environ.get("RECORD_UPLOAD", "1") != "0",
                   help="停用錄製檔自動回傳（改回落地後人工 scp）")
    #: **限速：地面站那條 5G 上可能還有別台機正在飛，而正在飛的那台優先。**
    #: 一趟十分鐘的錄製約 4 MB（UART 5.6 KB/s 的上限），256 KB/s 下約 16 秒。
    p.add_argument("--upload-kbps", type=int,
                   default=int(os.environ.get("RECORD_UPLOAD_KBPS", 256)),
                   help="回傳限速（KB/s）")
    #: **塊大小同時決定了「起飛後多久停手」**：守門在每一塊之前檢查，所以
    #: 停手的延遲上限就是送一塊要多久。預設與限速相同＝最遲一秒。
    p.add_argument("--upload-chunk-kb", type=int,
                   default=int(os.environ.get("RECORD_UPLOAD_CHUNK_KB", 256)),
                   help="回傳的塊大小（KB）；與限速同值時＝解鎖後最遲一秒停手")
    p.add_argument("--upload-settle-s", type=float,
                   default=float(os.environ.get("RECORD_UPLOAD_SETTLE_S", 20)),
                   help="落地後等多久才開始回傳（板凳上 arm/disarm 連續切換時"
                        "不要每次都戳一輪上傳）")
    p.add_argument("--report-interval", type=float, default=10.0)
    p.add_argument("--state-file",
                   default=os.environ.get("STATE_FILE",
                                          "/home/pi/uav-agent/state.json"))
    a = p.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s")
    log.info("uav-agent 啟動：%s@%d → %s", a.dev, a.baud, a.gs)
    # **這兩個值撞在一起的話，選項 C 會退化成 A**：處置要等 link_action_s 才
    # 開始評估，而單飛時間是從失聯那一刻起算——兩者相等時，第一次評估當下
    # 單飛時間就已經到上限，「任務中續飛」那條分支永遠走不到。
    # 不擅自改使用者給的數字，但一定要說出來：安靜地退化成另一種策略，
    # 比策略本身選錯更難發現
    if a.link_loss_max_solo_s <= a.link_action_s:
        log.warning(
            "⚠ 單飛上限 %.0fs ≤ 處置起算 %.0fs → **選項 C 會退化成「一律 %s」**："
            "第一次評估時單飛時間就已達上限，任務中續飛那條分支走不到。"
            "要讓續飛真的發生，把 LINK_ACTION_S 調小或把 LINK_LOSS_MAX_SOLO_S 調大",
            a.link_loss_max_solo_s, a.link_action_s, a.link_action.upper())
    log.info("地面失聯處置（選項 C／逐狀態）：%.0fs 告警／%.0fs 起算；"
             "任務中續飛（上限 %.0fs 或電量 %d%%，讀不到電量＝不續飛）、"
             "暫停中立刻 %s、已在 RTL/LAND 或飛手手上則不介入",
             a.link_warn_s, a.link_action_s, a.link_loss_max_solo_s,
             a.link_loss_min_batt, a.link_action.upper())
    # 兩條都是「預設不啟用」的安全前提，而預設不啟用最容易被忘記——啟動時
    # 說一次，免得事後從行為反推
    log.info("「跑完」的判定：mission_state；替代判定（seq 到底）%s",
             "已啟用（複裁 C——真機覆核過了嗎？）"
             if a.mission_complete_by_seq else
             "**未啟用**（複裁 C：待真機覆核 ArduPilot 4.7 的 seq 行為）")
    log.info("RC 前置條件（複裁 A）：rc_link 為 false 時擋下 %s；"
             "**未回報＝不知道＝不擋**", sorted(guard.RC_REQUIRED))
    # **選項 C 的第三條安靜退化途徑。** GCS 心跳是地面的指令服務每秒送的，
    # 經 5G 過本代理到飛控——5G 一斷，飛控那邊的心跳也真的停了。飛控那層
    # 若搶在代理前面動作，**代理完全不會知道**：我們只看得到模式突然變了，
    # 而 `_track_mode_owner` 會把它判成飛手切的，於是決定永不介入。
    # 2026-09-01 使用者裁定 FS_GCS_ENABLE=1、FS_GCS_TIMEOUT=45，不變式是
    #   FS_GCS_TIMEOUT > LINK_LOSS_MAX_SOLO_S > LINK_ACTION_S
    # 飛控那層的角色是**代理的後備**：只在代理沒動手時才出手。開機時讀一次
    # 參數自檢（`request_fs_gcs`／`_check_fs_gcs`），不週期重讀
    log.info("飛控 GCS failsafe：開機後會讀一次 FS_GCS_ENABLE／FS_GCS_TIMEOUT "
             "自檢，要求 FS_GCS_TIMEOUT > %.0fs（單飛上限）> %.0fs（處置起算）"
             "；讀不到與讀到但不合會分開報",
             a.link_loss_max_solo_s, a.link_action_s)
    log.info("訊號採樣：%s", a.modem_port or "停用")
    # **「什麼時候傳」是這個功能唯一危險的一格**，所以它要跟失聯處置那些
    # 政策一樣，在啟動時就說出口——事後從行為反推是最貴的查法
    log.info("機上錄製：%s（%s、%d MB 換檔、保留 %d 份）；自動回傳 %s",
             a.record_dir, "armed 時才錄" if not a.record_disarmed else "**連未解鎖也錄**",
             a.record_max_mb, a.record_keep,
             f"限速 {a.upload_kbps} KB/s、塊 {a.upload_chunk_kb} KB、"
             f"落地靜置 {a.upload_settle_s:.0f}s"
             "（**只在地面、鏈路通時傳，一解鎖立刻停**）"
             if a.upload else "**停用**（要人工 scp）")
    Agent(a).run()


if __name__ == "__main__":
    sys.exit(main())
