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

  但那件事留下的前提仍然成立且**更重要**：飛控的 GCS failsafe
  （`FS_GCS_ENABLE`）盯的是 `SYSID_MYGCS` 的心跳，也就是**本代理**——代理在
  機上，5G 斷了它照樣心跳，所以飛控不會因為地面失聯而觸發 failsafe。
  這條防線因此完全落在代理身上，見 `_check_gs_link`。
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

from intent import IntentLink
from modem import ModemSampler

log = logging.getLogger("uav-agent")

AGENT_VERSION = "0.2.0"


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
            s["autopilot_raw"] = msg.autopilot
            s["vehicle_type"] = msg.type
        elif t == "SYS_STATUS":
            v = msg.voltage_battery
            s["voltage"] = None if v == 0xFFFF else round(v / 1000, 3)
            s["battery_pct"] = None if msg.battery_remaining == -1 \
                else msg.battery_remaining
            s["cpu_load_pct"] = round(msg.load / 10, 1)
            s["sensors_health"] = msg.onboard_control_sensors_health
            if s["battery_pct"] is not None:
                for th in (20, 10, 5):
                    if s["battery_pct"] <= th:
                        self._change(f"batt_below_{th}", True,
                                     f"⚠ 電量低於 {th}%（{s['battery_pct']}%）")
                        break
        elif t == "GPS_RAW_INT":
            s["gps_fix"] = GPS_FIX.get(msg.fix_type, msg.fix_type)
            s["gps_sats"] = msg.satellites_visible
            self._change("gps_fix_state", s["gps_fix"],
                         f"GPS 定位 → {s['gps_fix']}（{s['gps_sats']} 顆）")
        elif t == "GLOBAL_POSITION_INT":
            s["lat"] = round(msg.lat / 1e7, 7)
            s["lon"] = round(msg.lon / 1e7, 7)
            s["alt_rel_m"] = round(msg.relative_alt / 1000, 2)
        elif t == "VFR_HUD":
            s["groundspeed_ms"] = round(msg.groundspeed, 2)
        elif t == "EXTENDED_SYS_STATE":
            # landed_state 是狀態機分辨「在地上／爬升中／降落中」的第一手依據。
            # UNDEFINED（0）＝**這台韌體沒回報**，不是「不在地上」——那時才退回
            # 用高度與地速猜，而且要讓人知道用的是猜的（見 Agent.flight_state）
            s["landed_state"] = LANDED.get(msg.landed_state)
        elif t == "MISSION_CURRENT":
            # 失聯處置的「跑完再 RTL」要知道任務**什麼時候算跑完**。
            # mission_state 是後來才加進 MAVLink 的欄位：舊韌體不填（0＝UNKNOWN），
            # 那時我們就宣告「測不到完成」，改由單飛時間上限收尾——
            # **不要用 seq 到底了來猜**，最後一個航點停留多久是任務自己的事。
            s["mission_seq"] = msg.seq
            s["mission_total"] = getattr(msg, "total", 0) or 0
            s["mission_state"] = getattr(msg, "mission_state", 0) or 0
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
            txt = msg.text.strip()
            if txt:
                self.events.append((now, "statustext",
                                    f"[sev{msg.severity}] {txt}"))
                log.info("飛控訊息 [sev%d] %s", msg.severity, txt)

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
        self.caps_requested = False
        self.hello_sent = False
        self.drone_id = None
        self.intent = None            # 意圖通道（拿到 board_uid 才起，見 say_hello）
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
        # 地面站方向的解析器：要認出搖桿封包才能改寫身分。
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
        self.modem = None

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
                self._position, self.a.modem_interval)
            self.modem.start()
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
        self.say_hello()
        self._track_mode_owner()
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
        self.mode_owner = "us" if req == mode and time.time() - ts < 10 else "pilot"

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
            log.info("已向地面站自報：%s（%s，board_uid=%s）", res.get("name"),
                     "**新建檔**" if res.get("created") else "既有記錄", uid)
            self._start_intent_link(uid, body)
        except Exception as e:
            # 失敗不設旗標＝下一拍重試。地面站可能還沒起來，或 5G 正好在抖。
            log.warning("自報身分失敗（%s），稍後重試", e)

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
             "inputs": ["telemetry"], "protocol": [1]})
        self.intent.start()

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
            "intent_id": None,          # intent 尚未實作，據實留 null
            "seq_step": None,
            "mission_seq": s.get("mission_seq"),
            "mission_total": s.get("mission_total"),
            "derived": {
                "armed": bool(s.get("armed")),
                "landed": s.get("landed_state"),
                "mode": s.get("mode"),
                "alt_rel": s.get("alt_rel_m"),
                "lat": s.get("lat"), "lon": s.get("lon"),
                "battery_pct": s.get("battery_pct"),
                "gps_fix": s.get("gps_fix"),
                # 地面站失聯是**代理這一側看到的事實**，與地面站自己看到的
                # 不一定同時發生——兩邊不一致本身就是要報警的事（狀態機 §6.4）
                "gs_link_ok": self.gs_link_ok,
            },
        })

    def _check_gs_link(self, now):
        """地面失聯偵測與處置。

        **為什麼這件事只能由代理做**：飛控的 GCS failsafe（`FS_GCS_ENABLE`）
        盯的是 `SYSID_MYGCS` 的心跳，而那個身分是本代理、就在機上——5G 斷掉時
        代理照樣心跳，飛控完全無感。整條鏈路上只有代理看得見「地面沒人了」。

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
                self.loss_warned = self.loss_acted = False   # 解除警戒
                self._solo_logged = False
                self.loss_started = None
            else:
                self.loss_started = self.last_gs_rx
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

        landed = s.get("landed_state")
        if landed is None:
            # 韌體不回報 landed_state：退回用高度與地速判「在地上」。
            # **寧可判成飛行中**——判錯成「在地上」會漏掉該做的處置，
            # 判錯成「飛行中」最多是多做一次
            alt, spd = s.get("alt_rel_m"), s.get("groundspeed_ms")
            landed = ("on_ground" if (alt is not None and alt < 1.0
                                      and spd is not None and spd < 0.5)
                      else "in_air")

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
        if mode == "LOITER" and self.mode_owner == "us":
            return "HOLDING"                # 地面站按的暫停
        return "PILOT_CONTROL"              # 含溯源不明的 LOITER 與所有手飛模式

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
        batt = self.mon.state.get("battery_pct")
        if batt is not None and batt < self.a.link_loss_min_batt:
            return f"電量 {batt}% 低於 {self.a.link_loss_min_batt}%"
        # 用 is None 而不是 `or`：loss_started 是 epoch 秒數，`or` 會把
        # 合法的 0.0 當成未設定——測試就是這樣抓到的（單飛上限永不觸發）
        base = now if self.loss_started is None else self.loss_started
        solo = now - base
        if solo > self.a.link_loss_max_solo_s:
            return f"無人監看已 {solo:.0f} 秒（上限 {self.a.link_loss_max_solo_s:.0f}）"
        if self.mon.state.get("mission_state") == 5:
            return "任務已完成"
        if not self._solo_logged:
            self._solo_logged = True
            ms = self.mon.state.get("mission_state")
            log.warning("地面失聯但任務進行中（%s/%s）→ 續飛，%.0f 秒或電量 %d%% "
                        "為止%s", self.mon.state.get("mission_seq"),
                        self.mon.state.get("mission_total") or "?",
                        self.a.link_loss_max_solo_s, self.a.link_loss_min_batt,
                        "" if ms else "（韌體不回報 mission_state＝測不到"
                                      "「跑完」，只能靠時間上限收尾）")
            self.mon.events.append(
                (now, "gs_link", "地面失聯，任務續飛中（上限內）"))
        return None

    def _do_link_loss_action(self, now, age, mode, why):
        self.loss_acted = True
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
                   default=float(os.environ.get("LINK_ACTION_S", 30.0)),
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
                   default=float(os.environ.get("LINK_LOSS_MAX_SOLO_S", 120.0)),
                   help="失聯後任務最多再自己飛多久（暫定 120s，待實機續航測試）")
    p.add_argument("--link-loss-min-batt", type=int,
                   default=int(os.environ.get("LINK_LOSS_MIN_BATT_PCT", 40)),
                   help="失聯續飛的電量下限，低於就立刻 RTL（暫定 40%%，待實測）")
    p.add_argument("--report-interval", type=float, default=10.0)
    p.add_argument("--state-file",
                   default=os.environ.get("STATE_FILE",
                                          "/home/pi/uav-agent/state.json"))
    a = p.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s")
    log.info("uav-agent 啟動：%s@%d → %s", a.dev, a.baud, a.gs)
    log.info("地面失聯處置（選項 C／逐狀態）：%.0fs 告警／%.0fs 起算；"
             "任務中續飛（上限 %.0fs 或電量 %d%%）、暫停中立刻 %s、"
             "已在 RTL/LAND 或飛手手上則不介入",
             a.link_warn_s, a.link_action_s, a.link_loss_max_solo_s,
             a.link_loss_min_batt, a.link_action.upper())
    log.info("訊號採樣：%s", a.modem_port or "停用")
    Agent(a).run()


if __name__ == "__main__":
    sys.exit(main())
