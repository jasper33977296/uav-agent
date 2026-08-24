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

* **手動控制由本代理具名重送**。ArduPilot 對 `MANUAL_CONTROL` 與
  `RC_CHANNELS_OVERRIDE` 有身分檢查：來源 sysid 不等於機上參數 `SYSID_MYGCS`
  就**靜默丟棄，不回任何錯誤**。機上目前 `SYSID_MYGCS=255`＝本代理，所以地面站
  以 254 送來的搖桿封包會被無聲吃掉。地面站自己的診斷也標了這件事。
  作法是把這兩種訊息**以本代理的身分重新編碼後送出**——代理是真正的下達者，
  地面站是意圖來源。其餘訊息（arm/模式/任務等 COMMAND_LONG）**原封轉發**，
  因為那些不受 SYSID_MYGCS 管制，改寫身分只會讓 ACK 的歸屬變糊。

  副作用要記住：飛控的 GCS failsafe（`FS_GCS_ENABLE`）盯的是 `SYSID_MYGCS`
  的心跳，也就是**本代理**——代理在機上，5G 斷了它照樣心跳，所以飛控不會因為
  地面失聯而觸發 failsafe。這條防線因此落在代理身上，見 `_check_gs_link`。
"""
import argparse
import json
import logging
import os
import select
import socket
import sys
import time
from collections import defaultdict

from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as apm

from modem import ModemSampler

log = logging.getLogger("uav-agent")

# 要飛控持續送的重點訊息（逐一設間隔，比舊式 ALL 串流精準）
WANTED = [1, 24, 27, 29, 30, 33, 36, 42, 62, 65, 74, 116, 125, 147, 193, 241]

COPTER_MODE_ID = {v: k for k, v in {
    6: "RTL", 9: "LAND", 21: "SMART_RTL", 5: "LOITER"}.items()}
#: 已經在「自己回家／降落」的模式，失聯處置不再介入
SELF_RECOVERING = {"RTL", "LAND", "SMART_RTL", "AUTO_RTL"}

COPTER_MODE = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
               5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 16: "POSHOLD",
               17: "BRAKE", 18: "THROW", 20: "GUIDED_NOGPS", 21: "SMART_RTL",
               27: "AUTO_RTL"}
GPS_FIX = {0: "NO_GPS", 1: "NO_FIX", 2: "2D", 3: "3D", 4: "DGPS",
           5: "RTK_FLOAT", 6: "RTK_FIXED"}


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
        self.last_telem_rx = 0.0
        self.last_report = 0.0
        self.fwd_to_fc = 0
        self.reissued = 0
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
        """地面站來的封包 → 飛控。搖桿類具名重送，其餘原封轉發。"""
        try:
            msgs = self.gs_parser.parse_buffer(data) or []
        except Exception:
            msgs = []
        if not msgs:                       # 解不出來就原封送，不要自作主張丟掉
            self._write(fd, data)
            return
        for m in msgs:
            t = m.get_type()
            if t in ("MANUAL_CONTROL", "RC_CHANNELS_OVERRIDE"):
                self._reissue(m, t)
            elif t != "BAD_DATA":
                self._write(fd, m.get_msgbuf())

    def _reissue(self, m, t):
        """以本代理的身分重新編碼送出（否則機上 SYSID_MYGCS 檢查會靜默丟棄）。"""
        tgt = self.mon.state.get("sysid", 1)
        try:
            if t == "MANUAL_CONTROL":
                self.serial.mav.manual_control_send(tgt, m.x, m.y, m.z, m.r,
                                                    m.buttons)
            else:
                self.serial.mav.rc_channels_override_send(
                    tgt, 1, m.chan1_raw, m.chan2_raw, m.chan3_raw, m.chan4_raw,
                    m.chan5_raw, m.chan6_raw, m.chan7_raw, m.chan8_raw)
            self.reissued += 1
            self.fwd_to_fc += 1
        except Exception as e:
            log.warning("重送 %s 失敗：%s", t, e)

    def _write(self, fd, data):
        try:
            os.write(fd, data)
            self.fwd_to_fc += 1
        except OSError as e:
            log.warning("寫入飛控失敗：%s", e)

    def _check_gs_link(self, now):
        """地面失聯偵測與處置。

        **為什麼這件事只能由代理做**：飛控的 GCS failsafe（`FS_GCS_ENABLE`）
        盯的是 `SYSID_MYGCS` 的心跳，而那個身分是本代理、就在機上——5G 斷掉時
        代理照樣心跳，飛控完全無感。整條鏈路上只有代理看得見「地面沒人了」。

        三層防線裡這是第二層：
          1. 飛控自己的 RC failsafe（遙控器待命，人隨時可接管）
          2. **本層**：地面失聯 → 逾時未恢復就自主返航
          3. 地面站的操作員判斷

        設計上的節制：
        * **只在 armed 時處置**。地面上做整合測試時鏈路斷來斷去是常態，
          板凳上絕不能因此送模式指令。
        * **已在 RTL/LAND 就不介入**——飛機已經在自己回家了，再送一次只會
          打斷它，或覆蓋掉操作員/飛控剛做的決定。
        * **一次失聯只動作一次**。動作後若人用遙控器接管改了模式，我們不再
          搶回來——人優先於自動化。
        * **鏈路恢復不自動續飛**。恢復只解除警戒並留痕，要不要繼續任務是人的
          決定（自動續飛等於在無人確認的狀態下重新起飛）。
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
            log.warning("⚠ 地面站失聯 %.0f 秒（armed=%s mode=%s）；%.0f 秒後%s",
                        age, armed, mode, self.a.link_action_s,
                        f"執行 {self.a.link_action.upper()}"
                        if self.a.link_action != "none" else "不動作（設定為 none）")
            self.mon.events.append((now, "gs_link", f"⚠ 地面站失聯（{age:.0f}s）"))

        if self.loss_acted or age < self.a.link_action_s:
            return
        if self.a.link_action == "none":
            self.loss_acted = True
            return
        if not armed:
            self.loss_acted = True          # 地面測試：只留痕，絕不送指令
            log.warning("地面站失聯逾時，但機體未 armed → 不送任何指令")
            return
        if mode in SELF_RECOVERING:
            self.loss_acted = True
            log.warning("地面站失聯逾時，但已在 %s → 不介入", mode)
            return

        self._do_link_loss_action(now, age, mode)

    def _do_link_loss_action(self, now, age, mode):
        self.loss_acted = True
        want = self.a.link_action.upper()
        code = COPTER_MODE_ID.get(want)
        if code is None:
            log.error("未知的失聯處置 %s，不動作", want)
            return
        tgt = self.mon.state.get("sysid", 1)
        log.error("⚠⚠ 地面站失聯 %.0f 秒且機體 armed（模式 %s）→ 下達 %s",
                  age, mode, want)
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
                                    f"⚠ 地面失聯 {age:.0f}s → 下達 {want}"))
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
            "reissued_manual": self.reissued,
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
    p.add_argument("--report-interval", type=float, default=10.0)
    p.add_argument("--state-file",
                   default=os.environ.get("STATE_FILE",
                                          "/home/pi/uav-agent/state.json"))
    a = p.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s")
    log.info("uav-agent 啟動：%s@%d → %s", a.dev, a.baud, a.gs)
    log.info("地面失聯處置：%.0fs 告警／%.0fs 執行 %s（僅在 armed 時；"
             "已在 RTL/LAND 則不介入）",
             a.link_warn_s, a.link_action_s, a.link_action.upper())
    log.info("訊號採樣：%s", a.modem_port or "停用")
    Agent(a).run()


if __name__ == "__main__":
    sys.exit(main())
