#!/usr/bin/env python3
"""飛控功能測試：主動請求串流，把這台機說得出來的東西全部攤開。

跑法（一定要用 venv 裡的 python）：
    ~/uav-agent/venv/bin/python ~/uav-agent/uav-test.py
    ... --seconds 30 --rate 4
    ... --dump                # 每種訊息印一筆完整欄位
    ... --no-request          # 不請求，看飛控自己主動送什麼

兩件實測學到的事，寫在這裡免得後人重踩：

1. **ArduPilot 預設不推遙測**。這條序列埠沒有 GCS 連過，串流速率是 0，
   不請求就只看得到 1Hz HEARTBEAT 與 TIMESYNC 兩種訊息（2026-08-24 實測）。
   所以要主動送 REQUEST_DATA_STREAM（舊式，ArduPilot 一直吃）＋
   SET_MESSAGE_INTERVAL（新式，逐訊息較精準），兩條都走。

2. **一定要驗 CRC**。第一版自己切訊框、不驗 CRC，靠「訊框接龍」判斷，結果在
   46KB/15s 的流量下解出大量幽靈訊框——sysid 冒出 15 個不存在的值、
   msgid 出現 12586844，還把假 HEARTBEAT 蓋掉真的。payload 裡湊巧出現 0xFD
   的機率遠比想像高。pymavlink 帶完整 CRC_EXTRA 表，這件事交給它。
"""
import argparse
import sys
import time
from collections import defaultdict

from pymavlink import mavutil

# ArduCopter 的 custom_mode
COPTER_MODE = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
               5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
               13: "SPORT", 14: "FLIP", 15: "AUTOTUNE", 16: "POSHOLD",
               17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS",
               21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW", 24: "ZIGZAG",
               25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL"}
GPS_FIX = {0: "無定位(NO_GPS)", 1: "無定位(NO_FIX)", 2: "2D", 3: "3D",
           4: "DGPS", 5: "RTK_FLOAT", 6: "RTK_FIXED"}
SENSOR_BITS = [
    (0x01, "3D 陀螺儀"), (0x02, "3D 加速度計"), (0x04, "3D 磁力計"),
    (0x08, "絕對氣壓"), (0x20, "GPS"), (0x400, "3D 角速率控制"),
    (0x800, "姿態穩定"), (0x1000, "偏航位置"), (0x2000, "Z/高度控制"),
    (0x4000, "X/Y 位置控制"), (0x8000, "馬達輸出"), (0x10000, "RC 接收"),
    (0x100000, "AHRS"), (0x4000000, "電池"),
]
# 明確逐一請求的重點訊息
WANTED = [1, 24, 27, 29, 30, 33, 36, 42, 62, 65, 74, 116, 125, 147, 193, 241]


def request_streams(m, rate):
    """兩條路都走：舊式 ALL 串流 ＋ 新式逐訊息間隔。"""
    m.mav.request_data_stream_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, rate, 1)
    for msgid in WANTED:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msgid, int(1e6 / rate), 0, 0, 0, 0, 0)
        time.sleep(0.02)          # 57600 很窄，別一次灌爆


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--rate", type=int, default=4)
    ap.add_argument("--no-request", action="store_true")
    ap.add_argument("--dump", action="store_true", help="每種訊息印一筆完整欄位")
    a = ap.parse_args()

    print(f"開啟 {a.dev} @ {a.baud}，聽 {a.seconds:.0f} 秒"
          f"{'' if a.no_request else f'，請求串流 {a.rate} Hz'} …\n")
    # 255/190 是 GCS 慣用身分；地面站的 command 服務用 254，刻意錯開，
    # 兩者在飛控眼中才是不同的 GCS，不會互相蓋掉。
    m = mavutil.mavlink_connection(a.dev, baud=a.baud, source_system=255,
                                   source_component=190,
                                   dialect="ardupilotmega")

    print("等待飛控心跳 …")
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        print("❌ 10 秒內沒收到 HEARTBEAT。鮑率不對／飛控沒上電／接線鬆脫。")
        return 1
    print(f"→ 飛控 sysid={m.target_system} compid={m.target_component} "
          f"autopilot={mavutil.mavlink.enums['MAV_AUTOPILOT'][hb.autopilot].name} "
          f"type={mavutil.mavlink.enums['MAV_TYPE'][hb.type].name}\n")

    if not a.no_request:
        request_streams(m, a.rate)
        print(f"→ 已送出 REQUEST_DATA_STREAM ＋ {len(WANTED)} 個 "
              f"SET_MESSAGE_INTERVAL\n")

    counts = defaultdict(int)
    samples, texts, acks = {}, [], []
    latest = {}
    sysids, compids = set(), set()
    t_end = time.time() + a.seconds
    next_hb = 0.0

    while time.time() < t_end:
        now = time.time()
        if now >= next_hb:          # GCS 心跳：讓飛控知道有人在聽
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            next_hb = now + 1.0
        msg = m.recv_match(blocking=True, timeout=0.2)
        if msg is None:
            continue
        t = msg.get_type()
        if t == "BAD_DATA":
            continue
        counts[t] += 1
        sysids.add(msg.get_srcSystem())
        compids.add(msg.get_srcComponent())
        samples.setdefault(t, msg.to_dict())
        latest[t] = msg
        if t == "STATUSTEXT":
            texts.append((msg.severity, msg.text.strip()))
        elif t == "COMMAND_ACK":
            acks.append((msg.command, msg.result))

    report(m, counts, samples, latest, texts, acks, sysids, compids,
           a.seconds, a.dump)
    return 0 if counts else 1


def report(m, counts, samples, latest, texts, acks, sysids, compids, dur, dump):
    errs = m.mav.total_receive_errors
    print("=" * 70)
    print(f"解出 {sum(counts.values())} 個**通過 CRC 驗證**的訊框，"
          f"{len(counts)} 種型別")
    print(f"CRC 失敗／雜訊：{errs}   sysid={sorted(sysids)}  "
          f"compid={sorted(compids)}")
    print("=" * 70)

    def g(t, f, d=None):
        msg = latest.get(t)
        return getattr(msg, f, d) if msg else d

    def row(label, val, unit=""):
        print(f"  {label:<16} {val}{unit}" if val is not None
              else f"  {label:<16} —（沒收到）")

    hb = latest.get("HEARTBEAT")
    print("\n【機體識別】")
    if hb:
        row("自駕儀", mavutil.mavlink.enums['MAV_AUTOPILOT'][hb.autopilot].name)
        row("機型", mavutil.mavlink.enums['MAV_TYPE'][hb.type].name)
        row("系統狀態", mavutil.mavlink.enums['MAV_STATE'][hb.system_status].name)
        print("\n【飛行狀態】")
        row("解鎖(armed)", "是 ⚠" if hb.base_mode & 0x80 else "否")
        row("飛行模式", COPTER_MODE.get(hb.custom_mode, hb.custom_mode))
    row("任務航點", g("MISSION_CURRENT", "seq"))

    print("\n【動力】")
    v = g("SYS_STATUS", "voltage_battery")
    row("電壓", None if v in (None, 0xFFFF) else round(v / 1000, 3), " V")
    c = g("SYS_STATUS", "current_battery")
    row("電流", None if c in (None, -1) else round(c / 100, 2), " A")
    r = g("SYS_STATUS", "battery_remaining")
    row("電量", None if r in (None, -1) else r, " %")
    row("已耗電量", g("BATTERY_STATUS", "current_consumed"), " mAh")
    l = g("SYS_STATUS", "load")
    row("CPU 負載", None if l is None else round(l / 10, 1), " %")

    print("\n【定位】")
    fx = g("GPS_RAW_INT", "fix_type")
    row("GPS 定位", GPS_FIX.get(fx, fx))
    row("衛星數", g("GPS_RAW_INT", "satellites_visible"))
    e = g("GPS_RAW_INT", "eph")
    row("HDOP", None if e in (None, 65535) else round(e / 100, 2))
    if latest.get("GLOBAL_POSITION_INT"):
        p = latest["GLOBAL_POSITION_INT"]
        row("融合座標", (round(p.lat / 1e7, 7), round(p.lon / 1e7, 7)))
        row("相對高度", round(p.relative_alt / 1000, 2), " m")
        row("航向", None if p.hdg == 65535 else round(p.hdg / 100, 1), " °")

    print("\n【姿態與運動】")
    if latest.get("ATTITUDE"):
        import math
        at = latest["ATTITUDE"]
        row("roll/pitch/yaw",
            tuple(round(math.degrees(x), 1) for x in (at.roll, at.pitch, at.yaw)),
            " °")
    row("地速", g("VFR_HUD", "groundspeed"), " m/s")
    row("爬升率", g("VFR_HUD", "climb"), " m/s")
    row("油門", g("VFR_HUD", "throttle"), " %")
    if latest.get("VIBRATION"):
        vb = latest["VIBRATION"]
        row("震動 x/y/z", tuple(round(x, 2) for x in
                              (vb.vibration_x, vb.vibration_y, vb.vibration_z)))

    ss = latest.get("SYS_STATUS")
    if ss:
        print("\n【感測器健康】")
        for bit, name in SENSOR_BITS:
            if ss.onboard_control_sensors_present & bit:
                en = "啟用" if ss.onboard_control_sensors_enabled & bit else "未啟用"
                ok = "✔" if ss.onboard_control_sensors_health & bit else "✘ 異常"
                print(f"  {name:<14} {en:<6} {ok}")

    if texts:
        print("\n【飛控訊息 STATUSTEXT】")
        for sev, txt in dict.fromkeys(texts):
            print(f"  [{sev}] {txt}")

    if acks:
        print("\n【指令回應 COMMAND_ACK】（證明飛控收得到我們的指令）")
        for cmd, res in dict.fromkeys(acks):
            cn = mavutil.mavlink.enums['MAV_CMD'].get(cmd)
            rn = mavutil.mavlink.enums['MAV_RESULT'].get(res)
            print(f"  {cn.name if cn else cmd} → "
                  f"{rn.name if rn else res}")

    print("\n【訊息型別普查】")
    print(f"  {'名稱':<26} {'次數':>6}  {'頻率':>8}")
    for t, c in sorted(counts.items(), key=lambda kv: -kv[1]):
        print(f"  {t:<26} {c:>6}  {c / dur:>6.1f} Hz")
    print(f"\n  共 {len(counts)} 種訊息型別")

    if dump:
        print("\n【每種訊息的完整欄位（各取一筆）】")
        for t in sorted(samples):
            print(f"\n  ── {t}")
            for k, val in samples[t].items():
                if k != "mavpackettype":
                    print(f"       {k:<28} {val}")


if __name__ == "__main__":
    sys.exit(main())
