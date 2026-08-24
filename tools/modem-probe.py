#!/usr/bin/env python3
"""找出 Fibocom FM160 的 AT 埠並撈訊號資訊。全部是唯讀查詢，不改 modem 設定。

usb0 是這台 Pi 唯一的對外網路，所以這裡只送查詢型指令（AT+XXX? / AT+XXX），
不送任何會改設定或重啟的指令。
"""
import sys, time
import serial

PORTS = ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB3"]

QUERIES = [
    ("ATI",                  "模組識別"),
    ("AT+CSQ",               "基本訊號品質（RSSI/BER）"),
    ("AT+CESQ",              "延伸訊號品質（含 LTE/NR 的 RSRP/RSRQ/SINR）"),
    ("AT+COPS?",             "目前電信商與接取technology"),
    ("AT+CREG?",             "2G/3G/4G 註冊狀態"),
    ("AT+CEREG?",            "LTE 註冊狀態（含 TAC/CellID）"),
    ("AT+C5GREG?",           "5G NR 註冊狀態"),
    ("AT+GTCCINFO?",         "Fibocom 服務細胞資訊（主力）"),
    ('AT+QENG="servingcell"', "Quectel 式服務細胞（預期不支援，對照用）"),
    ("AT+GTUSBMODE?",        "USB 模式"),
]


def talk(ser, cmd, wait=1.2):
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode())
    time.sleep(wait)
    out = ser.read(ser.in_waiting or 1).decode("utf-8", "replace")
    return out.strip()


def main():
    at_port = None
    print("=== 找 AT 埠 ===")
    for p in PORTS:
        try:
            ser = serial.Serial(p, 115200, timeout=1)
        except Exception as e:
            print(f"  {p}: 開不起來（{e}）")
            continue
        try:
            r = talk(ser, "AT", 0.6)
            ok = "OK" in r
            print(f"  {p}: {'✔ 回 OK' if ok else '無回應／非 AT 埠'}"
                  f"{'' if ok else '  ' + repr(r[:40])}")
            if ok and at_port is None:
                at_port = p
        finally:
            ser.close()

    if at_port is None:
        print("\n❌ 四個埠都沒有 AT 介面回應。")
        return 1

    print(f"\n=== 使用 {at_port} 撈訊號資訊 ===")
    ser = serial.Serial(at_port, 115200, timeout=1)
    try:
        for cmd, desc in QUERIES:
            r = talk(ser, cmd)
            print(f"\n── {cmd}   ({desc})")
            for line in r.splitlines():
                line = line.strip()
                if line and line != cmd:
                    print(f"     {line}")
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
