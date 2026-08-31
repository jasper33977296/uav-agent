#!/usr/bin/env python3
"""意圖回執：收到 `intent` 要先回 `ack`，再回 `event`（協定 §2，039 複裁 G）。

**為什麼順序本身要測**：回執與判決分開送的**唯一理由**就是讓地面站的逾時
說得出是哪一段斷的——「連回執都沒回」＝這則根本沒送達（5G 掉了），
「回執收到了但沒有判決」＝代理收到了卻算不出來（守門卡住、飛控沒回應）。
兩種都是不行，但要查的地方完全不同。順序顛倒或漏掉一則，這個分辨力就沒了，
而**它不會讓任何東西看起來壞掉**——只會在真的出事時少一條線索。

做法：起一個會說話的假地面站，握手後推一則 `intent` 進去，然後看機端**依序**
吐出什麼。用真的 `IntentLink`，不是模擬。

用法：python3 tools/intent-ack.py
"""
import base64
import hashlib
import json
import socket
import struct
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from intent import IntentLink  # noqa: E402

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
got = []                      # 機端送過來的訊息，**照順序**
stop = threading.Event()


def frame(payload: bytes) -> bytes:
    """伺服器→客戶端：不遮罩（RFC 6455 §5.1）。"""
    n = len(payload)
    if n < 126:
        return b"\x81" + bytes([n]) + payload
    return b"\x81\x7e" + struct.pack(">H", n) + payload


def unframe(buf: bytearray):
    """客戶端→伺服器：一定有遮罩。回 (訊息, 剩下的 buffer) 或 (None, buf)。"""
    if len(buf) < 2:
        return None, buf
    ln = buf[1] & 0x7F
    off = 2
    if ln == 126:
        if len(buf) < 4:
            return None, buf
        ln = struct.unpack(">H", buf[2:4])[0]
        off = 4
    masked = bool(buf[1] & 0x80)
    key = buf[off:off + 4] if masked else b""
    off += 4 if masked else 0
    if len(buf) < off + ln:
        return None, buf
    data = bytearray(buf[off:off + ln])
    if masked:
        for i in range(ln):
            data[i] ^= key[i % 4]
    return bytes(data), buf[off + ln:]


def server(sock, intents):
    conn, _ = sock.accept()
    req = conn.recv(4096).decode("latin-1")
    key = ""
    for line in req.split("\r\n"):
        if line.lower().startswith("sec-websocket-key:"):
            key = line.split(":", 1)[1].strip()
    acc = base64.b64encode(
        hashlib.sha1((key + GUID).encode()).digest()).decode()
    conn.send(("HTTP/1.1 101 Switching Protocols\r\n"
               "Upgrade: websocket\r\nConnection: Upgrade\r\n"
               f"Sec-WebSocket-Accept: {acc}\r\n\r\n").encode())
    conn.settimeout(0.2)
    buf, pushed, t0 = bytearray(), 0, time.time()
    while not stop.is_set():
        # 握手後隔一下再推 intent：讓 hello 先過去，順序才乾淨
        if pushed < len(intents) and time.time() - t0 > 0.4 + pushed * 0.8:
            conn.send(frame(json.dumps(intents[pushed]).encode()))
            pushed += 1
        try:
            chunk = conn.recv(65536)
            if not chunk:
                break
            buf += chunk
        except socket.timeout:
            continue
        except OSError:
            break
        while True:
            msg, buf = unframe(buf)
            if msg is None:
                break
            try:
                got.append(json.loads(msg))
            except ValueError:
                pass
    conn.close()


INTENTS = [
    {"v": 1, "type": "intent", "intent_id": "a1", "action": "rtl",
     "params": {"dry_run": True}},
    # 同一個 intent_id 重送：冪等短記憶會命中，但**回執仍然要回**
    {"v": 1, "type": "intent", "intent_id": "a1", "action": "rtl",
     "params": {"dry_run": True}},
]

srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", 0))
srv.listen(2)
port = srv.getsockname()[1]
threading.Thread(target=server, args=(srv, INTENTS), daemon=True).start()


def handle(msg):
    """扮代理的 on_intent：回一則 event（這裡不測守門，測的是通道的順序）。"""
    return {"event": "would_execute", "action": msg.get("action"),
            "state": "FLYING_MISSION", "reason": "測試用"}


link = IntentLink(f"ws://127.0.0.1:{port}/ws/agent",
                  {"agent_version": "test", "board_uid": "ack-test",
                   "inputs": ["telemetry"], "protocol": [1]},
                  on_intent=handle)
link.start()
time.sleep(3.0)
link.stop()
stop.set()
time.sleep(0.4)
try:
    srv.close()
except OSError:
    pass

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


kinds = [m.get("type") for m in got]
acks = [m for m in got if m.get("type") == "ack"]
events = [m for m in got if m.get("type") == "event"]

print("機端送出的順序：", " → ".join(kinds) or "（什麼都沒收到）")
chk("第一則是 hello", kinds[:1] == ["hello"], kinds[:1])
chk("收到 intent 之後有回 ack", len(acks) >= 1, f"ack={len(acks)}")
chk("ack 帶 of=intent 與 intent_id",
    acks and acks[0].get("of") == "intent"
    and acks[0].get("intent_id") == "a1", acks[:1])
chk("ack 帶信封（v／ts）",
    acks and acks[0].get("v") == 1 and acks[0].get("ts"), acks[:1])
chk("也回了 event（回執不取代判決）", len(events) >= 1, f"event={len(events)}")
# **順序就是這件事的全部價值**：先回執、再判決
if acks and events:
    chk("**ack 排在 event 前面**（逾時才分得出是哪一段斷的）",
        kinds.index("ack") < kinds.index("event"), kinds)
else:
    chk("**ack 排在 event 前面**", False, kinds)
# 重送的 intent：判決可以走冪等短記憶，但回執是「我收到了」，每則都要回
chk("重送的 intent 也回執（收到與處理過是兩件事）", len(acks) == 2,
    f"ack={len(acks)}（推了 {len(INTENTS)} 則 intent）")
chk("重送的判決標成 replayed（冪等短記憶命中）",
    any(e.get("replayed") for e in events), [e.get("replayed") for e in events])
chk("stats 數得出回執數", link.stats().get("acked") == len(acks),
    link.stats())

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
