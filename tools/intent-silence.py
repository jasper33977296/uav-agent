#!/usr/bin/env python3
"""靜默逾時測試：對面不回話時，機端要在幾秒內自己掐掉重連。

**為什麼非測不可**：這個機制存在的理由是「送得出去不代表送得到」。5G 路由掉
的時候 TCP 的 send() 只是塞進 kernel buffer 就回傳成功，機端會一路「送」幾十秒
才發現對面早就收不到——2026-08-25 實測到代理 sent 從 59 數到 79、地面站 30 秒
一則都沒收到。**這種失效不會有任何錯誤訊息**，所以只能用行為去釘。

做法：起一個**只收不回**的假地面站（WebSocket 握手照做，之後一句話都不說）。
真的地面站對每則 state 都會回 ack，所以「都不回話」就等同鏈路半開。

用法：python3 tools/intent-silence.py
"""
import base64
import hashlib
import socket
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from intent import IntentLink  # noqa: E402

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def mute_server(sock, stop):
    """完成 WebSocket 握手，然後**永遠不說話**（但保持 TCP 連著）。"""
    while not stop.is_set():
        try:
            conn, _ = sock.accept()
        except OSError:
            return
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
        # 收進來的都丟掉，一句都不回——半開鏈路的樣子
        conn.settimeout(0.5)
        while not stop.is_set():
            try:
                if not conn.recv(65536):
                    break
            except socket.timeout:
                continue
            except OSError:
                break
        conn.close()


srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", 0))
srv.listen(4)
port = srv.getsockname()[1]
stop = threading.Event()
threading.Thread(target=mute_server, args=(srv, stop), daemon=True).start()

SILENCE = 3.0                      # 測試用短逾時，機制與 10 秒版本相同
link = IntentLink(f"ws://127.0.0.1:{port}/ws/agent",
                  {"agent_version": "test", "board_uid": "silence-test",
                   "inputs": ["telemetry"], "protocol": [1]},
                  silence=SILENCE)
link.start()

for _ in range(60):
    if link.connected:
        break
    time.sleep(0.1)

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


chk("連得上這個裝聾的地面站", link.connected)
link.publish({"state": "READY", "sysid": 1, "derived": {"armed": False}})

t0 = time.time()
for _ in range(int((SILENCE + 4) * 10)):
    if link.silent_drops:
        break
    time.sleep(0.1)
took = time.time() - t0

chk("靜默逾時有觸發（不是傻等 TCP）", link.silent_drops >= 1,
    f"silent_drops={link.silent_drops}")
chk(f"觸發時間落在 {SILENCE:.0f}s 附近而不是幾十秒",
    SILENCE - 0.5 <= took <= SILENCE + 3.0, f"{took:.1f}s")
chk("掐掉之後會自己重連（不是就此放棄）",
    link.is_alive(), f"sent={link.sent}")

# **反向驗證**：靜默逾時不能誤殺會回話的連線。同一支程式接真的地面站，
# 撐過兩倍逾時仍不該掉線
link.stop(); time.sleep(0.3)
real = IntentLink("ws://localhost:38000/ws/agent",
                  {"agent_version": "test", "board_uid": "silence-test",
                   "inputs": ["telemetry"], "protocol": [1]},
                  silence=SILENCE)
real.start()
for _ in range(60):
    if real.connected:
        break
    time.sleep(0.1)
if not real.connected:
    print("skip：真地面站連不上，反向驗證沒跑（**skip ≠ pass**）")
    ok = False
else:
    real.publish({"state": "READY", "sysid": 1, "derived": {"armed": False}})
    time.sleep(SILENCE * 2 + 1)
    chk("會回話的連線撐過兩倍逾時仍不掉線（沒有誤殺）",
        real.silent_drops == 0 and real.connected,
        f"silent_drops={real.silent_drops} sent={real.sent}")
real.stop()

stop.set()
srv.close()
time.sleep(0.3)
print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
