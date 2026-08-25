"""意圖協定的機端連線（uav-system doc/agent-intent-protocol.md）。

**目前只做通到 `state` 為止**：代理連上地面站的 `/ws/agent`，先送 `hello`，
之後把飛行狀態推上去。地面站是鏡像，不會回下任何指令——`intent`／`proposal`
／`decision` 都還沒實作，收到未知型別一律回報並丟棄。

**跑在獨立執行緒**，理由與 `modem.py` 相同：連線、重連、送出都可能阻塞幾百
毫秒，而 agent 的路由主迴圈停一下，飛控與地面站就同時卡一下。主迴圈只做一件
事——把最新的狀態快照放進 `publish()`，其餘全在這裡。

**為什麼是 WebSocket 而不是繼續用 HTTP POST**：協定 §2 要的是「連線斷掉＝
失聯」這個乾淨語意。輪詢式的 HTTP 分不出「這一拍沒事可報」與「對面不見了」，
而那正是失聯處置要判的東西。

**為什麼用 websocket-client 而不自己切訊框**：與 README 裡「為什麼依賴
pymavlink」同一個理由——自己刻的訊框處理會安靜地產生壞資料，而壞資料在這條
鏈路上會變成錯誤的飛行判斷。
"""
import json
import logging
import threading
import time
from datetime import datetime, timezone

import websocket

log = logging.getLogger("uav-agent.intent")

PROTOCOL_V = 1
#: 狀態沒變化時的保活間隔（協定 §4.2：變化時立即送、另 1 Hz 保活）
KEEPALIVE_S = 1.0


def _now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds") \
        .replace("+00:00", "Z")


class IntentLink(threading.Thread):
    """一條往地面站的意圖通道。斷了就重連，永遠不放棄。

    `publish()` 由主迴圈每拍呼叫，只是覆寫一個快照，不阻塞、不排隊——
    **意圖通道塞住絕不能拖慢飛控的轉發**。舊快照被新的蓋掉是正確行為：
    這條通道要的是「現在是什麼狀態」，不是狀態的歷史。
    """

    def __init__(self, url, hello, interval=KEEPALIVE_S):
        super().__init__(daemon=True)
        self.url = url
        self.hello = hello
        self.interval = interval
        self._snapshot = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.connected = False
        self.sent = 0
        self.rejected = 0
        self.drops = 0                # 連上之後又斷掉的次數（＝鏈路品質指標）
        self._backoff = 1.0

    def publish(self, snapshot: dict) -> None:
        with self._lock:
            self._snapshot = snapshot

    def stop(self):
        self._stop.set()

    def stats(self) -> dict:
        """給代理的定期狀態列。**通道健康度要跟其他數字放在一起**——
        分開放就要兩邊對時間戳才看得出「那次遙測掉包時意圖通道在不在」。"""
        return {"up": self.connected, "sent": self.sent,
                "drops": self.drops, "rejected": self.rejected}

    # ── 執行緒本體 ─────────────────────────────────────────────
    def run(self):
        while not self._stop.is_set():
            try:
                self._session()
            except Exception as e:
                log.warning("意圖通道中斷（%s），%.0f 秒後重連", e, self._backoff)
            finally:
                self.connected = False
            self._stop.wait(self._backoff)
            self._backoff = min(self._backoff * 2, 30.0)  # 上限 30s

    def _session(self):
        ws = websocket.create_connection(self.url, timeout=10,
                                         suppress_origin=True)
        try:
            # **收的逾時要遠小於保活間隔**：這個迴圈是收與送共用的，recv 卡住
            # 多久，送就慢多久。實測用 5 秒時 7 秒內只送出 2 則（協定 §4.2 要求
            # 狀態變化「立即送」，那時最多會慢 5 秒才送出去）
            ws.settimeout(min(0.2, self.interval / 4))
            self._send(ws, dict(self.hello, type="hello"))
            self.connected = True
            # **退避在「連上」時重置，不是在 session 結束時**：session 結束的
            # 唯一方式是拋例外，所以原本寫在 try 之後的重置永遠不會執行。
            # 2026-08-25 實測後果：跑了 2 分半的連線被 peer reset 之後，
            # 退避已經爬到 30 秒，於是每斷一次就白白失聯半分鐘——而本場域的
            # 5G 抖動是常態，正是這條通道最需要立刻回來的時候
            self._backoff = 1.0
            log.info("意圖通道已連上 %s", self.url)
            last_sent = 0.0
            last_state = None
            while not self._stop.is_set():
                # 先收：地面站可能回 error（協定版本不合、型別未支援）。
                # **不能只顧著送**——被拒絕卻沒讀到，就會一路送到天荒地老
                try:
                    self._on_message(ws.recv())
                except websocket.WebSocketTimeoutException:
                    pass
                with self._lock:
                    snap = self._snapshot
                now = time.time()
                if snap is not None and (snap.get("state") != last_state
                                         or now - last_sent >= self.interval):
                    self._send(ws, dict(snap, type="state"))
                    last_state = snap.get("state")
                    last_sent = now
                    self.sent += 1
        finally:
            if self.connected:
                self.drops += 1       # 連上過才算「掉線」，連不上不算
            try:
                ws.close()
            except Exception:
                pass

    def _send(self, ws, msg):
        ws.send(json.dumps(dict(msg, v=PROTOCOL_V, ts=_now_iso())))

    def _on_message(self, raw):
        try:
            msg = json.loads(raw)
        except ValueError:
            log.warning("意圖通道收到非 JSON，丟棄")
            return
        t = msg.get("type")
        if t == "error":
            # 地面站說我們送的東西它不收。**大聲說出來**——這種錯誤如果只是
            # 靜靜地被丟掉，機上會以為狀態一直有推上去，而畫面其實是空的
            self.rejected += 1
            log.error("地面站拒絕：%s", msg.get("reason"))
        elif t in ("intent", "proposal", "decision"):
            log.warning("收到 %s，但本版只做到 state，尚未實作 → 丟棄", t)
        elif t != "ack":
            log.warning("收到未知型別 %s，丟棄（協定 §3：不半懂地解讀）", t)
