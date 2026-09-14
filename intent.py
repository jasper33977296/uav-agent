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
#: 多久沒**收到地面站任何一則訊息**就當這條連線死了，主動斷開重連（協定 §2）。
#:
#: **為什麼需要這個，而不是等 TCP 自己發現**：5G 路由掉的時候，UDP 立刻回
#: ENETUNREACH，但 TCP 的 send() 只是把資料塞進 kernel buffer 就回傳成功——
#: 要等重傳逾時（幾十秒）才會爆。2026-08-25 實測到這個現象：代理的 sent
#: 從 59 一路數到 79（自認 1Hz 正常），而地面站那頭 30 秒沒收到任何 state。
#:
#: 10 秒的取法：實測 RTT 30–180ms，正常抖動遠低於此；而 TCP 自己放棄要幾十秒
#: 到兩分鐘。夾在中間＝真的斷了幾秒內就發現，正常的塞車不會被誤殺。
SILENCE_S = 10.0


def _now_iso():
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds") \
        .replace("+00:00", "Z")


class IntentLink(threading.Thread):
    """一條往地面站的意圖通道。斷了就重連，永遠不放棄。

    `publish()` 由主迴圈每拍呼叫，只是覆寫一個快照，不阻塞、不排隊——
    **意圖通道塞住絕不能拖慢飛控的轉發**。舊快照被新的蓋掉是正確行為：
    這條通道要的是「現在是什麼狀態」，不是狀態的歷史。
    """

    def __init__(self, url, hello, interval=KEEPALIVE_S, silence=SILENCE_S,
                 on_intent=None, on_decision=None, on_progress=None):
        super().__init__(daemon=True)
        self.url = url
        self.hello = hello
        #: 收到 intent 時呼叫，回傳一則 event dict。**在這條執行緒上跑**——
        #: 所以它必須是快的（守門查表＋送幾個 MAVLink 封包），不能等回讀
        self.on_intent = on_intent
        #: 人的確認（協定 §4.5）與地面站執行序列的逐步回報（§4.6）。
        #: 兩者都回一則 event，走與 intent 相同的路徑
        self.on_decision = on_decision
        self.on_progress = on_progress
        self.interval = interval
        self.silence = silence
        self._snapshot = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.connected = False
        self.sent = 0
        self.rejected = 0
        self.drops = 0                # 連上之後又斷掉的次數（＝鏈路品質指標）
        self._pending = []            # 待送的 event（在收訊迴圈裡產生，送在同一圈）
        self._done = {}               # intent_id → 上次的結果（冪等用的短記憶）
        self.silent_drops = 0         # 其中「對面不回話」被我方主動掐掉的次數
        self.acked = 0                # 回給地面站的 intent 回執數（複裁 G）
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
                "drops": self.drops, "silent": self.silent_drops,
                "rejected": self.rejected, "acked": self.acked}

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
            # 靜默計時用單調時鐘：Pi 對時會讓牆鐘一次跳好幾分鐘，那一刻會被
            # 當成「地面站好久沒回話」而自己掐斷連線（2026-09-14）
            last_rx = time.monotonic()
            while not self._stop.is_set():
                # 先收：地面站可能回 error（協定版本不合、型別未支援）。
                # **不能只顧著送**——被拒絕卻沒讀到，就會一路送到天荒地老
                try:
                    self._on_message(ws.recv(), ws)
                    last_rx = time.monotonic()
                except websocket.WebSocketTimeoutException:
                    pass
                while self._pending:
                    self._send(ws, dict(self._pending.pop(0), type="event"))
                with self._lock:
                    snap = self._snapshot
                now = time.monotonic()
                # 靜默逾時：**送得出去不代表送得到**（見 SILENCE_S 的說明）。
                # 對面每則 state 都會回 ack，所以「一直沒人回話」＝這條連線已經
                # 沒有用了，主動掐掉重來比等 TCP 自己發現快幾十秒
                if now - last_rx > self.silence:
                    self.silent_drops += 1
                    raise TimeoutError(
                        f"地面站 {self.silence:.0f} 秒沒回話（送出 {self.sent} 則）"
                        "——TCP 還沒斷但這條連線已經沒用了")
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

    def _dispatch(self, msg, handler):
        """intent／decision／progress 都走這條：處理 → 回一則 event。

        **同一個 intent_id 重送＝冪等**（協定 §4.3）：斷線重連後地面站會安全
        重送，那時候**不能再做一次**——重做一次 rtl 沒差，重做一次 change_route
        會讓飛機再繞一圈。回上次的結果就好。

        冪等鍵是 (intent_id, 型別)：同一個 intent 的 decision 與 progress 是
        不同的事，用同一個鍵會讓 decision 之後的 progress 被當成重送吞掉。
        """
        iid = msg.get("intent_id")
        key = (iid, msg.get("type"), msg.get("step"))
        if iid and key in self._done:
            self._pending.append(dict(self._done[key], replayed=True))
            return
        ev = None
        try:
            if handler:
                ev = handler(msg)
        except Exception as e:
            log.exception("處理意圖失敗")
            ev = {"event": "failed", "reason": f"代理處理意圖時例外：{e}"}
        ev = dict(ev or {"event": "failed", "reason": "代理沒有掛對應的處理器"})
        ev.setdefault("intent_id", iid)
        if iid:
            self._done[key] = ev
            # 只留最近的：這是冪等用的短記憶，不是稽核紀錄（那在地面站的
            # command_audit）。無上限地留著＝一個慢慢長大的記憶體洩漏
            if len(self._done) > 64:
                self._done.pop(next(iter(self._done)))
        self._pending.append(ev)

    def _ack(self, ws, of, iid):
        """對一則收到的訊息回執（協定 §2，039 複裁 G）。

        **回執不是判決**：它只說「這則我收到了」，能不能做仍然由後面那則
        `event` 說。分開送是為了讓地面站的逾時**說得出是哪一段斷的**——
        「連回執都沒回」＝這則根本沒送達（5G 掉了），「回執收到了但沒有
        判決」＝代理收到了卻算不出來（守門卡住、飛控沒回應）。兩種都是不行，
        但要查的地方完全不同，而在這之前它們長得一模一樣。

        **當場送，不排進 `_pending`**：那條路出去的訊息會被統一蓋成
        `type="event"`，而回執必須是 `ack`。而且順序有意義——回執要排在判決
        前面，排進同一個佇列反而要多一層型別欄位才分得開。

        送不出去就讓例外往上走：此時這條連線已經沒用了，重連比吞掉好。
        地面站重連後會補送，冪等由 `_dispatch` 的短記憶擋（協定 §4.3）。
        """
        if not iid:
            return                      # 沒有 intent_id 就沒有東西可以回執
        self._send(ws, {"type": "ack", "of": of, "intent_id": iid})
        self.acked += 1

    def _on_message(self, raw, ws=None):
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
        elif t == "intent":
            # **先回執，再照原流程回 event**（協定 §2）。回執在 `_dispatch`
            # 之前：重送的 intent（冪等短記憶命中）一樣要回執——「我收到了」
            # 與「這則我先前就處理過」是兩件事
            if ws is not None:
                self._ack(ws, "intent", msg.get("intent_id"))
            self._dispatch(msg, self.on_intent)
        elif t == "decision":
            self._dispatch(msg, self.on_decision)
        elif t == "progress":
            self._dispatch(msg, self.on_progress)
        elif t == "proposal":
            log.warning("收到 proposal——那是**我方**送出去的型別，"
                        "地面站不該送回來 → 丟棄")
        elif t != "ack":
            log.warning("收到未知型別 %s，丟棄（協定 §3：不半懂地解讀）", t)
