"""代理的事件出口：「只有代理知道的事」送到地面站（uav-system issues/057）。

**為什麼要有這個**：代理以前有一個 `Monitor.events` 清單，18 處往裡面寫、
0 處讀——「代理接管期間做過什麼」「失聯處置實際送了什麼指令」這些飛安相關的
事實只活在機上的記憶體裡，還無上限地長大。

**只收地面站自己看不到的事。** 飛控的 STATUSTEXT、解鎖／模式、RC、電量、GPS
地面站都從 MAVLink 自己產生事件（`mavlink_rx`），代理再送一份只會讓事件流
每件事出現兩次。意圖的判決（守門擋下、已下達）走意圖通道的 `event` 回應，
也已經進事件流。剩下的才是這裡的：地面站鏈路、接管、失聯處置、失聯 failsafe
自檢、改航線序列逾時——**地面站不在的時候發生的事，唯一答得出來的是代理。**

**送不出去時要留著。** 這些事件多半就發生在送不出去的時候（地面失聯的那一刻，
意圖通道的 TCP 往往還沒被判死）。所以：

* 有上限的緩衝（`MAX`），滿了丟**最舊的**，丟掉幾則記在 `dropped`——
  地面站也從 `seq` 的缺號看得出來
* **地面站逐則回執才算送到**（`ack`）。TCP 的 `send()` 成功只代表塞進了
  核心緩衝（見 `intent.SILENCE_S` 的 2026-08-25 實測）。沒回執的重連後再送一次
* `seq` 從 1 起、每個行程一個 `boot`，地面站據此去重

**時間**：記單調時鐘，送出那一刻才換成牆鐘（`clock.iso`）。Pi 的 RTC 沒電池，
對時之前記下的牆鐘是錯的；對時之後換算出來的才對（見 `clock.py`）。

**節流只對 `gs_link`**：5G 抖動是本場域的常態（09-16 那天每 30 秒一次），
每則都送就是 002 那種狂噴的事件流。但不能靜靜丟掉——被節流的數量在窗口結束時
彙總成一則「另有 N 則沒有逐則送出」。**最後一則逐則送出的可能是「失聯」而它的
「恢復」被併掉了**——所以彙總那句話要叫人看狀態列，事件流在抖動期間說的是
「抖過幾次」，不是「現在通不通」。接管與失聯處置永遠不節流：它們只在解鎖時發生，而且就是
這整件事存在的理由。

**執行緒**：`note()` 由主迴圈呼叫，`unsent()`／`ack()` 由意圖通道的執行緒
呼叫，全部過同一把鎖。
"""
import os
import threading
import time
from collections import deque

import clock

#: 緩衝上限。每則約 200 bytes，500 則＝100 KB。**以量級取**：斷線十分鐘、
#: 每 30 秒一次抖動也才 40 則；到得了 500 代表對面很久沒回執了
MAX = 500

#: 節流：`gs_link` 在 `WINDOW_S` 內前 `BURST` 則照送，其餘併成彙總
#: `net`（dongle 斷線／換 IP 重連，2026-10-20）自己一份額度：原本跟 gs_link 共用，
#: 一次斷線時代理自己的「地面站失聯」「不介入」就把 4 則用完，最重要的那則
#: 「5G 重新連上」被併進 10 分鐘後的彙總
THROTTLED = {"gs_link", "net"}
WINDOW_S = 600.0
BURST = 4

LEVELS = ("info", "warning", "critical")


class Outbox:
    def __init__(self, maxlen=MAX, window_s=WINDOW_S, burst=BURST):
        self._lock = threading.Lock()
        self._q = deque()
        self._max = maxlen
        self._seq = 0
        #: 這個行程的代號。代理重啟 `seq` 會從 1 重來，地面站靠它分辨
        #: 「重送的舊事件」與「新行程的第一則」
        self.boot = os.urandom(4).hex()
        self.window_s, self.burst = window_s, burst
        self._win = {}          # kind → [窗口起點（單調）, 已送, 被併掉的]
        self.noted = 0          # 進來幾則（含被節流併掉的）
        self.acked = 0          # 地面站回執過幾則
        self.dropped = 0        # 緩衝滿了丟掉的最舊事件
        self.throttled = 0      # 被節流併進彙總的

    # ── 主迴圈這一側 ─────────────────────────────────────────────
    def note(self, kind, text, level="info", now=None):
        """記一則。**不阻塞、不送**：送是意圖通道那條執行緒的事。"""
        assert level in LEVELS, level
        now = time.monotonic() if now is None else now
        with self._lock:
            self.noted += 1
            self._flush_windows(now)
            if kind in THROTTLED:
                w = self._win.setdefault(kind, [now, 0, 0])
                if w[1] >= self.burst:
                    w[2] += 1
                    self.throttled += 1
                    return
                w[1] += 1
            self._push(kind, text, level, now)

    def _flush_windows(self, now):
        """窗口到期：被併掉的數量要**說出來**，不是默默歸零。"""
        for kind, w in list(self._win.items()):
            if now - w[0] < self.window_s:
                continue
            if w[2]:
                self._push(kind,
                           f"過去 {self.window_s / 60:.0f} 分鐘另有 {w[2]} 則"
                           f"同類事件沒有逐則送出（抖動中，節流）——現況看狀態列",
                           "info", now, throttled=w[2])
            del self._win[kind]

    def _push(self, kind, text, level, now, **extra):
        self._seq += 1
        self._q.append({"seq": self._seq, "boot": self.boot, "kind": kind,
                        "level": level, "text": text, "_m": now, **extra})
        while len(self._q) > self._max:
            self._q.popleft()
            self.dropped += 1

    def tick(self, now=None):
        """沒有新事件進來時，窗口也要能到期（彙總才送得出去）。"""
        now = time.monotonic() if now is None else now
        with self._lock:
            self._flush_windows(now)

    # ── 意圖通道這一側 ───────────────────────────────────────────
    def unsent(self, after_seq=0):
        """還沒回執、而且 seq 大於 `after_seq` 的，照順序。時間在這一刻換算。"""
        with self._lock:
            out = []
            for e in self._q:
                if e["seq"] <= after_seq:
                    continue
                d = {k: v for k, v in e.items() if k != "_m"}
                d["at"] = clock.iso(e["_m"])
                # 牆鐘還沒對過：時間照送（總比沒有好），但**要說它不可信**
                if not clock.synced():
                    d["at_unsynced"] = True
                out.append(d)
            return out

    def ack(self, boot, seq):
        """地面站回執到 `seq`。**累進**：同一條 TCP 上順序不會亂，
        回執到 N 就代表 N 以前的都到了。別的行程的回執不算數。"""
        if boot != self.boot:
            return
        with self._lock:
            while self._q and self._q[0]["seq"] <= seq:
                self._q.popleft()
                self.acked += 1

    def stats(self):
        with self._lock:
            return {"queued": len(self._q), "noted": self.noted,
                    "acked": self.acked, "dropped": self.dropped,
                    "throttled": self.throttled}
