#!/usr/bin/env python3
"""只有代理知道的事，送得到地面站（uav-system issues/057）。

以前 `Monitor.events` 18 處往裡寫、0 處讀——「代理接管期間做過什麼」只活在機上
記憶體裡。現在走 `outbox.Outbox` → 意圖通道的 `notice`。這份驗：

  * **送不出去時留著**、地面站回執才刪；沒回執的**下一條連線重送**
  * 地面站不收 `notice`（舊版）時不送——**看 hello 的 accepts，不用試的**
  * 緩衝有上限，滿了丟最舊的而且數得出來（`seq` 留缺號給地面站看）
  * `gs_link` 節流，但**被併掉的數量要說出來**；接管與失聯處置不節流
  * 時間記的是發生的時刻，不是送出的時刻

意圖通道用假的 WebSocket 跑真的 `_session`，不連任何地面站。

用法：python3 tools/notice-flow.py
"""
import collections
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

try:
    import websocket  # noqa: F401
except ImportError:
    print("skip：沒有 websocket-client。**skip ≠ pass**")
    sys.exit(2)

import intent as I  # noqa: E402
from datetime import datetime  # noqa: E402
from outbox import Outbox  # noqa: E402

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


print("── 緩衝與回執 ─────────────────────────────────────────")
o = Outbox()
o.note("takeover", "代理接管（失聯時 FLYING_MISSION）", "warning")
o.note("gs_link_action", "⚠ 地面失聯 30s → 下達 RTL", "critical")
u = o.unsent()
chk("記下的照順序排著，seq 從 1 起", [n["seq"] for n in u] == [1, 2])
chk("每則帶這個行程的 boot", all(n["boot"] == o.boot for n in u))
chk("送出去的東西不含內部欄位", all("_m" not in n for n in u))
o.ack("別的行程", 2)
chk("別的行程的回執不算數", len(o.unsent()) == 2, "代理重啟過的話 seq 會撞號")
o.ack(o.boot, 1)
chk("回執到 1 → 只刪 1", [n["seq"] for n in o.unsent()] == [2])
chk("`unsent(after_seq)` 跳過這條連線已送過的", o.unsent(after_seq=2) == [])

print("\n── 時間是發生的時刻 ─────────────────────────────────────")
o = Outbox()
o.note("takeover", "x", "warning", now=time.monotonic() - 120)
at = datetime.fromisoformat(o.unsent()[0]["at"])
age = time.time() - at.timestamp()
chk("兩分鐘前記下的，送出時仍說兩分鐘前", 119 < age < 121, f"{age:.1f}s")

print("\n── 上限 ───────────────────────────────────────────────")
o = Outbox(maxlen=3)
for i in range(5):
    o.note("takeover", f"#{i + 1}", "warning")
u = o.unsent()
chk("滿了丟最舊的", [n["text"] for n in u] == ["#3", "#4", "#5"])
chk("丟掉幾則數得出來", o.stats()["dropped"] == 2)
chk("seq 不重排——缺號留給地面站看", u[0]["seq"] == 3)

print("\n── 節流只對 gs_link，而且要說出來 ─────────────────────")
o = Outbox(window_s=600, burst=4)
t0 = 1000.0
for i in range(10):
    o.note("gs_link", f"抖 {i}", "warning", now=t0 + i)
o.note("takeover", "接管", "warning", now=t0 + 11)
o.note("gs_link_action", "下達 RTL", "critical", now=t0 + 12)
kinds = [n["kind"] for n in o.unsent()]
chk("gs_link 窗口內只送前 4 則", kinds.count("gs_link") == 4)
chk("接管與失聯處置不節流", "takeover" in kinds and "gs_link_action" in kinds)
chk("被併掉的有記數", o.stats()["throttled"] == 6)
o.tick(now=t0 + 599)
chk("窗口還沒到不彙總", len(o.unsent()) == 6)
o.tick(now=t0 + 601)
last = o.unsent()[-1]
chk("窗口到期送一則彙總", last["kind"] == "gs_link" and last.get("throttled") == 6,
    last["text"])
chk("彙總叫人看狀態列（事件流說的是抖過幾次，不是現在通不通）",
    "狀態列" in last["text"])
o.note("gs_link", "新窗口第一則", "info", now=t0 + 602)
chk("新窗口重新計數", o.unsent()[-1]["text"] == "新窗口第一則")


print("\n── 意圖通道：跑真的 _session ─────────────────────────")


class FakeWS:
    def __init__(self, inbox):
        self.inbox = collections.deque(inbox)
        self.sent = []

    def settimeout(self, t):
        pass

    def recv(self):
        if self.inbox:
            return json.dumps(self.inbox.popleft())
        time.sleep(0.01)
        raise I.websocket.WebSocketTimeoutException()

    def send(self, raw):
        self.sent.append(json.loads(raw))

    def close(self):
        pass


def session(link, inbox):
    ws = FakeWS(inbox)
    I.websocket.create_connection = lambda *a, **k: ws
    try:
        link._session()
    except TimeoutError:
        pass                       # 靜默逾時＝這個假地面站講完了
    return [m for m in ws.sent if m["type"] == "notice"]


def mk(outbox):
    return I.IntentLink("ws://fake", {"board_uid": "t"}, interval=0.05,
                        silence=0.3, outbox=outbox)


HELLO_OLD = {"type": "ack", "of": "hello", "accepts": ["state", "event", "ack"]}
HELLO_NEW = {"type": "ack", "of": "hello",
             "accepts": ["state", "event", "ack", "notice"]}

o = Outbox()
o.note("takeover", "代理接管", "warning")
link = mk(o)
chk("舊版地面站（accepts 沒有 notice）→ 一則都不送", session(link, [HELLO_OLD]) == [])
chk("沒送的還在緩衝", len(o.unsent()) == 1)

o = Outbox()
o.note("takeover", "代理接管", "warning")
o.note("gs_link_action", "下達 RTL", "critical")
link = mk(o)
sent = session(link, [HELLO_NEW])
chk("新版地面站 → 照順序送出", [n["seq"] for n in sent] == [1, 2])
chk("型別是 notice（不是意圖判決用的 event）", all(n["type"] == "notice" for n in sent))
chk("同一條連線上不重送", len(sent) == 2)
chk("**沒回執就不刪**", len(o.unsent()) == 2,
    "送得出去不代表送得到（TCP 只是塞進了核心緩衝）")

sent = session(link, [HELLO_NEW,
                      {"type": "ack", "of": "notice", "boot": o.boot, "seq": 2}])
chk("**下一條連線把沒回執的重送**", [n["seq"] for n in sent] == [1, 2])
chk("回執到 2 → 緩衝清空", o.unsent() == [] and o.stats()["acked"] == 2)

o.note("gs_link", "恢復", "info")
sent = session(link, [HELLO_NEW])
chk("回執過的不再送，只送新的", [n["seq"] for n in sent] == [3])
chk("通道狀態列說得出這台地面站收不收", link.stats()["notice_ok"] is True)

print("\n" + ("✓ 全部通過" if ok else "✗ 有失敗"))
sys.exit(0 if ok else 1)
