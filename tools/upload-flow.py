#!/usr/bin/env python3
"""機上錄製的自動回傳（uav-system issues/014）。

**這支測的是「什麼時候傳」，不是「傳得動嗎」。** 傳得動是幾行 HTTP；
危險的那一格是時機——飛行中送幾 MB，等於拿正在飛的那台機的遙測，
去換一份已經安全落地的紀錄。

七件事：
  1. **飛行中不傳**（armed ＝守門關上）
  2. **傳到一半起飛 → 停在原地**，而且下次從那裡接續（不是重來）
  3. **正在寫的那一份不碰**（內容還會變，sha256 與續傳都無從談起）
  4. 續傳的權威在地面站：**本地狀態檔掉了也不會重傳整份**
  5. 地面站回 409（位移不符）→ 自己對回來，不是整份重來
  6. 限速真的有生效
  7. **滾動刪掉沒傳成功的檔案時要大聲說**——安靜地刪，我們就會以為它還在

地面站那一端用真的協定、假的實作（`http.server`）：這裡要驗的是機端的
決策，backend 那半由 uav-system 的 `scripts/test-onboard-upload.py` 走真的。

用法：python3 tools/upload-flow.py
"""
import hashlib
import http.server
import json
import os
import sys
import tempfile
import threading
import time
import urllib.parse

sys.path.insert(0, "/home/k200/uav-agent")

import recorder    # noqa: E402
import uploader    # noqa: E402

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


# ── 假地面站：實作 offer/chunk 的協定 ────────────────────────────
class GS(http.server.BaseHTTPRequestHandler):
    store = {}          # stored_as → {"bytes", "sha256", "data": bytearray}
    fail_next = None    # 下一次 chunk 要回什麼錯（(code, body)）

    def log_message(self, *a):
        pass

    def _json(self, code, obj):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    lost = []           # 收到的「未回傳即被刪除」通報

    def do_POST(self):
        n = int(self.headers.get("Content-Length", 0))
        body = json.loads(self.rfile.read(n) or b"{}")
        if self.path.endswith("/abandoned"):
            if GS.fail_next:
                code, obj = GS.fail_next
                GS.fail_next = None
                return self._json(code, {"detail": obj})
            GS.lost.append(body["name"])
            return self._json(200, {"ok": True, "noted": True})
        name, sha = body["name"], body["sha256"]
        for k, v in GS.store.items():
            if v["sha256"] == sha:
                return self._json(200, {"stored_as": k, "have": len(v["data"]),
                                        "complete": len(v["data"]) == v["bytes"]})
        GS.store[name] = {"bytes": body["bytes"], "sha256": sha,
                          "data": bytearray()}
        self._json(200, {"stored_as": name, "have": 0, "complete": False})

    def do_PUT(self):
        q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
        name, off = q["name"][0], int(q["offset"][0])
        blk = self.rfile.read(int(self.headers.get("Content-Length", 0)))
        if GS.fail_next:
            code, obj = GS.fail_next
            GS.fail_next = None
            return self._json(code, {"detail": obj})
        e = GS.store[name]
        if off != len(e["data"]):
            return self._json(409, {"detail": {"code": "offset_mismatch",
                                               "have": len(e["data"])}})
        e["data"] += blk
        self._json(200, {"have": len(e["data"]),
                         "complete": len(e["data"]) == e["bytes"]})


srv = http.server.ThreadingHTTPServer(("127.0.0.1", 0), GS)
threading.Thread(target=srv.serve_forever, daemon=True).start()
PORT = srv.server_address[1]

FRAME = bytes([0xFD, 4, 0, 0, 7, 1, 1, 0, 0, 0]) + b"\xde\xad\xbe\xef" + b"\x00\x00"
d = tempfile.mkdtemp()

state = {"armed": False, "link": True, "current": None}


def gate():
    if state["armed"]:
        return False, "armed（飛行中不傳）"
    if not state["link"]:
        return False, "地面站失聯"
    return True, None


def make(rate=1 << 20, chunk=1 << 20, settle=0.0):
    return uploader.Uploader(d, "127.0.0.1", PORT, lambda name: ("board-test", None),
                             lambda: state["current"], gate,
                             rate_kbs=rate // 1024, chunk_kb=chunk // 1024,
                             settle_s=settle)


def write_file(name, size):
    with open(os.path.join(d, name), "wb") as f:
        f.write(os.urandom(size))
    return hashlib.sha256(open(os.path.join(d, name), "rb").read()).hexdigest()


print("── 1. 飛行中不傳 ──────────────────────────────────────")
sha_a = write_file("20260902-100000.tlog", 300_000)
u = make()
state["armed"] = True
u._pass()
chk("**armed 時一個 byte 都沒送出去**", not GS.store, list(GS.store))
chk("但它知道有東西在排隊（backlog 不是 0）", u.backlog == 1, u.stats())
chk("而且說得出為什麼", u.stats()["blocked"] == "armed（飛行中不傳）",
    u.stats()["blocked"])

print("\n── 2. 落地就傳，而且驗得出是同一份 ─────────────────────")
state["armed"] = False
u._pass()
got = GS.store.get("20260902-100000.tlog")
chk("整份送到了", got and len(got["data"]) == 300_000,
    got and len(got["data"]))
chk("**內容逐 byte 相同**（sha256 對得上）",
    got and hashlib.sha256(bytes(got["data"])).hexdigest() == sha_a)
chk("計數對得上", u.files == 1 and u.bytes == 300_000, u.stats())

print("\n── 3. 已經傳過的不再傳（本地狀態檔）────────────────────")
before = len(GS.store["20260902-100000.tlog"]["data"])
u._pass()
chk("沒有重送", len(GS.store["20260902-100000.tlog"]["data"]) == before)
chk("backlog 歸零", u.backlog == 0, u.stats())

print("\n── 4. 本地狀態檔掉了也不重傳整份（權威在地面站）─────────")
os.remove(os.path.join(d, ".uploaded.json"))
u2 = make()
chk("重開後認得它還在排隊", u2.pending() == ["20260902-100000.tlog"],
    u2.pending())
u2._pass()
chk("**地面站說「我有了」→ 一個 byte 都不重送**",
    len(GS.store["20260902-100000.tlog"]["data"]) == before)
chk("而且記回本地了", u2.is_uploaded("20260902-100000.tlog"))

print("\n── 5. 傳到一半起飛 → 停在原地，下次接續 ────────────────")
sha_b = write_file("20260902-110000.tlog", 400_000)
u3 = make(rate=100_000, chunk=100_000)     # 一塊 100 KB


def arm_after(delay):
    time.sleep(delay)
    state["armed"] = True


threading.Thread(target=arm_after, args=(1.2,), daemon=True).start()
u3._pass()
state["armed"] = False
part = GS.store.get("20260902-110000.tlog")
sent = len(part["data"]) if part else 0
chk("**停在半路**（不是整份送完）", 0 < sent < 400_000, sent)
chk("停手的延遲上限就是一塊的時間（≤2 塊）", sent <= 200_000, sent)
chk("本地沒有把它記成傳完", not u3.is_uploaded("20260902-110000.tlog"))
u3._pass()
chk("**下一輪從那裡接續、整份完成**",
    len(GS.store["20260902-110000.tlog"]["data"]) == 400_000)
chk("而且接得對（sha256 相符）",
    hashlib.sha256(bytes(GS.store["20260902-110000.tlog"]["data"])).hexdigest()
    == sha_b)

print("\n── 6. 正在寫的那一份不碰 ──────────────────────────────")
write_file("20260902-120000.tlog", 50_000)
state["current"] = "20260902-120000.tlog"
u4 = make()
chk("**不在待傳清單裡**（內容還會變）", u4.pending() == [], u4.pending())
state["current"] = None
chk("關檔之後才進清單", u4.pending() == ["20260902-120000.tlog"], u4.pending())

print("\n── 7. 位移不符：對回來，不是整份重來 ───────────────────")
GS.store["20260902-120000.tlog"] = {"bytes": 50_000, "sha256": "x" * 64,
                                    "data": bytearray(b"\x00" * 10_000)}
GS.fail_next = (409, {"code": "offset_mismatch", "have": 10_000})
u5 = make(rate=1 << 20, chunk=20_000)
u5._pass()
chk("**接在地面站說的位移之後**，總量剛好一份",
    len(GS.store["20260902-120000.tlog"]["data"]) == 50_000,
    len(GS.store["20260902-120000.tlog"]["data"]))

print("\n── 8. 限速真的有生效 ──────────────────────────────────")
write_file("20260902-130000.tlog", 400_000)
u6 = make(rate=200_000, chunk=100_000)     # 200 KB/s → 400 KB 要約 2 秒
t0 = time.monotonic()
u6._pass()
dt = time.monotonic() - t0
chk("400 KB 在 200 KB/s 下花了 ≥1.5 秒（不是一口氣灌出去）", dt >= 1.5,
    f"{dt:.2f}s")

print("\n── 9. 滾動刪掉沒傳成功的檔案要大聲說 ───────────────────")
d2 = tempfile.mkdtemp()
open(os.path.join(d2, "20260101-000000.tlog"), "wb").write(b"x" * 10)
said = []
r = recorder.Recorder(d2, max_mb=0, keep=1,
                      is_uploaded=lambda n: False)
import logging  # noqa: E402
logging.getLogger("recorder").addHandler(logging.NullHandler())
orig = recorder.log.warning
recorder.log.warning = lambda *a, **k: said.append(a[0] % a[1:])
r.feed(FRAME, armed=True)
r.feed(FRAME, armed=True)
recorder.log.warning = orig
chk("**刪掉一份從來沒有回傳成功的紀錄時，有人聽得見**",
    any("從來沒有回傳" in m for m in said), said)

print("\n── 10. 送上去的狀態說得出「為什麼沒在傳」──────────────")
state["armed"] = True
write_file("20260902-140000.tlog", 120_000)
u7 = make()
u7._pass()
st = u7.state()
chk("份數與檔名都有（清單才列得出「只在機上」那幾列）",
    st["pending"] == 1 and st["pending_files"] == ["20260902-140000.tlog"], st)
chk("**blocked 帶著理由**（只送份數的話，「沒東西要傳」與「傳不動」同形）",
    st["blocked"] == "armed（飛行中不傳）", st["blocked"])
state["armed"] = False
u7._pass()
chk("傳完之後 blocked 是 None、份數歸零", u7.state()["blocked"] is None
    and u7.state()["pending"] == 0, u7.state())
u8 = make()
chk("**剛啟動時不是「已同步」**（還沒判定過任何事）",
    u8.state()["blocked"] == "尚未判定", u8.state()["blocked"])

print("\n── 11. 未回傳即被刪除：報上去，而且報不成不會忘記 ──────")
GS.lost.clear()
u9 = make()
GS.fail_next = (503, "地面站掛了")
u9.note_pruned("20260101-000000.tlog", 4_000_000)
u9._pass()
chk("**報不成就留著**（那是一個不可挽回的損失，晚幾分鐘知道無妨）",
    not GS.lost and len(u9._lost) == 1, (GS.lost, u9._lost))
u10 = make()
chk("而且重開之後還記得（進了狀態檔）", len(u10._lost) == 1, u10._lost)
u10._pass()
chk("地面站回來就報成功了", GS.lost == ["20260101-000000.tlog"], GS.lost)
chk("報過就不再報", not u10._lost, u10._lost)
chk("計數也對", u10.state()["abandoned"] == 0 and u9.abandoned == 1,
    (u9.abandoned, u10.state()["abandoned"]))

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
srv.shutdown()
sys.exit(0 if ok else 1)
