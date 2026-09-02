"""機上錄製的自動回傳（uav-system issues/014）。

錄下來的東西如果要人 `scp` 才拿得到，那它**只對記得去拿的人存在**——而想起來
要去拿的時機，通常是出事之後、而那時卡上的滾動可能已經把它刪掉了。這支把
「落地後取回」變成不需要人記得的事。

## 為什麼這件事做得成，而飛控 `.BIN` 回收做不成

同一個算術，方向相反。`.BIN` 大，是因為**它從來不過那條 57600 的線**；
機上 tlog 小，是因為**它一定過那條線**——它的大小上限就是
`UART 速率 × 飛行時間`：

    5.6 KB/s（57600 8N1）× 600 s ≈ 3.4 MB ＋ 每框架 8 byte 時間戳 ≈ **4 MB／十分鐘**

四 MB 走 5G 是幾秒到幾十秒的事。**所以「什麼時候傳」不是量的問題**——
是「它與遙測共用同一條 5G」的問題。

## 什麼時候傳（本模組的全部重點）

**只在地面傳，一解鎖立刻停。** 與補傳（backfill.py）同一條紀律：
**補歷史不該拖慢現在**。差別在於補傳一筆 100 bytes、傳了也不痛，而這裡一次
是幾 MB——飛行中送它，等於拿正在飛的那台機的遙測去換一份已經安全落地的紀錄。

四道門，缺一不傳：

1. **飛機沒有 armed。** 用最後已知值判——`armed` 只會被新的心跳改掉，
   所以**「飛控失聯時最後看到它是 armed」自然就會擋下上傳**，不必另外寫。
2. **落地後靜置 `settle_s` 秒。** 剛落地時人還在飛機旁邊、可能馬上再起飛；
   板凳測試時的 arm/disarm 連續切換也不該每次都戳一輪上傳。
3. **地面站鏈路是通的**（`gs_link_ok`）。斷線時重試只是把時間花在必敗的事上。
4. **拿得到 board_uid。** 沒有身分就沒有歸屬，地面站也不該替一個不認得的
   東西開目錄（issues/038／040 同一條規則）。

**檢查在每一塊之前做**，所以停手的延遲上限就是「送一塊要多久」——
預設塊大小與限速相同（256 KB／256 KB/s），也就是**最遲一秒後停手**。

## 三個刻意的設計

* **只傳關好的檔案。** 正在寫的那一份不碰（`recorder.name`）。機上的檔名
  **不會重複使用**（同秒換檔會加 `-N`），所以「關好的檔案」等於「內容
  不會再變的檔案」——這是續傳與 sha256 能夠成立的前提。
* **可續傳，而且不靠自己記。** 進度的權威在地面站：宣告一次就知道要從哪裡
  接。本地那份 `.uploaded.json` 只是**省下重算 sha 的快取**，掉了不會出錯，
  只會多問一次。5G 在戶外會斷，**一次斷線就整份重來的話，鏈路愈差的那趟
  愈傳不回來**——而那正是最值得看的一趟。
* **傳完不刪本地檔。** 保留仍由錄製器的滾動負責。剛傳完就刪掉唯一的副本，
  是把「地面站那份一定沒問題」當成了前提。
"""
import hashlib
import json
import logging
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request

log = logging.getLogger("uav-agent.uploader")

#: 沒事做的時候多久看一次目錄。**便宜**（一次 listdir），但沒有必要更頻繁：
#: 檔案是在落地那一刻才出現的，早幾秒晚幾秒沒有差別
SCAN_S = 10.0
#: 單一 HTTP 請求的逾時。5G 抖的時候一塊真的會送很久，而**逾時當成失敗重試
#: 是安全的**——續傳靠地面站回報的位移，重送不會重複寫入
HTTP_TIMEOUT_S = 60.0


class Uploader(threading.Thread):
    """把關好的機上錄製檔在地面時送回地面站。"""

    def __init__(self, directory, gs, port, board_uid_fn, current_fn, gate_fn,
                 rate_kbs=256, chunk_kb=256, settle_s=20.0, retry_s=30.0):
        super().__init__(daemon=True)
        self.dir = directory
        self.base = f"http://{gs}:{port}/api/onboard-captures"
        self.board_uid_fn = board_uid_fn
        self.current_fn = current_fn        # 正在寫的那一份（不碰）
        self.gate_fn = gate_fn              # () → (可以傳嗎, 為什麼不行)
        self.rate = max(1, rate_kbs) * 1024.0
        self.chunk = max(1, chunk_kb) * 1024
        self.settle_s = settle_s
        self.retry_s = retry_s
        self._stop = threading.Event()
        self._done = {}                     # 本地快取：檔名 → {bytes, sha256, ...}
        self._state_path = os.path.join(directory, ".uploaded.json")
        #: 上一次不能傳的理由（只在變化時記錄）。**初始不是 None**——
        #: None 的意思是「可以傳而且沒在傳」＝已同步，而剛啟動時我們還沒
        #: 判定過任何事。把「還沒看」說成「已同步」是這一頁最容易犯的謊
        self._blocked_since = "尚未判定"
        self._next_try = 0.0                # 單調刻度：失敗後不要每 10 秒重試
        self._ground_since = None           # 單調刻度：從什麼時候開始在地面
        self.current = None                 # 正在傳的檔名
        self.progress = None                # 0..1
        #: 關好了、還沒回傳成功的檔案。**快取值**：`pending()` 要 listdir，
        #: 而狀態快照是每拍送的（最高 5 Hz）——每拍掃一次目錄只是浪費。
        #: 由本執行緒每輪更新，**而且在守門之前更新**：這個數字最有用的時候
        #: 正是傳不動的時候
        self.backlog = 0
        self.backlog_files: list[str] = []
        #: 滾動刪掉時還沒回傳成功的檔案，等著報給地面站。**要報，因為那是
        #: 永久的損失**：機上的滾動優先於保住紀錄（卡滿了會讓整台機出問題），
        #: 但沒有人聽見的話，我們事後會以為那一趟的紀錄還在機上。
        #: 存進狀態檔＝報成功之前不會因為重開而消失
        self._lost: list[dict] = []
        self.files = 0
        self.bytes = 0
        self.failed = 0
        self.abandoned = 0                  # 滾動刪掉時還沒傳成功的份數
        self._load()

    # ── 對外 ────────────────────────────────────────────────────
    def stop(self):
        self._stop.set()

    def is_uploaded(self, name: str) -> bool:
        """這一份已經回傳成功了嗎（錄製器滾動刪檔前會問）。"""
        try:
            size = os.path.getsize(os.path.join(self.dir, name))
        except OSError:
            return name in self._done
        return self._done.get(name, {}).get("bytes") == size

    def note_pruned(self, name: str, size: int = 0) -> None:
        """錄製器滾動刪掉了一份**從來沒有回傳成功**的檔案。

        **這要變成地面站的一則事件，不是機上的一行 log**：journald 會被清掉，
        而事件流是留痕的地方。這裡只排隊，送在 `_pass()`（要鏈路通）。
        """
        self.abandoned += 1
        self._lost.append({"name": name, "bytes": size, "at": time.time()})
        self._save()
        log.warning("⚠ %s 已被滾動刪除且**從未回傳成功**——已排隊通報地面站", name)

    def state(self) -> dict:
        """送上意圖通道的那一份（協定 §4.2 的頂層欄位）。

        **只讀快取，不碰檔案系統**——這個函式在主迴圈的每一拍上跑。

        `blocked` 是「為什麼沒在傳」那句話的來源：**「沒有東西要傳」與
        「傳不動」都是「沒在傳」，但一個是完成、一個是故障**。只送份數的話，
        畫面分不出這兩件事，而它們要做的處置完全相反。
        """
        return {"pending": self.backlog,
                "pending_files": self.backlog_files[:20],
                "blocked": self._blocked_since,
                "current": self.current,
                "progress": (round(self.progress, 3)
                             if self.progress is not None else None),
                "abandoned": self.abandoned}

    def pending(self) -> list[str]:
        """關好了、還沒回傳成功的檔案，舊的排前面。"""
        cur = self.current_fn()
        try:
            names = sorted(f for f in os.listdir(self.dir)
                           if f.endswith(".tlog"))
        except OSError:
            return []
        out = []
        for n in names:
            if n == cur:
                continue
            try:
                size = os.path.getsize(os.path.join(self.dir, n))
            except OSError:
                continue
            if size == 0:
                continue
            # **「傳過了」要連大小一起對。** 檔名會被重複使用：滾動刪掉
            # `X.tlog` 之後，同一秒內開的下一份會挑回這個空出來的名字
            # （`_open()` 只避開「現在存在的」名字）。只比名字的話，
            # 那份新的內容會被當成已經傳過而**永遠不傳**
            if self._done.get(n, {}).get("bytes") == size:
                continue
            out.append(n)
        return out

    def stats(self) -> dict:
        return {"pending": self.backlog, "files": self.files,
                "bytes": self.bytes, "failed": self.failed,
                "abandoned": self.abandoned, "current": self.current,
                "progress": (round(self.progress, 3)
                             if self.progress is not None else None),
                "blocked": self._blocked_since}

    # ── 主迴圈 ──────────────────────────────────────────────────
    def run(self):
        log.info("錄製回傳：%s → %s（限速 %.0f KB/s、塊 %d KB、落地靜置 %.0fs）",
                 self.dir, self.base, self.rate / 1024, self.chunk // 1024,
                 self.settle_s)
        while not self._stop.wait(SCAN_S):
            try:
                self._pass()
            except Exception:
                # 這支絕不能把代理弄死：它送的是已經安全落地的紀錄，
                # 而代理身上還有飛安職責
                log.exception("錄製回傳這一輪出錯，續跑")

    def _pass(self):
        self.backlog_files = self.pending()
        self.backlog = len(self.backlog_files)
        ok, why = self._gate()
        if not ok:
            self.current = self.progress = None
            return
        self._report_lost()
        if time.monotonic() < self._next_try:
            return
        for name in self.pending():
            ok, why = self._gate()
            if not ok:
                return                      # 中途起飛：停在這裡，下次接續
            if not self._send(name):
                self.failed += 1
                self._next_try = time.monotonic() + self.retry_s
                return
        self.current = self.progress = None

    def _report_lost(self):
        """把「滾動刪掉了一份沒回傳成功的檔案」報上去。

        **報不成就留著下次再報**：這件事沒有時效——它描述的是一個已經發生、
        而且不可挽回的損失，晚幾分鐘知道不影響它的意義。
        """
        while self._lost:
            it = self._lost[0]
            body = json.dumps({"board_uid": self.board_uid_fn(),
                               "name": it["name"], "bytes": it["bytes"],
                               "at": it["at"]}).encode()
            req = urllib.request.Request(
                self.base + "/abandoned", data=body, method="POST",
                headers={"Content-Type": "application/json"})
            try:
                with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S):
                    pass
            except urllib.error.HTTPError as e:
                if e.code < 500:
                    # 400/404＝地面站不接受這一則（舊版沒有這個端點、
                    # 或不認得這塊板子）。**重試不會變好**，丟掉並說一次
                    log.warning("通報 %s 遺失被拒（HTTP %d）——放棄這一則",
                                it["name"], e.code)
                    self._lost.pop(0)
                    self._save()
                    continue
                return
            except Exception:
                return              # 鏈路問題，下一輪再說
            log.info("已通報地面站：%s 未回傳即被刪除", it["name"])
            self._lost.pop(0)
            self._save()

    def _gate(self):
        """可以傳嗎。**理由要說得出來**——不然「沒在傳」與「傳不動」同形。"""
        why = None
        if not self.board_uid_fn():
            why = "還沒拿到 board_uid"
        else:
            ok, reason = self.gate_fn()
            if not ok:
                why = reason
                self._ground_since = None
            else:
                now = time.monotonic()
                if self._ground_since is None:
                    self._ground_since = now
                left = self.settle_s - (now - self._ground_since)
                if left > 0:
                    why = f"落地靜置中（還有 {left:.0f}s）"
        if why != self._blocked_since:
            if why:
                log.info("錄製回傳暫停：%s", why)
            else:
                log.info("錄製回傳可以進行了")
            self._blocked_since = why
        return why is None, why

    # ── 單一檔案 ────────────────────────────────────────────────
    def _send(self, name) -> bool:
        path = os.path.join(self.dir, name)
        try:
            size = os.path.getsize(path)
            sha = self._sha256(path)
        except OSError as e:
            log.warning("讀不到 %s（%s）——跳過", name, e)
            return True                     # 不是鏈路的錯，不要因此退避

        try:
            r = self._offer(name, size, sha)
        except Exception as e:
            log.warning("宣告 %s 失敗：%s", name, e)
            return False
        stored, off = r.get("stored_as", name), int(r.get("have", 0))
        if r.get("complete"):
            self._mark(name, size, sha, stored, already=True)
            return True

        self.current = name
        log.info("開始回傳 %s（%.1f MB，從 %.1f MB 接續）",
                 name, size / 1e6, off / 1e6)
        t_start = time.monotonic()
        with open(path, "rb") as f:
            while off < size:
                ok, why = self._gate()
                if not ok:
                    log.info("回傳 %s 中止於 %.1f/%.1f MB：%s（下次接續）",
                             name, off / 1e6, size / 1e6, why)
                    self.current = self.progress = None
                    return True             # 不是失敗，不要退避
                f.seek(off)
                blk = f.read(self.chunk)
                t0 = time.monotonic()
                try:
                    res = self._chunk(stored, off, blk)
                except _Restart:
                    log.warning("地面站要求 %s 從頭重傳", name)
                    return False
                except _Offset as e:
                    log.info("位移對回來：我方 %d、地面站 %d", off, e.have)
                    off = e.have
                    continue
                except Exception as e:
                    log.warning("回傳 %s 的第 %d byte 起這一塊失敗：%s",
                                name, off, e)
                    return False
                off = int(res.get("have", off + len(blk)))
                self.bytes += len(blk)
                self.progress = off / size if size else None
                if res.get("complete"):
                    break
                self._throttle(len(blk), time.monotonic() - t0)

        self._mark(name, size, sha, stored)
        log.info("回傳完成 %s → %s（%.1f MB，%.0f 秒）", name, stored,
                 size / 1e6, time.monotonic() - t_start)
        self.files += 1
        self.current = self.progress = None
        return True

    def _throttle(self, sent, elapsed):
        """限速。**地面站那條 5G 上可能還有別台機正在飛**，而正在飛的那台優先。"""
        want = sent / self.rate
        if want > elapsed:
            self._stop.wait(want - elapsed)

    # ── HTTP ────────────────────────────────────────────────────
    def _offer(self, name, size, sha) -> dict:
        body = json.dumps({"board_uid": self.board_uid_fn(), "name": name,
                           "bytes": size, "sha256": sha}).encode()
        req = urllib.request.Request(
            self.base + "/offer", data=body, method="POST",
            headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S) as r:
            return json.loads(r.read() or b"{}")

    def _chunk(self, stored, offset, blk) -> dict:
        q = urllib.parse.urlencode({"board_uid": self.board_uid_fn(),
                                    "name": stored, "offset": offset})
        req = urllib.request.Request(
            f"{self.base}/chunk?{q}", data=blk, method="PUT",
            headers={"Content-Type": "application/octet-stream"})
        try:
            with urllib.request.urlopen(req, timeout=HTTP_TIMEOUT_S) as r:
                return json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:
            detail = _detail(e)
            if e.code == 409 and isinstance(detail, dict) and "have" in detail:
                raise _Offset(int(detail["have"]))
            if e.code in (404, 422):
                # 宣告不見了或校驗不符——**兩者都要從 0 重來**，而重來的入口
                # 是重新宣告，所以這裡回失敗讓下一輪重跑整條路
                raise _Restart()
            if e.code == 507:
                log.warning("地面站空間不足：%s", detail)
            raise

    # ── 本地狀態 ────────────────────────────────────────────────
    def _mark(self, name, size, sha, stored, already=False):
        self._done[name] = {"bytes": size, "sha256": sha, "stored_as": stored,
                            "at": time.time()}
        self._save()
        self.backlog_files = [n for n in self.backlog_files if n != name]
        self.backlog = len(self.backlog_files)
        if already:
            log.info("%s 地面站已經有了（sha 相符）——不重傳", name)

    def _load(self):
        try:
            with open(self._state_path) as f:
                blob = json.load(f)
        except (OSError, ValueError):
            blob = {}
        # 舊格式（v0.6.0）整包就是 done 表。**認得舊格式**，不然升級當下
        # 已經傳過的檔案會全部重算一次 sha 並重問地面站
        if "done" in blob or "lost" in blob:
            self._done = blob.get("done") or {}
            self._lost = blob.get("lost") or []
        else:
            self._done, self._lost = blob, []

    def _save(self):
        try:
            tmp = self._state_path + ".tmp"
            with open(tmp, "w") as f:
                json.dump({"done": self._done, "lost": self._lost}, f)
            os.replace(tmp, self._state_path)     # 原子換檔
        except OSError:
            log.warning("寫不了回傳狀態檔——**不影響正確性**，"
                        "只會在重開後重算一次 sha 並多問地面站一次")

    @staticmethod
    def _sha256(path) -> str:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for blk in iter(lambda: f.read(1 << 20), b""):
                h.update(blk)
        return h.hexdigest()


class _Offset(Exception):
    def __init__(self, have):
        super().__init__(have)
        self.have = have


class _Restart(Exception):
    pass


def _detail(e: urllib.error.HTTPError):
    try:
        return json.loads(e.read() or b"{}").get("detail")
    except Exception:
        return None
