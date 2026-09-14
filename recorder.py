"""機上原始層錄製：把飛控送出的每一個 MAVLink 框架落盤（uav-system issues/014）。

## 為什麼在機上也錄一份

地面站已經在錄 tlog 了（`apps/backend/app/capture.py`）。但那份是
**「送到地面站的東西」**，而這份是**「飛控送出的東西」**——**5G 斷線期間，
這兩者相差的正是我們最想看的那一段**。

這與補傳（issues/039 C 層）是同一個道理，差別在範圍：補傳補的是 telemetry
表那幾個欄位（一筆約 100 bytes），這裡保留的是**整條原始流**，不挑、不解讀。

**成本幾乎是零**：代理本來就逐 byte 讀過每一個框架（轉發要用），這裡只是多
寫一份檔案。**零額外 UART 頻寬、零額外 5G 頻寬**。

## 為什麼不是「把飛控的 .BIN 拉下來」

飛控的 dataflash `.BIN`（`LOG_BACKEND_TYPE=1`，寫在飛控自己的 SD 卡）帶著
MAVLink 從來不送的東西——數百 Hz 的原始 IMU、控制器內部量。**那份過不了
57600 的序列埠**：ArduPilot 的 MAVLink logging backend（`LOG_BACKEND_TYPE`
bit 1）就是為伴飛電腦設計的，但它推的是同一份高頻資料，在這條線上會爆緩衝
然後丟塊。

**所以這支錄的是「MAVLink 看得到的一切」，不是「飛控知道的一切」**——
這個界線要講清楚，否則它會被當成後者。

## 三條紀律

1. **落盤失敗絕不拖垮轉發。** 錄製是附加價值，飛控↔地面站那條路不是。
2. **容量有上限而且會滾動。** Pi 的卡滿了會讓整台機出問題——包括代理自己。
3. **只在 armed 時錄**（可設）。待機時的資料量大而價值低，與飛控自己
   `LOG_DISARMED=0` 的選擇一致。
"""
import logging
import os
import struct
import time

import clock

log = logging.getLogger(__name__)


class Recorder:
    """滾動式 tlog 錄製器。格式與地面站那份、QGC、`mavlogdump.py` 相同：
    每則訊息前綴 8-byte big-endian 微秒時間戳。"""

    def __init__(self, path: str, max_mb: int = 512, keep: int = 20,
                 armed_only: bool = True, is_uploaded=None, on_discard=None,
                 rotate_min: float = 30.0):
        self.dir = path
        self.max_bytes = max_mb * 1024 * 1024
        self.keep = keep
        self.armed_only = armed_only
        #: 「這一份回傳成功了沒」（uploader.is_uploaded）。**只用來把滾動刪除
        #: 說出口**，不用來延後刪除——滾動的職責是保護那張卡，而卡滿了會讓
        #: 整台機出問題。但**刪掉一份從來沒有回傳成功的紀錄，一定要有人聽見**
        self.is_uploaded = is_uploaded
        #: 刪掉一份沒回傳成功的檔案時呼叫（uploader.note_pruned）。
        #: **log 不夠**：journald 會被清掉，而這是一個不可挽回的損失
        self.on_discard = on_discard
        #: 待機時多久換一次檔（分鐘）。**只在未解鎖時輪替**——飛行中換檔會
        #: 破壞「一趟飛行一個檔案」那個性質，而那是回放與比對的前提。
        #:
        #: 為什麼非有不可：檔案是在「解鎖→上鎖」那個邊界關的，而**開機就錄**
        #: 之後，一台從頭到尾沒解鎖的機永遠不會有那個邊界，檔案要寫滿
        #: `max_mb` 才關。實測 3.8 KB/s → 512 MB 要 37 小時，也就是待機
        #: 一整天那份錄製永遠不關、**永遠不會被自動回傳**（沒關好的檔案
        #: 內容還會變，sha256 與續傳都無從談起）。
        self.rotate_s = max(0.0, rotate_min * 60.0)
        self._opened_at = 0.0
        self._was_armed = False
        self.f = None
        self.name = None
        self.written = 0
        self.frames = 0
        self.dropped = 0
        self._warned = False
        #: 時鐘對過之前收到的框架：(單調時間, 框架)。對時後補寫、時間換算成牆鐘
        self._pre: list = []
        self._pre_bytes = 0
        self._gave_up = False

    # ── 對外 ────────────────────────────────────────────────────
    def feed(self, raw: bytes, armed: bool) -> None:
        """一個 MAVLink 框架。**這個函式不准拋例外**——它在轉發的熱路徑上。"""
        try:
            # **時鐘對過之前先不寫檔**：那段的牆鐘是錯的（換電池重開時是「上次
            # 存下的時間」），寫進去的時間戳對不上地面站。先記單調時間，對時後
            # 換算補寫——檔名也因此是對的時間
            if not (clock.synced() or self._gave_up):
                if not self.armed_only or armed:
                    self._hold(raw)
                return
            # **落地即換檔＝一趟飛行一個檔案。** armed_only 模式下這是順帶
            # 發生的（下面那行就會關檔），但 RECORD_DISARMED=1 時不換的話，
            # 檔案要等 512 MB 才會關——**而沒關好的檔案不會被自動回傳**
            # （正在寫的那一份內容還會變，sha256 與續傳都無從談起），
            # 於是「連未解鎖也錄」會安靜地連帶關掉自動回傳
            if self._was_armed and not armed:
                self._close()
            self._was_armed = armed
            if self.armed_only and not armed:
                self._close()
                return
            # **時間輪替只在未解鎖時做。** 飛行中換檔會把一趟飛行切成兩份，
            # 而「一趟飛行一個檔案」是回放與兩層比對的前提（014）。
            # 用單調時鐘：這台 Pi 的 RTC 沒有電池，牆鐘開機後會大幅跳一次
            if (not armed and self.f is not None and self.rotate_s
                    and time.monotonic() - self._opened_at >= self.rotate_s):
                self._close()
            if self.f is None:
                self._open()
            if self.f is None:
                return
            if self._pre:
                self._write_pre()
            self.f.write(struct.pack(">Q", int(time.time() * 1e6)))
            self.f.write(raw)
            self.written += len(raw) + 8
            self.frames += 1
            if self.written >= self.max_bytes:
                self._close()       # 下一個框架會開新檔，滾動在那裡做
        except Exception:
            # **只在第一次抱怨**：錄製壞掉時每個框架噴一行 log，會把真正的
            # 問題埋掉，而且 log 本身也會吃掉那張卡
            self.dropped += 1
            if not self._warned:
                self._warned = True
                log.exception("機上錄製失敗——**轉發不受影響**，但這段沒有錄到")

    def stats(self) -> dict:
        return {"file": self.name, "bytes": self.written,
                "frames": self.frames, "dropped": self.dropped}

    def flush(self) -> None:
        if self.f is not None:
            try:
                self.f.flush()
                os.fsync(self.f.fileno())
            except Exception:
                pass

    # ── 內部 ────────────────────────────────────────────────────
    #: 等對時最多先壓多少（位元組）。約 3.8 KB/s 的流量下是半個多小時；
    #: 等不到就照當下牆鐘開始寫——**錄不到比時間戳不準糟**
    PRE_MAX = 8 * 1024 * 1024

    def _hold(self, raw: bytes) -> None:
        self._pre.append((time.monotonic(), raw))
        self._pre_bytes += len(raw)
        if self._pre_bytes > self.PRE_MAX:
            self._gave_up = True
            log.warning("時鐘一直沒有對上（已壓 %.1f MB）——照牆鐘開始寫，"
                        "這份錄製的時間戳可能對不上地面站", self._pre_bytes / 1e6)

    def _write_pre(self) -> None:
        for m, raw in self._pre:
            self.f.write(struct.pack(">Q", int(clock.wall(m) * 1e6)))
            self.f.write(raw)
            self.written += len(raw) + 8
            self.frames += 1
        log.info("補寫對時前的 %d 個框架（時間已換算）", len(self._pre))
        self._pre, self._pre_bytes = [], 0

    def _open(self) -> None:
        self._opened_at = time.monotonic()
        os.makedirs(self.dir, exist_ok=True)
        # **檔名用開檔當下的時間**。開檔一定在時鐘對過之後（見 feed），
        # 除非一直等不到對時才照當下牆鐘開檔——那時檔名本身就說明了這件事
        base = time.strftime("%Y%m%d-%H%M%S", time.gmtime())
        # **同一秒內換檔要換名字。** 檔名只有秒解析度，而換檔可能在同一秒內
        # 發生（檔案填得快、或上限設得小）——同名會用 `ab` 接回原檔，
        # **看起來像換了檔，實際上還在寫同一個**，滾動也就永遠只看到一份。
        # 這是測試逼出來的：8 次換檔全落在同一秒，目錄裡只剩一個檔。
        self.name, n = f"{base}.tlog", 1
        while os.path.exists(os.path.join(self.dir, self.name)):
            n += 1
            self.name = f"{base}-{n}.tlog"
        self.f = open(os.path.join(self.dir, self.name), "ab", buffering=1 << 16)
        self.written = 0
        log.info("機上錄製開始：%s（換檔：落地時、寫滿 %.0f MB%s；保留 %d 份）",
                 self.name, self.max_bytes / 1e6,
                 f"、或待機滿 {self.rotate_s / 60:.0f} 分鐘"
                 if self.rotate_s else "",
                 self.keep)
        # **滾動在開檔時做，不是只在「寫滿 512 MB 換檔」時做。** 原本掛在
        # 換檔那條路上，而在真機上換檔幾乎不會發生：一趟十分鐘的飛行約 4 MB
        # （UART 5.6 KB/s 的上限），512 MB 要飛二十小時才寫得滿。於是
        # 「保留 20 份」從來沒有執行過——檔案一趟一份地無限累積，
        # **而部署查核表上寫著「約 10 GB 上限」。那句話當時是假的。**
        self._prune()

    def _close(self) -> None:
        if self.f is None:
            return
        try:
            self.flush()
            self.f.close()
            log.info("機上錄製結束：%s（%.1f MB、%d 個框架）",
                     self.name, self.written / 1e6, self.frames)
        except Exception:
            pass
        self.f, self.name = None, None

    def _prune(self) -> None:
        """留最新的 `keep` 份，其餘刪掉。

        **卡滿了會讓整台機出問題**——包括代理自己（寫不了 log、補傳緩衝也
        存不下）。所以滾動不是清潔工作，是**保護飛行路徑**。

        **自動回傳不會讓刪除變慢。** 保護那張卡的優先權高於保住一份紀錄；
        但**刪掉一份從來沒有回傳成功的東西，一定要有人聽見**——安靜地刪掉，
        我們就會在事後以為那趟飛行的紀錄還在機上。
        """
        try:
            # **依修改時間排，不是依檔名。** 檔名不是字典序遞增的：同一秒的
            # 第二份叫 `X-2.tlog`，而 `-`(0x2d) < `.`(0x2e)，所以 `X-2.tlog`
            # 排在 `X.tlog` **前面**——依檔名排會把最舊的那一份當成最新的、
            # 留著它而刪掉真正新的。冷開機的 1970 檔名也是同一顆時鐘寫的
            # mtime，兩者一致
            files = [e.name for e in sorted(
                (e for e in os.scandir(self.dir) if e.name.endswith(".tlog")),
                key=lambda e: e.stat().st_mtime)]
            for old in files[:-self.keep] if len(files) > self.keep else []:
                # **正在寫的那一份絕不刪。** 冷開機時檔名是 1970，排序上
                # 會排在所有 2026 的檔案前面＝被當成最舊的一份；unlink 一個
                # 開著的檔在 Linux 上不會報錯，只會讓後續的寫入流進一個
                # 沒有名字的 inode——**看起來一切正常，錄的東西全部消失**
                if old == self.name:
                    continue
                path = os.path.join(self.dir, old)
                if self.is_uploaded is not None and not self.is_uploaded(old):
                    log.warning("⚠ 機上錄製滾動刪除 %s，而它**從來沒有回傳"
                                "成功**——這一趟的機上紀錄到此為止", old)
                    if self.on_discard is not None:
                        try:
                            self.on_discard(old, os.path.getsize(path))
                        except Exception:
                            log.exception("通報遺失失敗（不影響滾動）")
                os.remove(path)
                log.info("機上錄製滾動：刪除 %s", old)
        except Exception:
            log.exception("錄製滾動失敗（不影響轉發）")
