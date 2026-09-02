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

log = logging.getLogger(__name__)


class Recorder:
    """滾動式 tlog 錄製器。格式與地面站那份、QGC、`mavlogdump.py` 相同：
    每則訊息前綴 8-byte big-endian 微秒時間戳。"""

    def __init__(self, path: str, max_mb: int = 512, keep: int = 20,
                 armed_only: bool = True):
        self.dir = path
        self.max_bytes = max_mb * 1024 * 1024
        self.keep = keep
        self.armed_only = armed_only
        self.f = None
        self.name = None
        self.written = 0
        self.frames = 0
        self.dropped = 0
        self._warned = False

    # ── 對外 ────────────────────────────────────────────────────
    def feed(self, raw: bytes, armed: bool) -> None:
        """一個 MAVLink 框架。**這個函式不准拋例外**——它在轉發的熱路徑上。"""
        try:
            if self.armed_only and not armed:
                self._close()
                return
            if self.f is None:
                self._open()
            if self.f is None:
                return
            self.f.write(struct.pack(">Q", int(time.time() * 1e6)))
            self.f.write(raw)
            self.written += len(raw) + 8
            self.frames += 1
            if self.written >= self.max_bytes:
                self._close()
                self._rotate()
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
    def _open(self) -> None:
        os.makedirs(self.dir, exist_ok=True)
        # **檔名用開檔當下的時間**。這台 Pi 的 RTC 沒有電池，冷開機時牆鐘從
        # 1970 起算——所以檔名可能是 1970，而那**不是 bug，是誠實**：
        # 它記錄了「這份是在時鐘校正之前錄的」。改用單調時鐘命名反而會讓
        # 檔案之間失去可比的時間軸（見 issues/039 C 層的同一顆 RTC）
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
        log.info("機上錄製開始：%s（上限 %.0f MB，保留 %d 份）",
                 self.name, self.max_bytes / 1e6, self.keep)

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

    def _rotate(self) -> None:
        """留最新的 `keep` 份，其餘刪掉。

        **卡滿了會讓整台機出問題**——包括代理自己（寫不了 log、補傳緩衝也
        存不下）。所以滾動不是清潔工作，是**保護飛行路徑**。
        """
        try:
            files = sorted(f for f in os.listdir(self.dir) if f.endswith(".tlog"))
            for old in files[:-self.keep] if len(files) > self.keep else []:
                os.remove(os.path.join(self.dir, old))
                log.info("機上錄製滾動：刪除 %s", old)
        except Exception:
            log.exception("錄製滾動失敗（不影響轉發）")
