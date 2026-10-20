"""兩種時間，用途不能混。

量「過了多久」一律用單調時鐘。這台 Pi 的 RTC 沒有電池：換電池重開機時，
系統時間從上次存下的那一刻開始，直到對時才一次跳到正確時間（2026-09-14
實測跳了 4～20 分鐘）。拿牆鐘量間隔，對時那一刻就會量出一次假的
「地面站失聯 1160 秒」——代理因此重綁 UDP 埠、重連意圖通道，**自己把連線弄斷**。

標「發生在什麼時候」才用牆鐘，而且要等時鐘對過才可信。在那之前記下的東西
先記單調時間，送出去（或寫進檔案）的那一刻再換算——對時之後換算出來的就是對的。
"""
import os
import time
from datetime import datetime, timezone

#: systemd-timesyncd 對時成功後建立；在 /run 裡，重開機就沒了
SYNC_FLAG = "/run/systemd/timesync/synchronized"
#: 沒有 timesyncd 的機器（開發機）退回舊判準：時間至少是 2020-09-13 以後
CLOCK_SANE_EPOCH = 1_600_000_000


_kernel_cache = (0.0, None)


def _kernel_synced():
    """問核心時鐘有沒有被校正過（adjtimex）。**跟用哪一支對時程式無關**——chrony、
    ntpd、timesyncd 都會清掉核心的「未同步」狀態。問不到回 None。結果快取 5 秒"""
    global _kernel_cache
    now = time.monotonic()
    if now - _kernel_cache[0] < 5:
        return _kernel_cache[1]
    try:
        import ctypes
        buf = ctypes.create_string_buffer(256)      # struct timex；modes=0＝只讀
        state = ctypes.CDLL(None, use_errno=True).adjtimex(buf)
        res = None if state < 0 else state != 5      # 5＝TIME_ERROR：時鐘未同步
    except Exception:
        res = None
    _kernel_cache = (now, res)
    return res


def synced() -> bool:
    """牆鐘可不可信。**有 timesyncd 就只認它的旗標**：換電池重開時牆鐘是
    「上次存下的時間」，不是 1970，光看年份分不出它還沒對過。
    沒有 timesyncd（chrony 等）→ 問核心；問不到才退回看年份"""
    if os.path.isdir(os.path.dirname(SYNC_FLAG)):
        return os.path.exists(SYNC_FLAG)
    k = _kernel_synced()
    if k is not None:
        return k
    return time.time() >= CLOCK_SANE_EPOCH


def wall(m: float | None = None) -> float:
    """單調時間 `m`（不給＝現在）換算成牆鐘秒數。偏移在換算這一刻取。"""
    if m is None:
        return time.time()
    return time.time() - (time.monotonic() - m)


def iso(m: float | None = None) -> str:
    return datetime.fromtimestamp(wall(m), timezone.utc).isoformat()
