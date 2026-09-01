"""這份驅動是從哪裡搬來的、現在是不是還一樣（026 B4-b）。

**放在獨立檔案而不是加在驅動的檔頭**，理由只有一個：讓那四支檔案與上游
`uav-system/libs/autopilot/` **逐字相同**，`diff -r` 出零差異。加一行檔頭就
永遠 diff 得到東西，於是「有沒有漂移」這個問題再也不能用 diff 回答。

搬家不是改寫（架構文件 §9.3）：唯一的驗收標準是新舊輸出完全相同，所以
這裡不改任何一行——**包括看起來像 bug 的**。目前已知有一處：
`ardupilot.py` 的 `GCS_SYSID = 254` 附註「與 mav.GCS_SYSID 一致」，而
`apps/command/app/mav.py` 已於 2026-08-25 改成 255（`capabilities.py` 跟著改了，
這份沒有，而地面站的啟動檢查只比對前兩者）。代理不使用這個常數，所以留著
不影響行為；已回報上游，**修要修在上游**——在這裡順手改掉會製造一個
B4-d 比不到的真實分歧（沒有任何測試比對 GCS_SYSID）。

版本字串的組成：上游 commit ＋ 本地四支檔案的內容雜湊。
**內容雜湊才是真正有用的那半**——commit 說的是「我從哪抄的」，雜湊說的是
「我現在手上這份是什麼」。機隊裡兩台機報出不同雜湊＝驅動漂移了
（026 待決點 6），而那正是本批要讓它「可見」的東西。
"""
import hashlib
from pathlib import Path

#: 搬過來時的上游 commit（uav-system）
PORTED_FROM = "6d6ac73"
#: 納入雜湊的檔案。**改這份清單要想清楚**：漏掉一支，那支的漂移就看不見了
_FILES = ("__init__.py", "driver.py", "ardupilot.py", "px4.py")


def content_hash() -> str:
    """四支驅動檔案的內容雜湊。讀不到檔案時回 "unknown"——**不要回空字串
    或假的雜湊**，那會讓兩台讀不到檔的機看起來版本一致。"""
    h = hashlib.sha256()
    here = Path(__file__).resolve().parent
    try:
        for name in _FILES:
            h.update((here / name).read_bytes())
    except OSError:
        return "unknown"
    return h.hexdigest()[:12]


def version() -> str:
    return f"{PORTED_FROM}+{content_hash()}"
