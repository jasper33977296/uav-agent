"""誰還開著飛控序列埠（uav-system issues/059 B）。

2026-09-21：`get-gps.py` 帶起的 `mavsdk_server` 與代理同開 `/dev/ttyAMA0`，
兩邊各拿隨機片段，校正永遠跑不完而且不報錯。當時的答案要 ssh 上去
`fuser -v /dev/ttyAMA0` 才看得到——而**它就是答案本身**。

059 A 的 `TIOCEXCL` 擋掉非 root 之後的 `open()`，擋不住兩種：root，以及
**比代理早開的人**。這裡把那兩種找出來：掃 `/proc/*/fd`，看哪個 fd 指向
同一個裝置節點。

**看不完整時要說看不完整。** 別的使用者（尤其 root）的 `/proc/<pid>/fd` 要兩道門：
**列出目錄**是一般檔案權限（`CAP_DAC_READ_SEARCH`），**讀每個 fd 的連結**是 ptrace
讀權（`CAP_SYS_PTRACE`）。2026-09-22 上機：只給 PTRACE 時 158 個行程讀不到、root
開著的完全抓不到。那時「沒找到別人」不等於「沒有別人」，結果裡的 `unreadable`
就是為了讓呼叫端說得出這個差別（而它當時也確實說了）。
（unit 給兩個能力；`pi` 本來就有免密碼 sudo，沒有多給任何權力。）
"""
import os
import pwd

#: 命令列太長時截斷。mavsdk_server 那一行就兩百多字元，全列只會把畫面撐爆
CMD_MAX = 160


def _user(uid):
    try:
        return pwd.getpwuid(uid).pw_name
    except KeyError:
        return str(uid)


def _cmdline(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            raw = f.read()
    except OSError:
        return "?"
    # 換行與連續空白壓成一格：`python3 -c "…多行…"` 原樣放上畫面會把一列撐成一整塊
    cmd = " ".join(raw.replace(b"\0", b" ").decode(errors="replace").split())
    if not cmd:                                   # 核心執行緒／殭屍沒有命令列
        try:
            with open(f"/proc/{pid}/comm") as f:
                cmd = f"[{f.read().strip()}]"
        except OSError:
            cmd = "?"
    return cmd if len(cmd) <= CMD_MAX else cmd[:CMD_MAX - 1] + "…"


def scan(dev, exclude_pid=None, proc="/proc"):
    """回傳 `(holders, unreadable)`。

    `holders`：開著 `dev` 的**別的**行程，`[{"pid", "user", "cmd"}]`，依 pid 排序。
    `unreadable`：有幾個行程的 fd 表讀不到（權限不足）——**不是 0 的話，
    `holders` 是空的也不代表沒有人**。

    `dev` 先解開符號連結（`/dev/serial0` → `/dev/ttyAMA0`）：`/proc/*/fd/N`
    的連結目標是解開過的真名，拿原名去比會永遠比不到。
    """
    real = os.path.realpath(dev)
    holders, unreadable = [], 0
    try:
        pids = [p for p in os.listdir(proc) if p.isdigit()]
    except OSError:
        return [], -1                              # 連 /proc 都讀不到：完全不知道
    for p in pids:
        pid = int(p)
        if pid == exclude_pid:
            continue
        fddir = f"{proc}/{p}/fd"
        try:
            fds = os.listdir(fddir)
        except PermissionError:
            unreadable += 1
            continue
        except OSError:
            continue                               # 掃到一半行程結束了
        for fd in fds:
            try:
                if os.readlink(f"{fddir}/{fd}") == real:
                    break
            except OSError:
                continue
        else:
            continue
        try:
            uid = os.stat(f"{proc}/{p}").st_uid
        except OSError:
            continue
        holders.append({"pid": pid, "user": _user(uid), "cmd": _cmdline(pid)})
    holders.sort(key=lambda h: h["pid"])
    return holders, unreadable


def describe(holders):
    """給人看的一句：`pid 2637（pi）mavsdk_server --sysid 245 …`。"""
    return "；".join(f"pid {h['pid']}（{h['user']}）{h['cmd']}" for h in holders)
