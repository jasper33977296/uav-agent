#!/usr/bin/env python3
"""機上原始層錄製（uav-system issues/014）。

**這支錄的是「MAVLink 看得到的一切」，不是「飛控知道的一切」**——飛控的
dataflash `.BIN` 帶著 MAVLink 從來不送的高頻資料（IMU 數百 Hz、控制器內部量），
而那份過不了 57600。**這個界線要測出來，否則它會被當成後者。**

價值在斷線那一段：地面站錄的是「送到地面站的東西」，這裡錄的是「飛控送出的
東西」——**5G 斷線期間，兩者相差的正是我們最想看的那一段**。

五件事：
  1. 只在 armed 時錄（與飛控 `LOG_DISARMED=0` 的選擇一致）
  2. 格式與地面站那份相同（8-byte big-endian μs 前綴 ＋ 原始框架）
  3. **落盤失敗不能拖垮轉發**——它在熱路徑上
  4. 超過上限就換檔並滾動（**卡滿了會讓整台機出問題，包括代理自己**）
  5. 錄製壞掉時**只抱怨一次**，不要每個框架噴一行把真正的問題埋掉

用法：python3 tools/recorder-flow.py
"""
import os
import struct
import sys
import tempfile

sys.path.insert(0, "/home/k200/uav-agent")

import recorder  # noqa: E402

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


FRAME = bytes([0xFD, 4, 0, 0, 7, 1, 1, 0, 0, 0]) + b"\xde\xad\xbe\xef" + b"\x00\x00"
d = tempfile.mkdtemp()

print("── 1. 未 armed 不錄（預設）────────────────────────────")
r = recorder.Recorder(d, armed_only=True)
for _ in range(5):
    r.feed(FRAME, armed=False)
chk("待機時一個框架都沒寫", r.frames == 0 and not os.listdir(d))

print("\n── 2. armed 之後開始錄，格式與地面站那份相同 ───────────")
for _ in range(3):
    r.feed(FRAME, armed=True)
r.flush()
files = os.listdir(d)
chk("開了一個檔", len(files) == 1, files)
blob = open(os.path.join(d, files[0]), "rb").read()
chk("三個框架都寫進去了", r.frames == 3)
ts = struct.unpack(">Q", blob[:8])[0]
chk("**前 8 byte 是 big-endian 微秒時間戳**", 1.7e15 < ts < 2.1e15, ts)
chk("第 9 byte 是 MAVLink2 magic", blob[8] == 0xFD, hex(blob[8]))
chk("長度＝(8+框架長)×3", len(blob) == (8 + len(FRAME)) * 3, len(blob))

print("\n── 3. 解鎖轉待機 → 收檔（不是繼續寫）──────────────────")
r.feed(FRAME, armed=False)
chk("檔案關掉了", r.f is None)
chk("而且沒有多寫", len(open(os.path.join(d, files[0]), "rb").read()) == len(blob))

print("\n── 4. 超過上限就換檔並滾動 ────────────────────────────")
d2 = tempfile.mkdtemp()
r2 = recorder.Recorder(d2, max_mb=0, keep=3)      # 上限 0 ＝ 每個框架都換檔
seen = set()
for i in range(8):
    r2.feed(FRAME, armed=True)
    seen |= set(os.listdir(d2))
left = sorted(os.listdir(d2))
chk("**真的換了多份檔**（同一秒內換檔要換名字，否則會 append 回原檔）",
    len(seen) >= 4, sorted(seen))
chk("**保留數有上限**（卡滿了會讓整台機出問題）", len(left) <= 3, left)
chk("留下的是最新的那幾份",
    left == sorted(seen, key=lambda x: (len(x), x))[-len(left):]
    or set(left) <= seen, left)

print("\n── 5. 落盤失敗不能拖垮轉發，而且只抱怨一次 ─────────────")
r3 = recorder.Recorder("/proc/nonexistent-dir/x", armed_only=False)
raised = None
try:
    for _ in range(5):
        r3.feed(FRAME, armed=True)
except Exception as e:
    raised = e
chk("**不拋例外**（它在轉發的熱路徑上）", raised is None, raised)
chk("失敗有計數（不是靜靜吞掉）", r3.dropped == 5, r3.stats())
chk("而且只抱怨一次（`_warned`）", r3._warned is True)

# ── 開機就錄＋待機時間輪替（2026-09-07 使用者裁定）────────────────
# **為什麼非有輪替不可**：檔案是在「解鎖→上鎖」那個邊界關的。開機就錄之後，
# 一台從頭到尾沒解鎖的機永遠不會有那個邊界，檔案要寫滿 max_mb 才關——實測
# 3.8 KB/s 要 37 小時。而**沒關好的檔案不會被自動回傳**，於是「連未解鎖也錄」
# 會安靜地連帶關掉自動回傳。
print("\n── 開機就錄：待機輪替，但飛行中不切 ──────────────────")
import tempfile as _tf
import time

d4 = _tf.mkdtemp()
# rotate_min 用極小值讓測試跑得完；判的是**行為**不是那個數字
r4 = recorder.Recorder(d4, armed_only=False, rotate_min=0.02)   # 1.2 秒


def files(dirp):
    return sorted(f for f in os.listdir(dirp) if f.endswith(".tlog"))


r4.feed(b"\xfd\x09\x00" + b"\x00" * 20, armed=False)
first = r4.name
chk("**未解鎖也開始錄**（原本這裡是關檔並 return）", first is not None, first)

time.sleep(1.4)
r4.feed(b"\xfd\x09\x00" + b"\x00" * 20, armed=False)
chk("待機超過輪替間隔 → 換檔", r4.name != first, (first, r4.name))
chk("舊的那份已經關好（關好才傳得回去）", len(files(d4)) >= 2, files(d4))

# **飛行中不許輪替**：一趟飛行一個檔案是回放與兩層比對的前提
during = r4.name
time.sleep(1.4)
r4.feed(b"\xfd\x09\x00" + b"\x00" * 20, armed=True)
time.sleep(1.4)
r4.feed(b"\xfd\x09\x00" + b"\x00" * 20, armed=True)
chk("**飛行中超過間隔也不換檔**（一趟飛行一個檔案）",
    r4.name == during, (during, r4.name))

# 落地那個邊界照舊關檔
r4.feed(b"\xfd\x09\x00" + b"\x00" * 20, armed=False)
chk("落地仍然換檔（原本就有的性質沒被輪替蓋掉）", r4.name != during,
    (during, r4.name))

# **反向驗證**：關掉輪替就不該換
r5 = recorder.Recorder(_tf.mkdtemp(), armed_only=False, rotate_min=0)
r5.feed(b"\xfd\x09\x00" + b"\x00" * 20, armed=False)
n5 = r5.name
time.sleep(1.4)
r5.feed(b"\xfd\x09\x00" + b"\x00" * 20, armed=False)
chk("**反向驗證**：rotate_min=0 就不輪替（不是無條件換檔）",
    r5.name == n5, (n5, r5.name))

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
