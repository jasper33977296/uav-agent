#!/usr/bin/env python3
"""時間的兩種用途不能混，以及「只綁一台飛控」——**不連飛控、不送任何東西**。

1. 量間隔的地方不准用牆鐘：Pi 對時會讓牆鐘一次跳好幾分鐘（2026-09-14），
   拿它量就量出假的失聯，代理會自己重綁埠、重連通道。這裡逐行掃，
   **新加的 time.time() 如果不是用來標時間戳，就會紅**。
2. 對時前記下的單調時間，對時後換算出來要是對的時刻。
3. 錄製：對時前不開檔，對時後補寫，時間戳換算過。
4. 只綁一台飛控：別的 sysid 不採信；換號中接受新號碼。

用法：python3 tools/clock-flow.py
"""
import os
import struct
import sys
import tempfile
import time
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
for name in ("serial",):
    sys.modules.setdefault(name, types.ModuleType(name))

import clock      # noqa: E402

fails = []


def ck(name, cond, got=""):
    print(f"  {'✓' if cond else '✗'} {name}" + (f"  ← {got}" if not cond else ""))
    if not cond:
        fails.append(name)


print("── 量間隔不准用牆鐘 ──")
#: 允許的 time.time()：**只有標時間戳**的地方
ALLOWED = {
    "agent.py": ['"ts": time.time()'],
    "intent.py": [],
    "modem.py": [],
    "recorder.py": ['struct.pack(">Q", int(time.time() * 1e6))'],
    "backfill.py": ["off = time.time() - time.monotonic()", "time.time = "],
    "uploader.py": ['"at": time.time()'],
    "clock.py": ["time.time()"],
}
for fn, ok in ALLOWED.items():
    bad = [f"{fn}:{i}: {line.strip()}"
           for i, line in enumerate((ROOT / fn).read_text(encoding="utf-8").splitlines(), 1)
           if "time.time()" in line and not line.lstrip().startswith("#")
           and not any(a in line for a in ok)
           and "`time.time()`" not in line]
    ck(f"{fn} 沒有拿牆鐘量間隔", not bad, bad[:3])

print("\n── 對時前記的時間，對時後換算要對 ──")
real = time.time
m = time.monotonic()
time.time = lambda: real() - 1200            # 開機時牆鐘慢 20 分鐘
wrong_then = time.time()
time.time = real                             # 對時：一次跳上來
got = clock.wall(m)
ck("換算出來的是真實時刻，不是開機時那個慢 20 分鐘的鐘",
   abs(got - (real() - (time.monotonic() - m))) < 0.05 and abs(got - wrong_then) > 1000,
   (got, wrong_then))

print("\n── 錄製：對時前不開檔，對時後補寫 ──")
import recorder   # noqa: E402
with tempfile.TemporaryDirectory() as d:
    synced = [False]
    clock.synced = lambda: synced[0]
    time.time = lambda: real() - 1200
    r = recorder.Recorder(d, armed_only=False, rotate_min=0)
    frames = [b"\xfd\x01aaaa", b"\xfd\x02bb", b"\xfd\x03c"]
    for f in frames:
        r.feed(f, False)
    ck("對時前不開檔", not os.listdir(d), os.listdir(d))
    ck("對時前的框架先壓著", len(r._pre) == 3, len(r._pre))
    synced[0] = True
    time.time = real
    r.feed(b"\xfd\x04dddd", False)
    r._close()
    files = os.listdir(d)
    ck("對時後開了一個檔", len(files) == 1, files)
    data = (Path(d) / files[0]).read_bytes() if files else b""
    stamps, pos = [], 0
    for f in frames + [b"\xfd\x04dddd"]:
        stamps.append(struct.unpack(">Q", data[pos:pos + 8])[0] / 1e6)
        pos += 8 + len(f)
    ck("四個框架都寫進去了", pos == len(data) and len(stamps) == 4, (pos, len(data)))
    ck("補寫的時間戳是真實時刻（不是慢 20 分鐘的鐘）",
       all(abs(t - real()) < 5 for t in stamps), [round(real() - t, 1) for t in stamps])
    clock.synced = __import__("importlib").reload(clock).synced

print("\n── 只綁一台飛控 ──")
import agent as A  # noqa: E402
ns = types.SimpleNamespace
ck("平常只認自己的 sysid", A.Agent._accept_sysids(ns(fc_sysid=1, reassigning=None)) == {1})
ck("換號中也認新號碼",
   A.Agent._accept_sysids(ns(fc_sysid=1, reassigning={"to": 5})) == {1, 5})

print()
if fails:
    print(f"✗ {len(fails)} 項沒過：" + "、".join(fails))
    sys.exit(1)
print("全部通過")
