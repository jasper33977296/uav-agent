#!/usr/bin/env python3
"""上下行實際用量的量測邏輯（uav-system issues/022 之後，2026-09-23 裁定）。

**重點全在「不知道」與「0」要分得開**：讀不到、第一次取樣、計數器被重設，
這三種都是不知道；0 的意思是「這段時間真的沒傳東西」。混在一起，事後就
分不出「那段沒量到」與「那段很安靜」。

跑法：`python3 tests/test_throughput.py`（不需要真的 5G 介面）
"""
import pathlib
import sys
import tempfile

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from modem import ModemSampler                                      # noqa: E402

ok = fail = 0


def check(name, cond, detail=""):
    global ok, fail
    if cond:
        ok += 1; print(f"  ✓ {name}")
    else:
        fail += 1; print(f"  ✗ {name} {detail}")


class Fake:
    """假的 /sys/class/net/<iface>/statistics。"""
    def __init__(self):
        self.root = pathlib.Path(tempfile.mkdtemp())
        (self.root / "statistics").mkdir()

    def set(self, tx, rx):
        (self.root / "statistics" / "tx_bytes").write_text(str(tx))
        (self.root / "statistics" / "rx_bytes").write_text(str(rx))


f = Fake()
m = ModemSampler("", "127.0.0.1", 38000, lambda: None)
# `_throughput` 讀的是 /sys/class/net/<iface>/statistics/*。**不另外抽一層
# 讓測試注入路徑**——那會變成測一份只有測試在用的邏輯。這裡換掉 open()，
# 測到的就是機上真的會跑的那一份。
_real_open = open


def fake_open(path, *a, **k):
    p = str(path)
    if p.startswith("/sys/class/net/usb0/statistics/"):
        return _real_open(str(f.root / "statistics" / p.rsplit("/", 1)[1]), *a, **k)
    return _real_open(path, *a, **k)


import builtins                                                     # noqa: E402
builtins.open = fake_open

print("── 讀不到介面 ──")
m.net_iface = "沒有這個介面"
check("回 (None, None) 而不是 (0, 0)", m._throughput(100.0) == (None, None))

print("\n── 第一次取樣（沒有前值）──")
m.net_iface = "usb0"
f.set(1000, 2000)
check("回 (None, None)", m._throughput(100.0) == (None, None))

print("\n── 正常差分 ──")
f.set(1000 + 125000, 2000 + 12500)      # 1 秒內 tx +125000 bytes = 1000 kbps
up, down = m._throughput(101.0)
check("up 算得對（125000 B/s → 1000 kbps）", up == 1000.0, up)
check("down 算得對（12500 B/s → 100 kbps）", down == 100.0, down)

print("\n── 真的沒傳東西：要是 0，不是 None ──")
up, down = m._throughput(102.0)
check("up 是 0.0", up == 0.0, up)
check("down 是 0.0", down == 0.0, down)

print("\n── 計數器被重設（介面重新列舉／重開）──")
f.set(5, 5)
check("丟掉那一段，回 (None, None)", m._throughput(103.0) == (None, None))
up, down = m._throughput(104.0)
check("下一拍就重新起算（不是永遠壞掉）", up == 0.0 and down == 0.0, (up, down))

print("\n── 時間沒前進 ──")
f.set(999999, 999999)
check("dt<=0 回 (None, None)", m._throughput(104.0) == (None, None))

builtins.open = _real_open
print(f"\n結果：{ok} 通過、{fail} 失敗")
sys.exit(1 if fail else 0)
