#!/usr/bin/env python3
"""取樣率與傳送率：**設定多少就要真的是多少**（2026-09-07）。

## 起因

`--modem-interval 1.0` 實際量到 **2.61 秒一筆**（0.38 Hz），而
`data-schema.md` §3.6 寫的是 1 Hz。兩個錯疊在一起：

1. `time.sleep(interval)` 睡在工作**之後** → 週期＝工作＋interval
2. `_at()` 無條件睡滿 `wait` 再讀 → 光等待就花掉 1.6 秒，而數據機實測
   **49 毫秒**就回完

第 1 個錯在工作很快時完全看不出來——那正是它活這麼久的原因。

## 這支怎麼驗

不接真數據機（CI 也要能跑）：把 `_at` 換成「回固定字串、耗時可控」的假貨，
量迴圈實際的週期。**假的是數據機，不是被測的邏輯**——`sample()`、`run()`、
節流、`stats()` 全都是真的。

用法：python3 tools/modem-rate.py [--live]   （--live 另外對真數據機量一次）
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from modem import ModemSampler                                   # noqa: E402

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


CESQ = "\r\n+CESQ: 99,99,255,255,255,255,65,93,103\r\n\r\nOK\r\n"
CCINFO = "\r\n+GTCCINFO: \r\nPCC,1,0,466,11,85,2179073,627264,79,-93,-11,25\r\n\r\nOK\r\n"
COPS = '\r\n+COPS: 0,0,"Private 5G",11\r\n\r\nOK\r\n'


def sampler(interval, live_interval, at_cost=0.0):
    m = ModemSampler("/dev/null", "127.0.0.1", 38000,
                     lambda: {"lat": 24.78, "lon": 120.99, "alt_rel": 30.0},
                     interval=interval, live_interval=live_interval,
                     airborne_fn=lambda: True)

    def fake_at(cmd, wait=0.5):
        time.sleep(at_cost)          # 假裝數據機花這麼久才回
        return (CESQ if "CESQ" in cmd else
                CCINFO if "GTCCINFO" in cmd else COPS)

    m._at = fake_at
    m._ping_rtt = lambda: 12.3
    m.posted_at = []
    def fake_post(sample):
        m.posted_at.append(time.monotonic()); m.posted += 1; return True
    m.post = fake_post
    m.flush = lambda: 0
    return m


def run_for(m, seconds):
    import threading
    t = threading.Thread(target=m.run, daemon=True)
    t0 = time.monotonic()
    t.start()
    time.sleep(seconds)
    return time.monotonic() - t0


print("── 週期＝設定值，不是「設定值＋工作時間」──────────────")
# 每次取樣的工作要 0.30 秒；設定 0.5 秒一筆
m = sampler(interval=0.5, live_interval=0.5, at_cost=0.10)   # 3 個 AT × 0.10
span = run_for(m, 4.0)
n = m.samples
period = span / max(n, 1)
print(f"   4 秒內取樣 {n} 筆，平均週期 {period:.3f}s（設定 0.5s、工作約 0.3s）")
chk("**週期貼近設定值**（舊版會是 0.5+0.3=0.8s）", 0.44 <= period <= 0.60, period)
chk("`stats().hz` 說得出實際取樣率（原本系統完全沒有這個數字）",
    m.stats()["hz"] is not None and 1.5 <= m.stats()["hz"] <= 2.4,
    m.stats()["hz"])

print("\n── 蒐集密、傳送疏：兩件事分開 ─────────────────────────")
m2 = sampler(interval=0.2, live_interval=1.0, at_cost=0.0)
span2 = run_for(m2, 3.0)
sent = len(m2.posted_at)
chk("取樣照 interval 走（3 秒 ≈ 15 筆）", 11 <= m2.samples <= 18, m2.samples)
chk("**即時通道只有每秒一筆**（不隨取樣變密）", 2 <= sent <= 4, sent)
gaps = [b - a for a, b in zip(m2.posted_at, m2.posted_at[1:])]
chk("送出間隔 ≈ live_interval", all(0.85 <= g <= 1.25 for g in gaps), 
    [round(g, 2) for g in gaps])
chk("**記錄通道拿到的是全部樣本**（研究資料不被畫面的需求稀釋）",
    len(m2.buf) == m2.samples, (len(m2.buf), m2.samples))

print("\n── 反向驗證：假數據機變慢，週期不該跟著爆 ──────────────")
m3 = sampler(interval=1.0, live_interval=1.0, at_cost=0.20)   # 工作 0.6s
span3 = run_for(m3, 4.0)
p3 = span3 / max(m3.samples, 1)
chk("工作 0.6s、設定 1.0s → 週期仍約 1.0s（舊版會是 1.6s）",
    0.90 <= p3 <= 1.15, round(p3, 3))

a = argparse.ArgumentParser(add_help=False)
a.add_argument("--live", action="store_true")
if a.parse_known_args()[0].live:
    print("\n── 對真數據機量一次 ───────────────────────────────────")
    real = ModemSampler("/dev/ttyUSB1", "127.0.0.1", 38000, lambda: {},
                        interval=1.0, live_interval=1.0)
    t0 = time.monotonic()
    for _ in range(3):
        real.sample()
    cost = (time.monotonic() - t0) / 3
    print(f"   單次取樣實際耗時 {cost * 1000:.0f} ms（舊版固定 1600 ms）")
    chk("真數據機一次取樣 < 300 ms", cost < 0.3, f"{cost * 1000:.0f} ms")

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
