#!/usr/bin/env python3
"""訊號記錄的離地閘門：**停在地面上就不記**（2026-09-07 使用者裁定）。

## 為什麼是「離地」不是「解鎖」

解鎖之後、起飛之前，飛機還停在地上——那段的訊號是停機坪的訊號，不是航路上
的。研究要的是後者。而原本的閘門在地面站那側，判準是「架次內」＝解鎖到上鎖，
比使用者要的寬。

## 不知道有沒有離地時，照記

`landed_state` 韌體不一定回報，退回高度＋地速也可能兩個都沒有。這時**當作
在飛**：飛行中的量測補不回來，多記幾筆地面資料刪得掉。與代理狀態機那條
「寧可判成飛行中」是同一個方向，而且用的是**同一個函式**（`Agent._landed`）。

用法：python3 tools/modem-ground-gate.py
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import agent as A                                              # noqa: E402
from modem import ModemSampler                                 # noqa: E402

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


# FM160 實測回應（2026-08-24 那組 65,93,103 就是本專案反推 NR 欄位順序的依據）
CESQ = "\r\n+CESQ: 99,99,255,255,255,255,65,93,103\r\n\r\nOK\r\n"
CCINFO = "\r\n+GTCCINFO: \r\nPCC,1,0,466,11,85,2179073,627264,79,-93,-11,25\r\n\r\nOK\r\n"
COPS = '\r\n+COPS: 0,0,"Private 5G",11\r\n\r\nOK\r\n'


def sampler(airborne_fn):
    m = ModemSampler("/dev/null", "127.0.0.1", 38000,
                     lambda: {"lat": 24.78, "lon": 120.99, "alt_rel": 30.0},
                     airborne_fn=airborne_fn)
    m._at = lambda cmd, wait=0.5: (          # 不開實體序列埠
        CESQ if "CESQ" in cmd else CCINFO if "GTCCINFO" in cmd else COPS)
    m._ping_rtt = lambda: 12.3
    m.post = lambda s: True                  # 即時通道：本測試不打網路
    return m


print("── 閘門本身 ─────────────────────────────────────────")
air = sampler(lambda: True)
for _ in range(3):
    air.sample()
chk("**離地：照記**", len(air.buf) == 3, f"buf={len(air.buf)} on_ground={air.on_ground}")

gnd = sampler(lambda: False)
for _ in range(3):
    gnd.sample()
chk("**停在地面：不記**", len(gnd.buf) == 0, f"buf={len(gnd.buf)}")
chk("而且丟掉幾筆看得見（不是無聲的）", gnd.on_ground == 3, gnd.on_ground)
chk("採樣本身照跑——即時通道要看得到起飛前的訊號",
    gnd.samples == 3 and gnd.last.get("rsrp") is not None,
    (gnd.samples, gnd.last.get("rsrp")))

none = sampler(None)
none.sample()
chk("沒接閘門就一律記（不要自作主張少記東西）", len(none.buf) == 1)

# 落地之後不該再長；重新離地要接得回去
mix = sampler(lambda: mix_air[0])
mix_air = [True]
mix.sample(); mix_air[0] = False
mix.sample(); mix.sample(); mix_air[0] = True
mix.sample()
chk("起飛→落地→再起飛：只記飛的那兩筆", len(mix.buf) == 2 and mix.on_ground == 2,
    (len(mix.buf), mix.on_ground))
seqs = [s["seq"] for s in mix.buf]
chk("**seq 不因為跳過而斷號**（地面站靠它回報收下了哪幾筆）",
    seqs == [1, 2], seqs)

print("\n── 判準：與狀態機共用同一個定義 ─────────────────────")


class FakeMon:
    def __init__(self, st):
        self.state = st


def agent_with(st):
    a = A.Agent.__new__(A.Agent)
    a.mon = FakeMon(st)
    return a


CASES = [
    ({"landed_state": "on_ground"}, False, "landed_state 說在地上"),
    ({"landed_state": "in_air"}, True, "landed_state 說在天上"),
    ({"landed_state": "takeoff"}, True, "起飛中算離地"),
    ({"landed_state": "landing"}, True, "降落中還沒落地，算離地"),
    ({"alt_rel_m": 0.2, "groundspeed_ms": 0.1}, False, "沒有 landed_state：低又靜＝在地上"),
    ({"alt_rel_m": 25.0, "groundspeed_ms": 6.0}, True, "沒有 landed_state：高又動＝在飛"),
    ({}, True, "**什麼都不知道 → 當作在飛**（量測補不回來）"),
    ({"alt_rel_m": 0.2}, True, "只有高度、沒有地速 → 不敢說在地上"),
]
for st, want, label in CASES:
    got = agent_with(st).airborne()
    chk(f"{label}｜airborne={got}", got is want)

chk("**只有一份定義**：`_landed` 是狀態機與訊號閘門共用的那一個",
    "landed = self._landed()" in Path(A.__file__).read_text())

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
