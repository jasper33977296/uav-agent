#!/usr/bin/env python3
"""`state.derived` 的正規化欄位：從 MAVLink 餵進去，看送出什麼（026 B4-b）。

`tools/driver-parity.py` 驗的是「算法與地面站相同」；**這一支驗的是另一半：
那些值真的從飛控的訊息裡被解出來了**。兩者都要——算法對但沒接上輸入，
送出去的就是一整排 None，而 None 在協定裡是合法值，不會有任何錯誤。

釘住的契約（協定 §4.2）：

* **缺欄位＝代理沒說，不是值是 false。** 送 `ready: false` 表示「還沒算出來」
  會讓所有機都變成不可飛——與 039 複裁 A 的 `rc_link` 同一個教訓。
* **判斷用 `mode_verb`，顯示用 `mode_name`。** 兩個都要有，不是二選一。
* `prearm_ok` 黏性：見過一次 PREARM 位元才信 health，從未見過維持 None。

用法：python3 tools/derived-fields.py
"""
import sys
import types
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

for name in ("serial",):
    sys.modules.setdefault(name, types.ModuleType(name))
try:
    import pymavlink  # noqa: F401
except ImportError:
    print("skip：沒有 pymavlink。**skip ≠ pass**")
    sys.exit(2)

import agent as A       # noqa: E402
import normalize as N   # noqa: E402

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


def hb(custom_mode=3, status=4, autopilot=3, armed=True):
    return types.SimpleNamespace(
        get_type=lambda: "HEARTBEAT", get_srcSystem=lambda: 1,
        base_mode=0x80 if armed else 0, custom_mode=custom_mode,
        system_status=status, autopilot=autopilot, type=2)


def sys_status(present=0, enabled=0, health=0, batt=80):
    return types.SimpleNamespace(
        get_type=lambda: "SYS_STATUS", voltage_battery=12000,
        battery_remaining=batt, load=250,
        onboard_control_sensors_present=present,
        onboard_control_sensors_enabled=enabled,
        onboard_control_sensors_health=health)


def gps(fix=3):
    return types.SimpleNamespace(get_type=lambda: "GPS_RAW_INT",
                                 fix_type=fix, satellites_visible=12)


def ekf(flags):
    return types.SimpleNamespace(get_type=lambda: "EKF_STATUS_REPORT",
                                 flags=flags)


GYRO, ACCEL, MAG = 1 << 0, 1 << 1, 1 << 2
PREARM = N.PREARM_CHECK

print("── 什麼都還沒收到 ─────────────────────────────────────")
m = A.Monitor()
d = N.derived(m.state)
chk("ready 是 None（不是 False——那會讓機變成不可飛）", d["ready"] is None, d["ready"])
chk("原因說得出是「還沒收到」而不是「不能飛」",
    d["not_ready_reasons"] == ["尚未收到預檢／EKF 狀態，無法判定就緒"],
    d["not_ready_reasons"])
chk("模式兩個欄位都在，值是 None（沒有 custom_mode 就不猜）",
    d["mode_name"] is None and d["mode_verb"] is None, d)
chk("**六個欄位一個都不能少**（缺欄位＝代理沒說，語意完全不同）",
    set(d) == {"mode_verb", "mode_name", "ready", "not_ready_reasons",
               "prearm_ok", "ekf_ok"}, sorted(d))

print("\n── 收到心跳：模式解得出來 ─────────────────────────────")
m.feed(hb(custom_mode=3))                    # ArduPilot AUTO
d = N.derived(m.state)
chk("mode_name＝AUTO（顯示用，廠牌自己的名字）", d["mode_name"] == "AUTO", d["mode_name"])
chk("mode_verb＝mission（判斷用，廠牌無關）", d["mode_verb"] == "mission", d["mode_verb"])
m.feed(hb(custom_mode=5))                    # LOITER
d = N.derived(m.state)
chk("LOITER → hold", (d["mode_name"], d["mode_verb"]) == ("LOITER", "hold"), d)
m.feed(hb(custom_mode=0))                    # STABILIZE
d = N.derived(m.state)
chk("**custom_mode=0 是 STABILIZE 不是「不知道」**（a92b8e1 修過的 bug）",
    d["mode_name"] == "STABILIZE", d["mode_name"])
chk("STABILIZE 沒有對應動詞 → verb None（飛手手飛，不在動詞表裡）",
    d["mode_verb"] is None, d["mode_verb"])

print("\n── PREARM 是黏性的 ────────────────────────────────────")
m2 = A.Monitor()
m2.feed(hb())
m2.feed(sys_status(present=GYRO, enabled=GYRO, health=GYRO))
chk("從沒見過 PREARM 位元 → None（韌體不回報，如實缺席）",
    m2.state["prearm_ok"] is None, m2.state["prearm_ok"])
m2.feed(sys_status(present=GYRO, enabled=GYRO, health=GYRO | PREARM))
chk("見到了且 health 有 → True", m2.state["prearm_ok"] is True)
m2.feed(sys_status(present=GYRO, enabled=GYRO, health=GYRO))
chk("**見過之後就一直信 health**：位元消失＝預檢未過，不是回到「不知道」",
    m2.state["prearm_ok"] is False, m2.state["prearm_ok"])
d = N.derived(m2.state)
# 2026-09-04：地面站那側現在說得出是哪一項預檢（從 `PreArm:` STATUSTEXT
# 收集），而代理還沒有那個能力——**兩邊的退路字串必須一樣**，否則
# `driver-parity` 會紅，而那個紅是對的。
chk("預檢未過會寫進原因，說得出是哪一家，**而且承認自己不知道是哪一項**",
    "ArduPilot 預檢未過（機上還沒說是哪一項）" in d["not_ready_reasons"],
    d["not_ready_reasons"])

print("\n── 感測器異常：present 且 enabled 且不 health ──────────")
m3 = A.Monitor()
m3.feed(hb())
m3.feed(sys_status(present=GYRO | ACCEL | MAG, enabled=GYRO | ACCEL | MAG,
                   health=GYRO | ACCEL))
chk("磁力計不健康 → 列出來", m3.state["sensors_unhealthy"] == ["磁力計"],
    m3.state["sensors_unhealthy"])
m4 = A.Monitor()
m4.feed(hb())
m4.feed(sys_status(present=GYRO | MAG, enabled=GYRO, health=GYRO))
chk("**沒 enable 的不算異常**（關掉的感測器不健康是廢話）",
    m4.state["sensors_unhealthy"] == [], m4.state["sensors_unhealthy"])

print("\n── EKF：兩個訊息名同一件事 ────────────────────────────")
m5 = A.Monitor()
m5.feed(hb())
m5.feed(ekf(N.EKF_READY_BITS))
chk("EKF_STATUS_REPORT（ArduPilot 的名字）解得出 ekf_ok",
    m5.state["ekf_ok"] is True, m5.state["ekf_ok"])
m5.feed(ekf(0x0011))                         # 少了水平位置那一位
chk("少一個必要位元 → False", m5.state["ekf_ok"] is False)
m6 = A.Monitor()
m6.feed(hb(autopilot=12))
m6.feed(types.SimpleNamespace(get_type=lambda: "ESTIMATOR_STATUS",
                              flags=N.EKF_READY_BITS))
chk("ESTIMATOR_STATUS（PX4 的名字）走同一條路", m6.state["ekf_ok"] is True)

print("\n── 湊齊之後才敢說「可以飛」─────────────────────────────")
m7 = A.Monitor()
m7.feed(hb())
m7.feed(gps(3))
m7.feed(ekf(N.EKF_READY_BITS))
m7.feed(sys_status(present=GYRO, enabled=GYRO, health=GYRO))
d = N.derived(m7.state)
chk("GPS 定位＋EKF 就緒＋無異常 → ready True", d["ready"] is True, d)
m7.feed(gps(1))
d = N.derived(m7.state)
chk("GPS 掉到 fix=1 → ready False 且說得出原因",
    d["ready"] is False and "GPS 未定位（fix=1）" in d["not_ready_reasons"], d)
m7.feed(gps(3))
m7.feed(hb(status=5))                        # CRITICAL
d = N.derived(m7.state)
chk("failsafe 狀態也擋", d["ready"] is False
    and "failsafe 狀態：CRITICAL" in d["not_ready_reasons"], d)

print("\n── hello 的 driver 欄位 ───────────────────────────────")
info = N.driver_info()
chk("帶得出版本（上游 commit ＋ 內容雜湊）",
    info["version"].startswith(info["ported_from"] + "+")
    and len(info["content_hash"]) == 12, info)
lim = N.driver_limits(3)
chk("limits 照驅動說的送，包括 unverified",
    lim["takeoff_alt_m"]["confidence"] == "unverified"
    and lim["takeoff_alt_m"]["min"] is None, lim)
chk("**未知廠牌的 limits 是空表**（不知道就不要湊一個通用值）",
    N.driver_limits(None) == {}, N.driver_limits(None))

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
