#!/usr/bin/env python3
"""驅動搬家的等價驗收（026 B4-d 的核心，**不需要 SITL**）。

**搬家不是改寫，唯一的驗收標準是「新舊輸出完全相同」**（架構文件 §9.3）。
而**兩份實作同時存在的這段期間，正是唯一能做這個比對的窗口**——地面站那份
刪掉之後就沒有對照組了。

這支把地面站的實作與機端移植的這份**同時載進同一個行程**，用同一批輸入掃過去
逐鍵比對。分兩半驗，因為兩半的風險完全不同：

* **逐字複製的那半**（`autopilot/` 四支檔案）：用內容雜湊證明與上游位元組相同。
  這一半沒有「搬錯」的空間，只有「漂移」的空間。
* **真正移植的那半**（`normalize.readiness()`，來自地面站的 `state.py`）：
  同一批輸入掃過所有組合。**這才是會出錯的地方。**

兩個 repo 都在本機時才跑得起來（機上沒有 uav-system）。找不到就 skip——
**skip ≠ pass**。

用法：python3 tools/driver-parity.py [uav-system 的路徑]
"""
import hashlib
import itertools
import sys
from pathlib import Path

AGENT = Path(__file__).resolve().parent.parent
GS = Path(sys.argv[1]) if len(sys.argv) > 1 else Path.home() / "uav-system"

if not (GS / "libs" / "autopilot").is_dir():
    print(f"skip：找不到 uav-system（{GS}）。**skip ≠ pass**")
    sys.exit(2)

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


# ── 掃描用的輸入空間 ────────────────────────────────────────────────
# 每一維都含「不知道」那一格——三態的分界正是最容易搬錯的地方
TRI = (None, True, False)
FIXES = (None, 0, 1, 2, 3, 4)
STATES = (None, "STANDBY", "ACTIVE", "CRITICAL", "EMERGENCY",
          "FLIGHT_TERMINATION")
SENSORS = ([], ["GPS"], ["磁力計", "電池"])
RAWS = (None, 3, 12)
CASES = list(itertools.product(TRI, TRI, FIXES, STATES, SENSORS, RAWS))

# ArduPilot 的 custom_mode 是模式號；PX4 的是 main<<16|sub<<24 的 union。
# 兩種編碼都掃，含表上沒有的值（那正是 decode 要老實回 MODE_n 的地方）
MODES_AP = list(range(0, 32))
MODES_PX4 = [0] + [(m << 16) | (s << 24) for m in range(1, 9) for s in range(0, 10)]


def sweep_readiness(readiness_fn):
    """readiness_fn(prearm, ekf, fix, mav_state, sensors, raw) → (ready, reasons)"""
    return [readiness_fn(*c) for c in CASES]


def sweep_modes(decode_mode, decode_verb, raw, values):
    return [(decode_mode(raw, v), decode_verb(raw, v)) for v in values]


# ── 第一階段：機端這份 ──────────────────────────────────────────────
sys.path.insert(0, str(AGENT))
import normalize as agent_norm                                    # noqa: E402
from autopilot import get_driver as agent_get_driver              # noqa: E402
import agent as agent_mod                                         # noqa: E402


def _agent_readiness(prearm, ekf, fix, mav_state, sensors, raw):
    return agent_norm.readiness({
        "prearm_ok": prearm, "ekf_ok": ekf, "gps_fix_raw": fix,
        "mav_state": mav_state, "sensors_unhealthy": sensors,
        "autopilot_raw": raw})


agent_ready = sweep_readiness(_agent_readiness)
agent_modes = {
    raw: sweep_modes(lambda r, v: agent_get_driver(r).decode_mode(v),
                     lambda r, v: agent_get_driver(r).decode_verb(v),
                     raw, vals)
    for raw, vals in ((3, MODES_AP), (12, MODES_PX4), (None, MODES_AP))}
agent_copter = dict(agent_mod.COPTER_MODE)
# **真的呼叫，不是 hasattr**：旗標會漏，方法一樣會漏而且更難發現——
# `hasattr` 過得了，要呼叫下去才知道（上游 test-driver-flags.py 同一個理由，
# 起因是 2026-08-26 `no_coord_frames` 漏了 PX4 直接 AttributeError）
agent_fsparams = {raw: agent_get_driver(raw).gcs_failsafe_params()
                  for raw in (3, 12, None)}
agent_ekf_types = agent_norm.EKF_MSG_TYPES
agent_ekf_bits = agent_norm.EKF_READY_BITS

# ── 第二階段：把機端的模組全部卸掉，換地面站那份 ────────────────────
# 兩邊的套件都叫 `autopilot`，sys.modules 只放得下一個。先算完再換，
# 比較的是**資料**不是物件——這樣才不會有一邊其實載到另一邊的檔案
for name in [n for n in sys.modules
             if n == "autopilot" or n.startswith("autopilot.")
             or n in ("normalize", "agent", "guard", "route")]:
    del sys.modules[name]
sys.path.remove(str(AGENT))
sys.path.insert(0, str(GS / "libs"))
sys.path.insert(0, str(GS / "apps" / "backend"))

from app.state import LiveState                                   # noqa: E402
from autopilot import get_driver as gs_get_driver                  # noqa: E402
from app import dialect as gs_dialect                              # noqa: E402


def _gs_readiness(prearm, ekf, fix, mav_state, sensors, raw):
    st = LiveState()
    st.prearm_ok, st.ekf_ok = prearm, ekf
    st.gps_fix, st.mav_state = fix, mav_state
    st.sensors_unhealthy, st.autopilot_raw = sensors, raw
    return st.readiness()


gs_ready = sweep_readiness(_gs_readiness)
gs_fsparams = {raw: gs_get_driver(raw).gcs_failsafe_params()
               for raw in (3, 12, None)}
gs_modes = {
    raw: sweep_modes(lambda r, v: gs_get_driver(r).decode_mode(v),
                     lambda r, v: gs_get_driver(r).decode_verb(v),
                     raw, vals)
    for raw, vals in ((3, MODES_AP), (12, MODES_PX4), (None, MODES_AP))}

# ── 比對 ────────────────────────────────────────────────────────────
print("── 逐字複製的那半：位元組相同嗎 ───────────────────────")
FILES = ("__init__.py", "driver.py", "ardupilot.py", "px4.py")
diffs = []
for f in FILES:
    a = (AGENT / "autopilot" / f).read_bytes()
    g = (GS / "libs" / "autopilot" / f).read_bytes()
    if a != g:
        diffs.append(f)
    print(f"   {f:<16} {hashlib.sha256(a).hexdigest()[:12]}"
          f"  {'＝' if a == g else '**≠**'}  "
          f"{hashlib.sha256(g).hexdigest()[:12]}")
chk("四支驅動與上游逐字相同（搬家不是改寫）", not diffs, diffs)

print("\n── 真正移植的那半：就緒判定 ───────────────────────────")
bad = [(c, a, g) for c, a, g in zip(CASES, agent_ready, gs_ready) if a != g]
chk(f"{len(CASES)} 組輸入的 (ready, reasons) 完全相同", not bad,
    "" if not bad else f"{len(bad)} 組不同，第一組：輸入={bad[0][0]} "
                       f"機端={bad[0][1]} 地面站={bad[0][2]}")
# **反向驗證**：這個掃描分得出差異嗎？故意改一個字元應該要被抓到
poisoned = list(agent_ready)
poisoned[0] = (poisoned[0][0], ["故意寫錯"])
chk("**反向驗證**：故意改一個原因字串，掃描抓得到",
    poisoned != gs_ready)
# 三態的分界單獨釘一次——它是這次移植最容易搬錯的一格
blank_a = _agent_readiness(None, None, None, None, [], 3)
chk("空狀態＝ready None（不是 True，也不是 False）", blank_a[0] is None, blank_a)

print("\n── 模式解讀：兩邊逐值相同 ─────────────────────────────")
for raw, label in ((3, "ArduPilot"), (12, "PX4"), (None, "Unknown")):
    d = [(v, a, g) for v, a, g in zip(
        MODES_AP if raw != 12 else MODES_PX4, agent_modes[raw], gs_modes[raw])
        if a != g]
    chk(f"{label}：decode_mode／decode_verb 逐值相同", not d, d[:2])
# 那個修過的 bug 不許回來（a92b8e1）
chk("ArduPilot custom_mode=0 是 STABILIZE（不是「—」）",
    agent_modes[3][0][0] == "STABILIZE", agent_modes[3][0])

print("\n── 代理自己那張 Copter 表 vs 驅動的 ───────────────────")
# 代理的 `flight_state()` 用的是自己的 COPTER_MODE，驅動另有一份 _COPTER。
# **兩份表一定會漂移**（驅動的註解自己這樣寫）。本批不改 flight_state 的
# 輸入來源（那是飛安路徑），但要讓漂移在測試裡就爆出來，而不是飛到一半
drift = [(k, v, gs_get_driver(3).decode_mode(k))
         for k, v in agent_copter.items()
         if gs_get_driver(3).decode_mode(k) != v]
chk("agent.COPTER_MODE 與 ArduPilot 驅動逐鍵一致", not drift, drift[:3])

print("\n── 失聯 failsafe 參數名：代理實際依賴的那個方法 ────────")
# 代理的開機自檢向驅動要參數名（`Agent._fs_gcs_names`），所以這個方法的
# 回傳值是**執行期契約**，不只是資料
chk("三個驅動的 gcs_failsafe_params() 兩邊相同",
    agent_fsparams == gs_fsparams,
    f"機端={agent_fsparams} 地面站={gs_fsparams}")
chk("ArduPilot 與 PX4 的名字**不同**（是方言不是改名）",
    agent_fsparams[3] != agent_fsparams[12],
    (agent_fsparams[3], agent_fsparams[12]))
chk("未知廠牌回空表（沒有意見就不要製造意見）",
    agent_fsparams[None] == {}, agent_fsparams[None])
chk("兩家都給得出 enable 與 timeout_s 兩個鍵",
    all(set(agent_fsparams[r]) == {"enable", "timeout_s"} for r in (3, 12)),
    {r: sorted(agent_fsparams[r]) for r in (3, 12)})

print("\n── 訊息層等價的推導 ───────────────────────────────────")
chk("EKF 訊息型別兩邊相同（都是從 MESSAGE_ADJUSTMENTS 推導）",
    set(agent_ekf_types) == set(gs_dialect.EKF_MSG_TYPES),
    f"機端={agent_ekf_types} 地面站={gs_dialect.EKF_MSG_TYPES}")
chk("EKF 就緒位元遮罩相同", agent_ekf_bits == gs_dialect.EKF_READY_BITS,
    f"機端=0x{agent_ekf_bits:04X} 地面站=0x{gs_dialect.EKF_READY_BITS:04X}")

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
