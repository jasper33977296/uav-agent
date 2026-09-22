#!/usr/bin/env python3
"""飛控 GCS failsafe 的開機自檢（uav-system 039，2026-09-01 裁定）。

**不變式**：`FS_GCS_TIMEOUT > LINK_LOSS_MAX_SOLO_S > LINK_ACTION_S`

飛控那層的角色是**代理的後備**——只在代理沒有動手時才出手。逾時若小於單飛
上限，飛控會在續飛的中途搶先，而使用者裁定的「失聯後跑完任務再 RTL」那一格
**永遠不會發生**。這是選項 C 的第三條安靜退化途徑，而它比另外兩條更難發現：
飛控搶先動作時代理只看得到模式突然變了，`_track_mode_owner` 會判成飛手切的，
於是決定永不介入——看起來完全正常。

三種結論必須分得開（缺一就等於沒說）：

  * **通過**：讀到了、不變式成立。
  * **讀到了但不合**：知道它設錯了 → 大聲說是哪一條不成立。
  * **讀不到**：不知道它怎麼設 → 這**不是**「設錯了」。可能是舊韌體沒有那個
    參數，也可能是請求或回應在 57600 上掉了，我們分不出來。

用法：python3 tools/fs-gcs-check.py
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

import agent as A  # noqa: E402

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


class Fake(A.Agent):
    """只借自檢那幾個方法，不跑 __init__。"""

    def __init__(self, *, raw=3, params=None, solo=30.0, act=10.0, sysid=1):
        st = {"sysid": sysid, "autopilot_raw": raw}
        if params is not None:
            st["params"] = dict(params)
        self.mon = types.SimpleNamespace(state=st)
        self.outbox = A.Outbox()
        self.a = types.SimpleNamespace(link_loss_max_solo_s=solo,
                                       link_action_s=act)
        self.fs_gcs_asked = 1
        self.fs_gcs_asked_at = 1000.0
        self.fs_gcs_said = False
        self.sent = []
        self.serial = types.SimpleNamespace(
            mav=types.SimpleNamespace(
                param_request_read_send=lambda *a: self.sent.append(a[2])))
        self.logs = []


def run(g, now=1000.0):
    """跑自檢，回傳 (說了沒有, 事件文字)。"""
    g._check_fs_gcs(now)
    return g.fs_gcs_said, [n["text"] for n in g.outbox.unsent()]


print("── 裁定值：通過 ───────────────────────────────────────")
g = Fake(params={"FS_GCS_ENABLE": 1.0, "FS_GCS_TIMEOUT": 45.0})
said, evs = run(g)
chk("45 > 30 > 10 → 通過且不留警告事件", said and not evs, evs)

print("\n── 讀到了但不合：要說得出是哪一條 ─────────────────────")
g = Fake(params={"FS_GCS_ENABLE": 1.0, "FS_GCS_TIMEOUT": 15.0})
said, evs = run(g)
chk("逾時 15s ≤ 單飛上限 30s → 報警", said and evs, evs)
chk("理由指名是 FS_GCS_TIMEOUT 與單飛上限的關係",
    evs and "FS_GCS_TIMEOUT" in evs[0] and "單飛上限" in evs[0], evs[:1])
# **這一格正是對方更正過的地方**：舊條件寫成「> LINK_ACTION_S（10s）」，
# 那太弱了——15 > 10 會通過，但飛控仍然在續飛中途搶先
chk("**15s 不因為「大於處置起算 10s」就放行**（舊條件太弱的那一格）",
    bool(evs))

g = Fake(params={"FS_GCS_ENABLE": 0.0, "FS_GCS_TIMEOUT": 45.0})
said, evs = run(g)
# 措辭是「不動作」而不是「關」：ArduPilot 的 FS_GCS_ENABLE 是開關，
# PX4 的 NAV_DLL_ACT 是行為列舉，**兩家共通的只有「0＝不動作」**
chk("FS_GCS_ENABLE=0 → 報「後備不存在」",
    said and evs and "不動作" in evs[0] and "FS_GCS_ENABLE" in evs[0], evs)
chk("而且說得出後果（代理掛掉沒人接手）",
    evs and "沒有任何人接手" in evs[0], evs[:1])

g = Fake(params={"FS_GCS_ENABLE": 1.0, "FS_GCS_TIMEOUT": 45.0},
         solo=10.0, act=10.0)
said, evs = run(g)
chk("單飛上限 ≤ 處置起算 → 也在這裡一併報（選項 C 退化成 A）",
    said and evs and "退化成 A" in evs[0], evs)

print("\n── 讀不到：與「設錯了」分開 ───────────────────────────")
g = Fake(params={"FS_GCS_ENABLE": 1.0})          # 少一個
said, evs = run(g, now=1000.0)
chk("還在等的時候不下結論", not said and not evs, (said, evs))
said, evs = run(g, now=1000.0 + A.Agent.FS_GCS_GIVEUP_S + 1)
chk("超過時限 → 下結論", said, said)
chk("說的是「讀不到」而不是「設錯了」",
    evs and "讀不到" in evs[0] and "衝突" not in evs[0], evs)
chk("指名缺的是哪一個", evs and "FS_GCS_TIMEOUT" in evs[0], evs)

print("\n── 參數名向驅動要，不是寫死 ───────────────────────────")
# 上游 c585825 把 `gcs_failsafe_params()` 加進三個驅動之後，代理這邊寫死的
# 那份就刪掉了——不留第二個會漂的地方
g = Fake(raw=12)                                 # PX4
g.fs_gcs_asked = 0
g.fs_gcs_asked_at = 0.0
g.request_fs_gcs(1000.0)
# 2026-09-04 起開機自檢除了失聯參數，也讀電量門檻（失聯接管的兩個判準之一
# ——代理不該自己編一個電壓門檻）。**這幾個 chk 當時沒跟著改**，於是這支
# 從那天起一直是紅的，而紅了三天沒有人動它。紅而不修等於把測試訓練成雜訊。
chk("PX4 問的是它自己的參數名（NAV_DLL_ACT／COM_DL_LOSS_T）",
    set(n.decode() if isinstance(n, bytes) else n for n in g.sent)
    == {"NAV_DLL_ACT", "COM_DL_LOSS_T", "BAT_LOW_THR", "BAT_V_EMPTY"}, g.sent)
g2 = Fake(raw=3)
g2.fs_gcs_asked = 0
g2.fs_gcs_asked_at = 0.0
g2.request_fs_gcs(1000.0)
chk("**對照**：ArduPilot 問的是 FS_GCS_*",
    set(n.decode() if isinstance(n, bytes) else n for n in g2.sent)
    == {"FS_GCS_ENABLE", "FS_GCS_TIMEOUT",
        "BATT_LOW_VOLT", "BATT_LOW_MAH"}, g2.sent)
# PX4 的 NAV_DLL_ACT 是**行為列舉不是開關**，共通的只有「0＝不動作」
g3 = Fake(raw=12, params={"NAV_DLL_ACT": 0.0, "COM_DL_LOSS_T": 45.0})
said, evs = run(g3)
chk("PX4：動作＝0 → 報「後備不存在」，訊息用它自己的參數名",
    said and evs and "NAV_DLL_ACT" in evs[0], evs)
g4 = Fake(raw=12, params={"NAV_DLL_ACT": 2.0, "COM_DL_LOSS_T": 45.0})
said, evs = run(g4)
chk("**反向驗證**：非 0 的行為列舉算「有動作」，不是只認 1",
    said and not evs, evs)

print("\n── 未知廠牌：沒有意見就不要製造意見 ───────────────────")
g = Fake(raw=99, params={})                      # 驅動不認得
said, evs = run(g)
chk("未知廠牌 → 跳過且不報警", said and not evs, evs)
g = Fake(raw=99)
g.fs_gcs_asked = 0
g.fs_gcs_asked_at = 0.0
g.request_fs_gcs(1000.0)
chk("**也不送請求**（拿別家的參數名去問，只會被誤讀成「讀不到＝沒設定」）",
    not g.sent, g.sent)
g = Fake(raw=None, params={})                    # 還沒收到心跳
said, evs = run(g)
chk("廠牌還不知道時**不下結論**（那是「還沒到」不是「沒有意見」）",
    not said and not evs, (said, evs))

print("\n── 一次就好：不週期重讀 ───────────────────────────────")
g = Fake(raw=3, params=None)
g.fs_gcs_asked = 0
g.fs_gcs_asked_at = 0.0
g.request_fs_gcs(1000.0)
n1 = len(g.sent)
chk("第一次會問（四個參數各送三次）", n1 == 12, n1)
g.request_fs_gcs(1000.5)
chk("重試間隔內不重問", len(g.sent) == n1, len(g.sent))
g.request_fs_gcs(1000.0 + A.Agent.FS_GCS_RETRY_S + 1)
chk("沒回應時會再問一次（開機期的丟包，不是週期重讀）", len(g.sent) == 24,
    len(g.sent))
g.request_fs_gcs(1000.0 + A.Agent.FS_GCS_RETRY_S * 5)
chk("**問過兩輪就永遠停手**（參數不會自己變）", len(g.sent) == 24, len(g.sent))
# 拿到答案之後也不再問
g2 = Fake(params={"FS_GCS_ENABLE": 1.0, "FS_GCS_TIMEOUT": 45.0,
                  "BATT_LOW_VOLT": 14.0, "BATT_LOW_MAH": 0.0})
g2.fs_gcs_asked = 0
g2.request_fs_gcs(1000.0)
chk("**反向驗證**：四個都拿到了就不開口", not g2.sent, g2.sent)

print("\n── PARAM_VALUE 真的解得進來 ───────────────────────────")
# 前面全是「拿到參數之後怎麼判」；這一段驗的是另一半：**參數真的從飛控的
# 訊息裡被解出來了**。算法對但沒接上輸入，結果就是永遠走「讀不到」那條
def param(name, value, ptype=None, as_bytes=False):
    pid = name.encode("ascii") + b"\x00" * (16 - len(name)) if as_bytes else name
    return types.SimpleNamespace(get_type=lambda: "PARAM_VALUE",
                                 param_id=pid, param_value=value,
                                 param_type=ptype)


def heartbeat(autopilot=3):
    return types.SimpleNamespace(
        get_type=lambda: "HEARTBEAT", get_srcSystem=lambda: 1, base_mode=0,
        custom_mode=0, system_status=3, autopilot=autopilot, type=2)


m = A.Monitor()
m.feed(heartbeat())
m.feed(param("FS_GCS_TIMEOUT", 45.0))
chk("字串型 param_id 解得出來", m.state["params"].get("FS_GCS_TIMEOUT") == 45.0,
    m.state.get("params"))
m.feed(param("FS_GCS_ENABLE", 1.0, as_bytes=True))
chk("**bytes 且補 \\x00 的 param_id 也解得出來**（pymavlink 兩種都可能給）",
    m.state["params"].get("FS_GCS_ENABLE") == 1.0, m.state.get("params"))
# 值的解讀是廠牌方言：ArduPilot 寫數值本身，PX4 把整數的位元組塞進 float32
m2 = A.Monitor()
m2.feed(heartbeat(autopilot=12))                 # PX4
m2.feed(param("SOME_INT", 4.2e-45, ptype=2))     # 位元其實是整數 3
chk("PX4 的整數參數走驅動的位元重解讀（不是原樣留著）",
    m2.state["params"]["SOME_INT"] == 3, m2.state["params"])
m3 = A.Monitor()
m3.feed(heartbeat(autopilot=3))                  # ArduPilot
m3.feed(param("SOME_INT", 3.0, ptype=2))
chk("**對照**：ArduPilot 同型別是數值本身，原樣留著",
    m3.state["params"]["SOME_INT"] == 3.0, m3.state["params"])

print("\n── 結論只說一次 ───────────────────────────────────────")
g = Fake(params={"FS_GCS_ENABLE": 1.0, "FS_GCS_TIMEOUT": 15.0})
run(g)
n = len(g.outbox.unsent())
run(g); run(g)
chk("重複呼叫不會重複刷事件", len(g.outbox.unsent()) == n, len(g.outbox.unsent()))

print("\n" + ("全部通過" if ok else "**有未通過項目**"))
print("**注意：這是邏輯測試，不是真機覆核。** FS_GCS_TIMEOUT 在這台飛控上"
      "存不存在、值域上限多少，要接飛控讀了才算數。")
sys.exit(0 if ok else 1)
