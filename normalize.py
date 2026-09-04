"""把飛控遙測正規化成協定 §4.2 的 `derived` 欄位（026 B4-b）。

**驅動是逐字搬過來的（`autopilot/`），這一支不是。** 就緒判定原本住在地面站的
`apps/backend/app/state.py::readiness()`，不在 `libs/autopilot/` 裡——所以它是
真正被「移植」的那一半，也是**唯一有機會搬錯的地方**。

移植的驗收標準只有一條：**同樣的輸入，兩邊算出完全相同的輸出**
（架構文件 §9.3 的 B4-d）。所以：

* 判定順序、原因字串、遮罩常數全部照抄，**包括中文字面**——`not_ready_reasons`
  會直接顯示給操作員，而地面站的 crosscheck 是逐鍵比對，差一個字就是不一致。
* `tools/driver-parity.py` 把地面站那份與這份同時載進來，用同一批輸入掃過去
  比對。**兩份實作並存的這段期間是唯一能做這個比對的窗口**（架構文件 §9.3）。

## 三態，而且三種「不知道」不是同一件事

* `ready = None`：**我算過了，但沒有依據**。剛連上只收到心跳時對預檢／EKF
  一無所知——那時候說「就緒」是憑空斷言。**缺乏證據不是通過的理由。**
* `ready` 這個鍵**整個不存在**：代理沒說（舊版代理、或這條路壞了）。
  地面站據此不做任何推論——把缺欄位讀成 `false` 會讓所有還沒升級的機都
  變成不可飛，與 039 複裁 A 的 `rc_link` 同一個教訓。
* `ready = False`：**我算過了，而且它不能飛**，`not_ready_reasons` 說得出為什麼。

本模組只負責前兩者的分界；「整個鍵不存在」由呼叫端決定（見 `agent.py`）。
"""
import time

from autopilot import autopilot_name, get_driver

#: MAV_STATE（`mavlink_rx.py` 的 `_MAV_STATE`，逐字）
MAV_STATE = {0: "UNINIT", 1: "BOOT", 2: "CALIBRATING", 3: "STANDBY",
             4: "ACTIVE", 5: "CRITICAL", 6: "EMERGENCY", 7: "POWEROFF",
             8: "FLIGHT_TERMINATION"}
FAILSAFE_STATES = ("CRITICAL", "EMERGENCY", "FLIGHT_TERMINATION")

#: 預檢失敗原因多久沒再聽到就當它已經解決（秒）。**與地面站
#: `state.PREARM_TTL_S` 逐字相同**——兩份實作漂了，`driver-parity` 就會紅。
#: ArduPilot 實測約每分鐘重講一次，取 3 分鐘＝漏掉兩次也還算數。
PREARM_TTL_S = 180.0


def prearm_said(s: dict) -> list[str]:
    """機上此刻還在講的預檢失敗原因（`agent.py` 從 STATUSTEXT 收進 state）。

    **要過期，而判準是「沒有再聽到」不是「時間到了」**：ArduPilot 只在被擋住
    時週期性重講，所以很久沒聽到多半代表那一項已經解決——把它繼續掛在畫面上，
    就是拿一個已經不成立的理由擋人。
    """
    msgs = s.get("prearm_msgs") or {}
    now = time.monotonic()
    return [t for t, at in sorted(msgs.items(), key=lambda kv: -kv[1])
            if now - at < PREARM_TTL_S]

#: SYS_STATUS 的感測器位元 → 人話名（`mavlink_rx.py` 的 `_SENSOR_BITS`，逐字）。
#: **名字是契約的一部分**：它們會出現在 `not_ready_reasons` 裡被逐字比對
SENSOR_BITS = [
    ("陀螺儀", 1 << 0),              # MAV_SYS_STATUS_SENSOR_3D_GYRO
    ("加速度計", 1 << 1),            # ..._3D_ACCEL
    ("磁力計", 1 << 2),              # ..._3D_MAG
    ("氣壓計", 1 << 3),              # ..._ABSOLUTE_PRESSURE
    ("GPS", 1 << 5),                 # ..._GPS
    ("RC 接收器", 1 << 16),          # ..._RC_RECEIVER（與 039 複裁 A 同一位元）
    ("AHRS 姿態解算", 1 << 21),      # MAV_SYS_STATUS_AHRS
    ("電池", 1 << 25),               # ..._BATTERY
]
#: MAV_SYS_STATUS_PREARM_CHECK
PREARM_CHECK = 1 << 28
#: 就緒所需的 EKF 位元：姿態＋水平速度＋水平絕對位置（`dialect.py` 逐字）。
#: 兩家只在 bit 1..512 同義，這三個位元在範圍內——**範圍見驅動的
#: `MESSAGE_ADJUSTMENTS.safe_field_bits`，不要憑印象擴大**
EKF_READY_BITS = 0x0013


def ekf_msg_types() -> tuple[str, ...]:
    """哪些訊息型別要當成 EKF 狀態讀。

    **從所有驅動的 `MESSAGE_ADJUSTMENTS` 推導，不是手寫清單**（與地面站的
    `dialect._ekf_types()` 同一個做法與同一個理由）：手寫的話新增驅動時這裡
    會漏，而漏掉的症狀是「某廠牌的 EKF 永遠是 None」——沒有任何錯誤訊息。
    """
    names = {"ESTIMATOR_STATUS"}
    for raw in (12, 3):
        for eq in get_driver(raw).MESSAGE_ADJUSTMENTS:
            if eq.dst_type == "ESTIMATOR_STATUS":
                names.add(eq.src_type)
    return tuple(sorted(names))


EKF_MSG_TYPES = ekf_msg_types()


def ekf_ready(flags: int) -> bool:
    return (flags & EKF_READY_BITS) == EKF_READY_BITS


def prearm_label(autopilot_raw) -> str:
    """預檢未過的原因文字。**廠牌名不寫死**（地面站 `dialect.prearm_label` 逐字）。"""
    return {"px4": "PX4", "ardupilot": "ArduPilot"}.get(
        autopilot_name(autopilot_raw), "自駕儀")


def readiness(s: dict) -> tuple[bool | None, list]:
    """就緒判定＋不就緒原因。**地面站 `state.readiness()` 的移植，逐行對應。**

    **沒有依據時回 None（未知），不回 True。** 「就緒」是對飛安狀態的斷言，
    而剛連上、只收到心跳的那段時間，我們對預檢／GPS／感測器一無所知。
    地面站 2026-08-12 實測抓到過：位置／GPS／電量全 null，卻因為「沒有任何
    反對證據」而回報 ready=true，前端據此點了綠燈。**缺乏證據不是通過的理由。**
    """
    prearm_ok = s.get("prearm_ok")
    ekf_ok = s.get("ekf_ok")
    gps_fix = s.get("gps_fix_raw")
    mav_state = s.get("mav_state")

    reasons = []
    if prearm_ok is False:
        # **與地面站 `state.py::readiness()` 逐字相同**（026 B4-d 的契約：
        # 同樣的輸入必須有同樣的輸出，`tools/driver-parity.py` 每次都比）。
        label = prearm_label(s.get("autopilot_raw"))
        said = prearm_said(s)
        if said:
            # **說得出是哪一項就說**——「預檢未過」是一句不可行動的話，
            # 而「Battery 1 low voltage failsafe」是一句可以去處理的話
            reasons += [f"{label} 預檢未過：{t}" for t in said]
        else:
            # 沒聽到機上說（韌體不講、或它講過而我們是之後才接上的）。
            # **要說出「不知道是哪一項」**，不要讓它看起來像一句完整的原因
            reasons.append(f"{label} 預檢未過（機上還沒說是哪一項）")
    reasons += [f"感測器異常：{n}" for n in (s.get("sensors_unhealthy") or [])]
    if ekf_ok is False:
        reasons.append("EKF 未就緒")
    if gps_fix is not None and gps_fix < 3:
        reasons.append(f"GPS 未定位（fix={gps_fix}）")
    if mav_state in FAILSAFE_STATES:
        reasons.append(f"failsafe 狀態：{mav_state}")
    if not reasons:
        # 沒有反對證據 ≠ 就緒——還要有**權威依據**才敢斷言「可飛」。
        # 權威訊號只有 prearm_ok（PX4 的 arming checks 總結）與 ekf_ok；
        # **GPS 好不算數**：GPS 定位良好但預檢未過（羅盤未校正、EKF 未收斂…）
        # 完全可能，拿 GPS 當「就緒」的依據就是用次級訊號冒充權威判斷。
        # 反向證據仍然有效——GPS 未定位這類「我們知道它不行」照樣回 False。
        if prearm_ok is None and ekf_ok is None:
            return None, ["尚未收到預檢／EKF 狀態，無法判定就緒"]
    return (not reasons and prearm_ok is not False), reasons


def sensors_unhealthy(present: int, enabled: int, health: int) -> list:
    """**present 且 enabled 且不 health** 才算異常（地面站逐字）。

    三個條件缺一不可：沒裝的感測器不健康是廢話，關掉的也是。
    """
    return [n for n, bit in SENSOR_BITS
            if (present & bit) and (enabled & bit) and not (health & bit)]


def prearm(present: int, enabled: int, health: int, seen: bool) -> tuple[bool | None, bool]:
    """PX4 預檢總結果，**黏性**。回 (prearm_ok, 更新後的 seen)。

    實測（地面站，SITL PX4 1.14）：PREARM 位元只出現在 health 遮罩，
    enabled/present 都不設——所以**見過一次就信 health 位元**；從未見過
    ＝韌體不支援，維持 None（就緒判定退回次級訊號）。

    ArduPilot 不回報這個位元（見驅動的 `readiness_signals()` 只有 `ekf`），
    所以在 ArduPilot 上這裡一直是 None，那是如實缺席不是壞掉。
    """
    seen = seen or bool((present | enabled | health) & PREARM_CHECK)
    return (bool(health & PREARM_CHECK) if seen else None), seen


def derived(s: dict) -> dict:
    """協定 §4.2 的正規化欄位。呼叫端負責把它併進 `state.derived`。"""
    drv = get_driver(s.get("autopilot_raw"))
    custom = s.get("custom_mode")
    ready, reasons = readiness(s)
    out = {
        # **判斷用 verb、顯示用 name**（協定 §4.2）。custom_mode 還沒收到時
        # 兩者都留 None——不是猜一個，也不是送空字串
        "mode_verb": drv.decode_verb(custom) if custom is not None else None,
        "mode_name": drv.decode_mode(custom) if custom is not None else None,
        "ready": ready,
        "not_ready_reasons": reasons,
        "prearm_ok": s.get("prearm_ok"),
        "ekf_ok": s.get("ekf_ok"),
    }
    return out


def driver_info() -> dict:
    """`hello` 的 `driver` 欄位（協定 §4.2，026 待決點 6）。

    **`limits` 照驅動說的送，包括 `unverified`。** `Limit` 的規格是「不知道就
    回 unverified 且三值皆 None，不得回通用值」——回一個通用預設會讓地面站
    無法區分「這是該廠牌的真實限制」與「這是我們湊的」。送 `unverified` 上去
    看起來像沒資訊，但那正是實話。
    """
    from autopilot._provenance import PORTED_FROM, content_hash, version
    return {"name": "uav-agent/autopilot", "version": version(),
            "ported_from": PORTED_FROM, "content_hash": content_hash()}


def driver_limits(autopilot_raw) -> dict:
    """這台機的廠牌驅動宣告的限制值，攤平成 JSON 送得出去的形狀。"""
    out = {}
    for key, lim in (get_driver(autopilot_raw).limits() or {}).items():
        out[key] = {"min": lim.min, "max": lim.max, "default": lim.default,
                    "confidence": lim.confidence, "source": lim.source}
    return out
