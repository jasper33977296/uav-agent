"""PX4 驅動（issue 026 B2）。

內容是從 `apps/command/app/mav.py` 的 `dialect()`、`apps/backend/app/dialect.py`
與 `apps/command/app/capabilities.py` **原樣提取**的 PX4 分支——B2 是搬遷，不是
改寫。任何行為變更都算回歸。
"""
import struct

from .driver import Limit

NAME = "px4"
AUTOPILOT_RAW = 12                  # MAV_AUTOPILOT_PX4

# DO_SET_MODE 的 param2/param3（main_mode, sub_mode）
MODES = {
    "position": (3, 0),   # POSCTL（手動位置控制：搖桿→速度，鬆手懸停）
    "mission":  (4, 4),   # AUTO.MISSION
    "hold":     (4, 3),   # AUTO.LOITER
    "rtl":      (4, 5),   # AUTO.RTL
    "land":     (4, 6),   # AUTO.LAND
}

# custom_mode 是 main<<16|sub<<24 的 union
_MAIN = {1: "MANUAL", 2: "ALTCTL", 3: "POSCTL", 5: "ACRO",
         6: "OFFBOARD", 7: "STABILIZED", 8: "RATTITUDE"}
_AUTO = {1: "READY", 2: "TAKEOFF", 3: "HOLD", 4: "MISSION",
         5: "RETURN_TO_LAUNCH", 6: "LAND", 8: "FOLLOW_ME", 9: "PRECLAND"}

#: MAV_PARAM_TYPE 的整數型別（1..8）；9=REAL32、10=REAL64 是真浮點。
_INT_PARAM_TYPES = frozenset(range(1, 9))

CAP_KEYS = ["arm", "takeoff", "land", "rtl", "hold",
            "mission_upload", "mission_start", "mission_fly"]


class Px4Driver:
    name = NAME
    autopilot_raw = AUTOPILOT_RAW
    modes = MODES

    #: 任務線序：PX4 不把 home 當 seq 0
    home_at_seq0 = False
    #: 2026-08-12 PX4 SITL 實測：RTL 配 frame 0/3/5/6 全拒、只有 2 過，
    #: 而且是**拒收整包任務**（MAV_MISSION_UNSUPPORTED）
    no_coord_frames = frozenset({2})
    #: NAV_TAKEOFF 的 param7 是絕對海拔
    takeoff_alt_is_relative = False
    #: 不需要先進 GUIDED
    takeoff_needs_guided = False

    #: PX4 不需要任何訊息層改名——它發的就是我們的標準形。
    MESSAGE_ADJUSTMENTS = ()


    # ── 參數值解碼 ──────────────────────────────────────────────────
    def decode_param(self, value: float, param_type=None):
        """PX4 把整數參數的**位元組原樣放進 float32 欄位**（不是數值轉型）。

        所以收到的 float 要**按 param_type 重新解讀位元**才拿得到真值。不做的話
        存下來的是非正規化浮點數——`BAT_N_CELLS` 的 3 會變成 4.2e-45，
        `CAL_ACC0_ID` 的 1310988 會變成 1.8e-39。

        **危險之處在於它長得像一個合理的數字**：不會拋例外、不會有紅字，只是
        參數快照裡 13% 的值（實測 851 個參數中的 112 個）默默是垃圾——而那份
        快照的存在理由正是「這一趟到底是用什麼設定飛的」（issue 021 Phase 2）。

        2026-08-13 查 PX4 為什麼不讓進 POSCTL 時順帶發現：`COM_RC_IN_MODE`
        顯示 1.4e-45，那不是任何合理的參數值，而是整數 1 的位元被當成浮點數。
        """
        if param_type in _INT_PARAM_TYPES:
            return struct.unpack("<i", struct.pack("<f", value))[0]
        return value

    # ── 訊息層 ──────────────────────────────────────────────────────
    def adjust_incoming(self, msg):
        return msg

    def adjust_outgoing(self, msg):
        return msg

    # ── 模式 ────────────────────────────────────────────────────────
    def decode_mode(self, custom_mode: int) -> str:
        if not custom_mode:          # 0＝尚未設定模式（開機瞬間），不顯 MODE_0
            return "—"
        main, sub = (custom_mode >> 16) & 0xFF, (custom_mode >> 24) & 0xFF
        if main == 4:
            return _AUTO.get(sub, f"AUTO_{sub}")
        return _MAIN.get(main, f"MODE_{main}")

    def decode_verb(self, custom_mode: int) -> str | None:
        """custom_mode → **廠牌無關的動詞**（hold／mission／rtl／land／position）。

        顯示層仍用原廠模式名（那是機端真的在跑的東西，不翻譯）；本方法補的是
        **語意層**，讓 UI 與 MCP 能問「這台在不在 hold」而不必知道 PX4 叫 HOLD、
        ArduPilot 叫 LOITER（UI/UX 2026-08-12 要求）。

        用 `mode_matches` 反查而不是另外維護一份反向表——兩份表一定會漂移。
        """
        if not custom_mode:
            return None
        for verb in self.modes:
            if self.mode_matches(custom_mode, verb):
                return verb
        return None

    def encode_mode(self, mode: str) -> tuple[int, int]:
        return MODES[mode]

    def mode_matches(self, custom_mode: int, mode: str) -> bool:
        return ((custom_mode >> 16) & 0xFF, (custom_mode >> 24) & 0xFF) == MODES[mode]

    # ── 動作 ────────────────────────────────────────────────────────
    def takeoff_plan(self, alt: float, ground_amsl: float | None) -> dict:
        """PX4 的 NAV_TAKEOFF param7 是**絕對海拔**，要用地面海拔＋目標高度。

        空白參數（偏航／經緯度）用 NaN＝「用當前值」。
        """
        if ground_amsl is None:
            raise ValueError("PX4 起飛需要地面海拔（GPS/EKF 未就緒）")
        return {"needs_guided": self.takeoff_needs_guided,
                "param7": ground_amsl + alt,
                "blank": float("nan"), "alt_semantics": "amsl"}

    def wire_seq(self, index: int) -> int:
        return index              # PX4 不補 home，索引即 seq

    def mission_line(self, items: list[dict]) -> list[dict]:
        return items                 # PX4 不把 home 當 seq 0

    # ── 連線與就緒 ──────────────────────────────────────────────────
    def on_connect(self) -> list:
        return []                    # PX4 預設就串流遙測

    def keepalive(self) -> list:
        return []

    def readiness_signals(self) -> tuple[str, ...]:
        return ("prearm", "ekf")     # PX4 有 PREARM_CHECK 健康位

    # ── 值域與能力 ──────────────────────────────────────────────────
    def limits(self) -> dict[str, Limit]:
        # **不照抄 QGC 的數字**——那等於把別人的驗證當成我們的驗證。
        # B3 一致性測試實測後才會升成 sitl。
        return {"takeoff_alt_m": Limit(confidence="unverified")}

    def capabilities(self, ctx: dict | None = None):
        return {k: "ok" for k in CAP_KEYS}, {}
