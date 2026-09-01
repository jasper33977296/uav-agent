"""自駕儀驅動層（issue 026）。

用法：

    from autopilot import get_driver
    drv = get_driver(msg.autopilot)      # MAV_AUTOPILOT_* 原值
    mode = drv.decode_mode(custom_mode)

**廠牌判斷只在 `get_driver()` 這一處。** 呼叫端拿到驅動之後不得再問「這是哪一
家」——那正是我們要消滅的東西（見 `doc/autopilot-driver-architecture.md`）。
"""
from .ardupilot import ArduPilotDriver
from .driver import Limit, MessageEquivalence
from .px4 import Px4Driver

__all__ = ["get_driver", "autopilot_name", "Limit", "MessageEquivalence",
           "Px4Driver", "ArduPilotDriver", "UnknownDriver"]

_BY_RAW = {Px4Driver.autopilot_raw: Px4Driver, ArduPilotDriver.autopilot_raw: ArduPilotDriver}
_NAMES = {Px4Driver.autopilot_raw: Px4Driver.name,
          ArduPilotDriver.autopilot_raw: ArduPilotDriver.name}

CAP_KEYS = ["arm", "takeoff", "land", "rtl", "hold",
            "mission_upload", "mission_start", "mission_fly"]


def autopilot_name(raw) -> str:
    return _NAMES.get(raw, "unknown")


class UnknownDriver:
    """非 MAVLink 或未知自駕儀：全鎖，且**不假裝懂它的方言**。

    模式解讀刻意沿用 PX4 的解法（維持既有行為），但能力全 `unsupported`——
    看得到、送不出去。
    """
    name = "unknown"
    autopilot_raw = None
    modes: dict = {}
    home_at_seq0 = False
    takeoff_alt_is_relative = False
    takeoff_needs_guided = False
    #: 無座標任務項（RTL／DO_*／CONDITION_*）可接受的 frame。
    #:
    #: **這是方言，不是通則。** 2026-08-12 在 PX4 SITL 實測「RTL 配 frame
    #: 0/3/5/6 全拒、只有 2 過」，然後那條規則被當成通則寫進預檢——結果 2026-08-26
    #: 擋下了一份**從 ArduPilot 自己下載回來的**任務：ArduPilot 存 RTL 用
    #: frame 0，下載時原樣回報，於是我們拿 PX4 的規則去否定 ArduPilot 自己的
    #: 表示法。**一家的實測結果被當成兩家的事實，正是 026 要收掉的那種洩漏。**
    no_coord_frames: frozenset = frozenset({2})
    MESSAGE_ADJUSTMENTS = ()

    def __init__(self):
        self._px4 = Px4Driver()

    def adjust_incoming(self, msg):
        return msg

    def adjust_outgoing(self, msg):
        return msg

    def decode_mode(self, custom_mode: int) -> str:
        # 既有行為：unknown 暫按 PX4 解（B2 是搬遷，不改行為）
        return self._px4.decode_mode(custom_mode)

    def decode_verb(self, custom_mode: int) -> str | None:
        return None                 # 不知道它的模式表，就不假裝解得出語意

    def decode_param(self, value: float, param_type=None):
        return value                # 不知道它的慣例，原樣留著

    def encode_mode(self, mode: str):
        raise KeyError(f"未知自駕儀，無法編碼模式 {mode!r}")

    def mode_matches(self, custom_mode: int, mode: str) -> bool:
        return False

    def takeoff_plan(self, alt, ground_amsl):
        raise KeyError("未知自駕儀，無起飛序列")

    def wire_seq(self, index):
        return index

    def mission_line(self, items):
        return items

    def on_connect(self):
        return []

    def keepalive(self):
        return []

    def readiness_signals(self):
        return ()

    def limits(self):
        return {}

    def capabilities(self, ctx=None):
        r = "非 MAVLink 或未知自駕儀，不支援指令"
        return {k: "unsupported" for k in CAP_KEYS}, {k: r for k in CAP_KEYS}


def get_driver(autopilot_raw):
    """依 `MAV_AUTOPILOT_*` 取驅動。未知廠牌回 `UnknownDriver`（不丟例外——
    未知機仍要能顯示遙測，只是不能下指令）。"""
    cls = _BY_RAW.get(autopilot_raw)
    return cls() if cls else UnknownDriver()
