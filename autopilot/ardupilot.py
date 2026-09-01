"""ArduPilot（Copter）驅動（issue 026 B2）。

內容是從 `apps/command/app/mav.py` 的 `dialect()`、`apps/backend/app/dialect.py`
與 `apps/command/app/capabilities.py` **原樣提取**的 ArduPilot 分支——B2 是搬遷，
不是改寫。任何行為變更都算回歸。

Plane／Rover 的模式表不同，待驗；本檔目前只涵蓋 Copter。
"""
from .driver import Limit, MessageEquivalence

NAME = "ardupilot"
AUTOPILOT_RAW = 3                   # MAV_AUTOPILOT_ARDUPILOTMEGA
#: **這裡原本有第三份 `GCS_SYSID = 254`，2026-09-01 刪除。** 它定義了卻從來
#: 沒有被用到，而且**已經漂移**：`mav.py` 與 `capabilities.py` 在 08-25 一起
#: 改成 255，這份沒跟上。command 服務開機時會比對那兩處不一致就拒絕啟動，
#: 但那個檢查只認得兩處——**漂掉的正好是它沒看的第三處**。
#:
#: 修法不是把它同步、也不是把檢查擴成三處，是**刪掉**：GCS sysid 是我方的
#: 身分，不是廠牌方言，本來就不該住在驅動裡。少一份就少一個會漂的地方，
#: 而多一道檢查只是讓漂移變得看得見而已。（機端 session 2026-09-01 移植
#: 驅動時發現，照「搬家不是改寫」原封留著並回報——那個判斷是對的。）

# DO_SET_MODE 的 **param2 直接是模式號**，沒有 sub。權威來源：
# reference/ardupilot/copter-mode.h。**gap-analysis 列為 critical**——送錯數字
# 會切到完全不同的模式。每個值都要在 SITL 讀回 HEARTBEAT 確認真的切到。
MODES = {
    "position": 16,   # POSHOLD：位置保持＋手動介入（對應 PX4 的 POSCTL 語意）
    "mission":   3,   # AUTO
    "hold":      5,   # LOITER
    "rtl":       6,   # RTL
    "land":      9,   # LAND
    "guided":    4,   # GUIDED（起飛序列要先進這個模式）
}

# custom_mode 直接是模式號
_COPTER = {0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED",
           5: "LOITER", 6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT",
           13: "SPORT", 16: "POSHOLD", 17: "BRAKE", 18: "THROW",
           20: "GUIDED_NOGPS", 21: "SMART_RTL", 27: "AUTO_RTL"}

CAP_KEYS = ["arm", "takeoff", "land", "rtl", "hold",
            "mission_upload", "mission_start", "mission_fly"]

#: 015 實測：**ArduPilot 預設幾乎不送遙測**——只收得到 HEARTBEAT／PARAM_VALUE／
#: STATUSTEXT／TIMESYNC 四種。送一次 REQUEST_DATA_STREAM 後變 32 種。
STREAM_HZ = 4
STREAM_REQ_S = 30.0


class ArduPilotDriver:
    name = NAME
    autopilot_raw = AUTOPILOT_RAW
    modes = MODES

    #: 任務線序：ArduPilot 把 home 當 seq 0，實際航點從 seq 1 起算
    home_at_seq0 = True
    #: ArduPilot 存 RTL 用 `MAV_FRAME_GLOBAL`(0)，下載任務時原樣回報
    #: （2026-08-26 實機證據：從這台機下載回來的 RTL 項就是 frame 0）。
    #: **它送得出來就一定收得下**，所以 0 與 2 都接受
    no_coord_frames = frozenset({0, 2})
    #: NAV_TAKEOFF 的 param7 是相對高度（送絕對海拔會差一整個地面海拔）
    takeoff_alt_is_relative = True
    #: Copter 必須先進 GUIDED 才能 arm 與起飛
    takeoff_needs_guided = True

    #: 訊息層等價：ArduPilot 發 EKF_STATUS_REPORT，PX4 發 ESTIMATOR_STATUS。
    #: **帶適用範圍**——只有 bit 1..512 逐位同義（見 B0 的實測）。
    MESSAGE_ADJUSTMENTS = (
        MessageEquivalence(
            "EKF_STATUS_REPORT", "ESTIMATOR_STATUS",
            safe_field_bits={"flags": 0x03FF},
            note="bit 1024 兩邊不同義（ESTIMATOR_GPS_GLITCH vs EKF_UNINITIALIZED）；"
                 "bit 32768 EKF_GPS_GLITCHING 為 ArduPilot 專有，無對應"),
    )


    # ── 參數值解碼 ──────────────────────────────────────────────────
    def decode_param(self, value: float, param_type=None):
        """ArduPilot 的慣例是**把數值本身寫進 float32**（不是位元重解讀）——
        與 PX4 相反。

        ⚠️ **這一條尚未實測驗證**（2026-08-13）。PX4 那邊是實測確認的（參數快照
        裡出現非正規化浮點數）；ArduPilot 這邊採用的是其文件慣例，但我們自己
        還沒抓一份 ArduPilot 的參數快照來對帳。**在對帳之前，不要把這裡的
        「原樣回傳」當成已驗證的事實**——它只是目前最合理的預設。

        驗法：抓一份 ArduPilot 快照，看整數型參數（例如 `SYSID_THISMAV` 應為
        10、`FRAME_CLASS`）是不是合理的整數值；若出現非正規化浮點數就要改成
        與 PX4 相同的位元重解讀。
        """
        return value

    # ── 訊息層：只改名，不解讀 ──────────────────────────────────────
    def adjust_incoming(self, msg):
        """把 ArduPilot 專有的訊息名正規化成標準形。

        **只做宣告過的改名**，不判斷任何值的意義（見介面規格 §2.1）。
        """
        return msg

    def adjust_outgoing(self, msg):
        return msg

    def normalized_type(self, msg_type: str) -> str:
        """訊息型別的標準形——`MESSAGE_ADJUSTMENTS` 的查表結果。"""
        for eq in self.MESSAGE_ADJUSTMENTS:
            if eq.src_type == msg_type:
                return eq.dst_type
        return msg_type

    # ── 模式 ────────────────────────────────────────────────────────
    def decode_mode(self, custom_mode: int) -> str:
        """**ArduPilot 的 custom_mode 0 是 STABILIZE，不是「未設定」。**

        這裡原本抄了 PX4 的 `if not custom_mode: return "—"`——那條對 PX4 成立
        （它把 main/sub 模式打包在高位元組，0＝什麼都沒設），對 ArduPilot 是錯的。
        後果不只是顯示難看：STABILIZE 正是**飛手用遙控器手飛的模式**，
        而我們用 `mode_verb` 判斷「飛手是不是接管了」。譯成「—」時 verb 是
        None，地面站就分不出「飛手拿著」與「不知道現在什麼模式」——
        那是狀態機文件 §1 說「必須是顯式狀態」的那一格。
        2026-08-26 實機看到：機在 STABILIZE，畫面顯示「—」。
        """
        return _COPTER.get(custom_mode, f"MODE_{custom_mode}")

    def decode_verb(self, custom_mode: int) -> str | None:
        """custom_mode → **廠牌無關的動詞**（hold／mission／rtl／land／position）。

        顯示層仍用原廠模式名（那是機端真的在跑的東西，不翻譯）；本方法補的是
        **語意層**，讓 UI 與 MCP 能問「這台在不在 hold」而不必知道 PX4 叫 HOLD、
        ArduPilot 叫 LOITER（UI/UX 2026-08-12 要求）。

        用 `mode_matches` 反查而不是另外維護一份反向表——兩份表一定會漂移。

        **這裡沒有 `if not custom_mode` 的捷徑**（PX4 那份有，因為它的 0 是
        「未設定」）。ArduPilot 的 0 是 STABILIZE，它本來就不在動詞表裡，
        所以走迴圈一樣回 None——但**不要留一份「0＝什麼都不是」的假設在
        ArduPilot 的檔案裡**，那正是上面 decode_mode 踩過的坑。
        """
        for verb in self.modes:
            if self.mode_matches(custom_mode, verb):
                return verb
        return None

    def encode_mode(self, mode: str) -> tuple[int, int]:
        return (MODES[mode], 0)

    def mode_matches(self, custom_mode: int, mode: str) -> bool:
        return custom_mode == MODES[mode]

    # ── 動作 ────────────────────────────────────────────────────────
    def takeoff_plan(self, alt: float, ground_amsl: float | None) -> dict:
        """Copter 的 NAV_TAKEOFF param7 是**相對高度**（送絕對海拔會差一整個
        地面海拔、數百公尺），而且必須**先進 GUIDED 才能 arm 與起飛**。

        空白參數用 **0.0 不用 NaN**——實測 2026-08-12：送 NaN 的 NAV_TAKEOFF
        連 ACK 都沒有，指令被靜默丟棄。它的慣例是 0＝當前位置。
        """
        return {"needs_guided": self.takeoff_needs_guided, "param7": alt,
                "blank": 0.0, "alt_semantics": "relative"}

    def wire_seq(self, index: int) -> int:
        return index + 1          # home 佔 seq 0（2026-08-25 SITL 實測：執行任務時 seq 從 1 起）

    def mission_line(self, items: list[dict]) -> list[dict]:
        """ArduPilot 把 **home 當 seq 0**，實際航點從 seq 1 起算。"""
        if not items:
            return items
        f = items[0]
        home = {**f, "seq": 0, "command": 16,      # MAV_CMD_NAV_WAYPOINT
                "frame": 0,                        # MAV_FRAME_GLOBAL
                "p1": 0, "p2": 0, "p3": 0, "p4": 0}
        return [home] + [{**it, "seq": i + 1} for i, it in enumerate(items)]

    # ── 連線與就緒 ──────────────────────────────────────────────────
    def on_connect(self) -> list:
        return [("REQUEST_DATA_STREAM", STREAM_HZ)]

    def keepalive(self) -> list:
        """**定期補送而不是只在註冊時送一次**：串流率設在自駕儀端，機端重開機、
        換連線通道、或我方重連之後就沒了——只送一次的話，那些情況下會靜默失去
        全部遙測（只剩心跳，看起來還「連著」）。
        """
        return [("REQUEST_DATA_STREAM", STREAM_HZ)]

    def readiness_signals(self) -> tuple[str, ...]:
        """**沒有 PREARM**：ArduPilot 不回報 PREARM_CHECK 位，只能靠 EKF。

        如實缺席讓 `readiness()` 在只有心跳時回 None（未知），而不是拿次級訊號
        （GPS 好）冒充權威判斷。
        """
        return ("ekf",)

    # ── 值域與能力 ──────────────────────────────────────────────────
    def limits(self) -> dict[str, Limit]:
        return {"takeoff_alt_m": Limit(confidence="unverified")}

    def gcs_failsafe_params(self) -> dict[str, str]:
        """地面站失聯 failsafe 的參數名（issues/033 §1.3，2026-09-01 裁定）。

        **這是真的方言**：同一件事兩家的參數名與語意都不同，而且它是飛安設定
        ——`FS_GCS_ENABLE` 必須開、`FS_GCS_TIMEOUT` 必須大於代理的
        `LINK_LOSS_MAX_SOLO_S`（30s），否則飛控會搶在代理的分狀態處置之前
        動作，039 選項 C 的每一格都不會發生。

        **值不在這裡**：這裡只回參數叫什麼名字。值是逐台機的事實，要讀回來
        （由代理讀——它就在飛控旁邊，而且手上有自己的門檻值可以比對）。
        """
        return {"enable": "FS_GCS_ENABLE", "timeout_s": "FS_GCS_TIMEOUT"}

    def capabilities(self, ctx: dict | None = None):
        """015 驗收進行中：**逐鍵開**，不整組開。只有實際驗過的鍵才是 ok。"""
        r = "ArduPilot 方言未經 SITL 驗證，僅觀察（見 issue 015）"
        caps = {k: "unverified" for k in CAP_KEYS}
        reasons = {k: r for k in CAP_KEYS}
        # **逐鍵開，只開實際在 SITL 驗過的**（2026-08-12 015 驗收）：
        #   mission_upload：方言分支（home 佔 seq 0）＋回讀逐項比對通過
        #   hold/rtl/land：DO_SET_MODE 用 ArduPilot 模式號，且**讀回 HEARTBEAT
        #     確認真的切到**（mode_engaged=True），不是只看 ACK
        #   arm/takeoff：GUIDED→arm→NAV_TAKEOFF（相對高度、空白參數用 0 不用
        #     NaN）實測爬到 15.0m
        #   mission_start/mission_fly：**2026-08-24 SITL 一致性測試通過**——
        #     上傳 4 項 → 起飛至 13.7m → **讀回 HEARTBEAT 確認機端實際進入
        #     AUTO**（不是看 ACK），當下高度 13.7m。證據檔
        #     `data/conformance/ardupilot.json`。
        #     ⚠ **證據強度**：測的是 ArduPilot **4.0.3** SITL（映像
        #     `radarku/ardupilot-sitl` 已五年），而機上實機是 **4.7.0**——
        #     差 7 個 minor 版本。所以這兩鍵的準確語意是「對 4.0.3 SITL 驗過」，
        #     **不是「對 4.7.0 真機保證可用」**。首飛前仍應以實機覆核。
        for _k in ("mission_upload", "hold", "rtl", "land", "arm", "takeoff",
                   "mission_start", "mission_fly"):
            caps[_k] = "ok"
            reasons.pop(_k, None)
        #
        return caps, reasons
