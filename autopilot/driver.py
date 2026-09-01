"""自駕儀驅動介面（issue 026 B1）。

**本檔是契約定義，還沒有實作、也還沒有人呼叫它。** B2／B3 才把
`apps/command/app/mav.py` 的 `dialect()` 與 `apps/backend/app/dialect.py` 的內容
搬進 `Px4Driver`／`ArduPilotDriver`。先定義後搬遷是刻意的：形狀先固定，搬運
才是「提取一處」而不是邊搬邊改介面。

介面規格與設計理由見 `doc/autopilot-driver-interface.md`；架構決策與 QGC 對照
見 `doc/autopilot-driver-architecture.md`。

## 驅動是無狀態的

方法一律把所需狀態當參數收，驅動自己不持有機體狀態——與 QGC 的
`FirmwarePlugin` 同形（它每個方法收 `Vehicle*`，本身沒有成員狀態）。機體狀態
留在 `backend/app/state.py` 與 command 端的 router。

**為什麼**：一個廠牌一個驅動實例，但機體是多台。驅動若持有狀態，多機就會共用
到同一份，或者被迫變成「每台一個驅動實例」——後者讓「這是 PX4 的行為」與
「這是第 3 號機的狀態」混在同一個物件裡，正是我們想拆開的兩件事。

## 兩類方言，兩種方法

- **訊息層**（`adjust_incoming`／`adjust_outgoing`）：同一件事、不同訊息名或
  欄位名。正規化成單一標準形之後，下游完全不必知道廠牌。
- **解讀層**（其餘方法）：同一個值、不同意義，必須知道語意才解得開。

**判準：需要知道「值代表什麼意思」的，就不屬於訊息層。** 正規化只認結構、
不認語意——不得對映模式、不得算就緒、不得構造指令。這一刀是規格不是建議：
沒有它，`adjust_*` 會變成什麼都往裡塞的垃圾抽屜，而那等於把方言問題從各處
搬到一個大函式裡，抽象效益歸零。

配套：`adjust_*` 必須用 `MESSAGE_ADJUSTMENTS` 明文宣告它碰哪些訊息型別，並由
測試釘住那份清單（同 `SEND_WHITELIST` 的手法）。往裡加東西就會測試失敗，逼
加的人回來面對「這到底該不該是 adjust」。
"""
from __future__ import annotations

from typing import Any, Protocol


class MessageEquivalence:
    """一個「A 訊息可以當 B 訊息讀」的宣告，**帶適用範圍**。

    **為什麼要帶範圍**（2026-08-12 B0 實測，這條是血的教訓）：
    `EKF_STATUS_REPORT` 與 `ESTIMATOR_STATUS` 的 flags 只有 bit 1..512 逐位同義；
    bit 1024 在兩邊分別是 `ESTIMATOR_GPS_GLITCH` 與 `EKF_UNINITIALIZED`——**同
    位元、不同意義**。

    「等價」若不帶範圍，最危險的地方不是寫錯，是**寫的時候是對的、用的時候
    沒人記得範圍**：日後有人讀第 1024 位，程式不會報錯，只會靜默給出錯的答案。

    所以本專案的規格是：**每個等價宣告都要能回答「在什麼範圍內成立」**，
    不能只寫「A 等於 B」。
    """

    def __init__(self, src_type: str, dst_type: str, *,
                 safe_field_bits: dict[str, int] | None = None,
                 note: str = ""):
        self.src_type = src_type              # 來源訊息型別（某廠牌獨有的名字）
        self.dst_type = dst_type              # 正規化後的標準名
        # 逐欄位的「可安全互換位元遮罩」；不在遮罩內的位元**不得跨廠牌解讀**
        self.safe_field_bits = safe_field_bits or {}
        self.note = note                      # 範圍之外為什麼不成立


class Limit:
    """一個因機型而異的數值限制。

    **不知道就回 `unverified` 且三個值皆 None，不得回通用值。**
    回一個通用預設會讓消費端無法區分「這是該廠牌的真實限制」與「這是我們湊
    的」——而那正是 `limits()` 存在的目的（現況：起飛高度 10.0 全域不分廠牌，
    對兩家剛好都安全所以沒出事）。

    與 `readiness()` 回 `bool | None`、影像五態、`msg_registry` 停掉的 hz 留
    null 同一條原則：**缺乏證據不是給出一個看似權威數字的理由。**
    """

    #: 證據強度。`sitl`＝SITL 實測驗過；`doc`＝依韌體文件／參數預設但未實測；
    #: `unverified`＝我們不知道（三個值必須皆 None）。
    CONFIDENCE = ("sitl", "doc", "unverified")

    def __init__(self, *, min=None, max=None, default=None,
                 confidence: str = "unverified", source: str = ""):
        if confidence not in self.CONFIDENCE:
            raise ValueError(f"未知的 confidence：{confidence}")
        if confidence == "unverified" and any(
                v is not None for v in (min, max, default)):
            # 擋住「有數字但標未驗證」這種讓消費端猶豫的組合——要嘛拿得出
            # 證據並標明強度，要嘛老實說不知道。
            raise ValueError("confidence=unverified 時 min/max/default 必須皆為 None")
        self.min, self.max, self.default = min, max, default
        self.confidence = confidence
        self.source = source              # 數字哪來的（韌體參數名／實測腳本）


class AutopilotDriver(Protocol):
    """一個廠牌的方言知識。**無狀態**：所需狀態一律由參數傳入。

    13 個成員。刻意維持粗粒度——QGC 的 `FirmwarePlugin` 在同一業務範圍內用了
    約 30 個方法，差別主要在粒度（它把模式拆成 `flightMode`／`flightModes`／
    `setFlightMode` 加 11 個具名模式存取子，我們用一張模式表涵蓋同一批知識）。
    未來要加動詞（`goto_location`、`pause` 等）就加方法，不預先開空方法。
    """

    #: ── 方言旗標（**每個驅動都要自己宣告**，這裡是 Protocol 不是基底類別）──
    #: 漏宣告的後果是執行期 AttributeError——2026-08-26 就發生過：
    #: `no_coord_frames` 加在 UnknownDriver 與 ArduPilotDriver 上、漏了 PX4，
    #: 於是預檢對 PX4 的任務直接炸掉。`test-driver-flags.py` 現在會擋住這件事。
    home_at_seq0: bool
    takeoff_alt_is_relative: bool
    takeoff_needs_guided: bool
    no_coord_frames: frozenset

    #: 廠牌識別（`MAV_AUTOPILOT_*` 的值）與人話名
    autopilot_raw: int
    name: str

    #: 本驅動宣告的訊息層等價。**改這份清單會讓一致性測試失敗**（刻意）。
    MESSAGE_ADJUSTMENTS: tuple[MessageEquivalence, ...]

    # ── 訊息層：只認結構、不認語意 ──────────────────────────────────
    def adjust_incoming(self, msg: Any) -> Any:
        """把收到的訊息正規化成標準形（改型別名／欄位名）。

        **不得做任何判斷**：不對映模式、不算就緒、不構造指令。只做
        `MESSAGE_ADJUSTMENTS` 宣告過的改名，且只在宣告的範圍內。
        """

    def adjust_outgoing(self, msg: Any) -> Any:
        """把要送出的訊息改成該廠牌認得的形式。限制同 `adjust_incoming`。"""

    def gcs_failsafe_params(self) -> dict[str, str]:
        """地面站失聯 failsafe 的參數名：`{"enable": …, "timeout_s": …}`。

        **只回名字不回值**：值是逐台機的事實，要向飛控讀。不知道的廠牌回空表
        ——沒有意見時不要製造意見（issues/033 §1.3）。
        """

    # ── 解讀層：模式 ────────────────────────────────────────────────
    def decode_mode(self, custom_mode: int) -> str:
        """HEARTBEAT.custom_mode → 顯示用模式名（差異 2）。"""

    def encode_mode(self, mode: str) -> tuple[int, int]:
        """動詞模式名 → DO_SET_MODE 要送的值（差異 1）。

        PX4 是 (main, sub)、ArduPilot 是 (模式號, 0)。
        """

    def mode_matches(self, custom_mode: int, mode: str) -> bool:
        """拿 HEARTBEAT 驗證真的切過去了——**送出成功不等於切成功**。"""

    # ── 解讀層：動作 ────────────────────────────────────────────────
    def takeoff_plan(self, alt: float, ground_amsl: float | None) -> dict:
        """起飛序列（差異 3、9）。

        涵蓋三件廠牌差異：要不要先進 GUIDED、param7 是相對高度還是絕對海拔
        （送錯會差一整個地面海拔）、空白參數用 NaN 還是 0.0（ArduPilot 對 NaN
        參數連 ACK 都不回，指令靜默丟棄）。
        """

    def wire_seq(self, index: int) -> int:
        """我方航點索引 → **機端的 mission seq**（差異 5 的另一面）。

        `mission_line()` 管的是「上傳時要不要補 home」，這裡管的是「**指定第
        幾項**時要送哪個數字」。兩者是同一個差異的一體兩面，但呼叫端不同：
        前者在上傳，後者在 `DO_SET_MISSION_CURRENT`。

        **少了這層換算，在 ArduPilot 上會差一個航點，而且不會有任何錯誤訊息**
        ——飛機安靜地飛到隔壁那點去。
        """

    def mission_line(self, items: list[dict]) -> list[dict]:
        """任務項的線序慣例（差異 5）。

        ArduPilot 把 home 當 seq 0，實際航點從 seq 1 起算；PX4 不用。
        """

    # ── 解讀層：連線與就緒 ──────────────────────────────────────────
    def on_connect(self) -> list[Any]:
        """連線時要送出的訊息（差異 6）。

        ArduPilot 預設幾乎不送遙測（實測只有 4 種訊息），要 `REQUEST_DATA_STREAM`
        才會串流；PX4 預設就串流。回傳空 list 代表不需要。
        """

    def keepalive(self) -> list[Any]:
        """要定期補送的訊息。

        **串流率設在自駕儀端**，機端重開機、換連線通道、或我方重連之後就沒了
        ——只在連線時送一次的話，那些情況下會靜默失去全部遙測（只剩心跳，看
        起來還「連著」）。
        """

    def readiness_signals(self) -> tuple[str, ...]:
        """本廠牌有哪些**權威**就緒訊號（差異 7）。

        PX4 有 PREARM_CHECK 位元；ArduPilot 不回報，只能靠 EKF。**沒有的訊號
        要如實缺席**，讓 `readiness()` 回 None（未知）而不是拿次級訊號
        （GPS 好）冒充權威判斷。
        """

    # ── 值域與能力 ──────────────────────────────────────────────────
    def limits(self) -> dict[str, Limit]:
        """因機型而異的數值限制（差異 11）。見 `Limit` 的「不知道就說不知道」。

        **這是對外契約而不是內部常數**：UI 據此決定輸入框的 min／max／預設
        （ui-spec §0.2c 條款 5b），MCP 據此避免送出必然被拒的值。
        """

    def capabilities(self, ctx: dict | None = None) -> dict[str, str]:
        """本驅動支援哪些動詞，四態。

        **值來自一致性測試結果＋執行期前提檢查的交集**（B3），不是人工寫死的
        字典：`ok` 當且僅當「驅動通過該動詞的一致性測試」**且**「這台機的執行期
        前提滿足」（例：某動詞需要機端參數設對）。

        把兩者混為一談會得到兩種錯誤——用測試結果宣告一台沒設好的機可用，或
        用單機設定否定驅動本身的正確性。
        """
