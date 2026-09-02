"""守門：這個意圖在**當下這個狀態**能不能做（狀態機文件 §2、協定 §5.2）。

**為什麼守門在機上**：它檢查的是**執行當下**的事實，而人是在看提案時確認的。
兩者之間飛機一直在動。所以守門的位階高於 `approved`——人按了確認，代理仍然
可以拒絕（協定 §5.2）。而「當下的事實」只有機上第一手看得到：地面站的狀態經
5G 回來已經是過期資料。

**執行者的分工（丙案，2026-08-25 裁定）**——切分依據是「失聯時還需不需要它」：

* `rtl`／`land`／`abort` → **代理執行**。這些是失聯時唯一還需要動作的東西，
  而且只要 `DO_SET_MODE`，不碰任務協定。
* `pause`／`resume`／`change_route`／`start_mission` → **地面站執行**。它們需要
  任務上傳與索引換算＝整套廠牌方言（`libs/autopilot`），在樹莓派上重寫一份
  等於複製一個會漂移的事實來源。而這些動作**在失聯時本來就不該發生**——
  沒有人在下意圖。

不論誰執行，**守門一律先過代理**。
"""

#: 代理自己執行（只需要模式指令）
AGENT_ACTIONS = {"rtl", "land", "abort"}
#: 地面站執行（需要完整的任務方言），代理只守門
GROUND_ACTIONS = {"pause", "resume", "change_route", "start_mission", "disarm"}
ACTIONS = AGENT_ACTIONS | GROUND_ACTIONS

#: **需要遙控器在線才放行的意圖**（039 複裁 A，2026-08-31 使用者裁定）。
#:
#: 為什麼是前置條件而不是查表的一格：它**橫跨狀態**。塞進 `_ALLOW` 會讓
#: `NOT_READY`／`READY`／`ARMED_GROUND` 各分裂成「RC 有」與「RC 無」兩格，
#: 而分裂出來的那幾格彼此只差這一個條件——表會變大一倍，錯一格卻更難看出來。
#:
#: 為什麼只有 `start_mission`：地面站把 `takeoff`／`mission_fly`／`arm`／群組
#: `upload` 全部映射到這個意圖（`apps/command/app/guard_client.py`），所以擋住
#: 它就同時擋住了單機起飛與群組同時起飛——裁定要的正是這兩件事。
#: **rtl／land／abort 不在此列**：那些是「把飛機弄回地面」的方向，RC 掉線時
#: 更需要它們能用。
RC_REQUIRED = {"start_mission"}

#: 每個狀態允許哪些意圖。**沒列到的一律拒絕**——白名單而不是黑名單：
#: 新增一個狀態時，忘了補這張表的後果是「什麼都不准」，不是「什麼都准」。
_ALLOW: dict[str, set[str]] = {
    # 連不上飛控時不得下任何指令（狀態機文件 §1）
    "DISCONNECTED": set(),
    # **飛手拿著就什麼都不做**（協定 §5.4）。這是最危險的一種分岔：
    # 系統以為自己在指揮，而人正在用遙控器飛
    "PILOT_CONTROL": set(),
    # 飛控自己的 failsafe 正在跑。允許把它往「停下來」的方向推，
    # 但不准開始新的任務——那會跟 failsafe 搶飛機
    "EMERGENCY": {"rtl", "land", "abort"},
    # 地面：可以開始任務，不能對著一台沒在飛的機下飛行中指令。
    # **abort 也不行**——地上沒有東西可以中止，而它仍然會送出一個模式指令。
    # 2026-08-25 實測：對著停在地上的真機打 abort，守門放行、LOITER 真的送出去了。
    # 無害，但「無害的多餘指令」不該由守門放行——守門的工作就是只放行有意義的
    "NOT_READY": {"start_mission"},
    "READY": {"start_mission"},
    # **上鎖只在地上允許。** 空中上鎖＝馬達停轉、飛機直接掉下來，那不是
    # 「停止」而是墜毀。真的需要緊急停止時用遙控器——那是本系統設計裡
    # 人接管的那一層，而且它不經過我們
    "ARMED_GROUND": {"start_mission", "disarm"},
    # 爬升中：可以中止，不能改航線（還沒開始飛任務，沒有東西可改）
    "TAKING_OFF": {"pause", "rtl", "land", "abort"},
    # 飛任務中：可以暫停、改航線、返航、降落
    "FLYING_MISSION": {"pause", "change_route", "rtl", "land", "abort"},
    # 空中暫停：可以續飛、改航線、返航、降落
    "HOLDING": {"resume", "change_route", "rtl", "land", "abort"},
    # 已經在回家／降落：**別打斷它**。要停下來就用 abort（改成懸停）
    "RETURNING": {"abort", "land"},
    "LANDING": {"abort"},
}

#: 拒絕時說人話。**只說「不允許」等於沒說**——操作員要知道現在能做什麼
_WHY: dict[tuple[str, str], str] = {
    ("PILOT_CONTROL", "*"): "飛手正用遙控器操控，系統只觀察不介入。"
                            "把模式切回 AUTO/GUIDED 才會交還控制權",
    ("DISCONNECTED", "*"): "收不到飛控心跳——不知道機體現在是什麼狀態，"
                           "此時下任何指令都是對著空氣喊",
    ("EMERGENCY", "*"): "飛控的 failsafe 正在跑，此時開始新任務會跟它搶飛機。"
                        "可以下返航或降落",
    ("FLYING_MISSION", "start_mission"): "已經在飛任務了。要換航線請用改航線"
                                         "（暫停→上傳→續飛），不要重下開始",
    ("FLYING_MISSION", "resume"): "已經在飛了，不需要續飛",
    ("HOLDING", "pause"): "已經在暫停懸停中",
    ("READY", "pause"): "機還在地上，沒有東西可以暫停",
    ("NOT_READY", "abort"): "機還在地上，沒有東西可以中止",
    # 改航線是**飛行中**的三步序列（暫停→上傳→續飛）。機在地上時沒有東西
    # 可以暫停、也沒有東西可以續飛——**要換飛哪一份任務，直接上傳那一份**，
    # 而上傳根本不經過本守門
    ("NOT_READY", "change_route"): "機還在地上，沒有航線在飛、也沒有東西可以"
                                   "暫停或續飛。要換飛哪一份任務，直接上傳那一份",
    ("READY", "change_route"): "機還在地上，沒有航線在飛、也沒有東西可以"
                               "暫停或續飛。要換飛哪一份任務，直接上傳那一份",
    ("ARMED_GROUND", "change_route"): "還沒起飛，沒有航線在飛。"
                                      "要換飛哪一份任務，直接上傳那一份",
    ("READY", "abort"): "機還在地上，沒有東西可以中止",
    ("ARMED_GROUND", "abort"): "機還在地上；要停下來請上鎖（disarm），"
                               "不是切懸停模式",
    ("READY", "rtl"): "機還在地上——返航會先爬升到返航高度，那是一次沒有人"
                      "按下的起飛",
    ("ARMED_GROUND", "rtl"): "機還在地上——返航會先爬升到返航高度，那是一次"
                             "沒有人按下的起飛",
    ("FLYING_MISSION", "disarm"): "**空中上鎖＝馬達停轉、飛機直接掉下來**。"
                                  "要讓它停下來請用 abort（原地懸停）或 rtl；"
                                  "真的要緊急切斷動力，用遙控器",
    ("HOLDING", "disarm"): "**空中上鎖＝馬達停轉、飛機直接掉下來**。"
                           "要降落請用 land",
    ("TAKING_OFF", "disarm"): "**爬升中上鎖＝飛機直接掉下來**",
    ("RETURNING", "*"): "已經在返航了。要停下來請用 abort（改成原地懸停）",
    ("LANDING", "*"): "已經在降落了。要停下來請用 abort（改成原地懸停）",
}


#: RC 前置條件的拒絕理由。**「只告警不 RTL」那格的前提就是這件事**：機在地上
#: 失聯時之所以敢不動作，靠的是「遙控器就在旁邊、人隨時可以接管」——RC 早就
#: 掉線的話這個前提不成立，而它從來沒有被驗證過（協定 §6）
_RC_WHY = ("遙控器未連線——自動起飛的前提是有人能隨時接管，"
           "而現在沒有人接得了手（039 複裁 A）"
           "；現在可以做：確認遙控器開機並與接收機綁定，連上後再開始任務")


def check(state: str, action: str,
          rc_link: bool | None = None) -> tuple[bool, str | None]:
    """回傳 (可不可以, 不行的理由)。理由一定說得出「那現在能做什麼」。

    `rc_link`：遙控器連上了沒有。**三態，`None` 是「不知道」而不是「沒有」**
    （039 複裁 A）——測不到 RC 的韌體不該因此什麼都不能做，那會讓所有還沒
    驗證過這個欄位的機都起飛不了（issues/036 的同一個教訓）。預設 `None`
    也讓既有的兩參數呼叫（含地面站的 `test-intent-guard.py`）維持原行為。
    """
    if action not in ACTIONS:
        return False, f"不認得的意圖 {action}（本站支援 {sorted(ACTIONS)}）"
    allowed = _ALLOW.get(state)
    if allowed is None:
        # 白名單的好處在這裡：沒列到的狀態＝什麼都不准，而且說得出原因
        return False, (f"狀態 {state} 不在守門表上——這是程式的疏漏，"
                       "此時不對飛機下任何指令")
    if action in allowed:
        # **RC 前置條件排在狀態判斷之後**：狀態本來就擋掉的那些格
        # （例如飛行中重下 start_mission），既有的理由比「遙控器未連線」
        # 更說得出當下該做什麼。排在後面，這道就只在真的會放行時才發言
        if action in RC_REQUIRED and rc_link is False:
            return False, _RC_WHY
        return True, None
    why = (_WHY.get((state, action))
           or _WHY.get((state, "*"))
           or f"在 {state} 狀態下不允許 {action}")
    now_can = sorted(allowed)
    if state == "NOT_READY":
        # **`NOT_READY` 的意思就是「它現在飛不了」**，所以照樣列出白名單裡的
        # `start_mission` 是一句誤導：那個動作會在更後面被飛控的預檢擋下，
        # 而使用者會照著我們的話去撞一次牆。
        #
        # 這條守門的紀律是「擋下要說得出**那現在能做什麼**」——而在這一格，
        # 真正能做的事不是任何一個意圖，是**讓它就緒**。
        tail = ("；但這台機現在的狀態是 NOT_READY（未就緒），"
                "**先讓它就緒**——預檢／EKF／GPS 定位，畫面的「未就緒原因」"
                "會列出卡在哪一項")
    else:
        tail = ("；現在可以做：" + "、".join(now_can)) if now_can else "；現在什麼都不做"
    return False, why + tail


def executor(action: str) -> str:
    """誰執行。守門一律先過代理，執行才分家。"""
    return "agent" if action in AGENT_ACTIONS else "ground"
