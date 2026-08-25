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
GROUND_ACTIONS = {"pause", "resume", "change_route", "start_mission"}
ACTIONS = AGENT_ACTIONS | GROUND_ACTIONS

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
    "ARMED_GROUND": {"start_mission"},
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
    ("READY", "abort"): "機還在地上，沒有東西可以中止",
    ("ARMED_GROUND", "abort"): "機還在地上；要停下來請上鎖（disarm），"
                               "不是切懸停模式",
    ("READY", "rtl"): "機還在地上——返航會先爬升到返航高度，那是一次沒有人"
                      "按下的起飛",
    ("ARMED_GROUND", "rtl"): "機還在地上——返航會先爬升到返航高度，那是一次"
                             "沒有人按下的起飛",
    ("RETURNING", "*"): "已經在返航了。要停下來請用 abort（改成原地懸停）",
    ("LANDING", "*"): "已經在降落了。要停下來請用 abort（改成原地懸停）",
}


def check(state: str, action: str) -> tuple[bool, str | None]:
    """回傳 (可不可以, 不行的理由)。理由一定說得出「那現在能做什麼」。"""
    if action not in ACTIONS:
        return False, f"不認得的意圖 {action}（本站支援 {sorted(ACTIONS)}）"
    allowed = _ALLOW.get(state)
    if allowed is None:
        # 白名單的好處在這裡：沒列到的狀態＝什麼都不准，而且說得出原因
        return False, (f"狀態 {state} 不在守門表上——這是程式的疏漏，"
                       "此時不對飛機下任何指令")
    if action in allowed:
        return True, None
    why = (_WHY.get((state, action))
           or _WHY.get((state, "*"))
           or f"在 {state} 狀態下不允許 {action}")
    now_can = sorted(allowed)
    tail = ("；現在可以做：" + "、".join(now_can)) if now_can else "；現在什麼都不做"
    return False, why + tail


def executor(action: str) -> str:
    """誰執行。守門一律先過代理，執行才分家。"""
    return "agent" if action in AGENT_ACTIONS else "ground"
