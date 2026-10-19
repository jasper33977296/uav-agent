# uav-agent — 機上智能代理

跑在無人機的伴飛電腦上，把飛控接上地面站，並在鏈路出事時自己做決定。

```
飛控 (MAVLink over UART)
  ↕
[ uav-agent ]
  ├── UDP → 地面站:14540   遙測入庫（唯讀通道）
  ├── UDP ↔ 地面站:14541   指令通道（雙向）
  ├── 5G 模組 AT 採樣 → POST /api/link-metrics/live
  └── 本地監控：串流保活、地面失聯處置、狀態變化留痕
```

---

## ⚠️ 相容性：這份程式適用於什麼

**「實測過」與「應該可以」是兩回事**，以下嚴格區分。安全路徑上的東西尤其
不要靠推測——寫錯不會報錯，只會在最需要它的那一刻做錯事。

### 飛控系統

| 自駕儀 | 狀態 | 說明 |
|---|---|---|
| **ArduPilot / ArduCopter** | ✅ **實測** | 四旋翼，`MAV_AUTOPILOT_ARDUPILOTMEGA`，MAVLink 2，UART 57600。遙測、指令、ACK、失聯處置全部驗證過 |
| ArduPilot / Plane・Rover | ⚠️ 未測 | 遙測與轉發應該可用；**失聯處置的模式號是 Copter 的**，見下方警告 |
| PX4 | ❌ **不可直接用** | 轉發與遙測會動，但**安全路徑會靜默做錯事**，見下方警告 |
| 其他 MAVLink 2 自駕儀 | ⚠️ 未測 | 純轉發與遙測不綁方言；模式相關功能一律要重做 |

#### 🔴 移植到非 ArduCopter 之前必須改的地方

1. **`COPTER_MODE_ID`（agent.py）是 ArduCopter 專屬的模式號**
   （RTL=6、LAND=9、SMART_RTL=21、LOITER=5）。失聯處置用它組
   `DO_SET_MODE`。ArduPilot 的 `DO_SET_MODE` 是 **param2 直接放模式號**，
   而 **PX4 是 main_mode/sub_mode 兩段式編碼**——同樣的數字在 PX4 上
   會切到完全不同的模式，而且**不會有任何錯誤訊息**。
   （這個坑本專案在地面站端踩過：issue 030，承諾 Hold 實送 GUIDED。）

2. **`SYSID_MYGCS` 決定飛控把誰當成「它的地面站」**（ArduPilot 專屬）。
   機端 `SYSID_MYGCS` 是 255，而以 sysid 255 發 GCS 心跳的是**地面的指令服務**
   （每秒一則，`uav-system` 的 `apps/command/app/mav.py`），**不是本代理**——
   代理只是那條路上的轉發者，自己從不送心跳。所以飛控盯的其實是**地面**，
   而那條心跳走 5G。這是安全模型的關鍵前提，見下方「為什麼第二層只能由
   代理做」。PX4 沒有這個機制，移植時該段前提要重想。

3. **串流請求走 `REQUEST_DATA_STREAM`（舊式）＋ `SET_MESSAGE_INTERVAL`（新式）**
   兩條路。ArduPilot 兩者都吃；PX4 已棄用前者。

### Modem 裝置

| 模組 | 狀態 | 說明 |
|---|---|---|
| **Fibocom FM160-JK** | ✅ **實測** | 5G SA。USB `2cb7:0105`，AT 埠 `/dev/ttyUSB1`，網路走 CDC-Ether `usb0` |
| Fibocom FM350 等同家族 | ⚠️ 推測可用 | 共用 `+GTCCINFO` / `+CESQ` 格式（本專案的欄位對照表就是取自 FM350 手冊，見 `reference/`）|
| **Quectel（RM500Q-GL 等）** | ❌ **未實作** | 它們用 `AT+QENG="servingcell"`，在 Fibocom 上回 `ERROR`；本代理沒有 QENG 的解析路徑。地面站 `LinkSample` schema 的註解假設的是 Quectel，**那份註解與這裡的實作不一致** |
| 無 modem | ✅ 支援 | `MODEM_PORT=""` 即停用訊號採樣，其餘功能不受影響 |

訊號欄位的取用策略與**進位／欄位序的推導過程**寫在
[`reference/README.md`](reference/README.md)。原則：
**沒把握的欄位一律留 `None`，原始回應整包存進 `raw`**——填錯不會報錯，
只會讓「干擾發生時哪個細胞在服務」的分析得到看似合理的錯誤答案。

### 執行平台

| 項目 | 實測環境 |
|---|---|
| 硬體 | Raspberry Pi 5 Model B Rev 1.1 |
| 系統 | Debian 13 (trixie)，kernel 6.18.34+rpt-rpi-2712 |
| 飛控接線 | 40-pin 排針 GPIO14/15，需 `dtoverlay=uart0-pi5` |
| Python | 3.13 |

> **Pi 5 的 `/dev/ttyAMA10` 是除錯接頭那顆 UART，不是排針上的。**
> 排針 UART 要靠 `uart0-pi5` overlay 才會出現 `/dev/ttyAMA0`。

### 地面站

搭配 [uav-system](https://github.com/jasper33977296/uav-system)：
backend `udpin://0.0.0.0:14540` ＋ HTTP `:38000`、
command `udpin://0.0.0.0:14541` ＋ HTTP `:38001`。
backend 需設 `LINK_SOURCE=modem` 才會收 `/api/link-metrics/*`。

---

## 安全模型

**三層防線，本代理只負責第二層。**

1. **飛控自己的 RC failsafe** — 遙控器在附近待命，人隨時可接管。
   **不要為了方便而關掉預檢的 RC 檢查**：那項檢查在確保接管手段存在。
2. **本代理的地面失聯處置** ← 這裡
3. **地面站操作員的判斷**

### 為什麼第二層只能由代理做

飛控的 GCS failsafe（`FS_GCS_ENABLE`）盯的是 `SYSID_MYGCS` 的心跳，而那則心跳
是**地面的指令服務**每秒送的（sysid 255），路徑是：

```
指令服務（1 Hz）→ UDP over 5G → 本代理 → 驗 CRC → 序列埠 → 飛控
```

**所以 5G 一斷，飛控那邊的 GCS 心跳也真的停了。** 但**飛控看得見的只有
「心跳沒了」，看不見那代表什麼**——是 5G 抖一下、地面站當掉、還是操作員關機
離開，它一律只能做同一件粗的事。分狀態的判斷（任務中續飛／暫停中立刻回／
飛手手上不介入）只有代理做得到，因為只有代理同時看得到飛控的全部狀態與
地面的死活。

**裁定（2026-09-01 使用者，uav-system issues/039）**：`FS_GCS_ENABLE = 1`、
`FS_GCS_TIMEOUT = 45`。飛控那層的角色是**代理的後備**——只在代理沒有動手時
才出手。不變式：

```
FS_GCS_TIMEOUT (45s)  >  LINK_LOSS_MAX_SOLO_S (30s)  >  LINK_ACTION_S (10s)
```

> **逾時只要大於 `LINK_ACTION_S` 是不夠的。** 代理在第 10 秒只是**開始判斷**，
> `FLYING_MISSION` 那一格會續飛到單飛滿 30 秒才動手。逾時設 15 秒的話飛控
> 仍然會在續飛中途搶先，「跑完任務再 RTL」照樣不會發生。要比的是 30 不是 10。

代理**開機時讀一次**這兩個參數自檢（`request_fs_gcs`／`_check_fs_gcs`），
**參數名向驅動要**（`gcs_failsafe_params()`）而不是寫死——PX4 的對應是
`NAV_DLL_ACT`／`COM_DL_LOSS_T`，而且 `NAV_DLL_ACT` 是**行為列舉不是開關**，
兩家共通的只有「0＝不動作」。未知廠牌回空表就跳過：**沒有意見時不要製造
意見**，拿別家的參數名去問只會被誤讀成「讀不到＝沒設定」。
不成立就大聲說，**而且「讀不到」與「讀到了但不合」分開報**——「不知道飛控
怎麼設定」和「知道它設錯了」處置完全不同。**一次就好不週期重讀**：參數不會
自己變，會變是因為有人去改，那種情況下重啟一次代理不算苛求。

> **這組數字的代價**：代理自己掛掉（Pi 死了、飛控還活著）時，**沒有任何人管
> 到第 45 秒**——出廠的 5 秒只要 5 秒就有人接手。這是在賭「代理活著」的機率
> 高於「任務被無謂打斷」的成本；5G 抖動在本場域是常態、Pi 掛掉罕見，所以
> 合理，但它是一個賭。**因此主迴圈那條 catch-all（「這個迴圈不准死」）的
> 重要性又高了一級**：它死掉的代價從「失聯處置不會發生」變成「失聯處置不會
> 發生，而且飛控要 45 秒後才接手」。
>
> **不要在代理裡補 `heartbeat_send`**——那會讓機上多一個心跳源，等於把
> `FS_GCS` 這條後備悄悄關掉。45 這個數字與 `FS_GCS_TIMEOUT` 的值域上限
> 尚未實機覆核。

### 處置行為

預設 `LINK_WARN_S=5` 告警、`LINK_ACTION_S=10` 起算，執行 `LINK_ACTION=rtl`。
處置**逐狀態決定**（選項 C，協定 §6）：飛任務中跑完再 RTL，暫停中／爬升中／
改航線序列進行中立刻 RTL，已在返航或飛手手上則不介入。節制之處：

* **只在 armed 時動作**。地面整合測試時鏈路斷來斷去是常態，板凳上絕不送指令。
* **已在 RTL/LAND/SMART_RTL 就不介入**——飛機已經在自己回家了。
* **一次失聯只動作一次**。人用遙控器接管改了模式後不搶回來，人優先於自動化。
* **鏈路恢復不自動續飛**。恢復只解除警戒並留痕，要不要繼續是人的決定。
* 保命指令送三次（57600 上單包丟失是真實風險，`DO_SET_MODE` 冪等無副作用）。

`LINK_ACTION=none` 可停用自動處置，只保留告警。

**續飛的兩個上限**（`LINK_LOSS_MAX_SOLO_S=30`、`LINK_LOSS_MIN_BATT_PCT=15`），
任一命中就立刻 RTL 而不是跑完。單飛時間從失聯那一刻起算，所以與
`LINK_ACTION_S=10` 相減，**續飛實際只有 20 秒可用**——這是已知並接受的
（2026-08-31 複裁 B），啟動時會印一行說明兩者的關係。

* **讀不到電量＝不准續飛**（複裁 E）。`battery_pct` 是 `None` 時立刻 RTL，
  不是跳過檢查——這條規則存在的前提就是知道還剩多少電。
* **「跑完」以 `mission_state` 為準**。韌體不回報時另有 seq 替代判定
  （`MISSION_COMPLETE_BY_SEQ=1`），**預設關**：`total` 含不含 home、最後一個
  航點的 seq 停在哪是廠牌與版本的行為，猜錯會在任務還沒飛完時打斷它。
  真機覆核 ArduPilot 4.7 之後才開。

### RC 未連線不得起飛（複裁 A，2026-08-31）

上面那條「機在地上失聯只告警不 RTL」的**整個理由**是「遙控器就在旁邊待命，
人處理比飛機自己起飛安全」。那是一個假設，而它從來沒有被驗證過——RC 早就掉線
的話前提不成立，機在地上失聯就變成沒有任何人能接管。所以守門多一道橫跨狀態的
前置條件：`rc_link` 為 `false` 時擋下 `start_mission`（地面站把單機起飛、
群組同時起飛、arm 全部映射到這個意圖）。

RC 狀態讀 `SYS_STATUS` 的 `MAV_SYS_STATUS_SENSOR_RC_RECEIVER` present／health
兩組位元，**三態**：present 沒這一位＝這台韌體沒回報＝`None`＝**不知道，不擋**。
把「不知道」當成「沒有 RC」會讓所有還沒驗證過的機都起飛不了。

> **`RC_CHANNELS.rssi` 不能用**：它沒有「不知道」這一態，而 `rssi = 255` 在
> MAVLink 是**無效值不是滿格**——讀成滿格的後果是 RC 掉線時守門照樣放行起飛。
>
> **真機覆核尚未進行**：ArduPilot 4.7 在 RC failsafe 期間是否真的清掉 health
> 位元，要接飛控跑 `tools/rc-link.py --live /dev/ttyAMA0` 才算數。

---

## 安裝與部署（一鍵，開發機上跑）

**空機與已裝好的機是同一行指令**（2026-10-19）：

```bash
./deploy.sh --env drone-e.env              # 空機：精靈帶著填安裝資訊，確認後從頭裝。之後也用這行更新
./deploy.sh --env drone-e.env --dry-run    # 先看（不動機上）
./deploy.sh                                # 已裝好的機（位址先問地面站）
./deploy.sh --rollback                     # 還原上一次部署前的備份
```

**安裝資訊精靈**（`tools/env-wizard.py`，在終端機上跑 `--env` 時自動出現）：只有人知道的——帳號、
密碼——用問的（問完立刻試登入）；找得到的——Pi 在哪（相連網段上網卡是樹莓派的鄰居）、地面站位址
（dongle 網段上這台自己的位址）、AT 埠（屬於 dongle 且回 OK 的）、飛控 sysid（聽排針 UART 的
HEARTBEAT）——找好帶著「怎麼找到的」給人看，Enter 接受或打別的值。**找的時候只讀**（密碼登入、
不裝金鑰），人確認整張表之前機上什麼都沒動。已經在跑代理的機排在 Pi 候選的最後並標警告。

**每一欄都填好才會開始**：`--env` 檔缺哪一欄就說哪一欄，連線都還沒連。空機沒給 `--env`
也一樣不開始——裝到一半才發現少一個值，機上會停在半套的狀態。填好的 `drone-*.env`
含密碼，已 gitignore。

順序固定，任何一步沒過就停：

1. 連線：沒有金鑰就用 `--env` 檔的密碼裝一次；sudo 要密碼就設成免密碼。
   帳號不必是 `pi`：不是的話 `install-service.sh` 用 drop-in 改服務的 `User=`
2. **5G dongle**（`tools/dongle-check.py`）：USB 在、網卡有 IPv4、**從 5G 那張** ping 得到地面站、
   AT 埠量得到訊號。**沒過就不部署**——這台機的用途就是經 5G 跟地面站講話、同時量 5G
3. 解鎖閘門：代理在跑時 `state.json` 要新鮮且未解鎖（不知道就中止）
4. 空機：建 `/opt/uav-agent`、**在開發機下載相依套件推上去**（機上不必有網路）、建 venv
5. 傳 git 追蹤的檔（不刪機上任何東西；不碰 `state.json`／`.fc_sysid`／`.fc_binding.json`／venv），
   相依有變就照第 4 步的方式裝
6. `--env` 檔的 `GS_HOST`／`MODEM_PORT`／`FC_SYSID` 寫進機上 `/etc/uav-agent.env`
7. `install-service.sh`（**每次都跑，可重跑**）：UART overlay、USB 供電上限、日誌持久化、
   unit 與執行帳號、開機自啟、**停用 ModemManager**（會搶 AT 埠）、**netplan 系統替 dongle
   網卡建 DHCP 設定**（Ubuntu 不會自己管它；用驅動比對，因為網卡名隨 MAC 每次列舉都變）、
   對時。改到 `config.txt` 就重開機並等它回來
8. 重啟、逐檔比 md5、驗收：commit 對、獨佔飛控序列埠、意圖通道、**代理量得到 5G**（硬條件）。
   飛控那格可以是「…」（桌上裝機時常常還沒接飛控）

**每台機不同的參數放機上 `/etc/uav-agent.env`**（例如 `MODEM_PORT=/dev/ttyUSB2`），不要改 unit——
unit 是所有機共用的一份，每次部署都會裝回 repo 的版本；那個檔部署不碰，而且蓋過 unit 的預設值。

代理住在 **`/opt/uav-agent`**、錄製 spool 在 **`/opt/uav-record`**（不放家目錄：躲誤刪）。
部署後機上的 `DEPLOYED` 記著 commit，代理開機時 log 出來、狀態列帶 `deployed`。
驗收失敗不自動回滾，保留現場。

## 一個代理只綁一塊飛控板（uav-system issues/076）

代理第一次從飛控問到板號（`AUTOPILOT_VERSION.uid2`）就**綁定**，寫進
`.fc_binding.json`（每台機自己的狀態，不進 git、部署不碰）。之後每一段飛控連線
都要重新確認——開機、或訊息中斷 3 秒以上恢復（換飛控一定會經過這一格：新飛控要開機）：

| 確認結果 | 轉發遙測 | 收地面站指令／意圖 | 註冊、補傳、訊號採樣 |
|---|---|---|---|
| 相符（`ok`） | ✅ | ✅ | ✅ |
| 確認中／報不出板號 | ❌ | ❌ | ❌ |
| **不符**（`mismatch`） | ❌ | ❌ | ❌，只以綁定的板號上線說「我鎖住了」，地面站判 `quarantined` |
| **armed 時任何結果** | ✅ | ✅ | 不符照樣大聲說（critical），**但不切**——在天上切斷比認錯更危險 |

機上錄檔照錄，每份旁邊有 `X.tlog.meta.json` 標著**錄製當下**的板號；回傳用那個板號，
不是「現在的」。說不出歸屬的檔案（不符期間錄的、換過飛控之後的無標記舊檔）留在機上不傳，
`record_upload.unowned` 會列出來。

### 換飛控：更換流程

在機上、飛機旁邊、已上鎖時跑：

```bash
python3 fc-rebind.py                                        # 看現況，不改任何東西
python3 fc-rebind.py --release --reason "原飛控 IMU 故障，換上備品"
python3 fc-rebind.py --release --reason "…" --fc-sysid 3   # 新飛控的號碼不是 FC_SYSID 時
```

它只**解除**綁定（原因與時間進 `history`）、刪掉配給舊板子的 `.fc_sysid`、重啟代理；
新板號由代理自己從飛控問出來並綁上，地面站會收到「更換流程完成：舊 → 新」。
**不讓人打新板號**（機器問得到的事讓人打只會打錯），**也不做成地面站按鈕**
（判斷「我剛剛換了飛控」的事實只在機旁的人身上）。

## 檔案

| 檔案 | 用途 |
|---|---|
| `agent.py` | 主程式：路由、串流保活、監控、失聯處置 |
| `guard.py` | 守門：這個意圖在當下這個狀態能不能做（含 RC 前置條件） |
| `intent.py` | 意圖通道（WebSocket）：state 推送、intent 回執與判決、`notice` 送出 |
| `outbox.py` | **代理的事件出口**：只有代理知道的事，送地面站（見〈事件要送到哪裡〉）|
| `route.py` | 改航線提案：續飛航點、過期重算 |
| `backfill.py` | 失聯期間的機上緩衝與恢復後補傳 |
| `recorder.py` | 機上原始層錄製：飛控送出的每一個框架落盤（tlog） |
| `uploader.py` | 那份錄製的自動回傳：**只在地面傳、一解鎖立刻停** |
| `modem.py` | 5G 模組 AT 採樣（獨立執行緒，不擋主迴圈） |
| `binding.py` | 飛控板綁定檔（`.fc_binding.json`）的讀寫（076） |
| `fc-rebind.py` | **在機上跑**：飛控更換流程（解除綁定） |
| `uav-test.py` | 一次性診斷：請求串流並把飛控說得出來的東西全部攤開 |
| `tools/link-loss-table.py` | 失聯處置的判定表（含反向驗證） |
| `tools/guard-table.py` | 守門全表 88 格 |
| `tools/rc-link.py` | RC 三態解析；`--live` 可接真飛控做覆核 |
| `tools/intent-ack.py` | 意圖回執的順序：先 ack 再 event |
| `tools/decision-flow.py` | 提案→確認→序列、乾跑、RC 前置 |
| `tools/notice-flow.py` | 事件出口：緩衝、回執才刪、重連重送、上限、節流要說出來 |
| `tools/uart-exclusive.py` | 飛控序列埠獨佔（TIOCEXCL）：別人開不起來、自己重開不被擋 |
| `tools/fc-link-watchdog.py` | 序列埠被動手腳、飛控心跳失聯、關掉重開 |
| `autopilot/` | 自駕儀驅動，**與 uav-system `libs/autopilot/` 逐字相同**（026 B4-b） |
| `normalize.py` | 正規化欄位：模式動詞／名稱、就緒判定（移植自地面站 `state.py`） |
| `tools/driver-parity.py` | 搬家的等價驗收：兩份實作同時載入逐鍵比對（B4-d 核心） |
| `tools/derived-fields.py` | 從 MAVLink 餵進去，看 `derived` 送出什麼 |
| `tools/udp-link.py` | 代理對 UDP 端點（SITL）的連線路徑 |
| `tools/fs-gcs-check.py` | 飛控 GCS failsafe 的開機自檢（不變式與三種結論） |
| `tools/backfill-flow.py` | 緩衝與補傳的邏輯（HTTP 用假的） |
| `tools/backfill-live.py` | 補傳端到端：真代理 → 跑著的地面站 → 真入庫 |
| `tools/recorder-flow.py` | 錄製器：armed 才錄、格式、滾動、失敗不拖垮轉發 |
| `tools/upload-flow.py` | 回傳的**時機**：飛行中不傳、半路起飛就停、續傳、限速 |
| `tools/fc-binding-flow.py` | 綁定：熱插拔換飛控、armed 不切、更換流程、錄檔歸屬 |
| `tools/modem-probe.py` | 找 modem 的 AT 埠並試各種查詢指令 |
| `deploy.sh` | **開發機上跑**：一鍵安裝／部署到 Pi（見〈安裝與部署〉） |
| `tools/dongle-check.py` | 5G dongle 檢查（只用標準函式庫；部署前在機上跑） |
| `systemd/uav-agent.service` | 服務定義（所有機共用的預設值；單機的值放 `/etc/uav-agent.env`） |
| `reference/` | Fibocom AT 指令手冊與欄位對照表 |

## 事件要送到哪裡（uav-system issues/057）

**要讓操作員看到的事，呼叫 `self.outbox.note(kind, text, level)`。**
`level` 是 `info`／`warning`／`critical`；`text` 寫整句中文（畫面原樣顯示）；
`now` 傳主迴圈那一拍的**單調時鐘**（送出時才換成牆鐘）。

以前有一個 `Monitor.events` 清單，18 處往裡寫、**0 處讀**——看起來像有在收集，
實際上是黑洞（「代理接管期間做過什麼」就丟在裡面）。已經刪掉，不要再做一個。

**先問：地面站自己看不看得到？** 看得到的就只寫 `log`，不要送：

| 地面站自己看得到（**不送**） | 從哪裡 |
|---|---|
| 飛控 STATUSTEXT、解鎖／模式、電量、GPS | 地面站的 `mavlink_rx` 從轉發過去的 MAVLink 自己產生事件 |
| RC 在不在 | 同上（`rc_link` 事件） |
| 守門擋下、意圖已下達、序列逐步 | 意圖通道的 `event` 回應，地面站寫成 `intent_*` |

| 只有代理知道（**送**） | kind |
|---|---|
| 地面站鏈路失聯／恢復／重綁／失聯但不介入／續飛中 | `gs_link`（**有節流**）|
| 代理接管開始與結束（結束時交代做過什麼） | `takeover` |
| 失聯處置實際下達了什麼 | `gs_link_action` |
| 失聯 failsafe 自檢讀不到或設定衝突 | `fs_gcs` |
| 改航線序列逾時、旗標被清掉 | `intent` |

送的方式：意圖通道的 `notice` 型別（uav-system `doc/agent-intent-protocol.md` §4.7）。
**斷線時留在機上**（上限 500 則，滿了丟最舊的），地面站逐則回執才刪，
沒回執的下一條連線重送。地面站 hello 回應的 `accepts` 沒有 `notice`（舊版）就不送。
狀態列的 `outbox` 看得到現況：`queued` 一直長＝地面站沒在回執。

`gs_link` 節流：10 分鐘內前 4 則照送，其餘併成一則「另有 N 則沒有逐則送出」。
新增的 kind 預設**不節流**；要節流加進 `outbox.THROTTLED`，而且想清楚
被併掉的那幾則會不會讓畫面說出不對的事。

## 機上錄製與自動回傳（uav-system issues/014）

代理逐 byte 讀過飛控送出的每一個 MAVLink 框架（轉發要用），所以**多寫一份
檔案是零額外 UART 頻寬、零額外 5G 頻寬**。檔案在 `RECORD_DIR`
（程式碼預設 `/home/pi/uav-record`，本部署的 unit 用 `Environment=RECORD_DIR=/opt/uav-record` 覆蓋成
`/opt/uav-record`），格式與地面站那份、QGC、`mavlogdump.py` 相同。

**它的價值不是備份。** 地面站錄的是「送到地面站的東西」，這裡錄的是
「**飛控送出的東西**」——**5G 斷線期間，兩者相差的正是我們最想看的那一段**。

> **界線：這支錄的是「MAVLink 看得到的一切」，不是「飛控知道的一切」。**
> 飛控內部的 dataflash `.BIN`（數百 Hz 的 IMU、控制器內部量）在飛控自己的
> SD 卡上，**過不了 57600**。機制存在（`LOG_BACKEND_TYPE` bit 1 的 MAVLink
> logging backend 就是為伴飛電腦設計的），算術不允許。

### 什麼時候傳（這個功能唯一危險的一格）

**先算術，再政策。** 錄製檔的大小上限就是那條 UART：

    5.6 KB/s（57600 8N1）× 600 s ＋ 每框架 8 byte 時間戳 ≈ **4 MB／十分鐘**

**所以「什麼時候傳」不是量的問題**——4 MB 走 5G 是幾十秒的事。問題是它與
遙測**共用同一條 5G**，而正在飛的那台機優先。與補傳（`backfill.py`）同一條
紀律：**補歷史不該拖慢現在**。

四道門，缺一不傳（`agent.py::_upload_gate`／`uploader.py`）：

| 門 | 為什麼 |
|---|---|
| 飛機沒有 `armed` | 飛行中送幾 MB，等於拿正在飛的那台機的遙測，換一份已經安全落地的紀錄 |
| 落地後靜置 `RECORD_UPLOAD_SETTLE_S` | 剛落地時人還在機旁、可能馬上再起飛；板凳測試的 arm/disarm 連續切換也不該每次都戳一輪 |
| 地面站鏈路通 | 斷線時重試只是把時間花在必敗的事上 |
| 拿得到 `board_uid` | 沒有身分就沒有歸屬（038／040 同一條規則） |

**守門在每一塊之前檢查**，所以停手的延遲上限就是「送一塊要多久」——
預設塊大小與限速相同（256 KB／256 KB/s）＝**最遲一秒後停手**。

三個刻意的設計：

* **只傳關好的檔案。** 正在寫的那一份不碰——內容還會變，sha256 與續傳都無從
  談起。**落地即換檔**，所以一趟飛行就是一個檔案。
* **可續傳，而且不靠自己記。** 進度的權威在地面站：宣告一次（帶 sha256）就
  知道要從哪裡接。本地 `.uploaded.json` 只是省下重算 sha 的快取。
  5G 在戶外會斷，**一次斷線就整份重來的話，鏈路愈差的那趟愈傳不回來**——
  而那正是最值得看的一趟。
* **傳完不刪本地檔。** 保留仍由錄製器的滾動負責（`RECORD_KEEP` 份）。
  剛傳完就刪掉唯一的副本，是把「地面站那份一定沒問題」當成前提。

**滾動刪除優先於保住紀錄**——卡滿了會讓整台機出問題，包括代理自己。但刪掉
一份**從來沒有回傳成功**的錄製時會大聲警告：安靜地刪，我們就會在事後以為
那趟飛行的紀錄還在機上。

### 回傳現況會上報（v0.7.0 起）

意圖協定的 `state` 帶一個頂層 `record_upload`：

    {pending, pending_files, blocked, current, progress, abandoned}

**只送份數不夠。** 「沒有東西要傳」與「傳不動」都是「沒在傳」，但一個是完成、
一個是故障，而處置完全相反——所以 `blocked` 帶著**「為什麼沒在傳」那句話**
一起上去（`armed（飛行中不傳）`／`落地靜置中（還有 12s）`／`地面站失聯`／
`尚未判定`）。地面站的 `/captures` 頁就是照這句話寫的。

**滾動刪掉一份從來沒回傳成功的檔案時，會 `POST /api/onboard-captures/abandoned`**
——那是永久的損失，而只寫進機上的 log 等於沒說（journald 會被清掉）。
報不成就留著下次再報，存進狀態檔，重開機也不會忘。

設定：`RECORD_UPLOAD`（0＝退回人工 `scp`）、`RECORD_UPLOAD_KBPS`、
`RECORD_UPLOAD_CHUNK_KB`、`RECORD_UPLOAD_SETTLE_S`。
地面站那端：`GET /api/onboard-captures`（含**沒傳完的**與**已遺失的**）、
`GET /api/onboard-captures/{drone_id}/{name}` 下載、
`GET /api/onboard-captures/coverage?session_id=` 兩層覆蓋對照。

## 自駕儀驅動（026 B4-b，2026-09-01）

廠牌方言從地面站搬到機上：由代理解讀飛控遙測後，把**正規化**的值送上去
（協定 §4.2）——`mode_verb`（判斷用，廠牌無關）、`mode_name`（顯示用，
廠牌自己的名字）、`ready`／`not_ready_reasons`／`prearm_ok`／`ekf_ok`。

**地面站的驅動不刪。** 「這台機有沒有代理」是執行期的事實不是編譯期的：
SITL 沒接代理就整個開發流程斷掉，而第三方的機永遠不會有代理。所以：

| 情況 | 處置 |
|---|---|
| 地面站**認得**那個廠牌 | 兩邊各自算，**不一致就報警**——不是拿來選一邊的 |
| 地面站**不認得**（新廠牌） | 用代理的值並標明來源。**這才是搬家真正的收穫** |
| 沒有代理 | 地面站自己解讀 |

**`autopilot/` 四支檔案與上游 `uav-system/libs/autopilot/` 逐字相同**——
搬家不是改寫，唯一的驗收標準是新舊輸出完全相同。所以那幾支不加檔頭、
不修看起來像 bug 的地方（來源與已知差異記在 `autopilot/_provenance.py`），
這樣 `diff -r` 出零差異才能回答「有沒有漂移」。

真正被移植的是**就緒判定**（原本住在地面站的 `state.py`，不在 `libs/autopilot`
裡）——那是唯一有機會搬錯的地方，所以 `tools/driver-parity.py` 把兩份實作
同時載進同一個行程，用 2916 組輸入逐鍵比對。**兩份實作並存的這段期間，
正是唯一能做這個比對的窗口**：地面站那份刪掉之後就沒有對照組了。

> **缺欄位＝代理沒說，不是值是 false。** 舊版代理不送這些欄位，把缺欄位讀成
> `ready: false` 會讓所有還沒升級的機都變成不可飛——與 `rc_link` 三態同一個
>教訓。`ready = None` 是「我算過了但沒有依據」，與「整個鍵不存在」不同。

## 設計決策（為什麼是這樣寫的）

**為什麼依賴 pymavlink 而不是自己解析。** 第一版為了零相依自己切訊框、
不驗 CRC，靠「訊框接龍」判斷。實測在 46KB/15s 的流量下解出大量幽靈訊框——
sysid 冒出 15 個不存在的值、msgid 出現 12586844，還把假的 HEARTBEAT 蓋掉真的。
payload 裡湊巧出現 `0xFD` 的機率遠比直覺高。飛行資料不能這樣賭。

**為什麼不用 `mavutil.recv_match()` 讀飛控。** 那個高階介面會做 per-sysid 的
訊息記帳，我們一項都沒用到，而它在 instance 訊息（`BATTERY_STATUS` 等）上會踩到
pymavlink 自己的 `_instances is None` → `TypeError`，每次吃掉一次讀取並噴多行
traceback。改成自己 `parse_buffer()`，CRC 一樣有驗，少一層不需要的狀態。

**為什麼轉發原始位元組。** `msg.get_msgbuf()` 原封轉出，不重新編碼——重編會換掉
seq 與簽章，也可能因方言表版本差異而丟欄位。**現在沒有例外**：先前為了繞過
ArduPilot 的 `SYSID_MYGCS` 檢查而以代理身分重新編碼搖桿封包的那條路，已隨地面站
移除虛擬搖桿一併刪掉（`7eb5f4b`，uav-system issues/035）。

> **但 `SYSID_MYGCS` 這個參數本身仍然要緊。** ArduPilot 用它決定誰的心跳算
> GCS 心跳，那是 `FS_GCS` 的判準——而送那則心跳的是地面的指令服務，經 5G
> 過本代理轉發（見上方「安全模型」）。刪掉搖桿重送那條路不影響它。

**為什麼串流要保活而不是設定一次。** ArduPilot 預設不推遙測；序列埠沒有 GCS
連過時串流速率是 0（實測：不請求就只有 1Hz HEARTBEAT 與 TIMESYNC）。而且飛控
重開、鏈路中斷重連之後速率會回到 0。所以偵測到「心跳還在但遙測停了」就重送。

**為什麼兩條 UDP 用不同 socket。** 地面站的 command 服務靠「收到封包的來源位址」
建回程路由表，兩條混用同一個 socket 會讓回程指向錯的通道。

**為什麼主迴圈包 catch-all。** 這條迴圈死掉＝飛控與地面站同時失聯，而外面看不
出來（HTTP 層與服務狀態都正常）。網路瞬斷是常態不是異常，吞掉續跑。
