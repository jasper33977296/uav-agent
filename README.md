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
| **ArduPilot / ArduCopter** | ✅ **實測** | 四旋翼，`MAV_AUTOPILOT_ARDUPILOTMEGA`，MAVLink 2，UART 57600。遙測、指令、ACK、手動控制重送、失聯處置全部驗證過 |
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
   本代理以 sysid 255 發心跳，機端 `SYSID_MYGCS` 也是 255——所以飛控的
   GCS failsafe 盯的是**代理**，不是真正的地面站。這是安全模型的關鍵前提，
   見下方「為什麼第二層只能由代理做」。PX4 沒有這個機制，移植時該段前提要重想。

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

飛控的 GCS failsafe（`FS_GCS_ENABLE`）盯的是 `SYSID_MYGCS` 的心跳，而那個身分
是**本代理、就在機上**。5G 斷掉時代理照樣心跳，**飛控完全無感**。
整條鏈路上只有代理看得見「地面沒人了」。

### 處置行為

預設 `LINK_WARN_S=5` 告警、`LINK_ACTION_S=30` 執行 `LINK_ACTION=rtl`。節制之處：

* **只在 armed 時動作**。地面整合測試時鏈路斷來斷去是常態，板凳上絕不送指令。
* **已在 RTL/LAND/SMART_RTL 就不介入**——飛機已經在自己回家了。
* **一次失聯只動作一次**。人用遙控器接管改了模式後不搶回來，人優先於自動化。
* **鏈路恢復不自動續飛**。恢復只解除警戒並留痕，要不要繼續是人的決定。
* 保命指令送三次（57600 上單包丟失是真實風險，`DO_SET_MODE` 冪等無副作用）。

`LINK_ACTION=none` 可停用自動處置，只保留告警。

---

## 安裝

```bash
git clone <this-repo> ~/uav-agent && cd ~/uav-agent
python3 -m venv venv
./venv/bin/pip install -r requirements.txt
sudo ./install-service.sh          # UART overlay + systemd unit
# 部署參數改 systemd/uav-agent.service 的 Environment= 行，不要改程式碼
sudo systemctl start uav-agent && journalctl -u uav-agent -f
sudo systemctl enable uav-agent    # 確認無誤後才開機自啟
```

## 檔案

| 檔案 | 用途 |
|---|---|
| `agent.py` | 主程式：路由、串流保活、監控、失聯處置 |
| `modem.py` | 5G 模組 AT 採樣（獨立執行緒，不擋主迴圈） |
| `uav-test.py` | 一次性診斷：請求串流並把飛控說得出來的東西全部攤開 |
| `tools/modem-probe.py` | 找 modem 的 AT 埠並試各種查詢指令 |
| `systemd/uav-agent.service` | 服務定義（部署參數在這裡） |
| `reference/` | Fibocom AT 指令手冊與欄位對照表 |

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
seq 與簽章，也可能因方言表版本差異而丟欄位。唯一的例外是手動控制，見上。

**為什麼串流要保活而不是設定一次。** ArduPilot 預設不推遙測；序列埠沒有 GCS
連過時串流速率是 0（實測：不請求就只有 1Hz HEARTBEAT 與 TIMESYNC）。而且飛控
重開、鏈路中斷重連之後速率會回到 0。所以偵測到「心跳還在但遙測停了」就重送。

**為什麼兩條 UDP 用不同 socket。** 地面站的 command 服務靠「收到封包的來源位址」
建回程路由表，兩條混用同一個 socket 會讓回程指向錯的通道。

**為什麼主迴圈包 catch-all。** 這條迴圈死掉＝飛控與地面站同時失聯，而外面看不
出來（HTTP 層與服務狀態都正常）。網路瞬斷是常態不是異常，吞掉續跑。
