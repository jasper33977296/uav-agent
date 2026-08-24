# 參考文件：Fibocom 模組 AT 指令

## 為什麼留這一份

機上的 5G 模組是 **Fibocom FM160-JK**（`ATI` 回報，IMEI 860939060062294），
不是地面站 `LinkSample` schema 註解假設的 Quectel RM500Q-GL。
`AT+QENG="servingcell"` 在這顆上直接回 **ERROR**，欄位對照完全不同。

FM160 的手冊沒公開流通，這裡用的是**同家族 FM350 的手冊**
（`Fibocom_FM350_AT_Commands_User_Manual_V2.10.pdf`，§11.1.15）。
兩者的 `+GTCCINFO` 格式一致——已用實測資料交叉驗證，見下。

## `AT+GTCCINFO?` NR service cell 欄位（手冊 §11.1.15 原文）

    <IsServiceCell>,<rat>,<mcc>,<mnc>,<tac>,<cellid>,<narfcn>,
    <physicalcellId>,<band>,<bandwidth>,<ss_sinr>,<rxlev>,<ss_rsrp>,<ss_rsrq>

實測對照（2026-08-24，本場域）：

| # | 欄位 | 我方值 | 進位 | 解讀 |
|---|------|--------|------|------|
| 1 | IsServiceCell | 1 | 十進位 | 是服務細胞 |
| 2 | rat | 9 | 十進位 | NR |
| 3 | mcc | 999 | 十進位 | 實驗網 |
| 4 | mnc | 66 | 十進位 | |
| 5 | tac | 8D | **十六** | 141 |
| 6 | **cellid** | 214001 | **十六** | NCI = 2179073 |
| 7 | narfcn | AFDA0 | **十六** | 720288 → 約 4804 MHz |
| 8 | **physicalcellId** | 85 | 十進位 | PCI = 85 |
| 9 | **band** | 5079 | 十進位 | n79（「5」前綴＝NR）|
| 10 | bandwidth | 100 | 十進位 | 100 MHz |
| 11 | ss_sinr | 97 | 索引 | |
| 12 | rxlev | 89 | 索引 | |
| 13 | ss_rsrp | 89 | 索引 | |
| 14 | ss_rsrq | 64 | 索引 | |

### 進位是怎麼確定的（手冊沒明寫）

拿另一份公開的實測輸出（T-Mobile US，310/260）做對照：

    範例  1,9,310,260,A2E700,101E0212F,7EFAE,290,5041,100,113,87,87,64
    我方  1,9,999, 66,    8D,   214001, AFDA0, 85,5079,100, 97,89,89,64

* 範例的 narfcn `7EFAE` = 519086，落在 **n41**（499200–537999）值域，
  而它的 band 欄是 **5041**。
* 我方 `AFDA0` = 720288，落在 **n79**（693334–733333）值域，
  而 band 欄是 **5079**。

兩份資料的 ARFCN 與 band 欄互相印證，同時確定了「這些欄位是十六進位」與
「band 欄的 5 前綴代表 NR」。

## `AT+CESQ` 的 NR 尾三碼順序

FM160 在 3GPP 標準的 6 個參數之後多回 3 個 NR 值，**順序手冊未載**。
先用物理可能性反推（實測 `65,93,103`）：

* 假設 rsrq,rsrp,sinr → −10.5 dB / −63 dBm / +28.5 dB　三者皆合理 ✔
* 假設 rsrp,rsrq,sinr → RSRQ = +3.5 dB　　　　　　　　　不可能 ✘
* 假設 sinr,rsrp,rsrq → RSRQ = +8.5 dB　　　　　　　　　不可能 ✘

後來拿到 GTCCINFO 的手冊定義（sinr/rxlev/rsrp/rsrq = `97,89,89,64`）
**獨立驗證了這個推論**：rsrq 65↔64、rsrp 93↔89、sinr 103↔97，逐項對得上
（兩個指令取樣時刻不同，故略有差異）。

## 換算公式（3GPP TS 38.133）

    SS-RSRP: dBm = index − 156        （0..127 → −156..−31）
    SS-RSRQ: dB  = index / 2 − 43     （0..127 → −43..+20，步進 0.5）
    SS-SINR: dB  = index / 2 − 23     （0..127 → −23..+40，步進 0.5）

## 目前的取用策略（見 `../modem.py`）

* **RSRP/RSRQ/SINR 走 `AT+CESQ`**——3GPP 標準指令、換算有明文規範。
* **PCI / cell_id / band 走 `AT+GTCCINFO?`**——依上表對照。
* `narfcn` 沒有對應的 schema 欄位，收進 `raw`。
* 兩個指令的原始回應**整包存進 `LinkSample.raw`**：對照表日後若有修正，
  可以拿歷史資料回頭重算，不必重飛。

## 檔案

* `Fibocom_FM350_AT_Commands_User_Manual_V2.10.pdf` — 原始手冊（344 頁）。
  注意：WebFetch 之類的工具會誤判它是掃描檔，實際上有文字層，用
  `pdftotext -layout` 抽得出來。
* `GTCCINFO-excerpt.txt` — §11.1.15 節錄（UMTS/LTE/NR/EN-DC 四種格式都在）。
* `CESQ-excerpt.txt` — CESQ 相關段落節錄。

來源：https://www.minipc.de/support_db/support_files/Fibocom_FM350_AT%20Commands%20User%20Manual_V2.10.pdf
