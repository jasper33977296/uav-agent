#!/bin/bash
# 代理啟動前的對時（uav-system issues/070）。由 unit 的 `ExecStartPre=+` 以 root 執行。
#
# 2026-09-22 兩次：Pi 重開後十幾分鐘才對上時，代理整段時間的牆鐘慢 21 分鐘
# （057 的 notice 帶 at_unsynced 抓到）。timesyncd 的設定是對的（NTP＝地面站），
# 只是 5G 起來得晚、它自己的重試又退避得很長。
#
# 規則：
#   * 已經對過（/run/systemd/timesync/synchronized 存在）→ 立刻放行
#   * 沒對過 → 重啟 timesyncd 催它馬上試，最多等 SYNC_WAIT_S 秒，中途每 20 秒再催一次
#   * **逾時照樣放行**（exit 0）：沒有橋比時間錯更糟（059 同一條原則）。
#     代理自己會在狀態列與 notice 標 clock_synced／at_unsynced
set -u
FLAG=${SYNC_FLAG:-/run/systemd/timesync/synchronized}   # SYNC_FLAG 只給測試用

# **不是用 timesyncd 的機**（chrony：Fedora 系、部分映像）。不處理的話下面會去等一個
# 永遠不會出現的旗標，每次啟動都白等 60 秒。chrony 的設定由 install-service.sh 寫
if [ -z "${SYNC_FLAG:-}" ] && ! systemctl cat systemd-timesyncd.service >/dev/null 2>&1; then
  if command -v chronyc >/dev/null; then
    echo "對時：這台用 chrony——最多等 60 秒"
    if chronyc waitsync 30 0 0 2 >/dev/null 2>&1; then echo "對時：chrony 已同步"
    else echo "對時：**chrony 60 秒內沒同步，照樣啟動代理**"; fi
  else
    echo "對時：⚠ 沒有 timesyncd 也沒有 chrony——機上時間沒人校正"
  fi
  exit 0
fi
WAIT=${SYNC_WAIT_S:-60}
DROPIN=${SYNC_DROPIN:-/etc/systemd/timesyncd.conf.d/50-gs.conf}
#: install-service.sh 早期另寫的一份（寫死安裝當下的 GS_HOST）
LEGACY=${SYNC_LEGACY:-/etc/systemd/timesyncd.conf.d/uav-gs.conf}

# ── 對時位址跟著 GS_HOST 走（2026-10-01）──────────────────────────────
# **寫死的下場看過了，而且代價是整天的研究資料。** 那天地面站換了 IP，
# timesyncd 還指著更早之前的位址，於是：對不到時 → 時鐘停在 7 天前 →
# 每一筆訊號樣本都帶著 7 天前的時戳 → 地面站用時間區間歸屬，**對不到任何
# 架次，整批當成「架次外」丟掉** → 11 趟飛行的採樣一筆都沒進資料庫。
# 而全程只有機上日誌裡的一行。
#
# GS_HOST 本來就是「地面站在哪」的單一事實源（遙測與記錄通道都用它），
# 對時沒有理由自己記一份。
#
# 2026-10-19 兩個洞：
#   * 比對永遠不相等——`$(cat)` 會吃掉結尾換行，而 want 帶著換行。於是**每次**
#     代理重啟都當成換了伺服器：重寫、重啟 timesyncd、刪掉「已對過」旗標。地面站
#     那一刻連不到的話，一台早就對好時的機會被自己打回 clock_synced=false。
#   * 舊的 uav-gs.conf 還在。timesyncd 的 NTP= 在多個 drop-in 之間是**累加**的，
#     所以伺服器清單是「新位址＋安裝當下的舊位址」，連不到新的就去試舊的
#     （當天日誌：.22 逾時 → 換試 .21）。這份由這裡全權負責，舊的刪掉，
#     FallbackNTP= 一併搬過來（不退回網路上的伺服器，見 install-service.sh）。
if [ -n "${GS_HOST:-}" ]; then
  want="[Time]
NTP=${GS_HOST}
FallbackNTP="
  changed=0
  if [ "$(cat "$DROPIN" 2>/dev/null)" != "$want" ]; then
    mkdir -p "$(dirname "$DROPIN")"
    printf '%s\n' "$want" > "$DROPIN"
    echo "對時：NTP 位址改成 ${GS_HOST}（跟著 GS_HOST，不另外記一份）"
    changed=1
  fi
  if [ -e "$LEGACY" ]; then
    echo "對時：刪掉舊的 $(basename "$LEGACY")（$(grep -h '^NTP=' "$LEGACY" 2>/dev/null)）——NTP= 會累加，留著就會去試舊位址"
    rm -f "$LEGACY"
    changed=1
  fi
  if (( changed )); then
    systemctl --no-block restart systemd-timesyncd 2>/dev/null || true
    rm -f "$FLAG" 2>/dev/null || true     # 換了伺服器，舊的「已對過」不算數
  fi
fi

if [ -e "$FLAG" ]; then
  echo "對時：已對過（$(date '+%F %T')），不必等"
  exit 0
fi
echo "對時：開機後還沒對過——催 systemd-timesyncd 立刻試，最多等 ${WAIT} 秒"
systemctl --no-block restart systemd-timesyncd 2>/dev/null || true

for ((i = 1; i <= WAIT; i++)); do
  sleep 1
  if [ -e "$FLAG" ]; then
    # 牆鐘剛被調過，用單調的 %N 差算等了多久會被跳動騙——改數迴圈次數
    echo "對時：成功（等了約 ${i} 秒）→ $(date '+%F %T')；$(timedatectl timesync-status 2>/dev/null | grep -E 'Server|Offset' | tr -s ' ' | tr '\n' ' ')"
    exit 0
  fi
  if (( i % 20 == 0 )); then
    systemctl --no-block restart systemd-timesyncd 2>/dev/null || true
  fi
done
echo "對時：**逾時 ${WAIT} 秒，照樣啟動代理**——機上時間可能不準（地面站 NTP 連不到？5G 還沒起來？）。"\
"代理會在狀態列標 clock_synced=false；timesyncd 仍會繼續試"
exit 0
