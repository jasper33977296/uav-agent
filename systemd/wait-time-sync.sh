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
WAIT=${SYNC_WAIT_S:-60}
DROPIN=${SYNC_DROPIN:-/etc/systemd/timesyncd.conf.d/50-gs.conf}

# ── 對時位址跟著 GS_HOST 走（2026-10-01）──────────────────────────────
# **寫死的下場看過了，而且代價是整天的研究資料。** 那天地面站換了 IP，
# timesyncd 還指著更早之前的位址，於是：對不到時 → 時鐘停在 7 天前 →
# 每一筆訊號樣本都帶著 7 天前的時戳 → 地面站用時間區間歸屬，**對不到任何
# 架次，整批當成「架次外」丟掉** → 11 趟飛行的採樣一筆都沒進資料庫。
# 而全程只有機上日誌裡的一行。
#
# GS_HOST 本來就是「地面站在哪」的單一事實源（遙測與記錄通道都用它），
# 對時沒有理由自己記一份。
if [ -n "${GS_HOST:-}" ]; then
  want="[Time]
NTP=${GS_HOST}
"
  if [ "$(cat "$DROPIN" 2>/dev/null)" != "$want" ]; then
    mkdir -p "$(dirname "$DROPIN")"
    printf '%s' "$want" > "$DROPIN"
    echo "對時：NTP 位址改成 ${GS_HOST}（跟著 GS_HOST，不另外記一份）"
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
