#!/bin/bash
# 讓 dongle 的網卡重新跟 dongle 要一次 IP（代理以 sudo -n 呼叫；netwatch 判斷要不要）。
#
# 2026-10-20 對調 dongle：drone C（NetworkManager）換上 drone E 的 dongle 後拿到的是
# dongle 自己的內部位址 192.168.187.22（不是電信 IP），而 NetworkManager 就一直留著
# 那個租約、不再重新要——6 分鐘連不到地面站，直到換回來。同一顆 dongle 在 drone E
# （networkd）上 14 秒就拿到電信 IP。重新要一次就好；這支只負責「重新要」，要不要重新要
# 由代理看「有 IP、但地面站兩個方向都不通」決定。
#
#   sudo dongle-renew.sh <網卡>
set -u
NIC=${1:?要給網卡名}
[ -e "/sys/class/net/$NIC" ] || { echo "沒有 $NIC"; exit 1; }
# 只准動數據機的網卡：防止被拿去重設乙太網或 Wi-Fi
drv=$(basename "$(readlink -f "/sys/class/net/$NIC/device/driver")" 2>/dev/null)
case "$drv" in cdc_ether|rndis_host|qmi_wwan|cdc_mbim|cdc_ncm) ;; *)
  echo "$NIC 的驅動是 ${drv:-?}，不是數據機——不動"; exit 1 ;; esac

if systemctl is-active --quiet NetworkManager; then
  # 先清掉這張網卡的租約快取：不清的話 NetworkManager 會再去要同一個舊位址
  rm -f /var/lib/NetworkManager/*-"$NIC".lease 2>/dev/null
  nmcli device disconnect "$NIC" >/dev/null 2>&1
  nmcli device connect "$NIC" >/dev/null 2>&1 && echo "NetworkManager：$NIC 重新連線（清掉舊租約）"
elif systemctl is-active --quiet systemd-networkd; then
  networkctl reconfigure "$NIC" 2>/dev/null || networkctl renew "$NIC"
  echo "networkd：$NIC 重新設定"
elif command -v dhcpcd >/dev/null; then
  dhcpcd -k "$NIC" >/dev/null 2>&1; dhcpcd -n "$NIC" >/dev/null 2>&1
  echo "dhcpcd：$NIC 重新要 IP"
else
  ip link set "$NIC" down; sleep 1; ip link set "$NIC" up
  echo "$NIC 網卡重開（認不出網路管理者）"
fi
