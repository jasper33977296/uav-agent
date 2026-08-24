#!/bin/bash
# 部署到 Raspberry Pi 5：UART overlay 永久化 + 安裝 systemd 服務。
#
#   sudo ./install-service.sh
#
# 前提：已建好 venv 並裝好相依（見 README「安裝」）。
set -e
[ "$EUID" -eq 0 ] || { echo "請用 sudo 執行"; exit 1; }
HERE="$(cd "$(dirname "$0")" && pwd)"
CFG=/boot/firmware/config.txt

echo "== 1. UART overlay 永久化 =="
# Pi 5 的 ttyAMA10 是**除錯接頭**那顆，不是 40-pin 排針上的。排針 UART
# （GPIO14/15）要靠這個 overlay 才會出現 /dev/ttyAMA0。
# 執行期 `dtoverlay uart0-pi5` 重開機就沒了，服務會找不到裝置。
if grep -q "^dtoverlay=uart0-pi5" $CFG; then
  echo "   已存在，跳過"
else
  cp -a $CFG $CFG.bak-$(date +%Y%m%d-%H%M%S)
  sed -i '/^\[pi5\]$/a dtoverlay=uart0-pi5' $CFG
  echo "   已加入 dtoverlay=uart0-pi5（備份：$(ls -t $CFG.bak-* | head -1)）"
fi
grep -n "\[pi5\]" -A 4 $CFG | sed 's/^/   /'

echo
echo "== 2. 安裝服務 =="
install -m 0644 "$HERE/systemd/uav-agent.service" /etc/systemd/system/uav-agent.service
systemctl daemon-reload
echo "   已安裝 /etc/systemd/system/uav-agent.service"
echo "   部署參數（GS_HOST 等）改在該檔的 Environment= 行，不要改程式碼"

echo
echo "== 3. 完成。尚未啟動 =="
echo "   先手動驗證： sudo systemctl start uav-agent && journalctl -u uav-agent -f"
echo "   確認無誤後 ： sudo systemctl enable uav-agent"
