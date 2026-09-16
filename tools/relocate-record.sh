#!/bin/bash
# 把錄製 spool 從 /home/pi/uav-record 搬到 /opt/uav-record（清掉家目錄裡最後一個 agent 痕跡）。
#
#   sudo bash relocate-record.sh
#
# 安全：上傳「已傳」是用檔名記在 .uploaded.json（跟著一起搬），伺服器端也去重，
# 所以整個目錄搬走不會重傳。冪等：搬過就跳過。
set -euo pipefail
[ "$EUID" -eq 0 ] || { echo "請用 sudo 執行"; exit 1; }

OLD=/home/pi/uav-record
NEW=/opt/uav-record
SVC=/etc/systemd/system/uav-agent.service

armed=$(python3 -c 'import json;print(json.load(open("/opt/uav-agent/state.json")).get("armed"))' 2>/dev/null || echo unknown)
echo "armed = $armed"
[ "$armed" = "True" ] && { echo "解鎖中——中止"; exit 1; }

echo "== 1. 停服務 =="
systemctl stop uav-agent
sleep 1

echo "== 2. 搬 spool =="
if [ -d "$OLD" ] && [ ! -e "$NEW" ]; then
  mv "$OLD" "$NEW"; chown pi:pi "$NEW"
  echo "  $OLD → $NEW（$(du -sh "$NEW" | cut -f1)，擁有者 $(stat -c '%U:%G' "$NEW")）"
elif [ -e "$NEW" ]; then
  echo "  $NEW 已存在，跳過搬移"
else
  echo "  找不到 $OLD（可能已搬過）"
fi

echo "== 3. unit 指到新 spool（沒有才加，先備份）=="
if grep -q '^Environment=RECORD_DIR=' "$SVC"; then
  echo "  unit 已有 RECORD_DIR：$(grep '^Environment=RECORD_DIR=' "$SVC")"
else
  cp -a "$SVC" "$SVC.bak-$(date +%Y%m%d-%H%M%S)"
  sed -i '/^Environment=RECORD_UPLOAD_SETTLE_S=/a Environment=RECORD_DIR=/opt/uav-record' "$SVC"
  echo "  已加入 Environment=RECORD_DIR=/opt/uav-record"
fi

echo "== 4. 重載並啟動 =="
systemctl daemon-reload
systemctl start uav-agent
sleep 8

echo "== 5. 驗證（錄製檔應落在 /opt/uav-record，上傳佇列保留）=="
systemctl is-active uav-agent
journalctl -u uav-agent -n 20 --no-pager 2>/dev/null | grep '"record"' | tail -1 | \
  python3 -c 'import sys,json,re
line=sys.stdin.read()
m=re.search(r"(\{.*\})", line)
d=json.loads(m.group(1)) if m else {}
print("  record.file =", d.get("record",{}).get("file"))
print("  upload =", {k:d.get("upload",{}).get(k) for k in ("pending","files","abandoned")})' 2>/dev/null || echo "  （用 journalctl -u uav-agent -n 20 自行確認）"
ls -la "$NEW"/.uploaded.json 2>/dev/null && echo "  .uploaded.json 在（上傳歷史保留）"
echo
echo "完成。家目錄裡的 uav-record 已移除。"
