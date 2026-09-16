#!/bin/bash
# 把機上 agent 從 /home/pi/uav-agent 搬到 /opt/uav-agent（不容易被誤刪：家目錄裡看不到，
# 父目錄 /opt 是 root 所有，rm 掉整個目錄也要 sudo）。
#
#   sudo bash relocate-to-opt.sh
#
# 冪等且會自保：解鎖中就中止、目標已存在就中止、改 unit 前先備份。
# venv 一起搬沒問題——unit 用的 venv/bin/python 是指向 /usr/bin/python3 的符號連結。
set -euo pipefail
[ "$EUID" -eq 0 ] || { echo "請用 sudo 執行"; exit 1; }

OLD=/home/pi/uav-agent
NEW=/opt/uav-agent
SVC=/etc/systemd/system/uav-agent.service

echo "== 前置檢查 =="
[ -d "$OLD" ] || { echo "找不到 $OLD（是不是已經搬過了？）"; exit 1; }
[ ! -e "$NEW" ] || { echo "$NEW 已存在，不覆蓋——請先確認它是什麼"; exit 1; }
[ -f "$SVC" ] || { echo "找不到 $SVC"; exit 1; }

# 安全：解鎖中一律不動（Pi 上沒有 healthz，看 agent 自己寫的 state.json）
armed=$(python3 -c 'import json,sys;print(json.load(open("'"$OLD"'/state.json")).get("armed"))' 2>/dev/null || echo unknown)
echo "   state.json armed = $armed"
[ "$armed" = "True" ] && { echo "飛控在解鎖狀態——中止。等上鎖後再搬"; exit 1; }

# 別的地方還有沒有指到舊路徑（除了這個 unit 本身）
echo "   其他指向舊路徑的地方："
grep -rl "$OLD" /etc/systemd /etc/cron* 2>/dev/null | grep -vx "$SVC" | sed 's/^/     /' || true
echo "     （以上若有，搬完要一起改；沒列出＝只有 unit 指到它）"

echo "== 1. 停服務 =="
systemctl stop uav-agent
sleep 1

echo "== 2. 搬目錄 =="
mv "$OLD" "$NEW"
chown pi:pi "$NEW"          # mv 已保留擁有者，這行只是保險
echo "   $OLD → $NEW（擁有者 $(stat -c '%U:%G' "$NEW")）"

echo "== 3. 改 unit 的路徑（先備份）=="
cp -a "$SVC" "$SVC.bak-$(date +%Y%m%d-%H%M%S)"
sed -i "s#${OLD}#${NEW}#g" "$SVC"
echo "   WorkingDirectory / ExecStart 已改指到 $NEW"
grep -E '^(WorkingDirectory|ExecStart)=' "$SVC" | sed 's/^/     /'

echo "== 4. 重載並啟動 =="
systemctl daemon-reload
systemctl start uav-agent
sleep 3

echo "== 5. 驗證 =="
systemctl is-active uav-agent
before=$(stat -c %Y "$NEW/state.json" 2>/dev/null || echo 0)
sleep 6
after=$(stat -c %Y "$NEW/state.json" 2>/dev/null || echo 0)
[ "$after" -gt "$before" ] && echo "   state.json 有在更新（agent 活著、正常寫檔）" \
                           || echo "   ⚠ state.json 沒更新——用 journalctl -u uav-agent -n 30 看一下"
echo
echo "完成。舊路徑 $OLD 已不存在；agent 現在住在 $NEW。"
echo "venv 裡的 pip / mav* 工具因為搬家 shebang 失效（agent 不用它們）；日後要用 pip 再重建 venv。"
