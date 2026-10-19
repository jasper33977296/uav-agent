#!/bin/bash
# 機上一次性的系統設定（以 root 在 Pi 上跑）。**平常不必手動跑**：
# `deploy.sh` 遇到空機時會自己傳上來執行（見 README〈部署〉）。
#
#   sudo ./install-service.sh
#
# **可以重跑**：每一項都先看現況，已經是對的就跳過。需要重開機才生效的
# 改動（config.txt）做了的話，最後一行印 `REBOOT_NEEDED`，由 deploy.sh 決定重開。
#
# 這裡收的是「換一張 SD 卡就會掉」的東西——有兩項（USB 供電上限、日誌持久化）
# 2026-08 是在第一台機上手動修的、沒寫進任何腳本，裝第二台時才發現要靠記憶。
set -e
[ "$EUID" -eq 0 ] || { echo "請用 sudo 執行"; exit 1; }
HERE="$(cd "$(dirname "$0")" && pwd)"
CFG=/boot/firmware/config.txt
REBOOT=0
#: 代理以哪個帳號執行。unit 預設 pi；不是 pi 的機（例如 Ubuntu 上的 k200）用 drop-in 改
AGENT_USER=${AGENT_USER:-pi}
id "$AGENT_USER" >/dev/null 2>&1 || { echo "沒有 $AGENT_USER 這個帳號"; exit 1; }

cfg_line() {   # cfg_line <整行>：config.txt 沒有這行就加在檔尾的 [all] 段
  local line="$1"
  if grep -qx "$line" "$CFG"; then
    echo "   $line：已存在"
    return
  fi
  [ -f "$CFG.bak-uav" ] || cp -a "$CFG" "$CFG.bak-uav"   # 第一次改之前的原廠版
  # 加在檔尾的 [all] 段：前面若停在 [pi4]／[cm5] 之類的條件段，接著寫會只對那種板子生效
  printf '\n[all]\n%s\n' "$line" >> "$CFG"
  echo "   $line：已加入（原檔備份 $CFG.bak-uav）"
  REBOOT=1
}

echo "== 1. 開機設定（config.txt）=="
# 排針 UART（GPIO14/15）。Pi 5 的 ttyAMA10 是**除錯接頭**那顆，不是排針上的；
# 要這個 overlay 才有 /dev/ttyAMA0，否則代理接不到飛控
cfg_line "dtoverlay=uart0-pi5"
# USB 供電上限 600mA → 1.6A。機上 5V 來自不做 USB-PD 協商的降壓模組，Pi 問不到
# 電源能力就把下游 USB 鎖在 600mA；5G dongle 申報 500mA，發射峰值時整顆重開
# （2026-08-26：0.5 Mbps 就掉線；修好後 52 Mbps 跑 90 秒零事件）
cfg_line "usb_max_current_enable=1"

echo
echo "== 2. 日誌留到重開機之後 =="
# Raspberry Pi OS 原廠 `40-rpi-volatile-storage.conf` 是 Storage=volatile：
# 任何跨重開機的故障事後都查不到。要用編號更大的 drop-in 蓋掉（改主檔沒用）
J=/etc/systemd/journald.conf.d/50-persistent.conf
WANT=$'[Journal]\nStorage=persistent\nSystemMaxUse=200M\nSystemMaxFiles=6'
if [ "$(cat "$J" 2>/dev/null)" = "$WANT" ]; then
  echo "   已是 persistent"
else
  mkdir -p "$(dirname "$J")" /var/log/journal
  printf '%s\n' "$WANT" > "$J"
  systemctl restart systemd-journald
  echo "   已設 persistent（上限 200M，顧 SD 卡壽命）"
fi

echo
echo "== 3. 目錄與每台機自己的設定 =="
install -d -o "$AGENT_USER" -g "$AGENT_USER" /opt/uav-record
echo "   /opt/uav-record（錄製 spool）"
# **每台機不一樣的參數放這裡，不要改 unit**：unit 是 repo 裡所有機共用的一份，
# deploy.sh 每次都會把它裝回 repo 的版本——在 unit 裡改的單機設定下次部署就沒了。
# 這個檔 deploy.sh 不碰；systemd 規定 EnvironmentFile 蓋過 Environment=
ENVF=/etc/uav-agent.env
if [ -f "$ENVF" ]; then
  echo "   $ENVF：已存在，不動"
else
  cat > "$ENVF" <<'EOF'
# 這台機自己的參數，蓋過 /etc/systemd/system/uav-agent.service 裡的預設值。
# deploy.sh 不會動這個檔。改完：sudo systemctl restart uav-agent
# 例：
# MODEM_PORT=/dev/ttyUSB2
# FC_BAUD=115200
EOF
  echo "   $ENVF：已建立（空的，全用 unit 的預設值）"
fi

echo
echo "== 4. 服務 =="
if cmp -s "$HERE/systemd/uav-agent.service" /etc/systemd/system/uav-agent.service; then
  echo "   unit 已是這一版"
else
  install -m 0644 "$HERE/systemd/uav-agent.service" /etc/systemd/system/uav-agent.service
  systemctl daemon-reload
  echo "   已安裝 /etc/systemd/system/uav-agent.service"
fi
# 帳號不是 pi：用 drop-in 改 User=，**不改 unit 本身**（unit 是所有機共用、每次部署會裝回）
DROP=/etc/systemd/system/uav-agent.service.d/10-user.conf
if [ "$AGENT_USER" = pi ]; then
  [ -f "$DROP" ] && rm -f "$DROP" && systemctl daemon-reload && echo "   執行帳號回到 pi"
else
  WANT_U=$'[Service]\nUser='"$AGENT_USER"
  if [ "$(cat "$DROP" 2>/dev/null)" != "$WANT_U" ]; then
    mkdir -p "$(dirname "$DROP")"; printf '%s\n' "$WANT_U" > "$DROP"; systemctl daemon-reload
  fi
  echo "   執行帳號：$AGENT_USER（$DROP）"
fi
chown -R "$AGENT_USER:$AGENT_USER" "$HERE" 2>/dev/null || true
systemctl enable --quiet uav-agent
echo "   開機自啟：$(systemctl is-enabled uav-agent)"

echo
echo "== 4b. 5G dongle 的網路與 AT 埠 =="
# **ModemManager 會搶 AT 埠**：代理直接對 AT 埠取樣，ModemManager 也會去探測、
# 送它自己的指令——兩邊互相吃掉回應。Fibocom FM160 自己撥號（GTRNDIS），不需要它
if systemctl is-enabled --quiet ModemManager 2>/dev/null || systemctl is-active --quiet ModemManager 2>/dev/null; then
  systemctl disable --now ModemManager >/dev/null 2>&1 || true
  echo "   ModemManager：已停用（會搶 AT 埠）"
else
  echo "   ModemManager：沒有在跑"
fi
# **netplan＋networkd 的系統（Ubuntu）不會自己管 dongle 的網卡**：沒有設定檔對到它，
# 網卡就一直是 off、沒有 IP（2026-10-19 第二台機：dongle 已撥上網、分到 IP，
# Pi 卻從沒把網卡打開）。NetworkManager 的系統（Raspberry Pi OS）會自動 DHCP，不必做。
# **用驅動比對，不寫死網卡名**：名字是 enx＋MAC，而這顆 dongle 每次重新列舉 MAC 都不一樣
NP=/etc/netplan/60-5g-dongle.yaml
if ! systemctl is-active --quiet NetworkManager && command -v netplan >/dev/null; then
  if [ -f "$NP" ]; then
    echo "   netplan：$NP 已存在"
  else
    cat > "$NP" <<'EOF'
# 5G dongle（Fibocom FM160，cdc_ether）。用驅動比對，不寫死網卡名（enx＋MAC，MAC 每次列舉都變）
network:
  version: 2
  ethernets:
    dongle5g:
      match:
        driver: cdc_ether
      dhcp4: true
      dhcp4-overrides:
        route-metric: 700
EOF
    chmod 600 "$NP"
    netplan apply 2>/dev/null || true
    echo "   netplan：已建 $NP 並套用"
  fi
else
  echo "   網路由 NetworkManager 管：dongle 網卡會自動 DHCP"
fi

echo
echo "== 5. 時間跟地面站對齊 =="
# 機上資料（訊號、補傳、錄製、事件）的時間戳要跟地面站**同一個時鐘**，
# 否則對不齊。只跟地面站對時、不退回網路上的伺服器：兩邊各自對到不同來源，
# 實測就差了 0.58 秒（2026-09-14）。地面站的 ntpsec 開著 orphan 模式，
# 現場沒有網路時仍然會給時間。
# 設定檔由 systemd/wait-time-sync.sh 每次代理啟動時照 GS_HOST 寫（50-gs.conf），
# 這裡不另寫一份——另寫的那份會寫死安裝當下的位址，而 NTP= 在 drop-in 之間是累加的
GS=$(sed -n 's/^GS_HOST=//p' "$ENVF" | tail -1)
[ -n "$GS" ] || GS=$(sed -n 's/^Environment=GS_HOST=//p' "$HERE/systemd/uav-agent.service")
GS_HOST=$GS SYNC_WAIT_S=0 "$HERE/systemd/wait-time-sync.sh" | grep -v "照樣啟動代理" || true
echo "   NTP＝$GS"

echo
if [ "$REBOOT" = 1 ]; then
  echo "== 完成。config.txt 有改：要重開機才生效 =="
  echo "REBOOT_NEEDED"
else
  echo "== 完成，不必重開機 =="
fi
