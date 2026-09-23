#!/bin/bash
# 機上供電／負載的 1 Hz 黑盒子（uav-system issues/072）。
#
# **為什麼需要這個**：Pi 無預警重開時，日誌在一行正常訊息之後直接斷掉——
# 沒有關機程序、沒有核心訊息。那有兩種來源，而現有證據分不出來：
#   1. **斷電**（整台瞬間消失）
#   2. **卡住**（userspace／systemd 停了 60 秒，RPi OS 預設的 systemd 看門狗
#      `/usr/lib/systemd/system.conf.d/40-rpi-enable-watchdog.conf` 把它硬重置）
#
# 這支記的是**斷掉前最後一刻的電壓與負載**。每寫一行就 `sync`——不 sync 的話
# 斷電會把最後那幾秒吃掉，而那幾秒正是要看的（09-22 就這樣丟過一次）。
#
# 搭配地面站的 `scripts/watch-pi-outage.py` 一起看：那邊記 ICMP（核心回的，
# 不需要 userspace），所以「日誌死了但還在回 ping」＝卡住，「兩邊同時消失」＝斷電。
set -u
LOG="${POWER_LOG:-/var/log/uav-power.log}"
MAX_BYTES=$((32 * 1024 * 1024))       # 超過就換檔，只留一份舊的

while :; do
  if [ -f "$LOG" ] && [ "$(stat -c %s "$LOG" 2>/dev/null || echo 0)" -gt "$MAX_BYTES" ]; then
    mv -f "$LOG" "$LOG.1"
  fi
  v=$(vcgencmd pmic_read_adc EXT5V_V 2>/dev/null | grep -o '=[0-9.]*V' | tr -d '=V')
  c=$(vcgencmd pmic_read_adc VDD_CORE_A 2>/dev/null | grep -o '=[0-9.]*A' | tr -d '=A')
  t=$(vcgencmd measure_temp 2>/dev/null | tr -d "temp='C")
  th=$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2)
  load=$(cut -d' ' -f1-3 /proc/loadavg)
  # **看門狗最後一次被餵是什麼時候**：systemd 卡住的話這個值會停住，
  # 而它停住 60 秒之後硬體就會重置——事後對得出來
  wd=$(systemctl show -p WatchdogLastPingTimestampMonotonic --value 2>/dev/null)
  printf '%s EXT5V=%s VDD_CORE_A=%s temp=%s throttled=%s load=%s wd_ping=%s\n' \
    "$(date '+%F %T')" "${v:-?}" "${c:-?}" "${t:-?}" "${th:-?}" "$load" "${wd:-?}" >> "$LOG"
  sync -d "$LOG" 2>/dev/null || sync
  sleep 1
done
