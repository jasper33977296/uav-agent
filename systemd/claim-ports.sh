#!/bin/bash
# 代理要用的序列埠，**代理在跑的時候完全佔用**（使用者 2026-10-19）。
# 由 unit 的 ExecStartPre=+（claim）與 ExecStopPost=+（release）以 root 執行。
#
# 為什麼要有這支：2026-10-19 drone E 上的 sdr-fc-telemetry.service 一直開著飛控埠。
# 代理一重啟，它搶先把埠重開而且上了 TIOCEXCL——代理開不回來（EBUSY），整台機
# 的橋就斷了。TIOCEXCL 只擋得住「之後才來開」的人，擋不住已經開著的。
# 兩個程式讀同一個序列埠，每個位元組只會被其中一個讀走：就算沒有獨佔，雙方
# 也都會漏訊息。
#
#   claim：找出開著 FC_DEV／MODEM_PORT 的行程。屬於某個 systemd unit 的
#          → `mask --runtime`（這次開機內不准再起，免得 Restart= 把它拉回來搶）＋stop；
#          不屬於 unit 的 → TERM，3 秒後還在就 KILL。做了什麼都寫進日誌，
#          也記在 /run/uav-agent/claimed，release 才知道要放誰回來
#   release：把 claim 時 mask 的 unit 解除 mask。**不主動把它們啟動**——
#          代理停下來的原因可能就是有人在查問題，自動把搶埠的程式拉起來只會添亂
set -u
MODE=${1:-claim}
STATE=/run/uav-agent/claimed
mkdir -p "$(dirname "$STATE")"

if [ "$MODE" = release ]; then
  [ -f "$STATE" ] || exit 0
  while read -r u; do
    [ -n "$u" ] || continue
    systemctl unmask --runtime "$u" >/dev/null 2>&1 && echo "佔埠：代理停了，$u 解除封鎖（沒有自動啟動它）"
  done < "$STATE"
  rm -f "$STATE"
  exit 0
fi

ports=()
for p in "${FC_DEV:-}" "${MODEM_PORT:-}"; do
  [ -n "$p" ] && [ -e "$p" ] && ports+=("$(readlink -f "$p")")
done
[ ${#ports[@]} -gt 0 ] || exit 0

holders() {   # 印出開著任何一個埠的 pid
  local fd t
  for fd in /proc/[0-9]*/fd/*; do
    t=$(readlink "$fd" 2>/dev/null) || continue
    for p in "${ports[@]}"; do
      if [ "$t" = "$p" ]; then echo "${fd#/proc/}" | cut -d/ -f1; fi
    done
  done | sort -u
}

for pid in $(holders); do
  [ "$pid" = $$ ] && continue
  cmd=$(tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null | cut -c1-80)
  # **只認系統服務**（/system.slice/ 底下）。使用者工作階段裡的行程 cgroup 會有
  # user@1000.service——把那個當成「它的服務」去停，等於把整個使用者工作階段關掉
  unit=$(sed -n 's|^0::/system\.slice/\([^/]*\.service\).*|\1|p' /proc/$pid/cgroup 2>/dev/null)
  if [ -n "$unit" ] && [ "$unit" != uav-agent.service ]; then
    systemctl mask --runtime "$unit" >/dev/null 2>&1
    systemctl stop "$unit" >/dev/null 2>&1
    grep -qx "$unit" "$STATE" 2>/dev/null || echo "$unit" >> "$STATE"
    echo "佔埠：$unit（pid $pid：$cmd）開著代理要用的序列埠——這次開機內封鎖並停止，代理停了才放回"
  elif [ -z "$unit" ]; then
    kill -TERM "$pid" 2>/dev/null
    echo "佔埠：pid $pid（$cmd）開著代理要用的序列埠，不屬於任何服務——已終止"
  fi
done
# 等它們真的放手
for i in 1 2 3; do
  left=$(holders | grep -vx $$ || true)
  [ -z "$left" ] && exit 0
  sleep 1
done
for pid in $left; do kill -KILL "$pid" 2>/dev/null && echo "佔埠：pid $pid 3 秒內沒放手——強制終止"; done
exit 0
