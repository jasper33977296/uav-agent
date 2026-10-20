#!/bin/bash
# 把這個 repo 的代理部署到機上的 Pi（uav-system issues/069）。**在開發機上跑。**
#
#   ./deploy.sh              # 部署目前的 commit
#   ./deploy.sh --env drone-e.env   # 空機：精靈帶著填安裝資訊（找得到的給你確認），再從頭裝
#   ./deploy.sh --dry-run    # 只看：dongle、閘門判定、哪些檔會變、unit 有沒有差——不動機上
#   ./deploy.sh --rollback   # 還原上一次部署前的備份並重啟
#
# 其他選項：
#   --host <addr>   指定 Pi（預設依序試 UAV_PI、地面站學到的、10.141.2.32、10.101.129.121）。
#                   IPv6 本地位址也行：--host 'fe80::...%eno2'
#   --dirty         工作目錄有未提交的改動也照部署（DEPLOYED 會標 dirty）
#   --env <檔>      這台機的安裝資訊（帳號、密碼、地面站、AT 埠、sysid）。**空機一定要給**，
#                   而且每一欄都要填才會開始——缺哪一欄就說哪一欄、什麼都不做。
#                   範本：drone.env.example；填好的 drone-*.env 不進 git
#
# **一鍵：空機與舊機同一行指令**（2026-10-19 使用者要求）。機上沒有 venv＝空機，
# 會先做初次安裝：裝 SSH 金鑰與免密碼 sudo → 傳程式 → **在開發機下載相依套件
# 再推上去**（機上不需要網路）→ 建 venv → 跑 install-service.sh（開機設定、
# 日誌、服務、對時）→ 需要的話重開機並等它回來 → 接著走一般部署與驗收。
#
# **5G dongle 沒過就不部署**（使用者 2026-10-19）：動機上任何東西之前先跑
# tools/dongle-check.py（USB、網卡、經 5G 到地面站、AT 埠量得到訊號），驗收時
# 也要代理自己真的量到訊號。這台機的用途就是經 5G 跟地面站講話、同時量 5G。
#
# 為什麼要有這支（069）：以前每次都是手動 scp＋手動 md5 比一個檔，機上說不出跑的是
# 哪個 commit，解鎖中不准重啟也全靠人記得。這支把那六個手動步驟變成固定的順序：
#
#   1. 前置：工作目錄乾淨、連得到 Pi
#   2. **解鎖閘門**：代理在跑 → state.json 30 秒內更新過而且 armed=false，否則中止
#      （**不知道就當作不行**——代理死了的話 state.json 停在最後一刻，armed=false
#      可能是舊的）。代理本來就停著 → 沒有橋會被打斷，照部署
#   3. 備份要被覆蓋的檔到 .bak-deploy-<時間>-<舊 commit>/，只留最近 5 份
#   4. rsync **git 追蹤的檔案**。不刪任何東西、不碰 state.json／.fc_sysid／venv
#   5. unit 有差：只裝 unit（備份舊的、daemon-reload），**不碰 config.txt**
#      ——那是 install-service.sh 的事
#   6. requirements.txt 有變：在開發機下載套件推上去裝（機上不必有網路）
#   7. install-service.sh（可重跑；系統設定收斂到 repo 的版本，要重開機才重開）
#   8. 寫 DEPLOYED、重啟、逐檔比 md5、等狀態列出現並驗收（含 5G 取樣）
#
# **驗收失敗不自動回滾**：自動回滾會把「新版哪裡不對」的現場蓋掉。失敗時印出
# 回滾指令，由人決定。
set -euo pipefail

# ── 離場保險 ─────────────────────────────────────────────────
# **代理是這支停的，就一定要由這支帶回來**（2026-10-20）。7b 要停代理才讀得到飛控參數；
# 那一輪 fc-setup 沒印出 JSON，`grep` 找不到東西在 pipefail 下讓整支安靜地結束——
# drone C 因此 15 分鐘沒有代理、地面站顯示未連接，而部署的輸出裡一個字都沒有。
# 不管怎麼離開（die、set -e、Ctrl-C），停過的代理都要啟動回來，而且要說出來
AGENT_STOPPED=0
TMPS=()
cleanup() {
  local rc=$?
  rm -f "${TMPS[@]}" 2>/dev/null || true
  if [ "$AGENT_STOPPED" = 1 ]; then
    printf '\n✗ 部署在代理停著的時候中斷（結束碼 %s）——**把代理啟動回來**\n' "$rc" >&2
    if R "sudo systemctl start uav-agent" 2>/dev/null; then
      printf '   代理：%s\n' "$(R "systemctl is-active uav-agent" 2>/dev/null)" >&2
    else
      printf '   ✗✗ 啟動不回來——**這台機現在沒有代理**，去機上 sudo systemctl start uav-agent\n' >&2
    fi
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP     # 被中斷也要走 EXIT 那條（bash 被訊號殺掉時不一定會跑 EXIT trap）

HERE="$(cd "$(dirname "$0")" && pwd)"
DEST="${UAV_DEST:-/opt/uav-agent}"   # UAV_DEST 只給測試閘門用（指向一個假的目錄）
UNIT=/etc/systemd/system/uav-agent.service
KEEP=5
STATE_FRESH_S=30
VERIFY_WAIT_S=45

DRY=0; ROLLBACK=0; DIRTY_OK=0; PIP=0; HOST="${UAV_PI:-}"; ENVFILE=""
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --rollback) ROLLBACK=1 ;;
    --dirty) DIRTY_OK=1 ;;
    --pip) PIP=1 ;;     # 舊選項：現在相依有變一律自動裝，留著不報錯
    --host) HOST="$2"; shift ;;
    --env) ENVFILE="$2"; shift ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "不認得的選項：$1" >&2; exit 2 ;;
  esac
  shift
done

say()  { printf '%s\n' "$*"; }
step() { printf '\n== %s ==\n' "$*"; }
die()  { printf '✗ %s\n' "$*" >&2; exit 1; }

# ── 本機模式 ─────────────────────────────────────────────────
# **在 Pi 上 git pull 下來直接跑**（使用者 2026-10-19）：沒給 --host、而且這台就是
# 樹莓派 → 裝到自己身上，不走 SSH。裝好之後**刪掉 pull 下來的這個資料夾**
# （程式只留在 /opt/uav-agent；資料夾裡的 drone-*.env 含密碼，一起刪）。
# 相依套件在機上自己下載，所以這個模式機上要能上網；不能的話從地面站跑
LOCAL=0
if [ -z "$HOST" ] && grep -qa "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
  LOCAL=1
  HOST=localhost
  [ -n "$ENVFILE" ] || ENVFILE=machine.env
fi

# ── 0. 這台機的安裝資訊 ──────────────────────────────────────
# **填好才開始**（使用者 2026-10-19）：機密與必要參數全部填齊之前什麼都不做——
# 裝到一半才發現少一個值，機上會停在一個半套的狀態。欄位見 drone.env.example
REQUIRED=(UAV_PI_HOST UAV_PI_USER UAV_PI_PASS GS_HOST MODEM_PORT FC_SYSID FC_BAUD)
# 本機模式不必連線：帳號就是正在跑的人，密碼只在 sudo 要密碼時才問
[ "$LOCAL" = 1 ] && REQUIRED=(GS_HOST MODEM_PORT FC_SYSID FC_BAUD)
#: 寫進機上 /etc/uav-agent.env 的那幾欄（蓋過 unit 的共用預設）
MACHINE_KEYS=(GS_HOST MODEM_PORT FC_SYSID FC_BAUD)
cd "$HERE"
if [ -n "$ENVFILE" ]; then
  step "0. 安裝資訊（$ENVFILE）"
  # **有終端機就走精靈**（tools/env-wizard.py）：只有人知道的（帳密）用問的，
  # 找得到的（Pi 在哪、地面站、AT 埠、飛控 sysid）先找好給人確認。找的時候只讀，
  # 人確認之前機上什麼都沒動。沒有終端機（自動化）就照檔案，缺欄位一樣停
  if [ -t 0 ]; then
    python3 tools/env-wizard.py $([ "$LOCAL" = 1 ] && echo --local) "$ENVFILE" \
      || die "安裝資訊沒有確認——**什麼都還沒做**"
  fi
  [ -f "$ENVFILE" ] || die "找不到 $ENVFILE——在終端機上跑會有精靈帶著填，或從 drone.env.example 複製一份填好"
  case "$ENVFILE" in /*) ;; *) ENVFILE="./$ENVFILE" ;; esac
  set -a; . "$ENVFILE"; set +a
  MISSING=()
  for k in "${REQUIRED[@]}"; do [ -n "${!k:-}" ] || MISSING+=("$k"); done
  [ ${#MISSING[@]} = 0 ] || die "$ENVFILE 還沒填完：${MISSING[*]}——**什麼都還沒做**，填好再跑"
  for k in "${REQUIRED[@]}"; do
    case $k in *PASS*) say "   $k=（已填，${#UAV_PI_PASS} 字）" ;; *) say "   $k=${!k}" ;; esac
  done
  [ -n "$HOST" ] || HOST=$UAV_PI_HOST
fi
U="${UAV_PI_USER:-pi}"
[ "$LOCAL" = 1 ] && U=$(id -un)

# ── 1. 前置 ──────────────────────────────────────────────────
step "1. 前置"
COMMIT=$(git rev-parse --short HEAD)
DIRTY=false
if ! git diff --quiet HEAD -- || [ -n "$(git ls-files --others --exclude-standard)" ]; then
  if [ "$ROLLBACK" = 0 ] && [ "$DIRTY_OK" = 0 ]; then
    git status --short | sed 's/^/   /'
    die "工作目錄有未提交的改動——部署出去的東西會對不上任何 commit。先提交，或明說 --dirty"
  fi
  DIRTY=true
fi
say "   commit $COMMIT$([ "$DIRTY" = true ] && echo '（dirty）')"

SSH_OPTS=(-o ConnectTimeout=8 -o BatchMode=yes -o StrictHostKeyChecking=accept-new)

# 機上位址**先問地面站**（2026-10-01）。指令服務會從實際收到的封包學對端位址
# 並寫進 peers.json——那是「這台機現在在哪」的事實源，而寫死的清單不是。
# 2026-10-01 就踩到：地面站與機上都換了網段，這支腳本的兩個預設值全部失效。
# 問不到才退回寫死的清單（手邊沒有地面站時仍然能用）。
learn_host() {
  docker exec uav-command cat /state/peers.json 2>/dev/null \
    | python3 -c "import json,sys
try:
    d = json.load(sys.stdin).get('peers') or {}
except Exception:
    raise SystemExit
for v in d.values():
    ip = v.get('ip')
    if ip:
        print(ip); break" 2>/dev/null
}

if [ "$LOCAL" = 1 ]; then
  say "   **本機模式**：裝到這台自己（$(hostname)，帳號 $U）"
elif [ -z "$HOST" ]; then
  LEARNED=$(learn_host || true)
  for h in $LEARNED 10.141.2.32 10.101.129.121; do
    if ssh "${SSH_OPTS[@]}" "$U@$h" true 2>/dev/null; then
      HOST=$h
      [ "$h" = "${LEARNED:-}" ] && say "   位址取自地面站學到的對端（peers.json）"
      break
    fi
  done
  [ -n "$HOST" ] || die "連不到 Pi。試過：${LEARNED:+$LEARNED（地面站學到的）、}10.141.2.32、10.101.129.121。用 --host <位址> 指定"
elif ! ssh "${SSH_OPTS[@]}" "$U@$HOST" true 2>/dev/null; then
  # **新機第一次連**：還沒有金鑰。裝一次，之後全程免密碼
  # 先分清楚是「要密碼」還是「根本連不到」——後者裝金鑰也沒用
  WHY=$(ssh "${SSH_OPTS[@]}" "$U@$HOST" true 2>&1 || true)
  case "$WHY" in *"Permission denied"*) ;; *)
    die "連不到 $U@$HOST（不是密碼的問題——位址、網路線、SSH 有沒有開）：$WHY" ;; esac
  say "   $U@$HOST 還沒有這台開發機的金鑰——裝一次（ssh-copy-id）"
  [ -f ~/.ssh/id_ed25519.pub ] || ssh-keygen -q -t ed25519 -N "" -f ~/.ssh/id_ed25519
  if [ -n "${UAV_PI_PASS:-}" ]; then
    AP=$(mktemp); TMPS+=("$AP")
    printf '#!/bin/sh\nprintf "%%s\\n" "$UAV_PI_PASS"\n' > "$AP"; chmod +x "$AP"
    UAV_PI_PASS="$UAV_PI_PASS" SSH_ASKPASS="$AP" SSH_ASKPASS_REQUIRE=force DISPLAY=:0 \
      setsid ssh-copy-id -o StrictHostKeyChecking=accept-new "$U@$HOST" </dev/null >/dev/null 2>&1 \
      || die "UAV_PI_PASS 登不進 $U@$HOST——帳號或密碼不對"
  else
    ssh-copy-id -o StrictHostKeyChecking=accept-new "$U@$HOST" \
      || die "裝金鑰失敗。沒有終端機可以問密碼的話，把密碼填進 --env 檔的 UAV_PI_PASS"
  fi
  ssh "${SSH_OPTS[@]}" "$U@$HOST" true || die "金鑰裝了還是連不到 $U@$HOST"
fi
say "   Pi：$HOST"
if [ "$LOCAL" = 1 ]; then
  R() { bash -c "$*"; }
  DSTP=$DEST
else
  R() { ssh "${SSH_OPTS[@]}" "$U@$HOST" "$@"; }
  # rsync 的目的地：IPv6 位址要加中括號
  case "$HOST" in *:*) DSTP="$U@[$HOST]:$DEST" ;; *) DSTP="$U@$HOST:$DEST" ;; esac
fi

# **免密碼 sudo**：重啟服務、裝 unit、改開機設定都要，而部署中途沒辦法問密碼。
# Raspberry Pi Imager 建的第一個帳號通常已經有；沒有就用 UAV_PI_PASS 補一次
if [ "$LOCAL" = 1 ] && ! sudo -n true 2>/dev/null && [ -t 0 ] && [ -z "${UAV_PI_PASS:-}" ]; then
  # 本機、有終端機：直接讓 sudo 問一次密碼
  sudo -v -p "   sudo 密碼（設定免密碼 sudo 用，只問這一次）：" && UAV_PI_PASS=__cached__
fi
if ! R "sudo -n true" 2>/dev/null || [ "${UAV_PI_PASS:-}" = __cached__ ]; then
  [ -n "${UAV_PI_PASS:-}" ] || die "$U 在 $HOST 上的 sudo 要密碼。在 --env 檔填 UAV_PI_PASS，會設成免密碼"
  { [ "$UAV_PI_PASS" = __cached__ ] && echo || printf '%s\n' "$UAV_PI_PASS"; } | R "sudo -S -p '' sh -c 'echo \"$U ALL=(ALL) NOPASSWD: ALL\" > /etc/sudoers.d/010_uav-nopasswd && chmod 440 /etc/sudoers.d/010_uav-nopasswd'" \
    && R "sudo -n true" || die "設定免密碼 sudo 失敗"
  say "   已設 $U 免密碼 sudo"
fi
FRESH=0
R "test -x $DEST/venv/bin/python" || FRESH=1
if [ "$FRESH" = 1 ]; then
  [ -n "$ENVFILE" ] || die "這是空機（沒有 $DEST/venv）。初次安裝用：./deploy.sh --env drone-<代號>.env（檔案不存在也行，精靈會帶著填）"
  say "   **空機**（沒有 $DEST/venv）→ 這次會先做初次安裝"
fi

# ── 1b. 5G dongle ────────────────────────────────────────────
# 參數：repo unit 的預設值，被機上 /etc/uav-agent.env 蓋過（與 systemd 同一個規則）
envval() {
  local k=$1 v
  if [ -n "$ENVFILE" ] && [ -n "${!k:-}" ]; then printf '%s' "${!k}"; return; fi
  v=$(R "sed -n 's/^$k=//p' /etc/uav-agent.env 2>/dev/null | tail -1" || true)
  [ -n "$v" ] || v=$(sed -n "s/^Environment=$k=//p" systemd/uav-agent.service)
  printf '%s' "$v"
}
dongle_check() {
  step "$1"
  local gs port
  gs=$(envval GS_HOST); port=$(envval MODEM_PORT)
  [ -n "$port" ] || die "沒有設 MODEM_PORT——代理不會量 5G，不部署"
  R "python3 - --gs '$gs' --port '$port' --state '$DEST/state.json'" < tools/dongle-check.py \
    || die "5G dongle 沒過——**不部署**（程式、服務、系統設定都還沒動）。修好再跑；dongle 的 AT 埠不是 $port 的話寫進機上 /etc/uav-agent.env"
}
dongle_check "1b. 5G dongle"

# ── 2. 解鎖閘門 ───────────────────────────────────────────────
step "2. 解鎖閘門"
GATE=$(R "python3 - <<'EOF'
import json, os, subprocess, time
active = subprocess.run(['systemctl', 'is-active', '--quiet', 'uav-agent']).returncode == 0
p = '$DEST/state.json'
try:
    age = time.time() - os.path.getmtime(p)
    armed = json.load(open(p)).get('armed')
except Exception:
    age, armed = None, None
print(json.dumps({'active': active, 'age': age, 'armed': armed}))
EOF")
read -r ACTIVE AGE ARMED < <(python3 -c "
import json,sys; g=json.loads(sys.argv[1])
print(g['active'], 'none' if g['age'] is None else round(g['age']), g['armed'])" "$GATE")
if [ "$ACTIVE" = True ]; then
  [ "$AGE" != none ] || die "代理在跑，但讀不到 state.json——不知道有沒有解鎖，中止"
  [ "$AGE" -le "$STATE_FRESH_S" ] || die "代理在跑，但 state.json 已經 ${AGE} 秒沒更新——不知道現在有沒有解鎖，中止"
  [ "$ARMED" = False ] || die "飛控是解鎖狀態（armed=$ARMED）——重啟代理等於把橋拆掉，中止"
  say "   代理在跑，state.json ${AGE} 秒前更新，armed=False → 可以重啟"
else
  say "   代理本來就停著——沒有橋會被打斷，照部署（部署完會啟動它）"
fi

# ── rollback ─────────────────────────────────────────────────
if [ "$ROLLBACK" = 1 ]; then
  step "還原上一次部署前的備份"
  LAST=$(R "ls -1d $DEST/.bak-deploy-* 2>/dev/null | sort | tail -1")
  [ -n "$LAST" ] || die "機上沒有 .bak-deploy-* 備份可還原"
  say "   來源：$LAST"
  [ "$DRY" = 1 ] && { say "   （--dry-run：不動）"; exit 0; }
  R "set -e; rsync -a '$LAST'/ '$DEST'/ --exclude=DEPLOYED.prev; \
     if [ -f '$LAST/DEPLOYED.prev' ]; then cp '$LAST/DEPLOYED.prev' '$DEST/DEPLOYED'; else rm -f '$DEST/DEPLOYED'; fi; \
     sudo systemctl restart uav-agent; sleep 3; systemctl is-active uav-agent"
  say "✓ 已還原並重啟。**這次部署新增的檔案不會被刪掉**（備份裡沒有它們）"
  exit 0
fi

# ── 3. 要傳哪些檔 ─────────────────────────────────────────────
step "3. 檔案"
LIST=$(mktemp); TMPS+=("$LIST")
git ls-files > "$LIST"
N=$(wc -l < "$LIST")
say "   git 追蹤的 $N 個檔（不刪機上任何東西；不碰 state.json／.fc_sysid／venv）"
if [ "$FRESH" = 1 ]; then CHANGES="（空機：全部 $N 個檔）"; else
CHANGES=$(rsync -ani --checksum --files-from="$LIST" ./ "$DSTP/" | grep -E '^[<>c]' || true); fi
if [ -z "$CHANGES" ]; then say "   沒有檔案不同"; else printf '%s\n' "$CHANGES" | sed 's/^/   /'; fi

# ── 4. unit ──────────────────────────────────────────────────
step "4. systemd unit"
UNIT_DIFF=$(R "cat $UNIT 2>/dev/null" | diff - systemd/uav-agent.service || true)
if [ -z "$UNIT_DIFF" ]; then say "   與 repo 相同"
elif ! R "test -f $UNIT"; then say "   機上還沒有 unit——會裝上 repo 那份"; else
  say "   **與 repo 不同**（部署時會裝上 repo 那份，舊的留備份）："
  printf '%s\n' "$UNIT_DIFF" | sed 's/^/     /'
fi

# ── 5. requirements ──────────────────────────────────────────
step "5. requirements.txt"
REQ_LOCAL=$(sha256sum requirements.txt | cut -c1-16)
# **連不上與「變了」要分開**：以前 ssh 一失敗 REQ_REMOTE 就是空的，被說成「相依有變」
REQ_REMOTE=$(R "if [ -f $DEST/requirements.txt ]; then sha256sum $DEST/requirements.txt | cut -c1-16; else echo none; fi") \
  || die "讀機上的 requirements.txt 時 ssh 失敗——Pi 可能剛斷線，機上沒有任何改動，稍後重跑"
if [ "$REQ_LOCAL" = "$REQ_REMOTE" ]; then say "   沒變"; else
  say "   **變了**（機上 ${REQ_REMOTE:-無} → $REQ_LOCAL）→ 在開發機下載套件推上去裝（機上不必有網路）"
fi

if [ "$DRY" = 1 ]; then
  [ "$FRESH" = 1 ] && say "" && say "   空機：實際部署時會先建 $DEST、推相依套件、建 venv、跑 install-service.sh（可能重開機）"
  step "--dry-run：到此為止，機上沒有任何改動"
  exit 0
fi

# 相依套件：**在開發機下載、推上去裝**。機上現場常常沒有網路（5G 只通地面站），
# 而 pymavlink／lxml 有編譯過的部分，要照機上的 Python 版本與 CPU 挑對的 wheel
push_wheels() {
  local pyv arch cache
  pyv=$(R "python3 -c 'import sys;print(\"%d.%d\" % sys.version_info[:2])'")
  arch=$(R "uname -m")
  [ "$arch" = aarch64 ] || die "機上是 $arch，這裡只準備了 aarch64 的套件"
  if [ "$LOCAL" = 1 ]; then
    R "mkdir -p $DEST/.wheels && python3 -m pip download -q --dest $DEST/.wheels --only-binary=:all: -r requirements.txt pip" \
      || die "在機上下載相依套件失敗（這個模式機上要能上網）。改從地面站跑：./deploy.sh --env <檔> --host <這台的位址>"
    say "   相依套件（Python $pyv／$arch）已在機上下載"
    return
  fi
  cache="${XDG_CACHE_HOME:-$HOME/.cache}/uav-agent-wheels/py$pyv"
  mkdir -p "$cache"
  python3 -m pip download -q --dest "$cache" --only-binary=:all: \
    --platform manylinux2014_aarch64 --platform manylinux_2_17_aarch64 \
    --platform manylinux_2_28_aarch64 --platform any --python-version "$pyv" \
    -r requirements.txt pip \
    || die "下載相依套件失敗（開發機要能上網，或 $cache 裡已經有）"
  rsync -a --delete "$cache"/ "$DSTP/.wheels/"
  say "   相依套件（Python $pyv／$arch）已推上去：$(ls "$cache" | wc -l) 個 wheel"
}

if [ "$FRESH" = 1 ]; then
  step "6a. 初次安裝"
  R "command -v rsync >/dev/null" || R "sudo apt-get install -y -q rsync" \
    || die "機上沒有 rsync，也裝不起來（沒網路）"
  R "sudo install -d -o $U -g $U $DEST"
  rsync -a --files-from="$LIST" ./ "$DSTP/"
  say "   程式已傳到 $DEST"
  push_wheels
  # Debian 把 venv 的 ensurepip 拆成另一個套件；沒裝的話用推上去的 pip wheel 自己補
  R "cd $DEST && (python3 -m venv venv 2>/dev/null || { rm -rf venv && python3 -m venv --without-pip venv \
       && W=\$(ls .wheels/pip-*.whl | head -1) && venv/bin/python \$W/pip install -q --no-index \$W; })" \
    || die "建 venv 失敗"
  R "$DEST/venv/bin/pip install -q --no-index --find-links $DEST/.wheels -r $DEST/requirements.txt" \
    || die "裝相依套件失敗"
  say "   venv 已建好：$(R "$DEST/venv/bin/python -c 'import pymavlink, serial, websocket; print(\"pymavlink\", pymavlink.__version__ if hasattr(pymavlink, \"__version__\") else \"ok\")'")"
  REQ_REMOTE=$REQ_LOCAL
fi

# ── 6. 備份、傳送、裝 unit ───────────────────────────────────
step "6. 備份與傳送"
OLD=$(R "python3 -c \"import json;print(json.load(open('$DEST/DEPLOYED')).get('commit','none'))\" 2>/dev/null || echo none")
BAK="$DEST/.bak-deploy-$(date +%Y%m%d-%H%M%S)-$OLD"
R "set -e; mkdir -p '$BAK'; cd '$DEST'; \
   rsync -a --files-from=- --ignore-missing-args ./ '$BAK'/; \
   [ -f DEPLOYED ] && cp DEPLOYED '$BAK/DEPLOYED.prev' || true" < "$LIST"
say "   備份：$BAK（機上原本的 $OLD）"
R "ls -1d $DEST/.bak-deploy-* | sort | head -n -$KEEP | xargs -r rm -rf"
rsync -a --checksum --files-from="$LIST" ./ "$DSTP/"
say "   已傳送"
if [ -n "$UNIT_DIFF" ]; then
  R "set -e; if [ -f $UNIT ]; then sudo cp $UNIT $UNIT.bak-$(date +%Y%m%d-%H%M%S); fi; \
     sudo install -m 0644 $DEST/systemd/uav-agent.service $UNIT; sudo systemctl daemon-reload"
  say "   unit 已更新並 daemon-reload（舊的留在 $UNIT.bak-*）"
fi
if [ "$REQ_LOCAL" != "$REQ_REMOTE" ]; then
  push_wheels
  R "$DEST/venv/bin/pip install -q --no-index --find-links $DEST/.wheels -r $DEST/requirements.txt" \
    || die "pip install 失敗——程式已傳上去但還沒重啟；要回到舊版用 ./deploy.sh --rollback"
fi

# ── 7. 系統設定 ──────────────────────────────────────────────
# 每次都跑：它可以重跑、已經對的就跳過，於是每台機的系統設定都收斂到 repo 這一版
# ——第一台機上那兩項手動修的設定，就是因為不在任何會重跑的地方才會漏
step "7. 系統設定（install-service.sh）"
if [ -n "$ENVFILE" ]; then
  # 這台機的參數寫進 /etc/uav-agent.env：只改這幾行，檔裡其他人加的設定不動
  for k in "${MACHINE_KEYS[@]}"; do
    R "sudo touch /etc/uav-agent.env && sudo sed -i '/^$k=/d' /etc/uav-agent.env && echo '$k=${!k}' | sudo tee -a /etc/uav-agent.env >/dev/null"
  done
  say "   /etc/uav-agent.env：${MACHINE_KEYS[*]} 照 $ENVFILE 寫入"
fi
SVC_OUT=$(R "sudo AGENT_USER=$U $DEST/install-service.sh") || { printf '%s\n' "$SVC_OUT" | sed 's/^/   /'; die "install-service.sh 失敗"; }
printf '%s\n' "$SVC_OUT" | grep -v '^REBOOT_NEEDED$' | sed 's/^/   /'
REBOOT=0; printf '%s\n' "$SVC_OUT" | grep -qx REBOOT_NEEDED && REBOOT=1

# ── 7b. 飛控參數基準（tools/fc-setup.py）──────────────────────
# 使用者 2026-10-20：MAVLink 版本、鮑率等預設參數配置要在部署裡。代理要先停（它握著
# 飛控埠），而且要先把別人從飛控埠上趕走——代理一停，像 sdr-fc-telemetry 那種服務會
# 自己回來搶（10-20 在 drone E 上實測：讀到一半 multiple access on port）
step "7b. 飛控參數基準"
FCP=${FC_PARAMS:-check}
FCDEV=$(envval FC_DEV); FCBAUD=$(envval FC_BAUD)
FCARGS="--dev ${FCDEV:-/dev/ttyAMA0} --baud ${FCBAUD:-57600}"
[ -n "${BATT_CELLS:-}" ] && FCARGS="$FCARGS --cells $BATT_CELLS"
[ -n "${BATT_CAPACITY_MAH:-}" ] && FCARGS="$FCARGS --capacity $BATT_CAPACITY_MAH"
AGENT_STOPPED=1
R "sudo systemctl stop uav-agent; sudo env FC_DEV=${FCDEV:-/dev/ttyAMA0} MODEM_PORT= $DEST/systemd/claim-ports.sh claim" | sed 's/^/   /' || true
fc_setup() { R "timeout 240 $DEST/venv/bin/python $DEST/tools/fc-setup.py $FCARGS $1"; }
FCOUT=$(fc_setup "") || true
printf '%s\n' "$FCOUT" | { grep -v '^{' || true; } | sed 's/^/ /'
# fc-setup 中途出錯就不會有 JSON——那是「沒有檢查」，不是讓整支腳本靜靜結束的理由
FCJ=$(printf '%s\n' "$FCOUT" | { grep '^{' || true; } | tail -1)
FCTODO=$(python3 -c "import json,sys
try: print(len(json.loads(sys.argv[1]).get('todo') or {}))
except Exception: print(-1)" "$FCJ")
if [ "$FCTODO" = -1 ]; then
  say "   ⚠ 讀不到飛控參數（飛控沒接、沒電？）——基準沒有檢查，不是當成正確"
elif [ "$FCTODO" -gt 0 ]; then
  DO=0
  if [ "$FCP" = apply ]; then DO=1
  elif [ -t 0 ]; then
    read -r -p "   要把這 $FCTODO 個寫進飛控嗎？（寫完讀回核對；序列埠設定會自動重開飛控）[y/N] " ans
    [ "$ans" = y ] || [ "$ans" = Y ] && DO=1
  fi
  if [ "$DO" = 1 ]; then
    FCOUT=$(fc_setup --apply) || { printf '%s\n' "$FCOUT" | sed 's/^/ /'; die "飛控參數寫入失敗（代理停著，修好再跑）"; }
    printf '%s\n' "$FCOUT" | { grep -E '✓|✗|重開' || true; } | sed 's/^/ /'
  else
    say "   **沒有寫**（FC_PARAMS=$FCP）。要寫：在 --env 檔設 FC_PARAMS=apply，或在終端機上跑時回 y"
  fi
fi
BY="$(git config user.name 2>/dev/null || whoami)@$(hostname)"
R "cat > $DEST/DEPLOYED" <<EOF
{"commit": "$COMMIT", "dirty": $DIRTY, "at": "$(date -Is)", "by": "$BY", "files": $N}
EOF

# ── 8. 重啟與驗收 ────────────────────────────────────────────
step "8. 重啟與驗收"
T0=$(R "date '+%Y-%m-%d %H:%M:%S'")
JSEL="--since '$T0'"
if [ "$REBOOT" = 1 ]; then
  # 開機設定要重開才生效。**重開後用 -b 取日誌**：這台沒有 RTC 電池，開機時鐘
  # 停在上次關機那一刻、對時後才跳，拿重開前的時間去 --since 會漏
  say "   開機設定有改 → 重開機（解鎖閘門已經過了）"
  R "sudo systemctl reboot" || true
  AGENT_STOPPED=0          # 開機自啟會把它帶起來
  sleep 15
  for i in $(seq 1 30); do R true 2>/dev/null && break; sleep 5; done
  R true || die "重開機後 150 秒還連不回 $HOST——去機旁看一下"
  say "   已重開回來（$(R uptime -p)）"
  JSEL="-b"
  VERIFY_WAIT_S=150      # 開機要等對時（最多 60 秒）、5G 註冊
else
  R "sudo systemctl restart uav-agent"
  AGENT_STOPPED=0
  [ "$FRESH" = 1 ] && VERIFY_WAIT_S=120
fi
FAIL=0
LOCAL_SUM=$(xargs -a "$LIST" -d '\n' md5sum | sort -k2)
REMOTE_SUM=$(R "cd $DEST && xargs -d '\n' md5sum" < "$LIST" | sort -k2)
if [ "$LOCAL_SUM" = "$REMOTE_SUM" ]; then say "   ✓ $N 個檔 md5 與本機逐一相同"
else say "   ✗ md5 對不上："; diff <(echo "$LOCAL_SUM") <(echo "$REMOTE_SUM") | sed 's/^/     /'; FAIL=1; fi

# **等到定下來才判**：第一條狀態列常常早於飛控與意圖通道連上（2026-09-22 第一次
# 部署就是這樣，兩格都是「…」）。每秒看最新一條，全部 ✓ 就結束，逾時才照最後一條判
EVAL=$(cat <<'EOF'
import json, sys
line, commit, final = sys.argv[1], sys.argv[2], sys.argv[3] == "1"
s = json.loads(line[line.index("{"):])
checks = [
    (s.get("deployed") == commit, "機上說它跑的是這個 commit", str(s.get("deployed"))),
    (True if s.get("fc_link_ok") else (None if s.get("fc_link_ok") is None else False),
     "飛控心跳", f"fc_link_ok={s.get('fc_link_ok')}（None＝還沒看到飛控）"),
    (s.get("port_exclusive") is not False, "獨佔飛控序列埠", f"port_exclusive={s.get('port_exclusive')}"),
    (True if (s.get("intent") or {}).get("up") else None, "意圖通道連上地面站",
     f"up={(s.get('intent') or {}).get('up')}"),
]
# **5G 取樣是硬條件**（使用者 2026-10-19）：等到最後還沒量到＝✗，不是「…」。
# 飛控那格可以是「…」（桌上裝機時常常還沒接飛控），5G 不行
m = s.get("modem")
sig = bool(m) and m.get("samples", 0) > 0 and (m.get("rsrp") is not None or m.get("sinr") is not None)
mok = True if sig else (False if (m is None or final) else None)
checks.append((mok, "代理量得到 5G",
               "沒有在取樣（MODEM_PORT？）" if m is None else
               f"取樣 {m.get('samples')} 次、失敗 {m.get('errors')}、RSRP {m.get('rsrp')} SINR {m.get('sinr')}"))
if final or all(c[0] is True for c in checks):
    for ok, label, note in checks:
        print(f"   {'✓' if ok is True else ('…' if ok is None else '✗')} {label}｜{note}")
    sys.exit(0 if all(c[0] is True for c in checks) else (1 if any(c[0] is False for c in checks) else 3))
sys.exit(2)          # 還沒定下來
EOF
)
LINE=""; RC=2
for i in $(seq 1 $VERIFY_WAIT_S); do
  LINE=$(R "journalctl -u uav-agent $JSEL --no-pager -o cat | grep ' 狀態 {' | tail -1" || true)
  if [ -n "$LINE" ]; then
    FINAL=0; [ "$i" = "$VERIFY_WAIT_S" ] && FINAL=1
    set +e; python3 -c "$EVAL" "$LINE" "$COMMIT" "$FINAL"; RC=$?; set -e
    [ "$RC" != 2 ] && break
  fi
  sleep 1
done
if [ -z "$LINE" ]; then
  say "   ✗ ${VERIFY_WAIT_S} 秒內沒有出現狀態列"
  R "systemctl is-active uav-agent; journalctl -u uav-agent $JSEL --no-pager -o cat | tail -15" | sed 's/^/     /'
  FAIL=1
elif [ "$RC" = 2 ]; then
  python3 -c "$EVAL" "$LINE" "$COMMIT" 1 || true; FAIL=1
elif [ "$RC" = 1 ]; then
  FAIL=1
elif [ "$RC" = 3 ]; then
  say "   （${VERIFY_WAIT_S} 秒內還有「…」的格子——不算失敗，但要去看一眼）"
fi

if [ "$FAIL" = 0 ]; then
  # 相依套件只在安裝時用，裝進 venv 之後就是暫存
  R "rm -rf $DEST/.wheels"
  say ""; say "✓ 部署完成：$COMMIT$([ "$DIRTY" = true ] && echo '（dirty）') → $U@$HOST:$DEST$([ "$FRESH" = 1 ] && echo '（初次安裝）')"
  if [ "$LOCAL" = 1 ]; then
    # **裝好了就刪掉 pull 下來的資料夾**（使用者 2026-10-19）：程式只該有一份，在
    # /opt/uav-agent。留著的話之後有人改了這份、以為改到的是在跑的那份；而且裡面的
    # drone-*.env 含密碼。只刪「確定是 uav-agent 的 git clone、而且不是安裝目的地」的
    SRC=$(cd "$HERE" && pwd -P)
    if [ "$SRC" != "$(cd "$DEST" && pwd -P)" ] && [ "$SRC" != / ] && [ "$SRC" != "$(cd ~ && pwd -P)" ] \
       && [ -d "$SRC/.git" ] && [ -f "$SRC/agent.py" ] && [ -f "$SRC/deploy.sh" ]; then
      cd /
      rm -rf "$SRC"
      say "   已刪除安裝來源 $SRC（程式只留在 $DEST；之後更新：在別處重新 pull 再跑，或從地面站跑）"
    else
      say "   安裝來源 $SRC 看起來不是一份單純的 clone，沒有刪——請自己確認"
    fi
  fi
else
  say ""; say "✗ 驗收沒過。**沒有自動回滾**（保留現場）。要回到上一版：./deploy.sh --rollback"
  exit 1
fi
