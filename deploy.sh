#!/bin/bash
# 把這個 repo 的代理部署到機上的 Pi（uav-system issues/069）。**在開發機上跑。**
#
#   ./deploy.sh              # 部署目前的 commit
#   ./deploy.sh --dry-run    # 只看：閘門判定、哪些檔會變、unit 有沒有差——不動機上
#   ./deploy.sh --rollback   # 還原上一次部署前的備份並重啟
#
# 其他選項：
#   --host <addr>   指定 Pi（預設依序試 UAV_PI、10.141.2.32、10.101.129.121）
#   --dirty         工作目錄有未提交的改動也照部署（DEPLOYED 會標 dirty）
#   --pip           requirements.txt 有變時順便 pip install（機上要有網路）
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
#   6. requirements.txt 有變：停下來，除非 --pip
#   7. 寫 DEPLOYED、重啟、逐檔比 md5、等狀態列出現並驗收
#
# **驗收失敗不自動回滾**：自動回滾會把「新版哪裡不對」的現場蓋掉。失敗時印出
# 回滾指令，由人決定。
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
DEST=/opt/uav-agent
UNIT=/etc/systemd/system/uav-agent.service
KEEP=5
STATE_FRESH_S=30
VERIFY_WAIT_S=45

DRY=0; ROLLBACK=0; DIRTY_OK=0; PIP=0; HOST="${UAV_PI:-}"
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --rollback) ROLLBACK=1 ;;
    --dirty) DIRTY_OK=1 ;;
    --pip) PIP=1 ;;
    --host) HOST="$2"; shift ;;
    -h|--help) sed -n '2,12p' "$0"; exit 0 ;;
    *) echo "不認得的選項：$1" >&2; exit 2 ;;
  esac
  shift
done

say()  { printf '%s\n' "$*"; }
step() { printf '\n== %s ==\n' "$*"; }
die()  { printf '✗ %s\n' "$*" >&2; exit 1; }

# ── 1. 前置 ──────────────────────────────────────────────────
step "1. 前置"
cd "$HERE"
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

SSH_OPTS=(-o ConnectTimeout=8 -o BatchMode=yes)
if [ -z "$HOST" ]; then
  for h in 10.141.2.32 10.101.129.121; do
    if ssh "${SSH_OPTS[@]}" "pi@$h" true 2>/dev/null; then HOST=$h; break; fi
  done
  [ -n "$HOST" ] || die "兩個位址都連不到 Pi（5G 側 10.141.2.32、eth0 10.101.129.121）"
else
  ssh "${SSH_OPTS[@]}" "pi@$HOST" true || die "連不到 pi@$HOST"
fi
say "   Pi：$HOST"
R() { ssh "${SSH_OPTS[@]}" "pi@$HOST" "$@"; }

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
LIST=$(mktemp); trap 'rm -f "$LIST"' EXIT
git ls-files > "$LIST"
N=$(wc -l < "$LIST")
say "   git 追蹤的 $N 個檔（不刪機上任何東西；不碰 state.json／.fc_sysid／venv）"
CHANGES=$(rsync -ani --checksum --files-from="$LIST" ./ "pi@$HOST:$DEST/" | grep -E '^[<>c]' || true)
if [ -z "$CHANGES" ]; then say "   沒有檔案不同"; else printf '%s\n' "$CHANGES" | sed 's/^/   /'; fi

# ── 4. unit ──────────────────────────────────────────────────
step "4. systemd unit"
UNIT_DIFF=$(R "cat $UNIT" | diff - systemd/uav-agent.service || true)
if [ -z "$UNIT_DIFF" ]; then say "   與 repo 相同"; else
  say "   **與 repo 不同**（部署時會裝上 repo 那份，舊的留備份）："
  printf '%s\n' "$UNIT_DIFF" | sed 's/^/     /'
fi

# ── 5. requirements ──────────────────────────────────────────
step "5. requirements.txt"
REQ_LOCAL=$(sha256sum requirements.txt | cut -c1-16)
REQ_REMOTE=$(R "sha256sum $DEST/requirements.txt 2>/dev/null | cut -c1-16" || true)
if [ "$REQ_LOCAL" = "$REQ_REMOTE" ]; then say "   沒變"; else
  say "   **變了**（機上 ${REQ_REMOTE:-無} → $REQ_LOCAL）"
  if [ "$PIP" = 0 ] && [ "$DRY" = 0 ]; then
    die "相依有變。機上現場不一定有網路——確認後用 --pip 重跑（會在 venv 裡 pip install -r）"
  fi
fi

if [ "$DRY" = 1 ]; then
  step "--dry-run：到此為止，機上沒有任何改動"
  exit 0
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
rsync -a --checksum --files-from="$LIST" ./ "pi@$HOST:$DEST/"
say "   已傳送"
if [ -n "$UNIT_DIFF" ]; then
  R "set -e; sudo cp $UNIT $UNIT.bak-$(date +%Y%m%d-%H%M%S); \
     sudo install -m 0644 $DEST/systemd/uav-agent.service $UNIT; sudo systemctl daemon-reload"
  say "   unit 已更新並 daemon-reload（舊的留在 $UNIT.bak-*）"
fi
if [ "$REQ_LOCAL" != "$REQ_REMOTE" ] && [ "$PIP" = 1 ]; then
  R "$DEST/venv/bin/pip install -q -r $DEST/requirements.txt" || die "pip install 失敗——程式已傳上去但還沒重啟；要回到舊版用 ./deploy.sh --rollback"
fi
BY="$(git config user.name 2>/dev/null || whoami)@$(hostname)"
R "cat > $DEST/DEPLOYED" <<EOF
{"commit": "$COMMIT", "dirty": $DIRTY, "at": "$(date -Is)", "by": "$BY", "files": $N}
EOF

# ── 7. 重啟與驗收 ────────────────────────────────────────────
step "7. 重啟與驗收"
T0=$(R "date '+%Y-%m-%d %H:%M:%S'")
R "sudo systemctl restart uav-agent"
FAIL=0
LOCAL_SUM=$(xargs -a "$LIST" -d '\n' md5sum | sort -k2)
REMOTE_SUM=$(R "cd $DEST && xargs -d '\n' md5sum" < "$LIST" | sort -k2)
if [ "$LOCAL_SUM" = "$REMOTE_SUM" ]; then say "   ✓ $N 個檔 md5 與本機逐一相同"
else say "   ✗ md5 對不上："; diff <(echo "$LOCAL_SUM") <(echo "$REMOTE_SUM") | sed 's/^/     /'; FAIL=1; fi

LINE=""
for _ in $(seq 1 $VERIFY_WAIT_S); do
  LINE=$(R "journalctl -u uav-agent --since '$T0' --no-pager -o cat | grep ' 狀態 {' | tail -1" || true)
  [ -n "$LINE" ] && break
  sleep 1
done
if [ -z "$LINE" ]; then
  say "   ✗ ${VERIFY_WAIT_S} 秒內沒有出現狀態列"
  R "systemctl is-active uav-agent; journalctl -u uav-agent --since '$T0' --no-pager -o cat | tail -15" | sed 's/^/     /'
  FAIL=1
else
  python3 - "$LINE" "$COMMIT" <<'EOF' || FAIL=1
import json, sys
line, commit = sys.argv[1], sys.argv[2]
s = json.loads(line[line.index("{"):])
bad = 0
def row(ok, label, note=""):
    global bad
    mark = "✓" if ok is True else ("…" if ok is None else "✗")
    if ok is False:
        bad = 1
    print(f"   {mark} {label}{('｜' + note) if note else ''}")
row(s.get("deployed") == commit, "機上說它跑的是這個 commit", str(s.get("deployed")))
row(True if s.get("fc_link_ok") else (None if s.get("fc_link_ok") is None else False),
    "飛控心跳", f"fc_link_ok={s.get('fc_link_ok')}（None＝還沒看到飛控，可能是它比 Pi 晚上電）")
row(s.get("port_exclusive") is not False, "獨佔飛控序列埠", f"port_exclusive={s.get('port_exclusive')}")
row(True if (s.get("intent") or {}).get("up") else None, "意圖通道連上地面站",
    "還沒連上時是「…」：第一條狀態列可能早於連線")
sys.exit(bad)
EOF
fi

if [ "$FAIL" = 0 ]; then
  say ""; say "✓ 部署完成：$COMMIT$([ "$DIRTY" = true ] && echo '（dirty）') → pi@$HOST:$DEST"
else
  say ""; say "✗ 驗收沒過。**沒有自動回滾**（保留現場）。要回到上一版：./deploy.sh --rollback"
  exit 1
fi
