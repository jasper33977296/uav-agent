"""5G 模組採樣：讀 Fibocom FM160 的訊號指標，送進地面站的 link-metrics 通道。

**跑在獨立執行緒**：AT 指令一來一回要等好幾百毫秒，絕不能擋到 agent 的路由
主迴圈——那條迴圈停一下，飛控與地面站就同時卡一下。

誠實原則（沿用 doc/ui-spec §0.2e 與 LinkSample.raw 的設計意圖）：
**沒把握的欄位一律留 None，不猜**。原始回應整包塞進 `raw`，日後對照表確認了
可以回頭重算，但絕不先填一個看起來合理的數字進資料庫。
"""
import json
import logging
import re
import subprocess
import threading
import time

import clock
from collections import deque
import urllib.error
import urllib.request
from datetime import datetime, timezone

import serial

log = logging.getLogger("uav-agent.modem")


# ── NR 索引 → dB（2026-09-09 用模組自報的值域定案）────────────────────
#
# `AT+CESQ=?` 會把每個參數的合法值域報出來，這台回：
#
#     (0-63,99),(0-7,99),(0-96,255),(0-49,255),(0-34,255),(0-97,255),
#     (0-126,255),(0-126,255),(0-127,255)
#
# 前 6 個正好是 TS 27.007 的標準參數（E-UTRA rsrq 0-34、rsrp 0-97 對得上），
# 所以後 3 個是 NR 擴充，而且**重複同一個 rsrq→rsrp 的順序**——原本靠物理
# 可能性反推的欄位順序，到這裡才算有憑據。
#
# **上限把公式釘死。** 值 0 是「低於下限」，值 1 是最低那一格：
#   RSRP 上限 126 → 值1=-156、值126=-31   ⇒ idx - 157
#   RSRQ 上限 126 → 值1=-43 、值126=19.5  ⇒ idx/2 - 43.5
#   SINR 上限 127 → 值1=-23 、值127=40    ⇒ idx/2 - 23.5
#
# 原本三個都少減一格（-156／-43／-23），也就是**每一個值都樂觀了一格**。


def _nr_rsrp(idx):
    """NR SS-RSRP 索引 → dBm。1..126 → -156..-31（步進 1）。"""
    return None if idx in (None, 255) else idx - 157


def _nr_rsrq(idx):
    """NR SS-RSRQ 索引 → dB。1..126 → -43..+19.5（步進 0.5）。"""
    return None if idx in (None, 255) else idx / 2.0 - 43.5


def _nr_sinr(idx):
    """NR SS-SINR 索引 → dB。1..127 → -23..+40（步進 0.5）。"""
    return None if idx in (None, 255) else idx / 2.0 - 23.5


#: 記錄緩衝的上限（筆）。1 Hz 取樣 → 1800 筆＝半小時。
#: **有上限而且滿了要說**：飛機上的記憶體有限，而真的斷超過半小時時，
#: 最舊那段的價值最低（與 backfill.py 同一個取捨）。
BUFFER_MAX = 1800
#: 一次送幾筆。**一包太大時，5G 抖一下整包就白送**（backfill.py 學到的）。
BATCH_MAX = 200
#: 多久送一次記錄通道。即時通道 1 Hz 是給畫面看的；記錄不必那麼即時，
#: 而**批次送比逐筆送省得多**——同樣的位元組數，少了 N 倍的 HTTP 開銷
BATCH_EVERY_S = 10.0


#: 地面站說「我最後收到你的遙測是 N 秒前」，N 要小於這個值才算「我的資料有到」。
#: 遙測實測約 97 則/秒，正常時這個年齡遠小於 1 秒；5 秒與畫面判定「資料變舊」
#: 的門檻一致。
UPLINK_FRESH_S = 5.0


class ModemSampler(threading.Thread):
    daemon = True

    def __init__(self, port, gs_host, gs_port, get_position, interval=1.0,
                 drone_id_fn=None, airborne_fn=None, live_interval=1.0,
                 on_uplink_ok=None, net_iface="usb0", hold_fn=None):
        super().__init__(name="modem-sampler")
        #: **現在不該替任何一台機量**（issues/076）：回傳理由字串就整拍跳過
        #: ——不取樣、不送即時、不送記錄通道。這個 Pi 接到的飛控不是它綁的
        #: 那塊時，量到的訊號屬於眼前這台飛機，而 `drone_id_fn` 給的是綁定的
        #: 那台：送出去就是把一台機的訊號記到另一台名下。None＝沒接＝舊行為
        self.hold_fn = hold_fn
        self.held = 0
        self._held_said = None
        #: **地面站確認「我收到你的遙測了」時呼叫**（issues/047 項次 6）。
        #: 這條每秒一次的通道是機上唯一能問到「我送的東西有沒有到」的地方
        #: ——順路帶回來，不必另開一個請求。
        self.on_uplink_ok = on_uplink_ok
        self.port, self.interval = port, interval
        #: 即時通道多久送一次（2026-09-07 使用者裁定：**蒐集可以密，傳送每秒
        #: 一次就夠**）。畫面看的是「現在訊號好不好」，不需要比人眼更快；
        #: 而記錄通道要的是密度，兩者的需求本來就不同，綁在一起等於讓畫面的
        #: 需求決定研究資料的解析度。
        self.live_interval = live_interval
        self._next_live = 0.0
        #: 最近幾次取樣的時間點，用來算**實際**取樣率（見 `stats`）
        self._ticks = deque(maxlen=32)
        base = f"http://{gs_host}:{gs_port}/api/link-metrics"
        self.url = f"{base}/live"          # 即時通道：只給畫面看，**不入庫**
        self.batch_url = f"{base}/batch"   # 記錄通道：**唯一的入庫路徑**
        self.gs_host = gs_host
        self.get_position = get_position      # 由 agent 提供：當下的 lat/lon/alt
        self.drone_id_fn = drone_id_fn
        #: 離地了沒（2026-09-07 使用者裁定：**只有飛離地面才記訊號**）。
        #: None＝沒接＝一律記（舊行為），呼叫端沒給就不要自作主張少記東西
        self.airborne_fn = airborne_fn
        #: 因為停在地面而沒有記的筆數。**要看得見**——原本「送出去」與
        #: 「記下來」之間的落差是無聲的，那正是訊號資料斷了三週沒人發現的原因
        self.on_ground = 0
        self.ser = None
        self.samples = self.posted = self.errors = 0
        self.last = {}
        self._rtt_ms = None
        self._last_ping = 0.0
        # ── 上下行實際流量（uav-system issues/022 之後，2026-09-23 使用者裁定）──
        #
        # **量的是「現在傳了多少」，不是「能傳多少」。** 讀 5G 介面的收發位元組
        # 計數器做差分，零額外流量——主動打流量會佔滿上行，而那條上行正是要
        # 研究的對象，量它等於弄壞它。
        #
        # 方向以**這台機**為準：`up`＝機→地面（介面的 tx）、`down`＝地面→機（rx）。
        #
        # **總量，不區分**（使用者裁定）：影像、遙測、樣本回傳全部算在一起。
        # 影像現在就走同一條（實測 2.1 Mbps），所以看到某段偏高時要對照那時候
        # 有沒有人在拉畫面。
        self.net_iface = net_iface
        self._net_at = None          # 上一次讀計數器的單調時刻
        self._net_tx = self._net_rx = None
        # ── 記錄通道（uav-system issues/021／doc/onboard-telemetry.md）──────
        #
        # **這條路以前不存在。** 採樣只往 `/live` 送，而那個端點的 docstring
        # 第一行就寫著「**不寫資料庫**」——它只更新畫面。於是這套系統的研究
        # 核心資料（5G 訊號時序）**從來沒有從真機被記錄過**：2026-09-02 實測
        # `link_metrics` 最新一筆停在 08-13，而當天四趟飛行的訊號筆數全是 0。
        # 記錄通道 `/batch` 一直是好的，唯一的呼叫者是 `fake-onboard-node.py`
        # ——**一支假的測試節點**。
        #
        # 兩條通道分工不變（職責不重疊就不需要去重）：`/live` 逐筆即時、
        # 送失敗不重試；`/batch` 緩衝、重送、由地面站回報收下了哪幾筆。
        self.buf: deque = deque(maxlen=BUFFER_MAX)
        self._seq = 0
        self.stored = 0          # 地面站說寫進去了幾筆
        self.dropped = 0         # 緩衝滿了被擠掉的（**要看得見**）
        self._next_batch = 0.0

    # ── AT ────────────────────────────────────────────────────
    def _open(self):
        if self.ser is None or not self.ser.is_open:
            self.ser = serial.Serial(self.port, 115200, timeout=1.5)
        return self.ser

    def _at(self, cmd, wait=0.5):
        """送一個 AT 指令並讀回應。**`wait` 是逾時上限，不是固定等待。**

        原本是 `write` 之後無條件 `time.sleep(wait)` 再讀——不管數據機多快回。
        2026-09-07 實測 FM160：三個指令合計實際回覆 **49 毫秒**，而程式等了
        **1.6 秒**，97% 是白等的。那 0.5／0.7／0.4 是「猜一個夠久的數字」，
        猜保守了 30 倍。

        改成讀到 `OK`／`ERROR` 為止。逾時上限保留原值，所以**最壞情況不變**，
        只是不再為了最壞情況付出每一次的代價。
        """
        s = self._open()
        s.reset_input_buffer()
        s.write((cmd + "\r\n").encode())
        deadline = time.monotonic() + wait
        buf = b""
        while time.monotonic() < deadline:
            n = s.in_waiting
            if n:
                buf += s.read(n)
                if b"OK" in buf or b"ERROR" in buf:
                    break
            else:
                time.sleep(0.005)
        return buf.decode("utf-8", "replace")

    # ── 解析 ──────────────────────────────────────────────────
    def _parse_cesq(self, txt):
        """+CESQ: <rxlev>,<ber>,<rscp>,<ecno>,<rsrq>,<rsrp>[,<nr...>]

        FM160 在標準 6 個之後多回 3 個 NR 值，**順序原廠文件未載**。
        2026-08-24 先用物理可能性反推出 rsrq,rsrp,sinr（另外兩種排法會讓
        RSRQ 落在 +3.5／+8.5 dB，不可能）。

        2026-09-09 拿到憑據：`AT+CESQ=?` 自報的值域是
        `…,(0-34,255),(0-97,255),(0-126,255),(0-126,255),(0-127,255)`
        ——前面那對正是 E-UTRA 的 rsrq,rsrp，後面三個**重複同一個順序**。
        另外 `AT+GTCCINFO?` 的具名欄位 ss_rsrq／ss_rsrp／ss_sinr 也對得上
        （SINR 索引完全相同；RSRP／RSRQ 差固定一格，見 modem_raw.py）。
        """
        m = re.search(r"\+CESQ:\s*([\d,\s]+)", txt)
        if not m:
            return {}
        v = [int(x) for x in m.group(1).split(",") if x.strip().isdigit()]
        out = {}
        if len(v) >= 9:
            out["rsrq"] = _nr_rsrq(v[6])
            out["rsrp"] = _nr_rsrp(v[7])
            out["sinr"] = _nr_sinr(v[8])
            out["nr_mode"] = "SA"
        elif len(v) >= 6:                      # 只有 LTE 欄位
            # TS 27.007 §8.69：<rsrp> 0..97（0=<-140、97=≥-44）、
            # <rsrq> 0..34（0=<-19.5、34=≥-3）。**值 0 是「低於下限」**，
            # 值 1 才是最低那一格——RSRQ 原本寫 -19.5，值1 會算成 -19、
            # 值34 算成 -2.5，整條刻度樂觀了半格（與 NR 那三個同一種錯）
            if v[5] != 255:
                out["rsrp"] = v[5] - 141
            if v[4] != 255:
                out["rsrq"] = v[4] / 2.0 - 20
        return out

    # **GTCCINFO 不在機上解。** 2026-09-09 抓到：這裡原本把 PCI 當十進位讀
    # （`int(f[7], 10)`），而那一欄跟 tac/cellid/narfcn 一樣是**十六進位**。
    # 現場 PCI 是 `85`＝0x85＝133，十進位讀成 85 **不會報錯、只會安靜地錯**
    # ——反而是 09-07 那個含字母的 `8D` 讓它丟例外、整批不填，於是由地面站
    # 補上正確的 141。錯得無聲的那一種，比錯得大聲的危險。
    #
    # 根因不是進位，是**同一個字串有兩份解析**（這裡一份、
    # `apps/backend/app/modem_raw.py` 一份），而且地面站那份只補 null、
    # 不覆蓋機上填的——於是錯的那份贏了。「機上是第一手」這個理由本身
    # 就是陷阱：**兩邊解的是同一個原始字串，沒有誰比較第一手**，
    # 第一手的是 raw 本身。所以機上只負責把 raw 原樣送上去，解碼只留一份。

    def _throughput(self, now):
        """5G 介面這一段時間的**實際**收發速率（kbps）。回 (up, down)。

        **讀不到就回 (None, None)，不是 0。** 介面不存在、計數器讀不到、
        第一次取樣沒有前值——這三種都是「不知道」，而 0 的意思是「這段時間
        什麼都沒傳」。兩者對事後分析的意義相反，不能混。

        計數器變小＝介面被重設或溢位（重開、usb0 重新列舉）。那一段的差分
        沒有意義，**丟掉並重新起算**，不要送出一個假的負數或暴衝值。
        """
        base = f"/sys/class/net/{self.net_iface}/statistics"
        try:
            with open(f"{base}/tx_bytes") as f:
                tx = int(f.read())
            with open(f"{base}/rx_bytes") as f:
                rx = int(f.read())
        except (OSError, ValueError):
            self._net_at = self._net_tx = self._net_rx = None
            return None, None
        prev_at, prev_tx, prev_rx = self._net_at, self._net_tx, self._net_rx
        self._net_at, self._net_tx, self._net_rx = now, tx, rx
        if prev_at is None:
            return None, None
        dt = now - prev_at
        if dt <= 0 or tx < prev_tx or rx < prev_rx:
            return None, None            # 計數器重設／時間沒前進＝這一段不算
        return (round((tx - prev_tx) * 8 / 1000.0 / dt, 1),
                round((rx - prev_rx) * 8 / 1000.0 / dt, 1))

    def _ping_rtt(self):
        """真的量一次 RTT，而不是拿 HTTP 往返充數（那含伺服器處理時間）。"""
        try:
            r = subprocess.run(["ping", "-c", "1", "-W", "2", self.gs_host],
                               capture_output=True, text=True, timeout=4)
            m = re.search(r"time=([\d.]+)\s*ms", r.stdout)
            return float(m.group(1)) if m else None
        except Exception:
            return None

    def sample(self):
        raw = {}
        out = {}
        try:
            cesq = self._at("AT+CESQ")
            raw["CESQ"] = cesq.strip()
            out.update(self._parse_cesq(cesq))
            cc = self._at("AT+GTCCINFO?", 0.7)
            # **原樣送上去，不在這裡解**（見上面那段）
            raw["GTCCINFO"] = cc.strip()
            cops = self._at("AT+COPS?", 0.4)
            raw["COPS"] = cops.strip()
            m = re.search(r'\+COPS:\s*\d+,\d+,"([^"]+)",(\d+)', cops)
            if m:
                raw["plmn"] = m.group(1)
                out["nr_mode"] = "SA" if m.group(2) == "11" else out.get("nr_mode")
        except Exception as e:
            self.errors += 1
            if self.errors % 30 == 1:
                log.warning("模組採樣失敗（%s），第 %d 次", e, self.errors)
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            return None

        now = time.monotonic()
        if now - self._last_ping >= 5.0:       # RTT 不必每秒量
            self._last_ping = now
            self._rtt_ms = self._ping_rtt()
        out["rtt_ms"] = self._rtt_ms
        up, down = self._throughput(now)
        out["throughput_up_kbps"], out["throughput_down_kbps"] = up, down
        out["raw"] = raw
        out["time"] = datetime.now(timezone.utc).isoformat()   # 必須含時區
        # 牆鐘還沒對過時，上面那個時間是錯的。即時通道照送，讓地面站改用收到
        # 的時刻；記錄通道則記單調時間，送出前再換算（見 flush）
        out["clock_synced"] = clock.synced()
        mono_at = time.monotonic()
        pos = self.get_position() or {}
        for k in ("lat", "lon", "alt_rel"):
            if pos.get(k) is not None:
                out[k] = pos[k]
        self.samples += 1
        self.last = out
        # **停在地面上就不記**（2026-09-07 使用者裁定）。即時通道在上面已經
        # 送過了——起飛前要看得到訊號好不好，那是畫面不是紀錄。
        #
        # 判準用代理的 `airborne()`（`landed_state` 優先，退回高度＋地速），
        # **不知道就照記**：飛行中的量測補不回來，多記幾筆地面資料刪得掉。
        if self.airborne_fn is not None and not self.airborne_fn():
            self.on_ground += 1
            return out
        # `seq` 是必要的：地面站回 `accepted_seq` 告訴我們它收下了哪幾筆
        # （**含它判定落在架次外而丟棄的**），沒有序號就只能猜要刪哪幾筆
        self._seq += 1
        if len(self.buf) == self.buf.maxlen:
            self.dropped += 1
        self.buf.append({**out, "seq": self._seq, "_m": mono_at})
        return out

    def post(self, sample):
        body = json.dumps(sample).encode()
        req = urllib.request.Request(
            self.url, data=body, method="POST",
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=3) as r:
                # 204 是舊版後端（沒有回應內容）；200 帶著上行確認。
                # **兩種都算送出成功**——不能因為地面站是舊版就說鏈路壞了
                if r.status in (200, 204):
                    self.posted += 1
                    if r.status == 200 and self.on_uplink_ok:
                        self._report_uplink(r)
                    return True
                log.warning("link-metrics 回 %s", r.status)
        except urllib.error.HTTPError as e:
            # 422/404 是我們送錯，值得吵；網路類的下一秒本來就會有新樣本
            log.warning("link-metrics HTTP %s：%s", e.code,
                        e.read()[:200].decode("utf-8", "replace"))
        except Exception as e:
            self.errors += 1
            if self.errors % 30 == 1:
                log.warning("link-metrics 送出失敗（%s）", e)
        return False

    def _report_uplink(self, r) -> None:
        """把地面站回報的「我最後收到你的遙測是多久以前」轉給代理。

        **只有夠新才算確認。** 這條 HTTP 通得過只證明 IP 路徑在，
        不證明 UDP 遙測有被收下——所以要看的是地面站說的那個年齡，
        不是「我 POST 成功了」。回應壞掉或缺欄位時什麼都不做：
        **沒有確認** 與 **確認說很舊** 都會讓代理那邊自然逾時，
        而在這裡瞎猜一個值會製造一個假的確認。
        """
        try:
            age = (json.loads(r.read() or b"{}").get("uplink")
                   or {}).get("telem_age_s")
        except Exception:                                  # noqa: BLE001
            return
        if age is not None and float(age) <= UPLINK_FRESH_S:
            self.on_uplink_ok()

    def flush(self):
        """把緩衝的樣本送進**記錄通道**。回傳送出去幾筆。

        **這個端點是冪等的、可以安全重送**（它自己的 docstring 這樣寫），
        所以我們的規則很簡單：送出去，然後**只刪掉地面站說它收下的那幾筆**。

        `accepted_seq` 刻意**包含落在架次外而被丟棄的樣本**——不回報那些的話，
        機上會永遠重送一批本來就不該保留的資料（backend 註解明載此意圖）。
        """
        if not self.buf:
            return 0
        if not clock.synced():
            return 0            # 換算基準還是錯的：留在緩衝裡等對時
        chunk = list(self.buf)[:BATCH_MAX]
        body = {"samples": [
            {**{k: v for k, v in x.items() if k not in ("_m", "clock_synced")},
             "time": clock.iso(x["_m"]) if "_m" in x else x["time"]}
            for x in chunk]}
        if self.drone_id_fn:
            did = self.drone_id_fn()
            if did:
                body["drone_id"] = did
        req = urllib.request.Request(
            self.batch_url, data=json.dumps(body).encode(), method="POST",
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=15) as r:
                res = json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:
            # 4xx＝我們送錯了，重送不會變好；5xx／網路＝留著下次再送
            detail = e.read()[:200].decode("utf-8", "replace")
            log.warning("記錄通道 HTTP %s：%s", e.code, detail)
            if 400 <= e.code < 500:
                self.dropped += len(chunk)
                for _ in range(min(len(chunk), len(self.buf))):
                    self.buf.popleft()
            return 0
        except Exception as e:
            if self.errors % 30 == 1:
                log.warning("記錄通道送不出去（%s），緩衝 %d 筆", e, len(self.buf))
            return 0
        ok = set(res.get("accepted_seq") or [])
        # **只刪它說收下的**。沒被收下的留著——那正是緩衝存在的理由
        keep = [x for x in self.buf if x["seq"] not in ok]
        self.buf.clear()
        self.buf.extend(keep)
        self.stored += int(res.get("stored") or 0)
        if res.get("outside_session"):
            # 架次外＝那時候飛機沒在飛，地面站照 issues/004 的規則丟掉。
            # **這不是錯誤**，但要看得見，不然「送了 1000 筆只存了 40 筆」
            # 會像是壞掉
            log.info("記錄通道：入庫 %s、重複 %s、架次外丟棄 %s（緩衝剩 %d）",
                     res.get("stored"), res.get("duplicate"),
                     res.get("outside_session"), len(self.buf))
        return len(ok)

    def run(self):
        log.info("模組採樣啟動：%s → %s（每 %.1fs）；記錄通道 %s（每 %.0fs 一批）",
                 self.port, self.url, self.interval, self.batch_url, BATCH_EVERY_S)
        while True:
            t_start = time.monotonic()
            try:
                why = self.hold_fn() if self.hold_fn else None
                if why != self._held_said:
                    self._held_said = why
                    if why:
                        log.warning("訊號採樣暫停：%s", why)
                    else:
                        log.info("訊號採樣恢復")
                if why:
                    self.held += 1
                    time.sleep(self.interval)
                    continue
                s = self.sample()
                self._ticks.append(t_start)
                # **即時通道節流**：畫面每秒一筆就夠，取樣可以更密。
                # 送失敗不重試——下一筆本來就會取代它（backend 註解明載此意圖）
                if s and t_start >= self._next_live:
                    self._next_live = t_start + self.live_interval
                    self.post(s)
                # 記錄通道：批次、可重送。**與即時通道分開**——即時送失敗
                # 就算了（下一秒有新的），記錄送失敗要留著
                now = time.monotonic()
                if now >= self._next_batch:
                    self._next_batch = now + BATCH_EVERY_S
                    self.flush()
            except Exception:
                log.exception("採樣迴圈例外，吞掉續跑")
            # **睡「還差多少」，不是固定睡滿。** 原本是工作完再睡 interval，
            # 於是週期＝工作時間＋interval：設定 1.0s、實際量到 2.61s
            # （2026-09-07）。工作很快時這個錯誤看不出來，正是它活這麼久的原因。
            time.sleep(max(0.0, self.interval - (time.monotonic() - t_start)))

    def stats(self):
        # **實際取樣率要報出來。** 系統原本沒有任何地方說得出自己多久取一次，
        # 於是「設定 1Hz、實際 0.38Hz」躺了很久沒人發現——唯一看得出來的地方
        # 是 link_metrics 的時間戳差，而那張表當時根本沒有真資料
        hz = None
        if len(self._ticks) >= 2:
            span = self._ticks[-1] - self._ticks[0]
            if span > 0:
                hz = round((len(self._ticks) - 1) / span, 2)
        return {"samples": self.samples, "posted": self.posted,
                "errors": self.errors, "hz": hz,
                # **入庫筆數要與送出筆數分開報。** 「送出去了」不等於
                # 「記下來了」——這整個 bug 就是活在那兩者之間
                "stored": self.stored, "buffered": len(self.buf),
                "dropped": self.dropped, "on_ground": self.on_ground,
                # 因為飛控身分不符而沒有量的拍數（076）。與 on_ground 分開：
                # 一個是「量了但不記」，一個是「根本沒量」
                "held": self.held, "held_why": self._held_said,
                "rsrp": self.last.get("rsrp"), "rsrq": self.last.get("rsrq"),
                "sinr": self.last.get("sinr"), "rtt_ms": self.last.get("rtt_ms"),
                # 實際用量（不是容量）。null＝讀不到，不是 0
                "up_kbps": self.last.get("throughput_up_kbps"),
                "down_kbps": self.last.get("throughput_down_kbps"),
                # pci／cell_id／band 由地面站從 raw 解（唯一一份解碼）
                "pci": self.last.get("pci"), "cell_id": self.last.get("cell_id"),
                "band": self.last.get("band")}
