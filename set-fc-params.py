#!/usr/bin/env python3
"""寫飛控參數：**一支工具，不是一個端點。**

本系統刻意不提供「寫任何飛控參數」的能力——`mavlink_rx` 的唯讀邊界寫著
「**`PARAM_SET` 永遠不得加入**」，command 服務也沒有這條路。理由是那條路一旦
存在，就再也說不出「地面站不會改機上的設定」。所以要改參數時，用這支工具，
由人在機上跑一次，而不是讓系統長出這個能力。

**預設是乾跑**：連上、讀出現值、印出差異，**不寫**。要真的寫要加 `--apply`，
寫完會**讀回來核對**——`PARAM_SET` 之後飛控不一定主動廣播，沒讀回就只是
「我送出去了」，那不等於「它收下了」。

用法（在機上跑，代理要先停掉——它握著序列埠）：
    sudo systemctl stop uav-agent
    python3 set-fc-params.py                 # 乾跑，只看差異
    python3 set-fc-params.py --apply         # 真的寫並讀回核對
    sudo systemctl start uav-agent

只寫 `PLAN` 表裡列的參數，**不接受命令列指定任意參數**：這支工具的目的是
把「已經裁定過的設定」套上去，不是給人一把萬用扳手。
"""
import argparse
import sys
import time

from pymavlink import mavutil

#: 要寫的參數：名稱 → (目標值, 為什麼)。
#: **每一項都要說得出理由**，不然三個月後沒有人知道這個數字是怎麼來的。
PLAN = {
    # ── 地面失聯：飛控那層是代理的後備（issues/039／033）──────────
    "FS_GCS_ENABLE": (1, "GCS failsafe 開。**它是代理的後備**——代理自己掛掉時"
                         "沒有它就沒有人接手（2026-09-01 裁定，但機上一直是 0）"),
    "FS_GCS_TIMEOUT": (45, "不變式：FS_GCS_TIMEOUT(45) > 單飛上限(30) > 處置起算。"
                           "飛控那層只在代理沒動手時才出手；小於單飛上限的話"
                           "它會搶在代理前面，「跑完任務再 RTL」那一格永遠走不到"),
    # ── 電池：這台機是 4S，而門檻停在 ArduPilot 的 3S 預設 ────────
    "BATT_LOW_VOLT": (14.0, "3.5 V/cell × 4S。**原值 10.5 是 3S 的預設**，"
                            "在 4S 上等於 2.63 V/cell——那個電壓下電池已經受損，"
                            "而飛機大概已經在掉下來了"),
    "BATT_CRT_VOLT": (13.2, "3.3 V/cell × 4S。原值 0＝critical 那層**連門檻都沒設**"),
    "BATT_FS_LOW_ACT": (2, "低電量 → RTL。原值 0＝None：偵測得到但不動作"),
    "BATT_FS_CRT_ACT": (1, "危險電量 → Land。就地降落優於飛回來的路上掉下來"),
    "BATT_CAPACITY": (4200, "實際電池是 4S **4200 mAh**（2026-09-07 使用者確認，"
                            "充電器滿電 16.8 V＝4×4.2 也對得上），而飛控上是 3300。"
                            "**剩餘電量就是拿這個算的**——`(容量−已耗)/容量`，"
                            "少報 900 mAh 會讓飛控在還有約 21% 電時就報 0%。"
                            "方向雖然偏保守，但那是**用錯的數字換來的保守**，"
                            "而失聯接管的兩個判準之一正是這個百分比"),
    # ── 模式開關（2026-09-02 使用者選第 5 格）──────────────────
    "FLTMODE5": (4, "第 5 格（1621–1749µs）改成 GUIDED（模式 4）。"
                    "**原本六格裡沒有 GUIDED**，所以每次要用地面站指揮都得靠"
                    "地面站自己切過去；而 GUIDED 不接受遙控器搖桿解鎖，"
                    "人就會卡在 `Arm: Guided mode not armable`。"
                    "第 5 格原本是重複的 STABILIZE（第 1、3、5 格都是），換掉不損失東西"),
}

#: **刻意不動的參數**，寫下來免得下次有人以為是漏掉的。
NOT_TOUCHED = {
    "BATT_LOW_MAH": "電流感測器的校正沒有驗過（BATT_AMP_PERVLT=59.5 是哪來的？）。"
                    "用沒驗過的數字當門檻，會在錯的時間觸發——而錯的時間可能是"
                    "任務中途。要用 mAh 門檻，先做一次耗電量校正",
    "BATT_CRT_MAH": "同上",
    "BATT_CAPACITY": "3300 mAh 是這顆電池的規格，不是安全設定",
}

ap = argparse.ArgumentParser()
ap.add_argument("--dev", default="/dev/ttyAMA0")
ap.add_argument("--baud", type=int, default=57600)
ap.add_argument("--apply", action="store_true", help="真的寫入（預設只乾跑）")
ap.add_argument("--timeout", type=float, default=20.0)
a = ap.parse_args()

print(f"連線 {a.dev}@{a.baud} …")
m = mavutil.mavlink_connection(a.dev, baud=a.baud)
hb = m.wait_heartbeat(timeout=15)
if hb is None:
    sys.exit("✗ 沒有心跳——代理還開著嗎？（它握著序列埠）")
tgt = (m.target_system, m.target_component)
print(f"✓ 飛控 sysid={tgt[0]} comp={tgt[1]}")

# **armed 就停手。** 改參數本身多半無害，但這支工具沒有任何理由在天上跑
base = getattr(hb, "base_mode", 0)
if base & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
    sys.exit("✗ 這台機是 armed 狀態——不在天上改參數")


def read(names, timeout):
    """把要的參數讀回來。逐一請求而不是整批 PARAM_REQUEST_LIST——
    整批要 2000 多筆、在 57600 上要好幾分鐘，而我們只關心六個。"""
    got, deadline = {}, time.time() + timeout
    want = set(names)
    for n in names:
        m.mav.param_request_read_send(tgt[0], tgt[1], n.encode(), -1)
        time.sleep(0.05)
    while want and time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is None:
            continue
        nm = msg.param_id.rstrip("\x00") if isinstance(msg.param_id, str) \
            else msg.param_id.decode().rstrip("\x00")
        if nm in want:
            got[nm] = float(msg.param_value)
            want.discard(nm)
    return got


cur = read(list(PLAN) + list(NOT_TOUCHED), a.timeout)
missing = [n for n in PLAN if n not in cur]
if missing:
    print(f"✗ 讀不到這幾個參數：{missing}——不繼續（讀不到就不知道改了什麼）")
    sys.exit(1)

print("\n── 差異 ─────────────────────────────────────────────")
todo = {}
for n, (want, why) in PLAN.items():
    now = cur[n]
    # **float32 往返**：13.2 存進飛控再讀回來是 13.199999809…。
    # 絕對容差 1e-6 在這個量級上是「剛好過」，換個大一點的值就會誤判成沒寫進去
    same = abs(now - want) <= max(1e-6, abs(want) * 1e-6)
    print(f"{'  ' if same else '→ '}{n:16} {now:>8.4g} → {want:<8.4g} "
          f"{'（已經是這個值）' if same else ''}")
    print(f"     {why}")
    if not same:
        todo[n] = want
print("\n── 刻意不動 ─────────────────────────────────────────")
for n, why in NOT_TOUCHED.items():
    print(f"  {n:16} = {cur.get(n, '讀不到')}")
    print(f"     {why}")

if not todo:
    print("\n沒有要改的，結束。")
    sys.exit(0)
if not a.apply:
    print(f"\n**乾跑**：有 {len(todo)} 個要改。真的要寫請加 --apply")
    sys.exit(0)

print(f"\n── 寫入 {len(todo)} 個 ───────────────────────────────")
for n, want in todo.items():
    m.mav.param_set_send(tgt[0], tgt[1], n.encode(), float(want),
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.2)

# **讀回來核對。** PARAM_SET 之後飛控不一定主動廣播，而「我送出去了」
# 不等於「它收下了」——這正是代理換 sysid 時學到的同一條（agent.py 的註解）
time.sleep(1.0)
back = read(list(todo), a.timeout)
ok = True
print("\n── 讀回核對 ─────────────────────────────────────────")
for n, want in todo.items():
    got = back.get(n)
    good = got is not None and abs(got - want) <= max(1e-6, abs(want) * 1e-6)
    ok &= good
    print(f"{'✓' if good else '✗'} {n:16} 讀回 {got}（要 {want}）")

print("\n" + ("全部寫入並核對成功。**重開飛控後生效的參數請重開一次**。"
              if ok else "**有參數沒寫進去**——不要當成已經設好了"))
sys.exit(0 if ok else 1)
