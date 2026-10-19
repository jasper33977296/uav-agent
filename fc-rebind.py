#!/usr/bin/env python3
"""飛控更換流程：**一支工具，不是一個端點**（uav-system issues/076）。

一個代理只綁一塊飛控板（`binding.py`）。綁定之後，序列埠上出現別塊板子時代理
會鎖住——不轉發、不收指令、不替它說話。真的要換飛控時，由人在機上跑這支：

    python3 fc-rebind.py                         # 看現況（不改任何東西）
    python3 fc-rebind.py --release --reason "原飛控 IMU 故障，換上備品"
    python3 fc-rebind.py --release --reason "…" --fc-sysid 3   # 新飛控的號碼不是 1

## 它做什麼

1. **確認現在可以動**：代理在跑的話，`state.json` 要新鮮而且沒有解鎖
   （與 `deploy.sh` 同一道閘——它會重啟代理，而重啟等於把橋拆掉）。
2. **解除綁定**，把原因與時間記進綁定檔的 `history`。
3. 處理號碼：舊的 `.fc_sysid` 是**配給舊板子的號碼**，對新板子沒有意義，刪掉；
   新飛控的號碼不是服務設定的 `FC_SYSID` 時用 `--fc-sysid` 指定。
4. 重啟代理，等它確認到新板號、綁上去，印出結果。

## 為什麼不讓人打新板號

**解除與綁定刻意分成兩步。** 這支只解除；新板號由代理自己從飛控問出來——
那是機器問得到的事，讓人打一串十六進位只會打錯。人負責的是「我知道我換了
飛控」這個判斷，以及說得出為什麼。

## 為什麼不是地面站的按鈕

換飛控是在飛機旁邊、手上拿著螺絲起子的時候發生的事，而這個判斷的事實
（「我剛剛換了」）只在機旁的人身上。做成遠端按鈕的話，按的人看不到
序列埠上接的是什麼——那正是這個機制要擋的情況。
"""
import argparse
import json
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import binding  # noqa: E402

STATE = os.path.join(HERE, "state.json")
FC_SYSID_FILE = os.path.join(HERE, ".fc_sysid")
STATE_FRESH_S = 30
WAIT_S = 60


def agent_active() -> bool:
    return subprocess.run(["systemctl", "is-active", "--quiet", "uav-agent"]
                          ).returncode == 0


def read_state():
    try:
        age = time.time() - os.path.getmtime(STATE)
        with open(STATE) as f:
            return age, json.load(f)
    except (OSError, ValueError):
        return None, {}


def show(b: binding.Binding, st: dict, age):
    print(f"綁定檔：{b.path}")
    if not b.exists:
        print("  （不存在）這個代理還沒綁過任何飛控——下次確認到的板號就會成為綁定")
    else:
        print(f"  綁定板號：{b.board_uid or '（已解除，等新板子）'}")
        print(f"  綁定時間：{b.blob.get('bound_at')}（{b.blob.get('how')}）")
        if b.released:
            r = b.released
            print(f"  解除：{r.get('at')}，原因：{r.get('reason')}（原 {r.get('from')}）")
        for h in b.blob.get("history") or []:
            print(f"  歷史：{h.get('board_uid')}  {h.get('bound_at')} → "
                  f"{h.get('released_at')}  {h.get('reason')}")
    fb = st.get("fc_binding") or {}
    print(f"代理現況（state.json，{'讀不到' if age is None else f'{age:.0f} 秒前'}）：")
    print(f"  確認狀態：{fb.get('state', '不知道')}——{fb.get('why', '')}")
    print(f"  序列埠上讀到的板號：{fb.get('observed') or '（沒有）'}")
    print(f"  armed：{st.get('armed')}　飛控心跳：{st.get('fc_link_ok')}")
    if st.get("foreign_sysids"):
        print(f"  序列埠上其他號碼：{st['foreign_sysids']}"
              "（綁定的飛控失聯而這裡有號碼的話，新飛控多半是它——用 --fc-sysid）")
    try:
        print(f"  .fc_sysid：{open(FC_SYSID_FILE).read().strip()}")
    except OSError:
        print("  .fc_sysid：（沒有，用服務設定的 FC_SYSID）")


def main():
    p = argparse.ArgumentParser(description="飛控更換流程（解除綁定）")
    p.add_argument("--release", action="store_true", help="解除目前的綁定")
    p.add_argument("--reason", help="為什麼換（必填，會留在綁定檔的歷史裡）")
    p.add_argument("--fc-sysid", type=int,
                   help="新飛控的 MAVLink 號碼（不是服務設定的 FC_SYSID 時才需要）")
    p.add_argument("--yes", action="store_true", help="不要再問一次")
    p.add_argument("--no-restart", action="store_true",
                   help="只改檔案，不重啟代理（之後自己重啟才會生效）")
    a = p.parse_args()

    b = binding.Binding(binding.BINDING_FILE)
    age, st = read_state()
    show(b, st, age)
    if not a.release:
        return 0

    if not (a.reason or "").strip():
        sys.exit("\n✗ 要寫 --reason：三個月後有人看到這筆更換，要說得出為什麼")
    #: 已經解除、只是新飛控號碼不對而綁不上（第二次跑、補 --fc-sysid）
    redo = b.board_uid is None and b.released is not None and a.fc_sysid is not None
    if b.board_uid is None and not redo:
        sys.exit("\n✗ 目前沒有綁定，不需要解除——直接重啟代理就會綁上眼前的飛控"
                 "（號碼不對的話加 --fc-sysid）")

    active = agent_active()
    if active and not a.no_restart:
        # 與 deploy.sh 同一道閘：**不知道就當作不行**
        if age is None:
            sys.exit("\n✗ 代理在跑，但讀不到 state.json——不知道有沒有解鎖，中止")
        if age > STATE_FRESH_S:
            sys.exit(f"\n✗ 代理在跑，但 state.json 已經 {age:.0f} 秒沒更新——"
                     "不知道現在有沒有解鎖，中止")
        if st.get("armed") is not False:
            sys.exit(f"\n✗ 飛控是解鎖狀態（armed={st.get('armed')}）——"
                     "重啟代理等於把橋拆掉，中止")

    print(f"\n要解除綁定：{b.board_uid}\n原因：{a.reason}" if not redo else
          f"\n綁定已在 {b.released.get('at')} 解除，這次只改新飛控的號碼")
    if a.fc_sysid is not None:
        print(f"新飛控的號碼：{a.fc_sysid}（寫進 .fc_sysid）")
    elif os.path.exists(FC_SYSID_FILE):
        print(".fc_sysid 是配給舊板子的號碼，會刪掉（改用服務設定的 FC_SYSID）")
    if not a.yes and input("確定？輸入 yes：").strip() != "yes":
        sys.exit("沒有改任何東西")

    old = b.board_uid or b.released.get("from")
    if not redo:
        b.release(a.reason.strip())
    if a.fc_sysid is not None:
        with open(FC_SYSID_FILE, "w") as f:
            f.write(str(a.fc_sysid))
    else:
        try:
            os.remove(FC_SYSID_FILE)
        except FileNotFoundError:
            pass
    print(f"✓ 已解除 {old}" if not redo else f"✓ .fc_sysid → {a.fc_sysid}")

    if a.no_restart:
        print("沒有重啟代理——重啟之後，它確認到的第一塊板子會成為綁定")
        return 0
    if not active:
        print("代理本來就停著——啟動之後，它確認到的第一塊板子會成為綁定")
        return 0
    t0 = time.time()
    subprocess.run(["sudo", "systemctl", "restart", "uav-agent"], check=True)
    print("已重啟代理，等它確認新的飛控板……")
    while time.time() - t0 < WAIT_S:
        time.sleep(2)
        age, st = read_state()
        fb = st.get("fc_binding") or {}
        if age is not None and age < time.time() - t0 and fb.get("bound"):
            if fb.get("bound") == old:
                print(f"⚠ 綁回了同一塊 {old}——序列埠上接的還是舊飛控")
            else:
                print(f"✓ 新綁定：{fb['bound']}（{fb.get('state')}）")
            return 0
    print(f"⚠ {WAIT_S} 秒內沒有看到新的綁定。看 journalctl -u uav-agent："
          "飛控有沒有心跳、號碼對不對（必要時 --fc-sysid 再跑一次）")
    return 1


if __name__ == "__main__":
    sys.exit(main())
