#!/usr/bin/env python3
"""意圖通道的重連退避測試——對**真的** /ws/agent 連，不是模擬。

**為什麼盯這一格**：退避的重置點寫錯不會讓任何東西壞掉，只會讓重連變慢，
而「慢」在日誌裡長得跟「網路本來就不好」一模一樣。2026-08-25 就是這樣：
`_session()` 結束的唯一方式是拋例外，所以寫在 `try` 之後的 `backoff = 1.0`
永遠不會執行——一條跑了兩分半的連線被 reset 之後，退避已經爬到 30 秒，
於是每斷一次就白白失聯半分鐘。本場域的 5G 抖動是常態，那正是最不能等的時候。

用法：python3 tools/intent-reconnect.py [ws://主機:38000/ws/agent]
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from intent import IntentLink  # noqa: E402

URL = sys.argv[1] if len(sys.argv) > 1 else "ws://localhost:38000/ws/agent"
UID = "2a0020001151333139383538"

link = IntentLink(URL, {"agent_version": "test", "board_uid": UID,
                        "inputs": ["telemetry"], "protocol": [1]})
# 假裝剛熬過一段長斷線：退避已經爬到上限
link._backoff = 30.0
link.start()

for _ in range(100):
    if link.connected:
        break
    time.sleep(0.1)

ok = True


def chk(label, cond, note=""):
    global ok
    ok &= bool(cond)
    print(f"{'✓' if cond else '✗'} {label}{('｜' + str(note)) if note else ''}")


chk("連得上", link.connected, URL)
chk("連上後退避重置成 1 秒（斷線不必再等 30 秒）",
    link._backoff == 1.0, f"_backoff={link._backoff}")

link.publish({"state": "READY", "sysid": 1, "derived": {"armed": False}})
time.sleep(2.5)
chk("1Hz 保活有在送", link.sent >= 2, f"sent={link.sent}")
chk("沒有被地面站拒絕", link.rejected == 0, f"rejected={link.rejected}")
chk("還沒掉過線", link.drops == 0, f"drops={link.drops}")

link.stop()
time.sleep(0.5)
print("\n" + ("全部通過" if ok else "**有未通過項目**"))
sys.exit(0 if ok else 1)
