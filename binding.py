"""一個代理只綁一塊飛控板（uav-system issues/076）。

## 為什麼要綁

代理在意圖通道上自報的身分，是飛控板的 `board_uid`（`AUTOPILOT_VERSION.uid2`，
issues/038）。但它只在**行程開始時問一次**——於是 Pi 不重開、直接把序列埠
接到另一塊飛控上，代理會繼續拿舊板號替新板子說話：新機的遙測記進舊機名下，
指揮舊機的指令送到新機上，**而且是靜默的**。

這裡把「這個代理是哪塊板子的」從「這個行程第一次問到什麼」變成一個**持久的
決定**：

* **第一次確認到板號就綁定**，寫進 `.fc_binding.json`。
* 之後每一段飛控連線（開機、心跳中斷後恢復、序列埠重開）都要重新確認；
  對不上就鎖住，不轉發、不收指令、不替它說話。
* **換飛控要走更換流程**（`fc-rebind.py`）：由人在機上明確解除綁定、寫明原因，
  代理下次確認到的板號才成為新的綁定。舊的那一筆留在 `history` 裡。

## 「解除」與「綁定」刻意分成兩步

更換流程只做**解除**，不讓人打新板號。新板號由代理自己從飛控問出來——
那是機器問得到的事，讓人打字只會打錯（與 `say_hello()` 同一個理由）。
人負責的是「我知道我換了飛控」這個判斷，不是那串十六進位。

## 檔案格式

    {"board_uid": "2a00…" | null,      # null ＝已解除、等下一塊板子
     "bound_at": "2026-10-19T…Z", "how": "first" | "rebind",
     "released": {"at": …, "reason": …, "from": "2a00…"} | null,
     "history": [{"board_uid": …, "bound_at": …, "released_at": …,
                  "reason": …}, …]}

**檔案不存在**＝這個代理從來沒有綁過（第一次部署、或升級到有這個機制的版本）。
與「解除後等新板子」是兩件事，兩者的事件措辭不同。
"""
import json
import logging
import os
import time

log = logging.getLogger("uav-agent.binding")

#: 跟著 agent.py 自己的位置走（與 `.fc_sysid` 同一處），不寫死絕對路徑
BINDING_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                            ".fc_binding.json")


def _iso(t: float | None = None) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(t))


class Binding:
    def __init__(self, path: str = BINDING_FILE):
        self.path = path
        self.exists = False
        self.board_uid: str | None = None
        self.blob: dict = {}
        self._load()

    # ── 查詢 ────────────────────────────────────────────────────
    @property
    def released(self) -> dict | None:
        """人走過更換流程、還沒有新板子被確認。"""
        return self.blob.get("released") if self.board_uid is None else None

    def ever_bound(self, uid: str | None) -> bool:
        """這塊板子**曾經**是這個代理的嗎（現在的或歷史上的）。

        機上錄製的回傳用它判斷「這份檔案歸誰」：舊板子在更換之前錄的檔案，
        換完之後仍然要以舊板號回傳——它錄的就是那台飛機。
        """
        if not uid:
            return False
        return uid == self.board_uid or any(
            h.get("board_uid") == uid for h in self.blob.get("history") or [])

    def first_uid(self) -> str | None:
        """有史以來第一塊綁定的板子。見 `uploader` 對沒有標記的舊錄檔的處理。"""
        hist = self.blob.get("history") or []
        return hist[0].get("board_uid") if hist else self.board_uid

    def wire(self) -> dict:
        return {"bound": self.board_uid, "bound_at": self.blob.get("bound_at"),
                "how": self.blob.get("how"), "released": self.released,
                "changes": len(self.blob.get("history") or [])}

    # ── 寫入 ────────────────────────────────────────────────────
    def bind(self, uid: str) -> str:
        """綁定。**只在沒有綁定時才可以呼叫**——已綁定時換板子要走更換流程。

        回傳 `first`（從來沒綁過）或 `rebind`（更換流程之後）。
        """
        assert self.board_uid is None, "已綁定時不可以直接改綁"
        how = "rebind" if self.released else "first"
        self.blob = {**self.blob, "board_uid": uid, "bound_at": _iso(),
                     "how": how, "released": None}
        self.blob.setdefault("history", [])
        # **先在記憶體裡綁上，再落盤**：寫不進去（卡壞了、唯讀）時，這個行程
        # 照樣守著這塊板子，由呼叫端大聲說「重開之後會忘記」。反過來的話，
        # 寫失敗會讓這一拍沒有綁定，下一拍又來一次——每一拍都是「第一次」
        self.board_uid = uid
        self._save()
        return how

    def release(self, reason: str) -> dict:
        """更換流程的那一步：解除目前的綁定，留下原因。由 `fc-rebind.py` 呼叫。"""
        old = self.board_uid
        if old is None:
            raise ValueError("目前沒有綁定，不需要解除")
        hist = list(self.blob.get("history") or [])
        hist.append({"board_uid": old, "bound_at": self.blob.get("bound_at"),
                     "released_at": _iso(), "reason": reason})
        rel = {"at": _iso(), "reason": reason, "from": old}
        self.blob = {"board_uid": None, "bound_at": None, "how": None,
                     "released": rel, "history": hist}
        self._save()
        self.board_uid = None
        return rel

    # ── 內部 ────────────────────────────────────────────────────
    def _load(self):
        try:
            with open(self.path) as f:
                blob = json.load(f)
        except FileNotFoundError:
            return
        except (OSError, ValueError) as e:
            # **讀不懂不等於沒有綁定。** 把壞掉的檔案當成「從來沒綁過」，
            # 代理就會把眼前那塊板子當成第一次綁定——而那正是這個機制要擋的
            # 「換了飛控沒有人知道」。所以讀不懂就當作綁定在一塊不存在的板子上：
            # 什麼都對不上、全部鎖住，直到人去看
            log.error("⚠ 綁定檔 %s 讀不懂（%s）——**一律當作對不上**，"
                      "請人工檢查後重寫或走更換流程", self.path, e)
            self.exists = True
            self.board_uid = "<unreadable>"
            return
        self.exists = True
        self.blob = blob if isinstance(blob, dict) else {}
        self.board_uid = self.blob.get("board_uid") or None

    def _save(self):
        tmp = self.path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self.blob, f, ensure_ascii=False, indent=2)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, self.path)      # 原子換檔：掉電時不會留下半截
        self.exists = True
