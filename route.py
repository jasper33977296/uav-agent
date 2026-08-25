"""續飛航點與提案的計算——**這一份是機上的權威版本**。

與地面站 `apps/command/app/change_route.py` 是**刻意的兩份相同實作**，因為
兩邊都要能算：機上是權威（位置最新），地面站是沒有代理的機的備援。

**兩份會漂移，所以有一支等價測試釘住它們**：uav-system 的
`scripts/test-route-equivalence.py` 拿同一組輸入餵兩邊，斷言結果完全相同。
改這個檔案就要改那個檔案，而測試會在忘記時說話。

**為什麼權威在機上**：地面站看到的位置經 5G 回來已經是過期資料——10 m/s
巡航下一次鏈路抖動就是數十公尺誤差，而「離當前位置最近的航點」正是對位置
最敏感的判斷（狀態機文件 §0.1）。2026-08-25 這條 5G 一次斷 98 秒，
那個差距不是理論值。
"""
import math

# NAV_TAKEOFF / NAV_LAND / NAV_RTL / NAV_VTOL_*：續飛過去會做出完全不同的事
_EXCLUDE = {20, 21, 22, 84, 85}
_NAV = {16, 17, 18, 19, 20, 21, 22, 82, 84, 85}


def _cmd(w: dict) -> int | None:
    if w.get("command") is not None:
        return int(w["command"])
    return {"takeoff": 22, "land": 21, "rtl": 20,
            "waypoint": 16}.get(w.get("action") or "waypoint")


def haversine(lat1, lon1, lat2, lon2) -> float:
    """水平大圓距離（公尺）。"""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp, dl = math.radians(lat2 - lat1), math.radians(lon2 - lon1)
    a = (math.sin(dp / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return 2 * r * math.asin(min(1.0, math.sqrt(a)))


def bearing(lat1, lon1, lat2, lon2) -> float:
    """真方位角（度，0＝正北）。"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def pick_resume_wp(wps: list[dict], lat: float, lon: float) -> dict | None:
    """§6.2 的選法。回傳 {index, seq, lat, lon, alt, distance_m, bearing_deg}。

    `index` 是**我方航點索引**（wps 內的位置）——機端 seq 由驅動的 `wire_seq()`
    換算，呼叫端不必知道兩家的差別。
    """
    best = None
    for i, w in enumerate(wps):
        c = _cmd(w)
        if c is not None and (c in _EXCLUDE or c not in _NAV):
            continue
        if w.get("lat") is None or w.get("lon") is None:
            continue
        d = haversine(lat, lon, w["lat"], w["lon"])
        if best is None or d < best["distance_m"]:
            best = {"index": i, "seq": w.get("seq", i),
                    "lat": w["lat"], "lon": w["lon"], "alt": w.get("alt"),
                    "distance_m": round(d, 1),
                    "bearing_deg": round(bearing(lat, lon, w["lat"], w["lon"]))}
    return best


def build_proposal(*, wps, cur, hold_alt, mission_name, mission_id,
                   cur_seq=None) -> dict:
    """組出確認畫面要顯示的東西（§6.3 逐項對應）。

    `cur` ＝機體現況 {lat, lon, alt_rel, heading}。缺什麼就**說缺什麼**，
    不填預設值：這份提案是要人據以判斷的，猜出來的數字比空白危險。
    """
    warnings, blockers = [], []
    # 0,0 在**這裡**也要擋一次：上游（mav.py）已經濾掉了，但這是會決定飛機
    # 往哪飛的地方，多一道不花錢，而漏掉一次的代價是一條指向一萬公里外的航線
    if not cur.get("lat") and not cur.get("lon"):
        cur = dict(cur, lat=None, lon=None)
    if cur.get("lat") is None or cur.get("lon") is None:
        blockers.append("不知道機體目前位置（GPS 未定位或遙測未到），"
                        "無法算續飛航點")
    resume = None
    if not blockers:
        resume = pick_resume_wp(wps, cur["lat"], cur["lon"])
        if resume is None:
            blockers.append("新航線裡沒有可續飛的導航航點"
                            "（起飛／降落／返航項不算——續飛過去會做出別的事）")

    cur_alt = cur.get("alt_rel")
    hold = {"alt": hold_alt,
            "alt_delta_m": (None if cur_alt is None or hold_alt is None
                            else round(hold_alt - cur_alt, 1))}
    if hold_alt is None:
        warnings.append("沒有指定懸停高度，暫停後維持當前高度")

    if resume:
        # **在航向後方**是真的會發生：兩家 SITL 實測都看過飛機為了回到指定
        # 航點而掉頭往回飛。人有權在按下去之前知道這件事
        hdg = cur.get("heading")
        if hdg is not None:
            diff = abs((resume["bearing_deg"] - hdg + 180) % 360 - 180)
            if diff > 120:
                warnings.append(
                    f"續飛航點在目前航向後方 {diff:.0f}°——機體會先掉頭")
        if resume["alt"] is not None and cur_alt is not None:
            dz = resume["alt"] - cur_alt
            resume["alt_delta_m"] = round(dz, 1)
            if dz < -10:
                warnings.append(f"續飛航點比現在低 {-dz:.0f} m，會下降")
        if resume["distance_m"] > 500:
            warnings.append(
                f"續飛航點離現在 {resume['distance_m']:.0f} m——"
                "系統沒有障礙物感知，這條直線的安全由人判斷")

    steps = [
        f"切 hold 懸停{'' if hold_alt is None else f'（{hold_alt:g} m）'}"
        "，並讀回機端確認真的進了 hold",
        f"上傳航線「{mission_name}」（機體懸停中，上傳不會造成移動）",
        (f"切回 mission 並從第 {resume['index']} 點續飛" if resume
         else "切回 mission"),
    ]
    return {
        "mission_id": mission_id, "mission_name": mission_name,
        "current": {"lat": cur.get("lat"), "lon": cur.get("lon"),
                    "alt_rel": cur_alt, "heading": cur.get("heading"),
                    "mission_seq": cur_seq},
        "hold": hold, "resume_wp": resume,
        "steps": steps, "warnings": warnings, "blockers": blockers,
        "ok": not blockers,
    }


#: 提案與執行當下的差異超過這些就中止重提（§5.1：人確認的是**那一份**提案）
DRIFT_M = 50.0
DRIFT_WP = 0            # 續飛航點換了就一定重提，沒有容忍值


def drift_reason(old: dict, new: dict) -> str | None:
    """執行前重算，與提案有實質差異就回一句話說明。沒差異回 None。

    **為什麼一定要重算**：提案是在人看它的那一刻算的，而飛機在那段時間裡一直
    在動。人確認的是那一份提案，不是「授權代理之後隨便怎麼飛」。
    """
    if not new.get("ok"):
        return "；".join(new.get("blockers") or ["現在算不出提案"])
    o, n = old.get("resume_wp") or {}, new.get("resume_wp") or {}
    if o.get("index") != n.get("index"):
        return (f"續飛航點已經從第 {o.get('index')} 點變成第 {n.get('index')} 點"
                "——機體在你確認的這段時間裡移動了")
    d = abs((n.get("distance_m") or 0) - (o.get("distance_m") or 0))
    if d > DRIFT_M:
        return (f"到續飛航點的距離變了 {d:.0f} m（提案時 "
                f"{o.get('distance_m')} m、現在 {n.get('distance_m')} m）")
    return None
