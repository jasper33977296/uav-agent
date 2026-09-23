# 機上相機 → RTSP（uav-system issues/022）

**地面站來拉，機上不推**（使用者 2026-09-23 裁定）。`runOnDemand` 讓相機與編碼
只在有人拉的時候才跑——沒人看、也沒在錄時，機上不開相機、不編碼、不送位元組。
影像與 5G 量測共用同一條上行，一直傳會讓**量到的不再是原本那條鏈路的品質**。

## 一次性安裝

```bash
# 1. MediaMTX 執行檔（**不進 git**：65 MB 二進位）。在地面站下載後送上來：
#    curl -sL -o mtx.tgz https://github.com/bluenviron/mediamtx/releases/download/v1.21.1/mediamtx_v1.21.1_linux_arm64.tar.gz
#    tar xzf mtx.tgz mediamtx && scp mediamtx pi@<機>:/opt/uav-agent/camera/
chmod +x /opt/uav-agent/camera/mediamtx

# 2. 服務（相機參數改 unit 的 Environment，不要改程式）
sudo install -m 0644 /opt/uav-agent/camera/uav-camera.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now uav-camera
```

程式檔（`mediamtx.yml`／`camera-source.sh`）由 `deploy.sh` 一起送，執行檔與 unit
是一次性的（換版本才要再做一次）。

## 地面站那一半

無人機管理頁 →「相機來源」填 `rtsp://<機IP>:8554/cam`。地面站會：

* 把它設成那台機在 MediaMTX 裡的來源（**path 綁機體身分，不是 sysid**——
  sysid 會被重新指派，綁錯會把這台的架次錄成另一台的畫面）
* 自動把「影像位址」填成地面站的 WHEP 播放位址
* 架次開始時開錄、結束時收錄（保留天數見地面站 `.env`）

## 相機

`camera-source.sh` **當場探測**格式，不寫死：

| 相機輸出 | 做法 | 代價 |
|---|---|---|
| H.264（部分 UVC 相機）| `-c copy` 直接轉送 | CPU 幾乎不耗 |
| 只有 MJPEG／YUYV | 軟體編碼 libx264 ultrafast | **吃 CPU**——Pi 5 沒有硬體 H.264 編碼器（Pi 4 有，Pi 5 拿掉了）|

實測 2026-09-23：**Logitech C920 PRO（046d:08e5）只有 YUYV 與 MJPEG**，走軟體編碼。
（同型號較舊的版本有 H.264 輸出，這一版沒有。）

換成 CSI 相機模組時改 `CAM_DEV`；`SupplementaryGroups` 可能要再加 `render`。

`CAM_TEST=1` ＝送合成測試畫面（畫面上寫著 TEST），相機還沒接時驗整條鏈路用。
