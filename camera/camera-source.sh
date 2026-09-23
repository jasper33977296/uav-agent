#!/bin/bash
# 把 USB 相機轉成 RTSP 推進本機的 MediaMTX（由它的 runOnDemand 啟動）。
# uav-system issues/022。**只在有人拉的時候才會被執行。**
#
# **Pi 5 沒有硬體 H.264 編碼器**（Pi 4 有，Pi 5 拿掉了）。所以：
#   * 相機自己輸出 H.264（Logitech C920／C922 這類）→ `-c copy`，CPU 幾乎不耗
#   * 只有 MJPEG／YUYV → 軟體編碼（libx264 ultrafast），會跟代理搶 CPU
# 探測是**當場做的**，不是寫死：換相機、換成 CSI 模組都不必改設定。
set -u
DEV="${CAM_DEV:-/dev/video0}"
SIZE="${CAM_SIZE:-1280x720}"
FPS="${CAM_FPS:-30}"
BITRATE="${CAM_BITRATE:-2M}"
OUT="${CAM_OUT:-rtsp://127.0.0.1:8554/cam}"

# **測試來源**（鷹架，同 uav-system 的 uav-video-testsrc）：相機還沒接上時，
# 用一段合成畫面驗整條鏈路（機上 RTSP → 地面站拉流 → 錄影 → 瀏覽器）。
# 畫面上直接寫著 TEST，**不會被誤認成真的相機畫面**。
if [ "${CAM_TEST:-0}" = "1" ]; then
  echo "CAM_TEST=1：送合成測試畫面，不是相機" >&2
  exec ffmpeg -nostdin -hide_banner -loglevel warning -re \
    -f lavfi -i "testsrc=size=${SIZE}:rate=${FPS},drawtext=text='TEST %{localtime}':fontsize=42:fontcolor=white:x=20:y=20" \
    -c:v libx264 -preset ultrafast -tune zerolatency -b:v "$BITRATE" \
    -pix_fmt yuv420p -g "$FPS" -an -f rtsp -rtsp_transport tcp "$OUT"
fi

if [ ! -e "$DEV" ]; then
  echo "相機不在：$DEV（USB 沒接？換成 CSI 模組要改 CAM_DEV）" >&2
  exit 1
fi

fmts=$(v4l2-ctl -d "$DEV" --list-formats 2>/dev/null || true)
if grep -qi "H264\|H.264" <<<"$fmts"; then
  echo "相機自己輸出 H.264 → 直接轉送，不編碼" >&2
  exec ffmpeg -nostdin -hide_banner -loglevel warning \
    -f v4l2 -input_format h264 -video_size "$SIZE" -framerate "$FPS" -i "$DEV" \
    -c:v copy -an -f rtsp -rtsp_transport tcp "$OUT"
fi

in_fmt=mjpeg
grep -qi "MJPG\|Motion-JPEG" <<<"$fmts" || in_fmt=yuyv422
echo "相機只有 $in_fmt → 軟體編碼（Pi 5 沒有硬體編碼器，會吃 CPU）" >&2
exec ffmpeg -nostdin -hide_banner -loglevel warning \
  -f v4l2 -input_format "$in_fmt" -video_size "$SIZE" -framerate "$FPS" -i "$DEV" \
  -c:v libx264 -preset ultrafast -tune zerolatency -b:v "$BITRATE" \
  -pix_fmt yuv420p -g "$FPS" -an -f rtsp -rtsp_transport tcp "$OUT"
