#!/bin/bash
# 把機上的相機轉成 RTSP 推進本機的 MediaMTX（由它的 runOnDemand 啟動）。
# uav-system issues/022。**只在有人拉（或地面站開錄）的時候才會被執行。**
#
# 支援兩種相機，**當場探測、不寫死**：
#
#   1. **CSI 模組**（走 libcamera／rpicam-vid，不是 /dev/videoN）
#   2. **USB 相機**（走 v4l2）
#
# **Pi 5 沒有硬體 H.264 編碼器**（Pi 4 有，Pi 5 拿掉了），所以兩條路的 H.264
# 都是軟體編的。實測（2026-09-23，720p30）：
#
#   * CSI IMX219 → rpicam-vid：**一顆核的 27%**
#   * USB C920   → ffmpeg：**一顆核的 55%**
#
# 差在 C920 送的是 MJPEG，ffmpeg 要**先解碼再編碼**；CSI 是 ISP 直接給 YUV，
# 只編一次。**不是因為誰有硬體編碼器**——兩邊都沒有。
set -u
DEV="${CAM_DEV:-/dev/video0}"
SIZE="${CAM_SIZE:-1280x720}"
FPS="${CAM_FPS:-30}"
BITRATE="${CAM_BITRATE:-2M}"
OUT="${CAM_OUT:-rtsp://127.0.0.1:8554/cam}"
# auto＝CSI 優先（比較省 CPU）、沒有才用 USB。要指定就給 csi／usb
SRC="${CAM_SRC:-auto}"

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

have_csi() { rpicam-hello --list-cameras 2>/dev/null | grep -q "^[0-9] *:"; }

# ── CSI 模組（libcamera）────────────────────────────────────────────────
# **用 rpicam-vid 自己的 libav 輸出直接推 RTSP，不接 ffmpeg。**
# 原本想用 `-o -` 管給 ffmpeg，2026-09-23 實測 **rpicam-apps v1.12 的 `-o -`
# 吐 0 bytes**（ffmpeg 看到空輸入，報 `unspecified size`）。改用 libav 之後
# 這支腳本只有一個行程，可以 exec——MediaMTX 收掉它就真的收乾淨了，不會留下
# rpicam-vid 把相機佔住害下一次拉不到。
start_csi() {
  local w="${SIZE%x*}" h="${SIZE#*x}"
  echo "CSI 相機（libcamera）→ rpicam-vid libav 軟體編碼 ${w}x${h}@${FPS} $BITRATE" >&2
  # --inline 在 libav 路徑上不適用（封裝由 libav 做）；關鍵影格間隔用 --intra，
  # 一秒一個，讓「開了多久才看得到畫面」有上限——拉流的人都是中途加入的。
  exec rpicam-vid -n -t 0 --width "$w" --height "$h" --framerate "$FPS" \
    --intra "$FPS" --codec libav --libav-format rtsp --libav-video-codec libx264 \
    -o "$OUT"
}

# ── USB 相機（v4l2）─────────────────────────────────────────────────────
start_usb() {
  local fmts in_fmt
  fmts=$(v4l2-ctl -d "$DEV" --list-formats 2>/dev/null || true)
  if grep -qi "H264\|H.264" <<<"$fmts"; then
    echo "USB 相機自己輸出 H.264 → 直接轉送，不編碼" >&2
    exec ffmpeg -nostdin -hide_banner -loglevel warning \
      -f v4l2 -input_format h264 -video_size "$SIZE" -framerate "$FPS" -i "$DEV" \
      -c:v copy -an -f rtsp -rtsp_transport tcp "$OUT"
  fi
  in_fmt=mjpeg
  grep -qi "MJPG\|Motion-JPEG" <<<"$fmts" || in_fmt=yuyv422
  echo "USB 相機只有 $in_fmt → 軟體編碼（Pi 5 沒有硬體編碼器，會吃 CPU）" >&2
  exec ffmpeg -nostdin -hide_banner -loglevel warning \
    -f v4l2 -input_format "$in_fmt" -video_size "$SIZE" -framerate "$FPS" -i "$DEV" \
    -c:v libx264 -preset ultrafast -tune zerolatency -b:v "$BITRATE" \
    -pix_fmt yuv420p -g "$FPS" -an -f rtsp -rtsp_transport tcp "$OUT"
}

case "$SRC" in
  csi) have_csi || { echo "CAM_SRC=csi 但 libcamera 看不到相機" >&2; exit 1; }
       start_csi ;;
  usb) [ -e "$DEV" ] || { echo "CAM_SRC=usb 但 $DEV 不在" >&2; exit 1; }
       start_usb ;;
  *)   if have_csi; then start_csi
       elif [ -e "$DEV" ]; then start_usb
       else
         # **分得開「沒接相機」與「接了但沒認到」**：後者要有人去看 config.txt
         echo "找不到任何相機：libcamera 說沒有，$DEV 也不在。" >&2
         echo "CSI 模組要在 /boot/firmware/config.txt 明寫 dtoverlay=<型號>——" >&2
         echo "自動偵測不是每顆都探得到（2026-09-23 的 IMX219 就是）。" >&2
         exit 1
       fi ;;
esac
