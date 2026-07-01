#!/bin/bash
STREAMING_DIR="$(dirname "$(realpath "$0")")"
PIDFILE="$STREAMING_DIR/stream.pid"
TAILSCALE_IP="100.110.244.54"

if [ -f "$PIDFILE" ] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
    echo "Stream already running (PID $(cat "$PIDFILE"))"
    echo "Watch: rtsp://$TAILSCALE_IP:8554/cam"
    echo "Watch: http://$TAILSCALE_IP:8888/cam (browser)"
    exit 0
fi

echo "Starting mediamtx..."
"$STREAMING_DIR/mediamtx" "$STREAMING_DIR/mediamtx.yml" >> "$STREAMING_DIR/mediamtx.log" 2>&1 &
MTXPID=$!

echo "Waiting for mediamtx..."
for i in $(seq 1 10); do
    nc -z "$TAILSCALE_IP" 8554 2>/dev/null && break
    sleep 0.5
done

echo "Starting webcam stream..."
ffmpeg -f v4l2 -framerate 30 -video_size 1280x720 -input_format mjpeg -i /dev/video2 \
    -c:v libx264 -preset ultrafast -tune zerolatency -r 30 -b:v 2M \
    -f rtsp "rtsp://$TAILSCALE_IP:8554/cam" \
    >> "$STREAMING_DIR/ffmpeg.log" 2>&1 &
FFPID=$!

echo "$MTXPID $FFPID" > "$PIDFILE"

echo "Stream live!"
echo "  RTSP (VLC, low latency): rtsp://$TAILSCALE_IP:8554/cam"
echo "  HLS  (browser):          http://$TAILSCALE_IP:8888/cam"
