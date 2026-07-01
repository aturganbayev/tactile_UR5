#!/bin/bash
STREAMING_DIR="$(dirname "$(realpath "$0")")"
PIDFILE="$STREAMING_DIR/stream.pid"

if [ ! -f "$PIDFILE" ]; then
    echo "No stream.pid found — killing any stray processes..."
    pkill -f 'mediamtx' 2>/dev/null
    pkill -f 'ffmpeg.*video0' 2>/dev/null
    echo "Done."
    exit 0
fi

read -r MTXPID FFPID < "$PIDFILE"

echo "Stopping stream (mediamtx PID $MTXPID, ffmpeg PID $FFPID)..."
kill "$FFPID" "$MTXPID" 2>/dev/null
sleep 1

kill -0 "$FFPID" 2>/dev/null && kill -9 "$FFPID" 2>/dev/null
kill -0 "$MTXPID" 2>/dev/null && kill -9 "$MTXPID" 2>/dev/null

rm -f "$PIDFILE"
echo "Stream stopped."
