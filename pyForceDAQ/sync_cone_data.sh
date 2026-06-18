#!/usr/bin/env bash
#
# Sync cone_data recordings from the remote DAQ PC (sshfs mount) to this local
# repo. Runs on THIS machine (gifty4), where both paths are visible.
#
# Usage:
#   ./sync_cone_data.sh                 one-shot copy of new/updated files
#   ./sync_cone_data.sh --watch [SECS]  keep copying every SECS seconds (default 10)
#   ./sync_cone_data.sh --move          copy, then delete the source files
#                                       (only safe when no recording is running)
#
# Copy is the default: it never deletes the originals and re-copies a file once
# it finishes being written, so it is safe to run while a session is recording.

set -euo pipefail

SRC="/home/gifty4/remote-server/ur_egg_v3_Aza_Minnesota/tactile_UR5/pyForceDAQ/cone_data/"
DST="/home/gifty4/github_local/tactile_UR5/pyForceDAQ/cone_data/"

WATCH=0
INTERVAL=10
MOVE=0

while [ $# -gt 0 ]; do
    case "$1" in
        --watch) WATCH=1; [[ "${2:-}" =~ ^[0-9]+$ ]] && { INTERVAL="$2"; shift; } ;;
        --move)  MOVE=1 ;;
        *) echo "Unknown option: $1" >&2; exit 2 ;;
    esac
    shift
done

if [ ! -d "$SRC" ]; then
    echo "ERROR: source not found: $SRC" >&2
    echo "Is the remote PC mounted at /home/gifty4/remote-server ?" >&2
    exit 1
fi

mkdir -p "$DST"

# --no-owner/--no-group/--no-perms: sshfs shows files as root, and we run as a
# normal user, so don't try to replicate ownership/permissions.
RSYNC_OPTS=(-rt --no-owner --no-group --no-perms --info=progress2)
[ "$MOVE" -eq 1 ] && RSYNC_OPTS+=(--remove-source-files)

do_sync() {
    echo "[$(date '+%H:%M:%S')] syncing $SRC -> $DST"
    rsync "${RSYNC_OPTS[@]}" "$SRC" "$DST"
}

if [ "$WATCH" -eq 1 ]; then
    echo "Watching every ${INTERVAL}s. Ctrl-C to stop."
    while true; do
        do_sync || echo "  (sync failed, will retry)"
        sleep "$INTERVAL"
    done
else
    do_sync
    echo "Done."
fi
