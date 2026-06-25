"""
Live-decode the UR5's Robot Message stream (port 30001, the Primary client
interface) -- the same feed PolyScope renders in the pendant's Log tab.

This is NOT a log file. /root/log_history.txt on the controller (see
fetch_robot_logs.py) is only flushed to disk at boot/shutdown boundaries, so
tailing it misses everything in between -- including textmsg() calls from a
running program. The pendant shows messages live because the GUI reads them
straight off this socket. We do the same thing here.

Packets on this port are framed as: 4-byte big-endian length (header
inclusive) + 1-byte packet type + payload. Type 20 is ROBOT_MESSAGE. Rather
than decode the exact field layout for this old CB2/SW1.8 protocol version
(undocumented for this generation), this pulls out the printable-ASCII runs
in the payload, which is where textmsg() strings, version banners, and mode
labels live.
"""

import os
import re
import socket
import struct
import sys
from time import strftime

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pose_utils import SIM_HOST, REAL_HOST

PORT = 30001
ROBOT_MESSAGE_TYPE = 20
LOCAL_LOG_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_logs")


def iter_packets(sock):
    buf = b""
    while True:
        chunk = sock.recv(65536)
        if not chunk:
            return
        buf += chunk
        while len(buf) >= 4:
            plen = struct.unpack(">I", buf[:4])[0]
            if len(buf) < plen:
                break
            yield buf[4], buf[5:plen]
            buf = buf[plen:]


def main():
    mode = input("Select mode ('sim' or 'real'): ").strip().lower()
    if mode == "sim":
        host = SIM_HOST
    elif mode == "real":
        host = REAL_HOST
    else:
        print("Invalid mode. Exiting.")
        return

    os.makedirs(LOCAL_LOG_ROOT, exist_ok=True)
    local_file = os.path.join(LOCAL_LOG_ROOT, f"{strftime('%Y-%m-%d_%H-%M-%S')}_messages.log")
    print(f"Connecting to {host}:{PORT} ... saving to {local_file}. Ctrl-C to stop.")

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, PORT))

    with open(local_file, "w") as f:
        try:
            for ptype, payload in iter_packets(s):
                if ptype != ROBOT_MESSAGE_TYPE:
                    continue
                strings = re.findall(rb"[\x20-\x7e]{4,}", payload)
                if not strings:
                    continue
                line = f"[{strftime('%H:%M:%S')}] " + " | ".join(b.decode("ascii") for b in strings)
                print(line)
                f.write(line + "\n")
                f.flush()
        except KeyboardInterrupt:
            pass
        finally:
            s.close()


if __name__ == "__main__":
    main()
