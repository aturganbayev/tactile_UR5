#!/usr/bin/env python3
"""
Print the real robot's current TCP pose from the real-time stream.

Usage:
    python3 print_tcp_pose.py

Reads one packet from port 30003 and prints a single space-separated pose:
x y z (mm) + rotation vector rx ry rz (rad).
"""

import os
import socket
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pose_utils import REAL_HOST

ROBOT_PORT = 30003

# Same layout handling as pyForceDAQ/record_cone_press.py: the byte offset of
# the Cartesian tool vector depends on the controller generation, and the
# total packet size tells us which layout the robot streams.
#   * CB2 / software 1.x  -> 812-byte packet, tool vector at double index 74
#   * CB3 / e-Series 3.x+ -> 1044+ byte packet, tool vector at double index 56
_LAYOUT_V3 = slice(56, 62)
_LAYOUTS = {812: slice(74, 80)}


def read_tcp_pose(host, port=ROBOT_PORT, timeout=5.0):
    """Read one real-time packet and return the TCP pose [x,y,z,rx,ry,rz]."""
    with socket.create_connection((host, port), timeout=timeout) as s:
        s.settimeout(timeout)

        def recv_exact(n):
            buf = b""
            while len(buf) < n:
                chunk = s.recv(n - len(buf))
                if not chunk:
                    raise ConnectionError("robot socket closed")
                buf += chunk
            return buf

        while True:
            size_bytes = recv_exact(4)
            total = struct.unpack(">I", size_bytes)[0]
            if total < 12 or (total - 12) % 8 != 0:
                if total > 4:
                    recv_exact(total - 4)   # drain unknown packet and skip
                continue
            rest = recv_exact(total - 4)
            n_doubles = (total - 12) // 8
            vals = struct.unpack(f">Id{n_doubles}d", size_bytes + rest)
            return list(vals[_LAYOUTS.get(total, _LAYOUT_V3)])


def main():
    x, y, z, rx, ry, rz = read_tcp_pose(REAL_HOST)
    print(f"{x * 1000:.3f} {y * 1000:.3f} {z * 1000:.3f} "
          f"{rx:.6f} {ry:.6f} {rz:.6f}")


if __name__ == "__main__":
    main()
