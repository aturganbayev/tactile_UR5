#!/usr/bin/env python3
"""
Move the real robot's TCP to an input Cartesian pose.

Usage:
    python3 move_to_pose.py                       (prompts for the pose)
    python3 move_to_pose.py x y z rx ry rz

Pose format matches print_tcp_pose.py output: 6 space-separated values,
x y z in mm and rotation vector rx ry rz in rad, base frame. The robot
moves linearly (movel) from its current pose at the slow approach speed.
"""

import os
import socket
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pose_utils import (REAL_HOST, A_approach_real, V_approach_real, pose_str)

# Secondary client interface, same as run_side_strip_poses.py (30003 would
# suspend the state broadcast that print_tcp_pose.py / the recorder read).
PORT = 30002


def main():
    if len(sys.argv) == 7:
        tokens = sys.argv[1:]
    else:
        tokens = input("Target pose (x y z [mm] rx ry rz [rad]): ").split()

    if len(tokens) != 6:
        print(f"Error: expected 6 space-separated values, got {len(tokens)}.")
        return
    try:
        vals = [float(t) for t in tokens]
    except ValueError as e:
        print(f"Error: could not parse pose ({e}).")
        return

    # mm -> m for the position; rotation vector is already in rad
    pose = [v / 1000.0 for v in vals[:3]] + vals[3:]

    print("Target TCP pose (base frame):")
    print(f"  x y z  = {vals[0]:.3f} {vals[1]:.3f} {vals[2]:.3f} mm")
    print(f"  rotvec = {vals[3]:.6f} {vals[4]:.6f} {vals[5]:.6f} rad")
    if input("Move the robot there? [y/N]: ").strip().lower() != "y":
        print("Aborted.")
        return

    ur_script = (
        "def move_to_pose():\n"
        f"  movel(p[{pose_str(pose)}], a={A_approach_real}, v={V_approach_real})\n"
        "end\n"
        "move_to_pose()\n"
    )

    print(f"Connecting to robot ({REAL_HOST}:{PORT}) ...")
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((REAL_HOST, PORT))
        s.sendall(ur_script.encode("ascii"))
        time.sleep(1)
        s.close()
        print("Script sent. The robot should be moving.")
    except Exception as e:
        print(f"Failed to connect to the robot: {e}")


if __name__ == "__main__":
    main()
