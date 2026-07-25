import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import socket
import time
from pose_utils import (pose_str, A_approach_sim, A_approach_real,
                        V_approach_sim, V_approach_real, SIM_HOST, REAL_HOST)
from taxel_geometry import taxel_hover_pose, HOVER_CLEARANCE_M

# Constants
# Secondary client interface (30002), same as execution/move_to_pose.py - the
# other "movel the TCP to a pose" script. A program streamed to the realtime
# port 30003 is not reliably executed by URSim, and 30002 also keeps the
# 30003 state broadcast alive for any concurrent pose reader.
PORT = 30002

# Which taxel's hover pose to move to (0 = the first taxel the calibration
# sweep visits, i.e. the touch-probed reference point). Pass a different index
# 0-15 as a command-line argument.
TAXEL_INDEX = int(sys.argv[1]) if len(sys.argv) > 1 else 0

while True:
    mode = input("Select mode ('sim' or 'real'): ").strip().lower()

    if mode == "sim":
        HOST = SIM_HOST

        # Slow Cartesian approach speed (same as move_to_pose.py), not the
        # joint-space V_sim/A_sim - this is a linear movel to a hover pose.

        A = A_approach_sim
        V = V_approach_sim

        break

    elif mode == "real":
        HOST = REAL_HOST

        A = A_approach_real
        V = V_approach_real

        break
    else:
        print("Invalid input. Please type 'sim' or 'real'.")


def main():
    start_pose = taxel_hover_pose(TAXEL_INDEX)
    start_pose_line = pose_str(start_pose)
    print(
        f"XELA start pose: taxel {TAXEL_INDEX} + "
        f"{HOVER_CLEARANCE_M * 1000:.0f} mm hover -> "
        f"[{start_pose[0]:.6f}, {start_pose[1]:.6f}, {start_pose[2]:.6f}, "
        f"{start_pose[3]:.1f}, {start_pose[4]:.1f}, {start_pose[5]:.1f}]"
    )

    ur_script = (
        "def my_program():\n"
        f"  movel(p[{start_pose_line}], a={A}, v={V})\n"
        "end\nmy_program()\n"
    )

    print("Connecting to robot...")
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        s.sendall(ur_script.encode('ascii'))
        time.sleep(1)
        s.close()
        print("Script sent successfully! The robot should be moving.")
    except Exception as e:
        print(f"Failed to connect to the robot: {e}")


if __name__ == "__main__":
    main()
