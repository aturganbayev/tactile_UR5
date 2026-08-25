import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import socket
import time
from pose_utils import (
    START_CLEARANCE_M, apex_start_tcp_pose, pose_str, A_sim, V_sim,
    ATI_A_real, ATI_V_real, XELA_A_real, XELA_V_real,
    SIM_HOST, REAL_HOST, ATI_START_POSE_ROTVEC, XELA_START_POSE_ROTVEC,
    ATI_DEFAULT_TCP, XELA_DEFAULT_TCP,
)

# Constants
PORT = 30003
while True:
    mode = input("Select mode ('sim' or 'real'): ").strip().lower()

    if mode == "sim":
        HOST = SIM_HOST

        #Simulation Accelaration and Velocity

        A = A_sim
        V = V_sim

        break

    elif mode == "real":
        HOST = REAL_HOST

        A = V = None          # set below, once the sensor is known

        break
    else:
        print("Invalid input. Please type 'sim' or 'real'.")

while True:
    sensor = input("Select sensor ('xela' or 'ati'): ").strip().lower()

    if sensor == "ati":
        ROTVEC = ATI_START_POSE_ROTVEC
        DEFAULT_TCP = ATI_DEFAULT_TCP
        USE_CSV = True
        # Speeds follow the sensor this script already asks about - the XELA
        # pad is far larger and heavier than the Nano17 indenter.
        if A is None:
            A, V = ATI_A_real, ATI_V_real
        break
    elif sensor == "xela":
        ROTVEC = XELA_START_POSE_ROTVEC
        DEFAULT_TCP = XELA_DEFAULT_TCP
        USE_CSV = False
        if A is None:
            A, V = XELA_A_real, XELA_V_real
        break
    else:
        print("Invalid input. Please type 'xela' or 'ati'.")


def main():
    start_pose = apex_start_tcp_pose(
        clearance_m=START_CLEARANCE_M, rotvec=ROTVEC, default_tcp=DEFAULT_TCP, use_csv=USE_CSV
    )
    start_pose_line = pose_str(start_pose)
    print(
        f"Start pose: apex TCP + {START_CLEARANCE_M * 1000:.0f} mm in Z -> "
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
