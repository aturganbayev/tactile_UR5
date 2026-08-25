import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import socket
import time
from pose_utils import (
    START_CLEARANCE_M, apex_start_tcp_pose, pose_str, A_sim, A_real, V_sim, V_real,
    SIM_HOST, REAL_HOST, ATI_START_POSE_ROTVEC, XELA_START_POSE_ROTVEC,
    ATI_DEFAULT_TCP, XELA_DEFAULT_TCP,
)


def main():
    while True:
        mode = input("Select mode ('sim' or 'real'): ").strip().lower()

        if mode == "sim":
            HOST = SIM_HOST
            A = A_sim
            V = V_sim
            break

        elif mode == "real":
            HOST = REAL_HOST
            A = A_real
            V = V_real
            break
        else:
            print("Invalid input. Please type 'sim' or 'real'.")

    while True:
        sensor = input("Select sensor ('xela' or 'ati'): ").strip().lower()

        if sensor == "ati":
            rotvec = ATI_START_POSE_ROTVEC
            default_tcp = ATI_DEFAULT_TCP
            use_csv = True
            break
        elif sensor == "xela":
            rotvec = XELA_START_POSE_ROTVEC
            default_tcp = XELA_DEFAULT_TCP
            use_csv = False
            break
        else:
            print("Invalid input. Please type 'xela' or 'ati'.")

    PORT = 30003

    pre_pose = [-1.57, -1.57, -1.57, -1.57, 1.57, -1.57]
    pre_pose_line = pose_str(pre_pose)
    start_pose = apex_start_tcp_pose(
        clearance_m=START_CLEARANCE_M, rotvec=rotvec, default_tcp=default_tcp, use_csv=use_csv
    )
    start_pose_line = pose_str(start_pose)

    ur_script = (
        "def my_program():\n"
        f"  movej([{pre_pose_line}], a={A}, v={V}, t=0, r=0)\n"
        f"  movel(p[{start_pose_line}], a={A}, v={V}, t=0, r=0)\n"
        "end\n"
        "my_program()\n"
    )

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
