import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import socket
import time
from pose_utils import pose_str, A_sim, A_real, V_sim, V_real, SIM_HOST, REAL_HOST
from taxel_geometry import taxel_hover_pose


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

    PORT = 30003

    # Which taxel's hover pose to move to (0 = the first taxel the
    # calibration sweep visits, i.e. the touch-probed reference point).
    # Pass a different index 0-15 as a command-line argument.
    taxel_index = int(sys.argv[1]) if len(sys.argv) > 1 else 0

    pre_pose = [-1.57, -1.57, -1.57, -1.57, 1.57, -1.57]
    pre_pose_line = pose_str(pre_pose)
    start_pose = taxel_hover_pose(taxel_index)
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
