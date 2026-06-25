import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import socket
import time
from pose_utils import pose_str, A_sim, A_real, V_sim, V_real, SIM_HOST, REAL_HOST


def main():
    mode = input("Select mode ('sim' or 'real'): ").strip().lower()
    if mode == "sim":
        HOST = SIM_HOST
        A = A_sim
        V = V_sim
    elif mode == "real":
        HOST = REAL_HOST
        A = A_real
        V = V_real
        confirm = input("Shutting down the REAL robot controller. Type 'yes' to confirm: ").strip().lower()
        if confirm != "yes":
            print("Aborted.")
            return
    else:
        print("Invalid mode. Exiting.")
        return

    PORT = 30003

    home_pose = [0, -1.57, 0, -1.57, 0, 0]
    home_pose_line = pose_str(home_pose)

    # Return to home before shutdown so the arm isn't left in an awkward pose
    # when the controller powers off.
    ur_script = (
        f"movej([{home_pose_line}], a={A}, v={V}, t=0, r=0)\n"
        "powerdown()\n"
    )
    print(ur_script)

    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((HOST, PORT))
        s.sendall(ur_script.encode('ascii'))
        time.sleep(1)
        s.close()
        print("Shutdown script sent. The robot should move home and power down.")
    except Exception as e:
        print(f"Failed to connect to the robot: {e}")


if __name__ == "__main__":
    main()
