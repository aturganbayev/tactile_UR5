import socket
import struct
import threading
import queue
import time
import csv
import os
from pose_utils import START_CLEARANCE_M, apex_start_tcp_pose, pose_str

# Constants
PORT = 30003
while True:
    mode = input("Select mode ('sim' or 'real'): ").strip().lower()

    if mode == "sim":
        HOST = "172.17.0.2"

        #Simulation Accelaration and Velocity

        A = 2.4
        V = 0.5

        break

    elif mode == "real":
        HOST = "192.168.0.153"  # <-- replace with your real robot IP

        A = 0.1
        V = 0.05

        break
    else:
        print("Invalid input. Please type 'sim' or 'real'.")


def main():
    start_pose = apex_start_tcp_pose(clearance_m=START_CLEARANCE_M)
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
