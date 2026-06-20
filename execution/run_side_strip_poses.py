import os
import socket
import sys
import time

import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
from pose_utils import (START_CLEARANCE_M, apex_start_tcp_pose, pose_str,
                        A_sim, A_real, V_sim, V_real,
                        A_approach_sim, A_approach_real, V_approach_sim, V_approach_real,
                        SIM_HOST, REAL_HOST)


def main():
    input_csv = paths.CONE_TOUCH_POSES
    try:
        poses_df = pd.read_csv(input_csv)
    except FileNotFoundError:
        print(f"Error: {input_csv} not found. Please run generate_side_strip_poses.py first.")
        return

    mode = input("Select mode ('sim' or 'real'): ").strip().lower()
    if mode == "sim":
        HOST = SIM_HOST
        A, V = A_sim, V_sim                       # fast transit
        A_app, V_app = A_approach_sim, V_approach_sim   # slow contact
    elif mode == "real":
        HOST = REAL_HOST
        A, V = A_real, V_real
        A_app, V_app = A_approach_real, V_approach_real
    else:
        print("Invalid mode. Exiting.")
        return

    # Secondary client interface (30002), NOT realtime (30003): pushing a
    # `def my_program() ... end` program to 30003 makes the CB2/SW1.8 controller
    # suspend its 125 Hz state broadcast, freezing record_cone_press.py's pose
    # reader. 30002 runs the program while 30003 keeps streaming state.
    PORT = 30002

    start_pose = apex_start_tcp_pose(clearance_m=START_CLEARANCE_M)
    start_pose_str = pose_str(start_pose)
    print(
        f"Start pose: apex TCP + {START_CLEARANCE_M * 1000:.0f} mm in Z -> "
        f"[{start_pose[0]:.6f}, {start_pose[1]:.6f}, {start_pose[2]:.6f}, "
        f"{start_pose[3]:.1f}, {start_pose[4]:.1f}, {start_pose[5]:.1f}]"
    )

    ur_script_lines = ["def my_program():"]
    # Safe reference configuration (pre-pose): joint 5 = 90° keeps the wrist away from singularity
    ur_script_lines.append("  qnear = [-1.57, -1.57, -1.57, -1.57, 1.57, -1.57]")

    # Move to safe start pose above the apex
    ur_script_lines.append(f"  movej(get_inverse_kin(p[{start_pose_str}], qnear), a={A}, v={V})")
    ur_script_lines.append("  sleep(0.5)")

    prev_strip = None
    for i, (_, row) in enumerate(poses_df.iterrows()):
        approach = [row["approach_x"], row["approach_y"], row["approach_z"],
                    row["approach_rx"], row["approach_ry"], row["approach_rz"]]
        press    = [row["press_x"],    row["press_y"],    row["press_z"],
                    row["press_rx"],   row["press_ry"],   row["press_rz"]]

        # Retract to the safe apex/start pose before swinging to the next
        # strip, instead of transiting directly between strips.
        strip = int(row["strip"])
        if prev_strip is not None and strip != prev_strip:
            ur_script_lines.append(f"  movej(get_inverse_kin(p[{start_pose_str}], qnear), a={A}, v={V})")
            ur_script_lines.append("  sleep(0.5)")
        prev_strip = strip

        # Log pose index and IK solution so the failing pose shows in the
        # pendant log (PolyScope -> Log tab) right before a protective stop.
        ur_script_lines.append(f'  textmsg("pose {i} strip {strip}")')
        ur_script_lines.append(f"  q = get_inverse_kin(p[{pose_str(approach)}], qnear)")
        ur_script_lines.append('  textmsg("q: ", q)')

        # Transit in joint space, always biased toward the fixed safe config.
        # (Chaining qnear from actual positions causes joint wind-up as the
        # strips wrap around the cone, eventually hitting joint limits.)
        ur_script_lines.append(f"  movej(q, a={A}, v={V})")
        ur_script_lines.append("  sleep(0.5)")
        # Press and retract in Cartesian at the slow contact speed so the tool
        # eases onto the cone instead of knocking it away.
        ur_script_lines.append(f"  movel(p[{pose_str(press)}], a={A_app}, v={V_app})")
        ur_script_lines.append("  sleep(0.5)")
        ur_script_lines.append(f"  movel(p[{pose_str(approach)}], a={A_app}, v={V_app})")
        ur_script_lines.append("  sleep(0.5)")

    # Return to start pose
    ur_script_lines.append(f"  movej(get_inverse_kin(p[{start_pose_str}], qnear), a={A}, v={V})")
    ur_script_lines.append("end\nmy_program()\n")

    ur_script = "\n".join(ur_script_lines)

    print(f"Connecting to robot ({HOST}:{PORT})...")
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
