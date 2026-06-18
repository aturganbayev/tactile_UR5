import socket
import time
import pandas as pd

from pose_utils import START_CLEARANCE_M, apex_start_tcp_pose, pose_str, A_sim, A_real, V_sim, V_real, SIM_HOST, REAL_HOST



def main():
    input_csv = "random_upper_touch_poses.csv"
    try:
        poses_df = pd.read_csv(input_csv)
    except FileNotFoundError:
        print(f"Error: {input_csv} not found. Please run generate_random_upper_poses.py first.")
        return

    mode = input("Select mode ('sim' or 'real'): ").strip().lower()
    if mode == "sim":
        HOST = SIM_HOST
        A = A_sim
        V = V_sim
    elif mode == "real":
        HOST = REAL_HOST
        A = A_real
        V = V_real
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

    # Initial safe move to start pose
    ur_script_lines.append(f"  movej(get_inverse_kin(p[{start_pose_str}], qnear), a={A}, v={V})")
    ur_script_lines.append("  sleep(0.5)")
    ur_script_lines.append("  qnear = get_actual_joint_positions()")

    for _, row in poses_df.iterrows():
        approach = [row["approach_x"], row["approach_y"], row["approach_z"], row["approach_rx"], row["approach_ry"], row["approach_rz"]]
        press = [row["press_x"], row["press_y"], row["press_z"], row["press_rx"], row["press_ry"], row["press_rz"]]

        # Transit to approach in joint space, biased toward last known good config
        ur_script_lines.append(f"  movej(get_inverse_kin(p[{pose_str(approach)}], qnear), a={A}, v={V})")
        ur_script_lines.append("  sleep(0.5)")
        ur_script_lines.append("  qnear = get_actual_joint_positions()")
        # Press and retract in Cartesian (short, controlled linear motion)
        ur_script_lines.append(f"  movel(p[{pose_str(press)}], a={A}, v={V})")
        ur_script_lines.append("  sleep(0.5)")
        ur_script_lines.append(f"  movel(p[{pose_str(approach)}], a={A}, v={V})")
        ur_script_lines.append("  sleep(0.5)")

    # Final move back to original start pose
    ur_script_lines.append(f"  movej(get_inverse_kin(p[{start_pose_str}], qnear), a={A}, v={V})")
    ur_script_lines.append("end\nmy_program()\n")
    
    ur_script = "\n".join(ur_script_lines)
    
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
