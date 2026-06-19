import csv
import os
import sys

import numpy as np

# pose_utils.py and paths.py live one level up at the repo root.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
from pose_utils import tcp_pose_to_contact


def parse_tcp_xyz(x, y, z):
    """Accept teach-pendant values in millimeters or meters."""
    xyz = np.array([x, y, z], dtype=float)
    if np.max(np.abs(xyz)) > 2.0:
        xyz /= 1000.0
    return xyz


def main():
    print("=== Calibration Point Recorder ===")
    print("Record poses while the SENSOR TIP touches the cone.")
    print("1. The FIRST point MUST be the exact top (apex) of the cone.")
    print("2. Record 10-15 more points spread around the upper sides.")
    print("3. For each point, read the full TCP pose from the teach pendant:")
    print("   X Y Z (mm or m — auto-detected), RX RY RZ in radians.")
    print("   Examples:  2.49 -513.52 130.51 -2.2 2.2 0.0")
    print("          or:  0.00249 -0.51352 0.13051 -2.2 2.2 0.0")
    print("4. Orient the tool so TCP +Z points into the surface at each touch.")
    print("")

    filename = paths.PHYSICAL_POINTS

    with open(filename, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "x_tcp", "y_tcp", "z_tcp", "rx", "ry", "rz",
                "x", "y", "z",
            ]
        )

    count = 1
    while True:
        if count == 1:
            print(f"\nPoint {count} (APEX): touch the TOP of the cone.")
        else:
            print(f"\nPoint {count}: touch a point on the SIDE of the cone.")

        user_input = input(
            "Enter 'x y z rx ry rz' from the teach pendant, or 'done': "
        ).strip()

        if user_input.lower() == "done":
            if count < 5:
                print("Warning: calibration works best with at least 10 points.")
            break

        parts = user_input.split()
        if len(parts) != 6:
            print("Invalid input. Example: 2.5 -513.5 133.5 -2.2 2.2 0.0")
            continue

        try:
            x, y, z, rx, ry, rz = map(float, parts)
            tcp_xyz = parse_tcp_xyz(x, y, z)
            rotvec = np.array([rx, ry, rz], dtype=float)
            contact = tcp_pose_to_contact(tcp_xyz, rotvec)

            with open(filename, mode="a", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        tcp_xyz[0], tcp_xyz[1], tcp_xyz[2],
                        rotvec[0], rotvec[1], rotvec[2],
                        contact[0], contact[1], contact[2],
                    ]
                )

            print(f"Recorded TCP (m):     [{tcp_xyz[0]:.6f}, {tcp_xyz[1]:.6f}, {tcp_xyz[2]:.6f}]")
            print(f"Recorded contact (m): [{contact[0]:.6f}, {contact[1]:.6f}, {contact[2]:.6f}]")
            count += 1
        except ValueError:
            print("Invalid input. Make sure all six values are numbers.")


    print(f"\nFinished! Recorded {count - 1} points to {filename}.")
    print("Next: python3 ur_calibration/calibrate_icp.py")
    print("Then: python3 ur_calibration/validate_calibration.py")


if __name__ == "__main__":
    main()

