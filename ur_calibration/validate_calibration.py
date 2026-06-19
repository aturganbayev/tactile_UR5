import os
import sys

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

# pose_utils.py and paths.py live one level up at the repo root.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
from pose_utils import tcp_pose_to_contact


def load_physical_contacts():
    df = pd.read_csv(paths.PHYSICAL_POINTS)
    if {"rx", "ry", "rz"}.issubset(df.columns):
        contacts = []
        for _, row in df.iterrows():
            contacts.append(
                tcp_pose_to_contact(
                    [row["x_tcp"], row["y_tcp"], row["z_tcp"]],
                    [row["rx"], row["ry"], row["rz"]],
                )
            )
        return np.asarray(contacts, dtype=float)

    if {"x", "y", "z"}.issubset(df.columns):
        return df[["x", "y", "z"]].to_numpy(dtype=float)

    raise ValueError("physical_points.csv must contain contact points or full TCP poses.")


def main():
    try:
        phys_points = load_physical_contacts()
    except FileNotFoundError:
        print("Error: physical_points.csv not found.")
        return

    try:
        T = np.loadtxt(paths.ICP_MATRIX)
    except OSError:
        print("Error: icp_transformation_matrix.txt not found. Run calibrate_icp.py first.")
        return

    surface_df = pd.read_csv(paths.SURFACE_POINTS_BASE)
    aligned = surface_df[["x", "y", "z"]].to_numpy(dtype=float)
    distances, _ = cKDTree(aligned).query(phys_points)
    print("=== Calibration validation (contact point -> calibrated STL mesh) ===")
    for i, dist in enumerate(distances):
        print(f"  Point {i}: {dist * 1000:.2f} mm")
    print(f"\nMean: {distances.mean() * 1000:.2f} mm")
    print(f"RMS:  {np.sqrt((distances ** 2).mean()) * 1000:.2f} mm")
    print(f"Max:  {distances.max() * 1000:.2f} mm")

    if distances.mean() > 0.005:
        print(
            "\nCalibration still looks poor (>5 mm mean error). "
            "Re-record physical points with full TCP pose (x,y,z,rx,ry,rz) "
            "and verify TOOL_TIP_OFFSET in pose_utils.py."
        )
    else:
        print("\nCalibration looks good (<5 mm mean error).")


if __name__ == "__main__":
    main()
