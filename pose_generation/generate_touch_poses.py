import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
from pose_utils import approach_distance, press_distance, approach_and_press_poses

input_csv = paths.SURFACE_POINTS_BASE
output_csv = paths.TOUCH_POSES



df = pd.read_csv(input_csv)

poses = []

# Calculate approximate center to determine outward direction
top_idx = df["z"].idxmax()
center_x = df.loc[top_idx, "x"]
center_y = df.loc[top_idx, "y"]

for _, row in df.iterrows():
    p = np.array([row["x"], row["y"], row["z"]])
    n = np.array([row["nx"], row["ny"], row["nz"]])
    n = n / np.linalg.norm(n)

    # Ensure normal always points outward from the cone
    v_out = np.array([p[0] - center_x, p[1] - center_y, 0])
    if np.linalg.norm(v_out) > 1e-5:
        v_out = v_out / np.linalg.norm(v_out)
        if np.dot(n[:2], v_out[:2]) < 0:
            n = -n  # Flip normal to point outward

    approach_p, (rx, ry, rz), press_p, _ = approach_and_press_poses(
        p, n, approach_distance, press_distance
    )

    poses.append([
        approach_p[0], approach_p[1], approach_p[2], rx, ry, rz,
        press_p[0], press_p[1], press_p[2], rx, ry, rz
    ])

out = pd.DataFrame(
    poses,
    columns=[
        "approach_x", "approach_y", "approach_z",
        "approach_rx", "approach_ry", "approach_rz",
        "press_x", "press_y", "press_z",
        "press_rx", "press_ry", "press_rz"
    ]
)

out.to_csv(output_csv, index=False)
print("Saved:", output_csv)
print(out.head())