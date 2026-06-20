import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
from pose_utils import (
    approach_distance,
    press_distance,
    approach_and_press_poses,
    MAX_ORIENTATION_TILT_DEG,
)

# --- Parameters ---
NUM_STRIPS = 12             # number of strips evenly distributed around the cone
NUM_POINTS = 8           # number of touch points per strip (top → bottom)
MIN_HEIGHT_FRACTION = 0.6  # lower bound as a fraction of cone height
                           # (0.0 = base, 1.0 = apex)


def cone_axis_from_calibration():
    """True symmetry axis of the cone in the robot base frame.

    surface_points_base.csv is the canonical cone mapped through the ICP
    calibration's rigid transform, which can tilt the cone's axis away from
    world Z. Height/angle bands must follow this axis, or they cut across
    the cone instead of tracing rings around it.
    """
    T = np.loadtxt(paths.ICP_MATRIX)
    axis = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return axis / np.linalg.norm(axis)


def perpendicular_basis(axis):
    ref = np.array([0.0, 0.0, 1.0])
    if abs(np.dot(axis, ref)) > 0.999:
        ref = np.array([0.0, 1.0, 0.0])
    u = np.cross(axis, ref)
    u = u / np.linalg.norm(u)
    v = np.cross(axis, u)
    return u, v


def main():
    input_csv = paths.SURFACE_POINTS_BASE
    df = pd.read_csv(input_csv)

    axis = cone_axis_from_calibration()
    u, v = perpendicular_basis(axis)

    pts = df[["x", "y", "z"]].to_numpy()
    origin = pts.mean(axis=0)
    rel = pts - origin
    t = rel @ axis
    perp = rel - np.outer(t, axis)

    df["t"] = t
    df["perp_x"], df["perp_y"], df["perp_z"] = perp[:, 0], perp[:, 1], perp[:, 2]
    df["angle"] = np.degrees(np.arctan2(perp @ v, perp @ u))

    t_max = t.max()
    t_min = t.min()
    t_lower = t_min + MIN_HEIGHT_FRACTION * (t_max - t_min)

    df_height = df[df["t"] >= t_lower].copy()
    if len(df_height) == 0:
        print("Error: no points found in the specified height range.")
        return

    strip_angles = np.linspace(0, 360, NUM_STRIPS, endpoint=False)
    t_bins = np.linspace(t_max, t_lower, NUM_POINTS + 1)

    all_poses = []
    for strip_idx, strip_angle_raw in enumerate(strip_angles):
        # Normalise to [-180, 180] to match arctan2 output
        target = (strip_angle_raw + 180) % 360 - 180

        # Every strip runs top→bottom; the execution script retracts to the
        # apex/start pose between strips, so direction doesn't need to match.
        for i in range(NUM_POINTS):
            t_hi, t_lo = t_bins[i], t_bins[i + 1]
            band = df_height[(df_height["t"] <= t_hi) & (df_height["t"] >= t_lo)]
            if len(band) == 0:
                continue
            ang_dist = band["angle"].apply(lambda a: abs((a - target + 180) % 360 - 180))
            row = band.loc[ang_dist.idxmin()]

            p = np.array([row["x"], row["y"], row["z"]])
            n = np.array([row["nx"], row["ny"], row["nz"]])
            n = n / np.linalg.norm(n)

            v_out = np.array([row["perp_x"], row["perp_y"], row["perp_z"]])
            if np.linalg.norm(v_out) > 1e-5:
                v_out = v_out / np.linalg.norm(v_out)
                if np.dot(n, v_out) < 0:
                    n = -n

            # Tilt the tool toward vertical for holder clearance:
            # 0 deg at the apex band, MAX_ORIENTATION_TILT_DEG at the lowest.
            height_frac = (row["t"] - t_lower) / (t_max - t_lower)
            tilt_deg = MAX_ORIENTATION_TILT_DEG * (1.0 - height_frac)

            approach_p, (rx, ry, rz), press_p, _ = approach_and_press_poses(
                p, n, approach_distance, press_distance, tilt_deg=tilt_deg
            )

            all_poses.append({
                "strip": strip_idx,
                "strip_angle_deg": round(strip_angle_raw, 1),
                "tilt_deg": round(tilt_deg, 2),
                "x": p[0], "y": p[1], "z": p[2],
                "nx": n[0], "ny": n[1], "nz": n[2],
                "approach_x": approach_p[0], "approach_y": approach_p[1], "approach_z": approach_p[2],
                "approach_rx": rx, "approach_ry": ry, "approach_rz": rz,
                "press_x": press_p[0], "press_y": press_p[1], "press_z": press_p[2],
                "press_rx": rx, "press_ry": ry, "press_rz": rz,
            })

    poses_df = pd.DataFrame(all_poses)
    output_file = paths.CONE_TOUCH_POSES
    poses_df.to_csv(output_file, index=False)
    print(f"Saved {len(poses_df)} touch poses  "
          f"({NUM_STRIPS} strips × {NUM_POINTS} points)  →  {output_file}")

    
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")

    ax.scatter(df["x"], df["y"], df["z"],
               c="dimgray", s=3, alpha=0.3, label="All surface points")

    colors = plt.cm.hsv(np.linspace(0, 1, NUM_STRIPS, endpoint=False))
    for strip_idx, strip_angle_raw in enumerate(strip_angles):
        strip_df = poses_df[poses_df["strip"] == strip_idx]
        c = [colors[strip_idx]]
        ax.scatter(strip_df["x"], strip_df["y"], strip_df["z"],
                   c=c, s=80, marker="o", zorder=5,
                   label=f"Strip {strip_idx}  ({strip_angle_raw:.0f}°)")
        ax.quiver(strip_df["x"], strip_df["y"], strip_df["z"],
                  strip_df["nx"], strip_df["ny"], strip_df["nz"],
                  length=0.015, color=colors[strip_idx])

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title(
        f"Cone touch poses — {NUM_STRIPS} strips × {NUM_POINTS} points, "
        f"height ≥ {MIN_HEIGHT_FRACTION * 100:.0f}% of cone"
    )
    ax.legend(loc="upper right", fontsize=7)

    os.makedirs(paths.FIGURES, exist_ok=True)
    plt.savefig(paths.SIDE_STRIP_PLOT, dpi=300, bbox_inches="tight")
    print(f"Saved plot to {paths.SIDE_STRIP_PLOT}")
    plt.show()


if __name__ == "__main__":
    main()
