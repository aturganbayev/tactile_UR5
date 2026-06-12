import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from pose_utils import (
    approach_distance,
    press_distance,
    approach_and_press_poses,
    MAX_ORIENTATION_TILT_DEG,
)

# --- Parameters ---
NUM_STRIPS = 4             # number of strips evenly distributed around the cone
NUM_POINTS = 8            # number of touch points per strip (top → bottom)
MIN_HEIGHT_FRACTION = 0.6  # lower bound as a fraction of cone height
                           # (0.0 = base, 1.0 = apex)


def main():
    input_csv = "surface_points_base.csv"
    df = pd.read_csv(input_csv)

    max_z = df["z"].max()
    min_z = df["z"].min()
    z_lower = min_z + MIN_HEIGHT_FRACTION * (max_z - min_z)

    top_idx = df["z"].idxmax()
    center_x = df.loc[top_idx, "x"]
    center_y = df.loc[top_idx, "y"]

    df["angle"] = np.degrees(np.arctan2(df["y"] - center_y, df["x"] - center_x))

    df_height = df[df["z"] >= z_lower].copy()
    if len(df_height) == 0:
        print("Error: no points found in the specified height range.")
        return

    strip_angles = np.linspace(0, 360, NUM_STRIPS, endpoint=False)
    z_bins = np.linspace(max_z, z_lower, NUM_POINTS + 1)

    all_poses = []
    for strip_idx, strip_angle_raw in enumerate(strip_angles):
        # Normalise to [-180, 180] to match arctan2 output
        target = (strip_angle_raw + 180) % 360 - 180

        for i in range(NUM_POINTS):
            z_hi, z_lo = z_bins[i], z_bins[i + 1]
            band = df_height[(df_height["z"] <= z_hi) & (df_height["z"] >= z_lo)]
            if len(band) == 0:
                continue
            ang_dist = band["angle"].apply(lambda a: abs((a - target + 180) % 360 - 180))
            row = band.loc[ang_dist.idxmin()]

            p = np.array([row["x"], row["y"], row["z"]])
            n = np.array([row["nx"], row["ny"], row["nz"]])
            n = n / np.linalg.norm(n)

            v_out = np.array([p[0] - center_x, p[1] - center_y, 0.0])
            if np.linalg.norm(v_out) > 1e-5:
                v_out = v_out / np.linalg.norm(v_out)
                if np.dot(n[:2], v_out[:2]) < 0:
                    n = -n

            # Tilt the tool toward vertical for holder clearance:
            # 0 deg at the apex band, MAX_ORIENTATION_TILT_DEG at the lowest.
            height_frac = (p[2] - z_lower) / (max_z - z_lower)
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
    output_file = "cone_touch_poses.csv"
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

    os.makedirs("figures", exist_ok=True)
    plt.savefig("figures/cone_touch_poses.png", dpi=300, bbox_inches="tight")
    print("Saved plot to figures/cone_touch_poses.png")
    plt.show()


if __name__ == "__main__":
    main()
