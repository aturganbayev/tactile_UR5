import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from pose_utils import approach_distance, press_distance, approach_and_press_poses

# --- Parameters ---
NUM_POINTS = 10          # number of touch points along the strip
SIDE_ANGLE_DEG = 180.0   # direction of the target side in degrees
                         # (0=+X, 90=+Y, 180=-X, 270/-90=-Y)
                         # run the script once to see the angle map printed below
ANGLE_WIDTH_DEG = 10   # angular width of the slice (±half of this around SIDE_ANGLE_DEG)
MIN_HEIGHT_FRACTION = 0.4  # lower bound of the strip as a fraction of cone height
                             # (0.0=base, 1.0=apex); 0.25 means the bottom of the
                             # strip is at 1/4 of the total cone height


def angle_in_slice(a, target, half_width):
    diff = (a - target + 180) % 360 - 180
    return abs(diff) <= half_width


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

    # Print angle map so the user can identify which angle is "left", "right", etc.
    print("=== Angle reference (from cone apex, looking down) ===")
    for label, angle in [("  +X (right)", 0), ("  +Y (forward)", 90),
                          ("  -X (left)",  180), ("  -Y (back)",  -90)]:
        print(f"  {label:20s} → {angle}°")
    print(f"\nUsing SIDE_ANGLE_DEG = {SIDE_ANGLE_DEG}°, width ±{ANGLE_WIDTH_DEG/2:.0f}°")
    print(f"Height range: z >= {z_lower*1000:.1f} mm  (bottom {MIN_HEIGHT_FRACTION*100:.0f}% cut off)\n")

    # Filter by height (from apex down to MIN_HEIGHT_FRACTION)
    df_height = df[df["z"] >= z_lower].copy()

    if len(df_height) == 0:
        print("Error: no points found in the specified height range.")
        return

    # Divide height range into NUM_POINTS equal z-bands; in each band pick the
    # point whose angle is closest to SIDE_ANGLE_DEG — this keeps all points on
    # the same curve along the cone surface.
    half_width = ANGLE_WIDTH_DEG / 2.0
    z_bins = np.linspace(max_z, z_lower, NUM_POINTS + 1)
    selected_rows = []
    for i in range(NUM_POINTS):
        z_hi, z_lo = z_bins[i], z_bins[i + 1]
        band = df_height[(df_height["z"] <= z_hi) & (df_height["z"] >= z_lo)]
        if len(band) == 0:
            continue
        ang_dist = band["angle"].apply(lambda a: abs((a - SIDE_ANGLE_DEG + 180) % 360 - 180))
        selected_rows.append(band.loc[ang_dist.idxmin()])

    selected_df = pd.DataFrame(selected_rows)

    # Strip candidates for visualisation only (points within the angular band)
    mask = df_height["angle"].apply(lambda a: angle_in_slice(a, SIDE_ANGLE_DEG, half_width))
    df_strip = df_height[mask].copy()

    print(f"Selected {len(selected_df)} points (top → bottom):")
    for _, row in selected_df.iterrows():
        print(f"  z={row['z']*1000:.1f} mm,  angle={row['angle']:.1f}°")

    # Generate touch poses with outward-corrected normals
    poses = []
    for _, row in selected_df.iterrows():
        p = np.array([row["x"], row["y"], row["z"]])
        n = np.array([row["nx"], row["ny"], row["nz"]])
        n = n / np.linalg.norm(n)

        v_out = np.array([p[0] - center_x, p[1] - center_y, 0.0])
        if np.linalg.norm(v_out) > 1e-5:
            v_out = v_out / np.linalg.norm(v_out)
            if np.dot(n[:2], v_out[:2]) < 0:
                n = -n

        approach_p, (rx, ry, rz), press_p, _ = approach_and_press_poses(
            p, n, approach_distance, press_distance
        )

        poses.append({
            "x": p[0], "y": p[1], "z": p[2],
            "nx": n[0], "ny": n[1], "nz": n[2],
            "approach_x": approach_p[0], "approach_y": approach_p[1], "approach_z": approach_p[2],
            "approach_rx": rx, "approach_ry": ry, "approach_rz": rz,
            "press_x": press_p[0], "press_y": press_p[1], "press_z": press_p[2],
            "press_rx": rx, "press_ry": ry, "press_rz": rz,
        })

    poses_df = pd.DataFrame(poses)
    output_file = "side_strip_touch_poses.csv"
    poses_df.to_csv(output_file, index=False)
    print(f"\nSaved {len(poses_df)} touch poses to {output_file}")

    # Plot — all cone points shown; strip candidates and selected points highlighted
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # All cone points
    ax.scatter(df["x"], df["y"], df["z"],
               c='lightgray', s=1, alpha=0.15, label="All surface points")

    # Strip candidates (within the angular band, for visual reference)
    ax.scatter(df_strip["x"], df_strip["y"], df_strip["z"],
               c='steelblue', s=8, alpha=0.5, label=f"Strip ±{half_width:.0f}°")

    # Selected points — large red stars
    ax.scatter(poses_df["x"], poses_df["y"], poses_df["z"],
               c='red', s=120, marker='*', zorder=5, label="Selected points")

    # Surface normals at selected points
    ax.quiver(poses_df["x"], poses_df["y"], poses_df["z"],
              poses_df["nx"], poses_df["ny"], poses_df["nz"],
              length=0.015, color='green', label="Normals")

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title(
        f"Side strip — angle={SIDE_ANGLE_DEG}° ±{half_width:.0f}°, "
        f"{len(poses_df)} points, "
        f"height ≥ {MIN_HEIGHT_FRACTION*100:.0f}% of cone"
    )
    ax.legend()

    os.makedirs("figures", exist_ok=True)
    plt.savefig("figures/side_strip_touch_poses.png", dpi=300, bbox_inches="tight")
    print("Saved plot to figures/side_strip_touch_poses.png")
    plt.show()


if __name__ == "__main__":
    main()
