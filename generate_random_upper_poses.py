import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from pose_utils import approach_distance, press_distance, approach_and_press_poses

NUM_POINTS = 5
# Keep points close to the apex row in base-frame Y (meters).
MAX_Y_OFFSET_M = 0.01


def main():
    input_csv = "surface_points_base.csv"
    print(f"Loading data from {input_csv}...")
    df = pd.read_csv(input_csv)
    
    # Filter points for the upper 1/4th
    max_z = df["z"].max()
    min_z = df["z"].min()
    threshold_z = min_z + 0.75 * (max_z - min_z)
    df_upper = df[df["z"] >= threshold_z].copy()

    top_idx = df["z"].idxmax()
    center_x = df.loc[top_idx, "x"]
    center_y = df.loc[top_idx, "y"]

    y_offset = (df_upper["y"] - center_y).abs()
    df_upper = df_upper[y_offset <= MAX_Y_OFFSET_M].copy()
    print(
        f"Kept {len(df_upper)} upper points within "
        f"±{MAX_Y_OFFSET_M * 1000:.0f} mm of apex Y ({center_y:.4f} m)."
    )

    if len(df_upper) == 0:
        print("Error: no points left after Y filter. Increase MAX_Y_OFFSET_M.")
        return

    # The Y-band cuts a thin cross-section slice through the apex, so the
    # points already lie along the cone's profile curve. Spread the picks
    # evenly along that curve by binning on x (horizontal position across the
    # slice) instead of polar angle, which would collapse onto the apex.
    df_upper["y_dist"] = y_offset.loc[df_upper.index]

    bins = np.linspace(df_upper["x"].min(), df_upper["x"].max(), NUM_POINTS + 1)
    df_upper["bin"] = pd.cut(df_upper["x"], bins, labels=False, include_lowest=True)

    selected_points = []
    for i in range(NUM_POINTS):
        bin_points = df_upper[df_upper["bin"] == i]
        if len(bin_points) == 0:
            continue
        # Pick the point nearest the apex-Y plane so picks stay on the curve.
        selected_points.append(bin_points.loc[bin_points["y_dist"].idxmin()])

    selected_df = pd.DataFrame(selected_points)
    selected_df = selected_df.sort_values("x").reset_index(drop=True)

    print(f"Selected {len(selected_df)} points spread along the slice curve.")
    for _, row in selected_df.iterrows():
        dy_mm = abs(row["y"] - center_y) * 1000
        print(
            f"  x={row['x'] * 1000:.1f} mm, z={row['z'] * 1000:.1f} mm, "
            f"|dy|={dy_mm:.1f} mm"
        )
    
    # Generate touch poses

    poses = []
    
    for _, row in selected_df.iterrows():
        p = np.array([row["x"], row["y"], row["z"]])
        n = np.array([row["nx"], row["ny"], row["nz"]])
        n = n / np.linalg.norm(n)
        
        
        # Some STL files have inconsistent triangle orientations (flipped normals)
        v_out = np.array([p[0] - center_x, p[1] - center_y, 0])
        if np.linalg.norm(v_out) > 1e-5:
            v_out = v_out / np.linalg.norm(v_out)
            if np.dot(n[:2], v_out[:2]) < 0:
                n = -n  # Flip normal to point outward
        
        approach_p, (rx, ry, rz), press_p, _ = approach_and_press_poses(
            p, n, approach_distance, press_distance
        )
        
        poses.append({
            "x": p[0], "y": p[1], "z": p[2],
            "nx": n[0], "ny": n[1], "nz": n[2],
            "approach_x": approach_p[0], "approach_y": approach_p[1], "approach_z": approach_p[2],
            "approach_rx": rx, "approach_ry": ry, "approach_rz": rz,
            "press_x": press_p[0], "press_y": press_p[1], "press_z": press_p[2],
            "press_rx": rx, "press_ry": ry, "press_rz": rz
        })
        
    poses_df = pd.DataFrame(poses)
    output_file = "random_upper_touch_poses.csv"
    poses_df.to_csv(output_file, index=False)
    print(f"Saved random poses to {output_file}")
    
    # Plotting
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    ax.scatter(df["x"], df["y"], df["z"], c='gray', alpha=0.05, s=1, label="Cone")
    ax.scatter(df_upper["x"], df_upper["y"], df_upper["z"], c='blue', alpha=0.1, s=5, label="Upper 1/4")
    ax.scatter(poses_df["x"], poses_df["y"], poses_df["z"], c='red', s=50, label="Selected Points")
    
    ax.quiver(poses_df["x"], poses_df["y"], poses_df["z"], 
              poses_df["nx"], poses_df["ny"], poses_df["nz"], 
              length=0.015, color='green', label="Normals")
              
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title(f'{NUM_POINTS} Random Upper Surface Points and Normals')
    ax.legend()
    
    os.makedirs("figures", exist_ok=True)
    plt.savefig("figures/random_upper_points_plot.png")
    print("Saved plot to figures/random_upper_points_plot.png")
    
    plt.show()

if __name__ == "__main__":
    main()
