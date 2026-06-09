import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

from pose_utils import approach_and_press_poses

NUM_POINTS = 2
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

    angles = np.arctan2(df_upper["y"] - center_y, df_upper["x"] - center_x)
    df_upper["angle"] = angles

    bins = np.linspace(-np.pi, np.pi, NUM_POINTS + 1)
    df_upper["bin"] = pd.cut(df_upper["angle"], bins, labels=False, include_lowest=True)

    selected_points = []
    for i in range(NUM_POINTS):
        bin_points = df_upper[df_upper["bin"] == i]
        if len(bin_points) == 0:
            continue
        # Prefer points nearer the apex Y row within each bin.
        bin_points = bin_points.assign(y_dist=y_offset.loc[bin_points.index])
        closest = bin_points["y_dist"].min()
        bin_points = bin_points[bin_points["y_dist"] <= closest + 1e-9]
        selected_points.append(bin_points.sample(n=1).iloc[0])

    selected_df = pd.DataFrame(selected_points)

    print(f"Selected {len(selected_df)} random points (Y-limited, spread by angle).")
    for _, row in selected_df.iterrows():
        dy_mm = abs(row["y"] - center_y) * 1000
        print(f"  angle={np.degrees(row['angle']):.1f} deg, |dy|={dy_mm:.1f} mm")
    
    # Generate touch poses
    approach_distance = 0.015
    press_distance = 0.005
    poses = []
    
    for _, row in selected_df.iterrows():
        p = np.array([row["x"], row["y"], row["z"]])
        n = np.array([row["nx"], row["ny"], row["nz"]])
        n = n / np.linalg.norm(n)
        
        # FIX: Ensure normal always points outward from the cone
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
    
    plt.savefig("random_upper_points_plot.png")
    print("Saved plot to random_upper_points_plot.png")
    
    try:
        plt.show(block=False)
        plt.pause(2)
    except Exception:
        pass # Ignore display errors if no GUI is available

if __name__ == "__main__":
    main()
