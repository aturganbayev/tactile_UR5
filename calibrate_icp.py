import numpy as np
import pandas as pd
import trimesh
import trimesh.registration

from pose_utils import tcp_pose_to_contact


def load_physical_contacts():
    df = pd.read_csv("physical_points.csv")

    if {"x", "y", "z"}.issubset(df.columns) and not {"x_tcp", "y_tcp", "z_tcp"}.issubset(df.columns):
        print(
            "Warning: physical_points.csv contains only x,y,z without orientation.\n"
            "Those values are likely TCP positions, not surface contact points.\n"
            "Re-run record_icp_points.py and record full TCP poses."
        )
        return df[["x", "y", "z"]].to_numpy(dtype=float)

    raise ValueError("physical_points.csv has an unexpected format.")


def mesh_apex_meters(mesh):
    verts = mesh.vertices / 1000.0
    return verts[np.argmax(verts[:, 2])]


def main():
    print("=== ICP Calibration ===")

    try:
        stl_df = pd.read_csv("surface_points.csv")
    except FileNotFoundError:
        print("Error: surface_points.csv not found. Run extract_points.py first.")
        return

    try:
        phys_points = load_physical_contacts()
    except FileNotFoundError:
        print("Error: physical_points.csv not found. Run record_icp_points.py first.")
        return

    if len(phys_points) < 4:
        print("Error: need at least 4 physical contact points.")
        return

    mesh = trimesh.load("cone.STL")
    stl_points = stl_df[["x", "y", "z"]].to_numpy(dtype=float) / 1000.0
    stl_apex = mesh_apex_meters(mesh)
    physical_apex = phys_points[0]

    print(f"Loaded {len(phys_points)} physical contact points and {len(stl_points)} STL points.")

    T_init = np.eye(4)
    T_init[:3, 3] = stl_apex - physical_apex

    print("Running ICP...")
    T_icp, _, cost = trimesh.registration.icp(
        phys_points,
        stl_points,
        initial=T_init,
        max_iterations=200,
        scale=False,
        reflection=False,
    )

    print(f"ICP cost: {cost:.6f} m")

    T_stl_to_robot = np.linalg.inv(T_icp)
    np.savetxt("icp_transformation_matrix.txt", T_stl_to_robot)

    ones = np.ones((len(stl_points), 1))
    aligned_points = (T_stl_to_robot @ np.hstack([stl_points, ones]).T).T[:, :3]

    stl_normals = stl_df[["nx", "ny", "nz"]].to_numpy(dtype=float)
    rotation = T_stl_to_robot[:3, :3]
    aligned_normals = (rotation @ stl_normals.T).T

    out_df = pd.DataFrame(
        {
            "x": aligned_points[:, 0],
            "y": aligned_points[:, 1],
            "z": aligned_points[:, 2],
            "nx": aligned_normals[:, 0],
            "ny": aligned_normals[:, 1],
            "nz": aligned_normals[:, 2],
        }
    )
    out_df.to_csv("surface_points_base.csv", index=False)

    from scipy.spatial import cKDTree

    mesh_distances, _ = cKDTree(aligned_points).query(phys_points)

    print("\nContact-point error after calibration:")
    for i, dist in enumerate(mesh_distances):
        print(f"  Point {i}: {dist * 1000:.2f} mm")
    print(
        f"\nMean: {mesh_distances.mean() * 1000:.2f} mm | "
        f"RMS: {np.sqrt((mesh_distances ** 2).mean()) * 1000:.2f} mm | "
        f"Max: {mesh_distances.max() * 1000:.2f} mm"
    )

    print("\nSaved surface_points_base.csv and icp_transformation_matrix.txt")
    print("Next: python3 validate_calibration.py")
    print("Then: python3 generate_random_upper_poses.py")


if __name__ == "__main__":
    main()
