import os
import sys

import numpy as np
import pandas as pd
import trimesh

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


def mesh_apex_meters(mesh):
    verts = mesh.vertices / 1000.0
    return verts[np.argmax(verts[:, 2])]


def main():
    print("=== ICP Calibration ===")

    try:
        stl_df = pd.read_csv(paths.SURFACE_POINTS)
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

    mesh = trimesh.load(paths.CONE_STL)
    stl_points = stl_df[["x", "y", "z"]].to_numpy(dtype=float) / 1000.0
    stl_apex = mesh_apex_meters(mesh)
    physical_apex = phys_points[0]

    print(f"Loaded {len(phys_points)} physical contact points and {len(stl_points)} STL points.")

    # Constrained calibration: keep the cone axis VERTICAL (robot +Z) instead of
    # letting ICP rotate it freely. The calibration points all sit near the apex
    # (top ~half of the cone), which barely constrains the axis orientation, so a
    # free ICP fits a spurious ~20deg tilt. The cone physically stands upright on
    # a level surface, so we fix the rotation (the STL +z, which is the cone's
    # symmetry axis, -> robot +Z) and fit ONLY the position by translation-only
    # ICP against the physical points. Rotation about the vertical is irrelevant
    # for a surface of revolution, so it is left at identity.
    from scipy.spatial import cKDTree

    R_vert = np.eye(3)
    t = physical_apex - stl_apex          # init: align apexes
    print("Fitting position (cone axis pinned vertical)...")
    for _ in range(100):
        _, idx = cKDTree(stl_points + t).query(phys_points)
        correction = (phys_points - (stl_points[idx] + t)).mean(axis=0)
        t = t + correction
        if np.linalg.norm(correction) < 1e-9:
            break

    T_stl_to_robot = np.eye(4)
    T_stl_to_robot[:3, :3] = R_vert
    T_stl_to_robot[:3, 3] = t
    np.savetxt(paths.ICP_MATRIX, T_stl_to_robot)

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
    out_df.to_csv(paths.SURFACE_POINTS_BASE, index=False)

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
    print("Next: python3 ur_calibration/validate_calibration.py")
    print("Then: python3 pose_generation/generate_side_strip_poses.py")


if __name__ == "__main__":
    main()
