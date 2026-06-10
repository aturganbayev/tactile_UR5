import numpy as np
from scipy.spatial.transform import Rotation as R


def pose_str(p):
    return ",".join([f"{x:.6f}" for x in p])

def rotvec_to_matrix(rotvec):
    return R.from_rotvec(np.asarray(rotvec, dtype=float)).as_matrix()


def tcp_pose_to_contact(xyz, rotvec, tip_offset=None):
    """Convert a TCP pose to the sensor tip (surface contact) position in base frame."""
    if tip_offset is None:
        tip_offset = TOOL_TIP_OFFSET
    return np.asarray(xyz, dtype=float) + rotvec_to_matrix(rotvec) @ np.asarray(tip_offset, dtype=float)


def contact_to_tcp_position(contact_xyz, rotvec, tip_offset=None):
    """Convert a desired contact point to the TCP position for that orientation."""
    if tip_offset is None:
        tip_offset = TOOL_TIP_OFFSET
    return np.asarray(contact_xyz, dtype=float) - rotvec_to_matrix(rotvec) @ np.asarray(tip_offset, dtype=float)


def normal_to_rotvec(normal):
    normal = np.asarray(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)

    # TCP +Z points into the surface (opposite outward normal).
    z_tcp = -normal

    world_z = np.array([0.0, 0.0, 1.0])
    y_tcp = np.cross(world_z, z_tcp)
    if np.linalg.norm(y_tcp) < 1e-3:
        y_tcp = np.array([0.0, 1.0, 0.0])
    else:
        y_tcp = y_tcp / np.linalg.norm(y_tcp)

    x_tcp = np.cross(y_tcp, z_tcp)
    x_tcp = x_tcp / np.linalg.norm(x_tcp)

    return R.from_matrix(np.column_stack((x_tcp, y_tcp, z_tcp))).as_rotvec()


# Default orientation used when hovering above the cone apex.
TOOL_TIP_OFFSET = np.array([0.0, 0.0, 0.086])
START_POSE_ROTVEC = np.array([-2.2, 2.2, 0.0])
START_CLEARANCE_M = 0.01

# Approach & press distance m
approach_distance = 0.015
press_distance = 0.005


def apex_start_tcp_pose(clearance_m=None, physical_points_csv="physical_points.csv"):
    """
    Safe hover pose above the cone apex.

    Uses the recorded apex-touch TCP pose and lifts it by clearance along base Z.
    With vertical-ish approach this matches ~1 cm above the cone top in practice.
    """
    if clearance_m is None:
        clearance_m = START_CLEARANCE_M

    default_tcp = np.array([0.002490, -0.513500, 0.1355])
    rotvec = START_POSE_ROTVEC.copy()

    try:
        import pandas as pd

        row = pd.read_csv(physical_points_csv).iloc[0]
        tcp = np.array([row["x_tcp"], row["y_tcp"], row["z_tcp"]], dtype=float)
    except (OSError, KeyError, IndexError, ValueError):
        tcp = default_tcp.copy()

    tcp[2] += clearance_m
    return np.concatenate([tcp, rotvec])


def approach_and_press_poses(surface_point, normal, approach_distance, press_distance, tip_offset=None):
    """Build TCP approach/press poses for a desired surface contact point."""
    if tip_offset is None:
        tip_offset = TOOL_TIP_OFFSET

    surface_point = np.asarray(surface_point, dtype=float)
    normal = np.asarray(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)

    rotvec = normal_to_rotvec(normal)
    tip_approach = surface_point + approach_distance * normal
    tip_press = surface_point - press_distance * normal

    approach_tcp = contact_to_tcp_position(tip_approach, rotvec, tip_offset)
    press_tcp = contact_to_tcp_position(tip_press, rotvec, tip_offset)

    return approach_tcp, rotvec, press_tcp, rotvec
