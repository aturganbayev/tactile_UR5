import os

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares

# Calibration artifacts live in ur_calibration/ next to this file.
_DEFAULT_PHYSICAL_POINTS_CSV = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "ur_calibration", "physical_points.csv"
)


def pose_str(p):
    return ",".join([f"{x:.6f}" for x in p])

def rotvec_to_matrix(rotvec):
    return R.from_rotvec(np.asarray(rotvec, dtype=float)).as_matrix()


def tcp_pose_to_contact(xyz, rotvec, tip_offset=None):
    # Convert a TCP pose to the sensor tip (surface contact) position in base frame
    if tip_offset is None:
        tip_offset = TOOL_TIP_OFFSET
    return np.asarray(xyz, dtype=float) + rotvec_to_matrix(rotvec) @ np.asarray(tip_offset, dtype=float)


def contact_to_tcp_position(contact_xyz, rotvec, tip_offset=None):
    # Convert a desired contact point to the TCP position for that orientation
    if tip_offset is None:
        tip_offset = TOOL_TIP_OFFSET
    return np.asarray(contact_xyz, dtype=float) - rotvec_to_matrix(rotvec) @ np.asarray(tip_offset, dtype=float)


def normal_to_rotvec(normal, y_hint=None):
    normal = np.asarray(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)

    # TCP +Z points into the surface (opposite outward normal).
    z_tcp = -normal

    world_z = np.array([0.0, 0.0, 1.0])
    y_tcp = np.cross(world_z, z_tcp)
    if np.linalg.norm(y_tcp) < 1e-3:
        # z_tcp is (anti)parallel to world Z - e.g. a cone apex, where the
        # outward normal is purely axial and azimuth is geometrically
        # undefined. cross(world_z, z_tcp) can't pick a horizontal axis here,
        # so without a hint every such point silently falls back to the same
        # fixed [0, 1, 0] frame regardless of which strip it belongs to. That
        # produced a real (up to ~165 deg, mean ~78 deg across strips)
        # reorientation between a strip's apex point and its very next point,
        # since every OTHER point on the strip locks its horizontal axis to
        # the strip's azimuth. Callers that know the intended azimuth (e.g.
        # generate_side_strip_poses.py, via the meridian-perpendicular
        # direction cross(e_r, axis)) should pass it as y_hint so the apex
        # frame is consistent with the rest of its strip.
        y_tcp = np.array([0.0, 1.0, 0.0]) if y_hint is None else np.asarray(y_hint, dtype=float)
        y_tcp = y_tcp / np.linalg.norm(y_tcp)
    else:
        y_tcp = y_tcp / np.linalg.norm(y_tcp)

    x_tcp = np.cross(y_tcp, z_tcp)
    x_tcp = x_tcp / np.linalg.norm(x_tcp)

    return R.from_matrix(np.column_stack((x_tcp, y_tcp, z_tcp))).as_rotvec()


def tilt_normal_toward_vertical(normal, tilt_deg):
    """Rotate a normal toward world +Z by tilt_deg degrees.

    Used to tilt the tool orientation away from the surface so the sensor
    holder clears the cone below the contact point. Clamped so the result
    never tilts past vertical.
    """
    normal = np.asarray(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)
    if tilt_deg <= 0.0:
        return normal

    world_z = np.array([0.0, 0.0, 1.0])
    axis = np.cross(normal, world_z)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-6:
        return normal  # already vertical

    angle_to_z = np.arccos(np.clip(np.dot(normal, world_z), -1.0, 1.0))
    tilt = min(np.radians(tilt_deg), angle_to_z)
    return R.from_rotvec(axis / axis_norm * tilt).apply(normal)


# Robot IP addresses for Sim and Real PC
SIM_HOST = "172.17.0.2"
REAL_HOST = "192.168.0.110"

# Velocity and Acceleration for Sim and Real Robot.
# Transit moves are joint-space (movej): v is rad/s, a is rad/s^2. Sim is pushed
# near the UR5 joint limit (~3.14 rad/s) since there's no hardware to protect.
#
# The real-robot transit speed is TOOL DEPENDENT. The ATI figures were tuned
# with the Nano17 and its small pointed indenter; the XELA pad is a much larger,
# heavier end-effector with a cable, so it transits at roughly a third of that.
# Running ATI speeds with the XELA fitted swings a big tool fast near the
# phantom, and the XELA speeds make the ATI sweep needlessly slow.
ATI_A_real = 0.35
ATI_V_real = 0.7
XELA_A_real = 0.2
XELA_V_real = 0.1

# Unprefixed names are the ATI values, which is what the generic movers in
# execution/ (go_home, go_pre_pose, shutdown_robot) use. Scripts that KNOW
# which tool is fitted select explicitly instead: home_start.py and
# start_pose.py already ask, and run_side_strip_poses.py is ATI-only.
A_sim = 8.0
A_real = ATI_A_real
V_sim = 3.0
V_real = ATI_V_real

# Approach/contact speed: used only for the short press-into-surface and retract
# moves. Kept slow so the tool eases onto the cone instead of knocking it away.
# Tune V_approach_real down if the cone still shifts on contact.
A_approach_sim = 2.5
A_approach_real = 0.2
V_approach_sim = 1
V_approach_real = 0.1

# Default orientation used when hovering above the cone apex.
TOOL_TIP_OFFSET = np.array([0.0, 0.0, 0.086])
START_POSE_ROTVEC = np.array([0.0, 3.12, 0.03])
ATI_START_POSE_ROTVEC = np.array([-2.2, 2.2, 0.0])
XELA_START_POSE_ROTVEC = np.array([0.0, 3.12, 0.03])

# Default apex TCP position (m), used when physical_points.csv can't be read.
ATI_DEFAULT_TCP = np.array([0.002490, -0.513500, 0.21185])
XELA_DEFAULT_TCP = np.array([-0.000988, -0.514494, 0.21185])

START_CLEARANCE_M = 0.03

# Approach & press distance m
approach_distance = 0.015
press_distance = 0.015

# --------------------------------------------------------------------------- #
#                            UR5 KINEMATICS (offline IK)                       #
# --------------------------------------------------------------------------- #
# Official UR5 Denavit-Hartenberg parameters (metres, radians). Used to solve
# joint targets in software so we don't depend on the controller's
# get_inverse_kin, whose single fixed seed cannot converge for poses spread all
# the way around the cone. Joints are commanded directly with movej([...]).

_UR5_D = np.array([0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823])
_UR5_A = np.array([0.0, -0.425, -0.39225, 0.0, 0.0, 0.0])
_UR5_ALPHA = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0])

# Seed for the very first IK solve (a safe, mid-range elbow configuration).
UR5_IK_SEED = np.array([-1.57, -1.57, -1.57, -1.57, 1.57, -1.57])


def _ur5_dh(theta, i):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(_UR5_ALPHA[i]), np.sin(_UR5_ALPHA[i])
    return np.array([
        [ct, -st * ca, st * sa, _UR5_A[i] * ct],
        [st, ct * ca, -ct * sa, _UR5_A[i] * st],
        [0.0, sa, ca, _UR5_D[i]],
        [0.0, 0.0, 0.0, 1.0],
    ])


def ur5_fk(q):
    """Forward kinematics: joint vector (6) -> 4x4 TCP (flange) pose."""
    T = np.eye(4)
    for i in range(6):
        T = T @ _ur5_dh(q[i], i)
    return T


def _ur5_pose_residual(q, target_T):
    T = ur5_fk(q)
    pos_err = T[:3, 3] - target_T[:3, 3]
    rot_err = R.from_matrix(T[:3, :3].T @ target_T[:3, :3]).as_rotvec()
    return np.concatenate([pos_err, rot_err])


def ur5_ik_near(pose, seed, max_pos_err=2e-3):
    """Numerical IK: joint solution for `pose` (x,y,z,rx,ry,rz) nearest `seed`.

    Chaining the seed from the previous solved pose keeps the trajectory smooth
    and the branch consistent. Returns (q, ok) where ok is False if the solver
    could not reach the pose (position error above max_pos_err).
    """
    pose = np.asarray(pose, dtype=float)
    target_T = np.eye(4)
    target_T[:3, :3] = R.from_rotvec(pose[3:]).as_matrix()
    target_T[:3, 3] = pose[:3]
    sol = least_squares(_ur5_pose_residual, np.asarray(seed, dtype=float),
                        method="lm", args=(target_T,), max_nfev=300)
    q = sol.x
    # Wrap each joint to the 2*pi-equivalent nearest the seed (same pose,
    # preserves continuity and keeps within the UR5 +/-2*pi joint range).
    q = np.asarray(seed, dtype=float) + ((q - np.asarray(seed, dtype=float) + np.pi) % (2 * np.pi) - np.pi)
    ok = np.linalg.norm(ur5_fk(q)[:3, 3] - target_T[:3, 3]) <= max_pos_err
    return q, ok

"""

Tool tilt toward vertical (deg). Two purposes:
  * holder clearance: tilt the tool away from the cone so the printed sensor
    holder clears the surface below the contact point.
  * self-collision avoidance: a more top-down (vertical) approach keeps the
    wrist extended, so the tool flange stays away from the lower arm. Pure
    horizontal approaches fold the wrist and risk clamping the forearm.

Generators scale tilt with height between MIN (apex band) and MAX (lowest
band): MIN is a floor applied to ALL points so even the apex approaches come
in from above; MAX adds extra tilt low down for holder clearance. Raise
MIN_ORIENTATION_TILT_DEG if the wrist still folds toward the forearm on the
near-horizontal strips.
"""
MIN_ORIENTATION_TILT_DEG = 7
MAX_ORIENTATION_TILT_DEG = 14


def apex_start_tcp_pose(clearance_m=None, physical_points_csv=None, rotvec=None, default_tcp=None, use_csv=True):
    """
    Safe hover pose above the cone apex.

    Uses the recorded apex-touch TCP pose and lifts it by clearance along base Z.
    With vertical-ish approach this matches ~1 cm above the cone top in practice.

    rotvec defaults to START_POSE_ROTVEC; pass ATI_START_POSE_ROTVEC or
    XELA_START_POSE_ROTVEC to select the sensor-specific start orientation.
    default_tcp (used when use_csv is False, or when physical_points_csv can't
    be read) defaults to ATI_DEFAULT_TCP; pass ATI_DEFAULT_TCP or
    XELA_DEFAULT_TCP explicitly to match the sensor in use.
    use_csv: physical_points.csv holds ATI's calibrated apex touch point, not
    XELA's - pass False (as done for the xela sensor) to use default_tcp
    directly instead of reading it.
    """
    if clearance_m is None:
        clearance_m = START_CLEARANCE_M
    if physical_points_csv is None:
        physical_points_csv = _DEFAULT_PHYSICAL_POINTS_CSV
    if rotvec is None:
        rotvec = START_POSE_ROTVEC
    if default_tcp is None:
        default_tcp = ATI_DEFAULT_TCP

    default_tcp = np.asarray(default_tcp, dtype=float)
    rotvec = np.asarray(rotvec, dtype=float).copy()

    tcp = None
    if use_csv:
        try:
            import pandas as pd

            row = pd.read_csv(physical_points_csv).iloc[0]
            tcp = np.array([row["x_tcp"], row["y_tcp"], row["z_tcp"]], dtype=float)
        except (OSError, KeyError, IndexError, ValueError):
            tcp = None
    if tcp is None:
        tcp = default_tcp.copy()

    tcp[2] += clearance_m
    return np.concatenate([tcp, rotvec])


def approach_and_press_poses(surface_point, normal, approach_distance, press_distance, tip_offset=None, tilt_deg=0.0, y_hint=None):
    """Build TCP approach/press poses for a desired surface contact point.

    tilt_deg tilts only the tool ORIENTATION toward vertical (holder
    clearance); the tip positions and press direction stay on the true
    surface normal. y_hint is forwarded to normal_to_rotvec, used only when
    the normal is (anti)parallel to world Z (e.g. a cone apex) - pass the
    meridian-perpendicular direction there to keep the orientation consistent
    with the rest of the point's strip instead of an azimuth-blind default.
    """
    if tip_offset is None:
        tip_offset = TOOL_TIP_OFFSET

    surface_point = np.asarray(surface_point, dtype=float)
    normal = np.asarray(normal, dtype=float)
    normal = normal / np.linalg.norm(normal)

    rotvec = normal_to_rotvec(tilt_normal_toward_vertical(normal, tilt_deg), y_hint=y_hint)
    tip_approach = surface_point + approach_distance * normal
    tip_press = surface_point - press_distance * normal

    approach_tcp = contact_to_tcp_position(tip_approach, rotvec, tip_offset)
    press_tcp = contact_to_tcp_position(tip_press, rotvec, tip_offset)

    return approach_tcp, rotvec, press_tcp, rotvec
