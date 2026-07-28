"""
Physical taxel grid geometry for this specific XR1944 sensor mount, derived
from touch-probing (see conversation notes / memory) rather than nominal
datasheet dimensions - those were shown to be unreliable (the first point-0
touch attempt wasn't actually touching the sensor).

Layout:
    taxel_index = 4*row + col
    col (the +1 index direction, 0->1->2->3 within a row) runs along -X
    row (the +4 index direction, 0->4->8->12) runs along +Y

Axis mapping CONFIRMED against XELA response data (2026-07-25): with the
indenter pressed at each grid position and the XELA taxel that lights up
recorded at low force, the -X neighbour of taxel 0 responds as XELA taxel 1
(NOT 4), the next -X step as taxel 2, and the +Y neighbour as taxel 4. An
earlier guess had row/col swapped (-X=+4, +Y=+1); the sensor data overrides
it. This makes plain range(16) iterate X-first (taxels 0-3 run down -X) and
label sequentially, matching the XELA data order.

SURFACE TILT (measured 2026-07-28, run 20260728_122448): a plane fitted to
the 16 touch-probed contact heights shows the sensor face is NOT level - it
tilts 2.45 deg from horizontal (0.93 mm drop across the 15x15 mm grid, mostly
along +Y). Two consequences, both handled here:

  1. The grid steps are taken IN that tilted plane, so each taxel's hover pose
     sits at its own correct height. Previously every hover shared one height,
     so the low corner needed several extra mm of descent while the high
     corner was already nearly touching.

  2. The press direction is aligned to the measured surface normal instead of
     the tool axis recorded at point 0. Those differ by 3.75 deg, which over
     the ~1.8 mm working indentation dragged the tip ~0.12 mm sideways across
     the skin. That small drag matters: the taxels' shear (X/Y) channels are
     far more sensitive to lateral motion than the normal (Z) channel is to
     depth, and in run 20260728_122448 the shear response swamped the normal
     response on nearly every taxel (mean |shear|/|normal| >> 100%), with the
     peak response migrating to the +Y neighbour above ~1 N.

*** TOOL_TIP_OFFSET DEPENDENCY - READ BEFORE TRUSTING A RUN ***
Aligning the press direction rotates the tool by 3.75 deg about the tip. To
keep the tip on the SAME contact point, the TCP must shift by ~5.6 mm - more
than one taxel pitch - and that shift is computed from
pose_utils.TOOL_TIP_OFFSET (86 mm). That value came from the cone-press
tooling; if it is stale for the currently fitted indenter, every press lands
off-target by roughly 0.066 * (offset error).

So VERIFY before a full sweep: run xela_start_pose.py 0 and check the tip
still sits over taxel 0. If it is off, either correct TOOL_TIP_OFFSET, or set
ALIGN_PRESS_TO_SURFACE = False below to fall back to the original
(unrotated, tip-offset-independent) press direction.

The cleanest permanent fix is to re-probe taxel 0 with the tool already at
PRESS_ROTVEC and store that as POINT0_EE_POSE_MM: measuring in the same
orientation that is used to press makes the tip offset cancel out exactly
again, removing this dependency.

Only reads from pose_utils - does not modify anything there.
"""

import os
import sys

import numpy as np
from scipy.spatial.transform import Rotation as Rot

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
sys.path.insert(0, REPO_ROOT)

from pose_utils import rotvec_to_matrix, TOOL_TIP_OFFSET

# --------------------------------------------------------------------------- #
#                          MEASURED REFERENCE (hardcoded)                     #
# --------------------------------------------------------------------------- #

# Raw EE/TCP pose at taxel 0 (mm, rad), measured in the ORIGINAL tool
# orientation. Re-measure and update this if the sensor or mount ever moves.
#
# The z here is 128.86, NOT the originally probed 131.5: the 16 contact heights
# recorded in run 20260728_122448 showed the real surface sits a consistent
# 2.64 mm lower (spread only +/-0.37 mm across the grid, i.e. a genuine offset,
# not noise - the tilt is handled separately by SURFACE_NORMAL). Whether the
# mount settled or the swapped indenter is shorter, the old value made every
# hover ~4.1 mm above contact instead of HOVER_CLEARANCE_M, wasting most of the
# approach. With this correction the predicted contact points match the
# measured ones to ~0.1 mm laterally and ~0 mm in height.
POINT0_EE_POSE_MM = np.array(
    [-2.039, -531.009, 128.86, -2.200144, 2.199824, -0.000074])

PITCH_M = 0.005          # 5 mm between adjacent taxel centers, both axes
# Confirmed against XELA response data (see module docstring):
ROW_AXIS = np.array([0.0, 1.0, 0.0])    # +Y -> taxel index +4 (next row, 0->4)
COL_AXIS = np.array([-1.0, 0.0, 0.0])   # -X -> taxel index +1 (next col, 0->1)

# Outward normal of the sensor face (base frame), from the plane fitted to the
# 16 measured contact heights: z = +0.00637x -0.04235y + 106.388 (mm).
# Residual RMS 0.215 mm - quantisation-limited by the 0.3 mm approach step.
SURFACE_NORMAL = np.array([-0.00637, 0.04231, 0.99908])

# Press along SURFACE_NORMAL (True) or along the tool axis recorded at point 0
# (False). See the TOOL_TIP_OFFSET warning in the module docstring.
ALIGN_PRESS_TO_SURFACE = True

HOVER_CLEARANCE_M = 0.0015  # non-contact standoff above each taxel, along the
                             # surface normal. Small because the tilt-aware
                             # grid now puts every hover at its own correct
                             # height, so the descent to contact is short.


def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def _press_orientation():
    """(rotvec, rotation matrix) for the press orientation.

    Uses the MINIMAL rotation that swings the tool +Z onto the inward surface
    normal, so the wrist yaw recorded at point 0 is preserved (a full
    normal_to_rotvec() rebuild would pick an unrelated yaw and swing the wrist
    far more than the 3.75 deg actually needed).
    """
    R0 = rotvec_to_matrix(POINT0_EE_POSE_MM[3:])
    if not ALIGN_PRESS_TO_SURFACE:
        return POINT0_EE_POSE_MM[3:].copy(), R0

    z_cur = R0[:, 2]              # current press direction (tool +Z, into surface)
    z_des = -_unit(SURFACE_NORMAL)  # desired press direction (into surface)
    axis = np.cross(z_cur, z_des)
    s = np.linalg.norm(axis)
    if s < 1e-9:                   # already aligned
        return POINT0_EE_POSE_MM[3:].copy(), R0
    angle = np.arccos(np.clip(np.dot(z_cur, z_des), -1.0, 1.0))
    R_new = Rot.from_rotvec(axis / s * angle).as_matrix() @ R0
    return Rot.from_matrix(R_new).as_rotvec(), R_new


PRESS_ROTVEC, _R_PRESS = _press_orientation()


def _taxel0_contact_m():
    """Contact point of taxel 0 (base frame, metres), from the measured pose
    in the orientation it was measured in."""
    R0 = rotvec_to_matrix(POINT0_EE_POSE_MM[3:])
    return POINT0_EE_POSE_MM[:3] / 1000.0 + R0 @ TOOL_TIP_OFFSET


def _plane_axes():
    """Grid step directions projected into the tilted sensor plane."""
    n = _unit(SURFACE_NORMAL)
    row = ROW_AXIS - np.dot(ROW_AXIS, n) * n
    col = COL_AXIS - np.dot(COL_AXIS, n) * n
    return _unit(row), _unit(col)


def taxel_row_col(index):
    if not 0 <= index < 16:
        raise ValueError(f"taxel index must be 0-15, got {index}")
    return divmod(index, 4)   # (row, col)


def taxel_contact_point(index):
    """Surface contact point of a taxel (base frame, metres), stepped from
    taxel 0 within the measured tilted plane."""
    row, col = taxel_row_col(index)
    row_dir, col_dir = _plane_axes()
    return _taxel0_contact_m() + row * PITCH_M * row_dir + col * PITCH_M * col_dir


def taxel_tcp_position(index):
    """(TCP position in metres, rotvec) placing the tool tip on the taxel's
    contact point at the press orientation."""
    return taxel_contact_point(index) - _R_PRESS @ TOOL_TIP_OFFSET, PRESS_ROTVEC


def taxel_hover_pose(index, clearance_m=HOVER_CLEARANCE_M):
    """TCP pose (metres, rad) hovering clearance_m above the taxel along the
    surface normal, ready to hand to send_movel(). The press session's approach
    phase finds exact contact from here via force feedback, so this only needs
    to be in the right neighbourhood, not perfectly precise.
    """
    xyz, rotvec = taxel_tcp_position(index)
    return np.concatenate([xyz + clearance_m * _unit(SURFACE_NORMAL), rotvec])


if __name__ == "__main__":
    # Quick sanity printout of the derived grid.
    print(f"ALIGN_PRESS_TO_SURFACE = {ALIGN_PRESS_TO_SURFACE}")
    print(f"press rotvec = [{PRESS_ROTVEC[0]:.6f}, {PRESS_ROTVEC[1]:.6f}, "
          f"{PRESS_ROTVEC[2]:.6f}]")
    print(f"press direction (tool +Z) = {np.round(_R_PRESS[:, 2], 5)}\n")
    for i in range(16):
        r, c = taxel_row_col(i)
        xyz, _ = taxel_tcp_position(i)
        print(f"taxel {i:2d} (row {r}, col {c}): TCP position = "
              f"[{xyz[0]*1000:7.2f}, {xyz[1]*1000:7.2f}, "
              f"{xyz[2]*1000:7.2f}] mm")
