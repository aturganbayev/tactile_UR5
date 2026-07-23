"""
Physical taxel grid geometry for this specific XR1944 sensor mount, derived
from touch-probing (see conversation notes / memory) rather than nominal
datasheet dimensions - those were shown to be unreliable (the first point-0
touch attempt wasn't actually touching the sensor).

Layout (2019 hardware manual, Fig. 1 - confirmed against measured directions):
    row 0 (point 0's row): [ 3  2  1  0]   (cable side, taxel 0 rightmost)
    row 1:                 [ 7  6  5  4]
    row 2:                 [11 10  9  8]
    row 3:                 [15 14 13 12]
    taxel_index = 4*row + col   (col 0 = point 0's column)

Measured (base frame, robot):
    point 0 raw EE pose (mm, rad)  = POINT0_EE_POSE_MM - empirically confirmed
        to sit only ~0.1-0.15mm above true contact (force appears around
        z=131.35-131.4 vs. this pose's z=131.5).
    row step (0 -> 4, i.e. +1 row) = -X, 5 mm
    col step (0 -> 1, i.e. +1 col) = +Y, 5 mm

Note: this works entirely in raw TCP-pose space, not surface-contact space.
The sensor tip offset (pose_utils.TOOL_TIP_OFFSET, used for the cone presses)
would be added when converting to a contact point and then immediately
subtracted again when converting back to a TCP pose for the SAME fixed
orientation used throughout this grid - the two cancel out exactly, so it's
left out entirely rather than round-tripped through it for no effect.

Only reads from pose_utils - does not modify anything there.
"""

import os
import sys

import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
sys.path.insert(0, REPO_ROOT)

from pose_utils import rotvec_to_matrix

# --------------------------------------------------------------------------- #
#                          MEASURED REFERENCE (hardcoded)                     #
# --------------------------------------------------------------------------- #

# Raw EE/TCP pose at taxel 0 (mm, rad). Re-measure and update this if the
# sensor or mount ever moves.
POINT0_EE_POSE_MM = np.array(
    [-2.039, -531.009, 131.5, -2.200144, 2.199824, -0.000074])

PITCH_M = 0.005          # 5 mm between adjacent taxel centers, both axes
ROW_AXIS = np.array([-1.0, 0.0, 0.0])   # base-frame direction of +1 row (0->4)
COL_AXIS = np.array([0.0, 1.0, 0.0])    # base-frame direction of +1 col (0->1)

HOVER_CLEARANCE_M = 0.003   # safe non-contact standoff above each taxel


def taxel_row_col(index):
    if not 0 <= index < 16:
        raise ValueError(f"taxel index must be 0-15, got {index}")
    return divmod(index, 4)   # (row, col)


def taxel_tcp_position(index):
    """Raw TCP position (base frame, metres) directly above/at a taxel,
    at the same height as the measured point-0 reading - i.e. the point 0
    reading + the grid step for this taxel, no sensor-tip offset involved
    (see module docstring for why it cancels out)."""
    row, col = taxel_row_col(index)
    point0_xyz = POINT0_EE_POSE_MM[:3] / 1000.0
    rotvec = POINT0_EE_POSE_MM[3:]
    offset = row * PITCH_M * ROW_AXIS + col * PITCH_M * COL_AXIS
    return point0_xyz + offset, rotvec


def taxel_hover_pose(index, clearance_m=HOVER_CLEARANCE_M):
    """TCP pose (metres, rad) hovering clearance_m above the taxel's surface,
    ready to hand to send_movel(). The approach phase in the press session
    finds exact contact from here via force feedback, so this only needs to
    be in the right neighbourhood, not perfectly precise.
    """
    xyz, rotvec = taxel_tcp_position(index)
    tool_z_axis = rotvec_to_matrix(rotvec)[:, 2]   # points INTO the surface
    hover_xyz = xyz - clearance_m * tool_z_axis     # back off along +normal
    return np.concatenate([hover_xyz, rotvec])


if __name__ == "__main__":
    # Quick sanity printout of the derived grid.
    for i in range(16):
        r, c = taxel_row_col(i)
        xyz, _ = taxel_tcp_position(i)
        print(f"taxel {i:2d} (row {r}, col {c}): TCP position = "
              f"[{xyz[0]*1000:7.2f}, {xyz[1]*1000:7.2f}, "
              f"{xyz[2]*1000:7.2f}] mm")
