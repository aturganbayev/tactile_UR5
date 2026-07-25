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

Measured (base frame, robot):
    point 0 raw EE pose (mm, rad)  = POINT0_EE_POSE_MM - empirically confirmed
        to sit only ~0.1-0.15mm above true contact (force appears around
        z=131.35-131.4 vs. this pose's z=131.5).
    col step (0 -> 1, i.e. +1 col) = -X, 5 mm   (confirmed via XELA response)
    row step (0 -> 4, i.e. +1 row) = +Y, 5 mm

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
# Confirmed against XELA response data (see module docstring):
ROW_AXIS = np.array([0.0, 1.0, 0.0])    # +Y -> taxel index +4 (next row, 0->4)
COL_AXIS = np.array([-1.0, 0.0, 0.0])   # -X -> taxel index +1 (next col, 0->1)

HOVER_CLEARANCE_M = 0.0015  # safe non-contact standoff above each taxel -
                             # kept small so the descent to contact is short
                             # (the sensor is flat and point0 is well measured)


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