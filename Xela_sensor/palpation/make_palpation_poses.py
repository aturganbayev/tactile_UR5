#!/usr/bin/env python3
"""
Build a XELA-specific palpation pose grid from the existing cone touch poses,
with a shallower press depth.

WHY: data/cone_touch_poses.csv presses `press_distance` = 15 mm PAST the
phantom surface, open-loop. That was safe with the small Nano17 indenter tip,
but the XELA sensor presents a FLAT ~24x28 mm pad instead of a point. Force at
a given indentation scales roughly with contact area, so the same 15 mm can
produce far more force - against a module whose limit is ~10 N - and if the
Nano17 is out of the load path, nothing is measuring force to abort on.

This does NOT redo any cone geometry, ICP, or normals: the source CSV already
stores each pose's surface point (x,y,z) and outward normal (nx,ny,nz), so the
press TCP position is simply recomputed at a new depth along that same normal,
exactly as pose_utils.approach_and_press_poses() does. Everything else -
strips, orientations, approach poses - is copied through unchanged.

Nothing in the cone pipeline is modified; this writes a separate CSV.

Usage:
    python3 make_palpation_poses.py [press_mm] [-o OUTPUT_CSV]

    press_mm defaults to 3.0 (deliberately conservative for a first run -
    raise it only after seeing what force/response it actually produces).
    Pass --verify to check that regenerating at 15 mm reproduces the original
    press poses, which validates this script against the real generator.
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
sys.path.insert(0, REPO_ROOT)

import paths
from pose_utils import contact_to_tcp_position, TOOL_TIP_OFFSET

DEFAULT_OUTPUT = os.path.join(paths.DATA, "xela_palpation_poses.csv")
DEFAULT_PRESS_MM = 3.0


def rebuild(df, press_m, tip_offset=TOOL_TIP_OFFSET):
    """Recompute press_{x,y,z} for a new press depth along each surface normal."""
    out = df.copy()
    for i, row in df.iterrows():
        p = np.array([row["x"], row["y"], row["z"]], dtype=float)
        n = np.array([row["nx"], row["ny"], row["nz"]], dtype=float)
        n = n / np.linalg.norm(n)
        # Orientation is unchanged - reuse the rotvec already solved for this
        # pose (approach and press share it in the source generator).
        rotvec = np.array([row["press_rx"], row["press_ry"], row["press_rz"]],
                          dtype=float)
        tip_press = p - press_m * n
        tcp = contact_to_tcp_position(tip_press, rotvec, tip_offset)
        out.at[i, "press_x"], out.at[i, "press_y"], out.at[i, "press_z"] = tcp
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("press_mm", nargs="?", type=float, default=DEFAULT_PRESS_MM,
                    help="press depth past the surface, in mm")
    ap.add_argument("-o", "--output", default=DEFAULT_OUTPUT)
    ap.add_argument("-i", "--input", default=paths.CONE_TOUCH_POSES)
    ap.add_argument("--verify", action="store_true",
                    help="regenerate at 15mm and compare to the source poses")
    a = ap.parse_args()

    df = pd.read_csv(a.input)
    print(f"source: {a.input}  ({len(df)} poses)")

    if a.verify:
        chk = rebuild(df, 0.015)
        d = np.linalg.norm(
            chk[["press_x", "press_y", "press_z"]].values
            - df[["press_x", "press_y", "press_z"]].values, axis=1)
        print(f"VERIFY @15mm: max deviation from source press poses = "
              f"{d.max() * 1000:.4f} mm  -> "
              f"{'MATCH' if d.max() < 1e-6 else 'MISMATCH'}")
        return

    press_m = a.press_mm / 1000.0
    out = rebuild(df, press_m)

    # How far the press poses moved compared with the source grid.
    d = np.linalg.norm(
        out[["press_x", "press_y", "press_z"]].values
        - df[["press_x", "press_y", "press_z"]].values, axis=1)
    out.to_csv(a.output, index=False)
    print(f"press depth: {a.press_mm:.1f} mm (was 15.0 mm)")
    print(f"press poses pulled back by {d.min()*1000:.2f}-{d.max()*1000:.2f} mm")
    print(f"wrote {len(out)} poses -> {a.output}")
    print("\nRun it with:")
    print(f"  python3 execution/run_side_strip_poses.py --poses {a.output}")


if __name__ == "__main__":
    main()
