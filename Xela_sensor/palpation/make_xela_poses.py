#!/usr/bin/env python3
"""
Generate a pose grid sized for the XELA PAD, not for a point indenter.

WHY A SEPARATE GRID
data/cone_touch_poses.csv was built for the Nano17's 0.7 mm tip: 24 strips x
12 points, with consecutive poses 0.8 mm apart around a ring and 2.9-5.6 mm
apart down a strip. At that spacing a point tip samples 288 distinct spots.

The XELA pad is 24 x 28 mm - roughly half the width of the whole phantom
(which spans ~50 x 50 x 31 mm). Consecutive poses in that grid therefore
overlap almost completely: 288 presses would re-measure the same contact
patch over and over. Counting properly, the usable band has ~3900 mm2 of
surface against a 672 mm2 pad footprint, so there are only ~6 non-overlapping
placements, or ~12 at 50% overlap.

This generator places poses by PAD FOOTPRINT instead of by point count:
  * rings spaced SPACING_MM apart down the slant,
  * within each ring, ceil(circumference / SPACING_MM) poses,
so coverage adapts to the cone's taper - one pose at the apex where the ring
is tiny, more further down where it is wide.

WHAT IT REUSES, AND WHAT IT DOES NOT
Reuses the existing ICP registration and the cone-axis / meridian machinery
from pose_generation/generate_side_strip_poses.py - same surface, same normals,
same tilt-toward-vertical rule for wrist clearance. It does NOT re-probe the
phantom and does NOT touch TOOL_TIP_OFFSET, which is still the Nano17's 86 mm.
That offset is known to be ~2-4 mm short for this tool (contact occurred after
10-13 mm of approach travel where 15 mm was predicted), but under press-to-
count that error only changes how far the approach search travels before
finding contact, which the 20 mm cap absorbs.

A GEOMETRY CAVEAT WORTH KNOWING
The phantom is a steep cone: of 3000 measured surface points only 6 have
normals within 20 deg of vertical and 62 within 40 deg. Everything else is a
near-vertical wall. A rigid flat pad cannot conform to that - against a ~30 mm
radius the pad's edges stand ~2.5 mm off when its centre touches - so away
from the apex, contact is a BAND along the cone's slant rather than a full
face. That is geometry, not a fault, and it is why the earlier runs showed
load concentrated on a few taxels. Poses are still generated across the band
because a line contact still produces a usable response; just do not expect
the whole pad to load.

Output schema matches cone_touch_poses.csv exactly, so xela_palpation.py and
run_side_strip_poses.py both read it unchanged.

Usage:
    python3 make_xela_poses.py [--spacing 12] [--out CSV] [--dry-run]
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "pose_generation"))

import paths
from pose_utils import (approach_distance, press_distance, rotvec_to_matrix,
                        ur5_ik_near, UR5_IK_SEED, contact_to_tcp_position,
                        approach_and_press_poses,
                        MIN_ORIENTATION_TILT_DEG, MAX_ORIENTATION_TILT_DEG)
# Same axis/basis derivation as the point-tip generator, so both grids sit on
# the identical registered surface rather than two slightly different ones.
from generate_side_strip_poses import perpendicular_basis


def cone_axis(matrix_path):
    """Cone symmetry axis in base frame, from an ICP matrix.

    Same derivation as generate_side_strip_poses.cone_axis_from_calibration,
    but takes the matrix path so the XELA registration can be used instead of
    the Nano17 one.
    """
    T = np.loadtxt(matrix_path)
    axis = T[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return axis / np.linalg.norm(axis)

PAD_W_MM = 24.0        # XELA XR1944 short side
PAD_L_MM = 28.0        # long side
DEFAULT_SPACING_MM = 12.0   # ~50% overlap of the pad's short side
MIN_HEIGHT_FRACTION = 0.65  # Lower bound as a fraction of cone height.
                            #
                            # 0.45 was the point-tip value. 0.50 was not enough:
                            # in run 20260820_183618 the lowest ring sat at
                            # surface z = 84.4 mm and every one of its 13 poses
                            # "reached target" in 2.2-3.9 mm - the signature of
                            # hitting the rigid PLATFORM, not the phantom. That
                            # whole ring's data is contaminated.
                            #
                            # The constraint is not the contact height but the
                            # PAD EDGE: the pad reaches ~14 mm past its centre,
                            # so at the 14 deg tilt used low down its lower edge
                            # sits roughly 10 mm below whatever it touches. The
                            # ring at 93.8 mm behaved correctly (8/11 target,
                            # 8.9 mm depths), so 0.65 -> 92.8 mm reproduces
                            # known-good clearance. Lower it only after
                            # measuring the actual platform height.


def build(spacing_mm, surface_csv, matrix_path, approach_m=approach_distance,
          min_height=MIN_HEIGHT_FRACTION):
    df = pd.read_csv(surface_csv)
    pts = df[["x", "y", "z"]].to_numpy()
    normals = df[["nx", "ny", "nz"]].to_numpy()

    axis = cone_axis(matrix_path)
    u, v = perpendicular_basis(axis)
    origin = pts.mean(axis=0)
    rel = pts - origin
    t = rel @ axis
    perp = rel - np.outer(t, axis)
    radius = np.linalg.norm(perp, axis=1)
    tree = cKDTree(pts)

    t_max, t_min = t.max(), t.min()
    t_lower = t_min + min_height * (t_max - t_min)

    # Ring heights spaced by the pad footprint along the SLANT, not along the
    # axis - on a tapered cone those differ, and the slant is what the pad
    # actually lies against.
    band = (t >= t_lower)
    r_lo = float(np.median(radius[band & (t < t_lower + 0.002)])) if band.any() else 0.0
    r_hi = float(np.median(radius[band & (t > t_max - 0.002)])) if band.any() else 0.0
    axial = t_max - t_lower
    slant = float(np.hypot(axial, r_lo - r_hi))
    n_rings = max(2, int(round(slant * 1000.0 / spacing_mm)) + 1)
    t_targets = np.linspace(t_max, t_lower, n_rings)

    rows = []
    for ring, t_c in enumerate(t_targets):
        sel = band & (np.abs(t - t_c) < max(axial / (n_rings * 2), 0.002))
        r_c = float(np.median(radius[sel])) if sel.any() else 0.0

        circumference_mm = 2 * np.pi * r_c * 1000.0
        n_pts = max(1, int(round(circumference_mm / spacing_mm)))
        angles = np.linspace(0, 360, n_pts, endpoint=False)

        height_frac = (t_c - t_lower) / max(t_max - t_lower, 1e-9)
        tilt_deg = (MIN_ORIENTATION_TILT_DEG
                    + (MAX_ORIENTATION_TILT_DEG - MIN_ORIENTATION_TILT_DEG)
                    * (1.0 - height_frac))

        for a in angles:
            th = np.radians((a + 180) % 360 - 180)
            e_r = np.cos(th) * u + np.sin(th) * v
            e_theta = np.cross(e_r, axis)
            p = origin + t_c * axis + r_c * e_r

            # Measured normal at the nearest cloud point, projected into the
            # meridian plane and forced outward. Same rule as the point-tip
            # generator; abs() also covers the apex, where the outward normal
            # is the axis itself.
            n_meas = normals[tree.query(p)[1]]
            n_ax = abs(float(np.dot(n_meas, axis)) / np.linalg.norm(n_meas))
            n = n_ax * axis + np.sqrt(max(0.0, 1.0 - n_ax ** 2)) * e_r
            n /= np.linalg.norm(n)

            ap, (rx, ry, rz), pr, _ = approach_and_press_poses(
                p, n, approach_m, press_distance,
                tilt_deg=tilt_deg, y_hint=e_theta)

            rows.append({
                "strip": ring, "strip_angle_deg": round(float(a), 1),
                "tilt_deg": round(float(tilt_deg), 2),
                "x": p[0], "y": p[1], "z": p[2],
                "nx": n[0], "ny": n[1], "nz": n[2],
                "approach_x": ap[0], "approach_y": ap[1], "approach_z": ap[2],
                "approach_rx": rx, "approach_ry": ry, "approach_rz": rz,
                "press_x": pr[0], "press_y": pr[1], "press_z": pr[2],
                "press_rx": rx, "press_ry": ry, "press_rz": rz,
            })
    return pd.DataFrame(rows), slant * 1000.0, n_rings


def minimise_wrist(out, approach_m=approach_distance):
    """Choose each pose's spin ABOUT ITS OWN PRESS AXIS to keep wrist 3 near
    neutral. Returns the modified frame plus a before/after report.

    Two problems share one cause. normal_to_rotvec builds the tool frame as
    cross(world_Z, press_dir), so the wrist yaw follows the ring azimuth: a
    ring sweeping 0->360 deg spins J6 a full turn, and four rings accumulate
    ~1400 deg ALL IN ONE DIRECTION. Measured on the 36-pose grid, J6 ran
    -1401.9 to +4.8 deg. That is
        (a) past the UR5's +/-360 deg joint limit, and
        (b) four turns of cable wind-up on the sensor lead.

    Rotation about the press axis is FREE here - the pad is read as a whole-pad
    scalar, so its in-plane angle carries no information - which makes this
    fixable at generation time rather than by re-routing cable. For each pose
    the spin is searched over a full turn and the one whose IK puts J6 closest
    to zero is kept. Measured result on the same grid: J6 travel 1407 -> 69 deg,
    range -2.5..+2.3 deg.
    """
    from scipy.spatial.transform import Rotation as Rot

    seed = UR5_IK_SEED
    before, after = [], []
    new = out.copy()
    for i, row in out.iterrows():
        pos = np.array([row["approach_x"], row["approach_y"], row["approach_z"]])
        rv0 = np.array([row["approach_rx"], row["approach_ry"], row["approach_rz"]])
        q0, ok0 = ur5_ik_near(np.concatenate([pos, rv0]), seed)
        if ok0:
            before.append(q0[5])

        z = rotvec_to_matrix(rv0)[:, 2]
        best = None
        for phi in np.radians(np.arange(-180, 180, 5.0)):
            M = rotvec_to_matrix(rv0) @ Rot.from_rotvec(
                phi * np.array([0.0, 0.0, 1.0])).as_matrix()
            rv = Rot.from_matrix(M).as_rotvec()
            q, ok = ur5_ik_near(np.concatenate([pos, rv]), seed)
            if not ok:
                continue
            cost = abs(q[5])
            if best is None or cost < best[0]:
                best = (cost, q, rv)
        if best is None:          # leave the pose alone if nothing solved
            continue
        _, q, rv = best
        after.append(q[5])
        seed = q
        for c, val in zip(("approach_rx", "approach_ry", "approach_rz"), rv):
            new.at[i, c] = val
        for c, val in zip(("press_rx", "press_ry", "press_rz"), rv):
            new.at[i, c] = val
        # The press/approach POSITIONS were built from the old rotvec via
        # contact_to_tcp_position, so they must be rebuilt for the new one or
        # the tool tip lands somewhere else entirely.
        p_surf = np.array([row["x"], row["y"], row["z"]])
        n = np.array([row["nx"], row["ny"], row["nz"]])
        n = n / np.linalg.norm(n)
        ap = contact_to_tcp_position(p_surf + approach_m * n, rv)
        pr = contact_to_tcp_position(p_surf - press_distance * n, rv)
        for c, val in zip(("approach_x", "approach_y", "approach_z"), ap):
            new.at[i, c] = val
        for c, val in zip(("press_x", "press_y", "press_z"), pr):
            new.at[i, c] = val
    return new, np.degrees(np.array(before)), np.degrees(np.array(after))


def pad_corners(centre, rotvec):
    """The four corners of the pad face, in base frame.

    The pose's tool +Z is the press direction, so the pad face is the tool XY
    plane. Drawing the actual FOOTPRINT rather than a dot is the whole point
    of this plot: with a 24 x 28 mm pad on a ~50 mm phantom, whether poses
    overlap or leave gaps is invisible from centre markers alone.

    Assumes pad-u is along tool X and pad-v along tool Y, the same (unverified)
    mounting assumption as record_icp_points_xela.PAD_U_IN_TOOL.
    """
    R = rotvec_to_matrix(np.asarray(rotvec, dtype=float))
    hw, hl = PAD_W_MM / 2000.0, PAD_L_MM / 2000.0
    local = np.array([[-hw, -hl, 0.0], [hw, -hl, 0.0],
                      [hw, hl, 0.0], [-hw, hl, 0.0]])
    return np.asarray(centre, dtype=float) + local @ R.T


def plot(out, surface_csv, spacing_mm, path, show=False):
    import matplotlib
    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    surf = pd.read_csv(surface_csv)[["x", "y", "z"]].to_numpy()
    rings = sorted(out["strip"].unique())
    colors = plt.cm.viridis(np.linspace(0, 0.9, len(rings)))

    fig = plt.figure(figsize=(17, 8))

    ax = fig.add_subplot(121, projection="3d")
    ax.scatter(surf[:, 0], surf[:, 1], surf[:, 2], c="lightgray", s=2,
               alpha=0.25)
    for ri, r in enumerate(rings):
        g = out[out["strip"] == r]
        quads = [pad_corners([row["x"], row["y"], row["z"]],
                             [row["press_rx"], row["press_ry"], row["press_rz"]])
                 for _, row in g.iterrows()]
        ax.add_collection3d(Poly3DCollection(
            quads, facecolors=colors[ri], edgecolors=colors[ri],
            alpha=0.28, linewidths=0.6))
        ax.scatter(g["x"], g["y"], g["z"], color=colors[ri], s=18, zorder=6,
                   label=f"ring {int(r)}  ({len(g)} poses)")
        ax.quiver(g["x"], g["y"], g["z"], g["nx"], g["ny"], g["nz"],
                  length=0.008, linewidth=0.7, color=colors[ri])
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    c = (surf.max(axis=0) + surf.min(axis=0)) / 2
    h = (surf.max(axis=0) - surf.min(axis=0)).max() / 2
    ax.set_xlim(c[0] - h, c[0] + h); ax.set_ylim(c[1] - h, c[1] + h)
    ax.set_zlim(c[2] - h, c[2] + h)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=22, azim=-60)
    ax.set_title(f"pad footprints ({PAD_W_MM:.0f} x {PAD_L_MM:.0f} mm)")
    ax.legend(loc="upper right", fontsize=7)

    ax2 = fig.add_subplot(122)
    ax2.scatter(surf[:, 0], surf[:, 1], c="lightgray", s=2, alpha=0.25)
    for ri, r in enumerate(rings):
        g = out[out["strip"] == r]
        for _, row in g.iterrows():
            q = pad_corners([row["x"], row["y"], row["z"]],
                            [row["press_rx"], row["press_ry"], row["press_rz"]])
            ax2.fill(q[:, 0], q[:, 1], color=colors[ri], alpha=0.22, lw=0.5,
                     edgecolor=colors[ri])
        ax2.scatter(g["x"], g["y"], color=colors[ri], s=14, zorder=5)
    ax2.set_xlabel("X (m)"); ax2.set_ylabel("Y (m)")
    ax2.set_aspect("equal")
    ax2.set_title("top view - overlap between footprints")

    d = cKDTree(out[["x", "y", "z"]].to_numpy()).query(
        out[["x", "y", "z"]].to_numpy(), k=2)[0][:, 1] * 1000.0
    fig.suptitle(f"XELA pose grid - {len(out)} poses, spacing {spacing_mm:.0f} mm, "
                 f"median NN {np.median(d):.1f} mm "
                 f"({100*(1-np.median(d)/PAD_W_MM):.0f}% pad overlap)")

    os.makedirs(paths.FIGURES, exist_ok=True)
    plt.savefig(path, dpi=200, bbox_inches="tight")
    print(f"saved plot -> {path}")
    if show:
        plt.show()
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spacing", type=float, default=DEFAULT_SPACING_MM,
                    help="pose spacing in mm (default %(default)s = ~50%% "
                         "overlap of the pad's 24 mm short side)")
    ap.add_argument("--xela", action="store_true",
                    help="use the XELA-probed registration "
                         "(surface_points_base_xela.csv) instead of the "
                         "Nano17 one")
    ap.add_argument("--out", default=None)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--min-height", type=float, default=MIN_HEIGHT_FRACTION,
                    help="lowest ring as a fraction of cone height (default "
                         "%(default)s; 0 = base, 1 = apex). Raise it if the pad "
                         "gets near the platform.")
    ap.add_argument("--approach", type=float, default=10.0,
                    help="mm from the surface to park the pad before "
                         "descending (default %(default)s, was 15). Every mm "
                         "here is a mm the contact search has to travel at "
                         "every pose. Do NOT go below ~8: ICP error is "
                         "1.5-2.2 mm and TOOL_TIP_OFFSET is ~2-4 mm stale, so "
                         "a nominal 5 mm gap can be zero on some poses - and "
                         "starting already in contact corrupts the baseline.")
    ap.add_argument("--no-wrist-opt", action="store_true",
                    help="skip the wrist-3 minimisation. The raw grid winds J6 "
                         "~1400 deg in one direction, which both exceeds the "
                         "joint limit and wraps the sensor cable.")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--show", action="store_true",
                    help="open the plot window as well as saving it")
    a = ap.parse_args()

    if a.xela:
        surface, matrix = paths.SURFACE_POINTS_BASE_XELA, paths.ICP_MATRIX_XELA
        default_out = os.path.join(paths.DATA, "xela_poses.csv")
    else:
        surface, matrix = paths.SURFACE_POINTS_BASE, paths.ICP_MATRIX
        default_out = os.path.join(paths.DATA, "xela_poses_nano17reg.csv")
    if a.out is None:
        a.out = default_out
    for f in (surface, matrix):
        if not os.path.exists(f):
            sys.exit(f"missing {f}" + ("  - run calibrate_icp.py --xela first"
                                       if a.xela else ""))
    print(f"registration: {os.path.basename(surface)}")
    out, slant_mm, n_rings = build(a.spacing, surface, matrix,
                                   a.approach / 1000.0, a.min_height)
    p = out[["x", "y", "z"]].to_numpy()
    print(f"pad {PAD_W_MM:.0f} x {PAD_L_MM:.0f} mm, spacing {a.spacing:.1f} mm, "
          f"approach {a.approach:.0f} mm, min-height {a.min_height:.2f}")
    print(f"usable slant length : {slant_mm:.1f} mm  -> {n_rings} rings")
    print(f"poses generated     : {len(out)}")
    print("  ring  n_poses  z (mm)   tilt")
    for r, g in out.groupby("strip"):
        print(f"   {int(r):3d}   {len(g):5d}    {g['z'].iloc[0]*1000:6.1f}"
              f"   {g['tilt_deg'].iloc[0]:5.1f}")
    # Nearest-neighbour spacing actually achieved, which is what decides
    # whether presses overlap the way we intended.
    if len(p) > 1:
        d = cKDTree(p).query(p, k=2)[0][:, 1] * 1000.0
        print(f"\nnearest-neighbour spacing: {d.min():.1f} - {d.max():.1f} mm "
              f"(median {np.median(d):.1f})")
        print(f"overlap of a {PAD_W_MM:.0f} mm pad at median spacing: "
              f"{100 * (1 - np.median(d) / PAD_W_MM):.0f}%")
    if not a.no_wrist_opt:
        print("\nminimising wrist-3 rotation ...")
        out, j6_before, j6_after = minimise_wrist(out, a.approach / 1000.0)
        if len(j6_before) and len(j6_after):
            tb = np.abs(np.diff(j6_before)).sum()
            ta = np.abs(np.diff(j6_after)).sum()
            print(f"  J6 range  : {j6_before.min():8.1f}..{j6_before.max():7.1f} "
                  f"-> {j6_after.min():6.1f}..{j6_after.max():5.1f} deg")
            print(f"  J6 travel : {tb:8.0f} -> {ta:6.0f} deg "
                  f"({tb/360:.1f} -> {ta/360:.1f} turns of cable wind-up)")

    if a.dry_run:
        print("\n--dry-run: nothing written.")
        return
    out.to_csv(a.out, index=False)
    print(f"\nwrote {len(out)} poses -> {a.out}")
    if not a.no_plot:
        stem = "xela_poses" + ("" if a.xela else "_nano17reg")
        plot(out, surface, a.spacing,
             os.path.join(paths.FIGURES, stem + ".png"), show=a.show)
    print(f"\nCheck reachability, then run:")
    print(f"  python3 Xela_sensor/palpation/xela_palpation.py --poses {a.out} --check")


if __name__ == "__main__":
    main()
