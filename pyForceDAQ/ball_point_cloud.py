#!/usr/bin/env python3
"""
3D point cloud + sphere fit of the embedded ball, from press recordings.

Every press probes along a known ray into the tissue. Comparing a phantom
press's force-vs-indentation curve against the EMPTY phantom's curve at the
same grid location, the depth where the two curves start to diverge is where
the tip began to feel the ball - and the tip's measured 3D position at that
instant is a point on (just outside) the ball's boundary. Collecting those
divergence points over all presses that show a clear ball influence yields a
partial point cloud of the ball's outward-facing surface; a least-squares
sphere fit turns it into a center + radius estimate.

No prior knowledge of the ball (size, depth, on/off axis) is used. Bias to
keep in mind: the silicone shell compresses ahead of the tip, so the points
sit a little outside the true boundary and the fitted radius is slightly
overestimated.

Usage:
    python3 ball_point_cloud.py PHANTOM_DIR [REFERENCE_DIR]

REFERENCE_DIR defaults to the "empty" folder next to PHANTOM_DIR (the data
directory keeps the empty-phantom run there). Works from any working
directory - paths are taken from the arguments, and outputs go into
PHANTOM_DIR.

Outputs (into PHANTOM_DIR, named like the batch plots):
    <folder>_ball_cloud.png    - static 3D view
    <folder>_ball_cloud.html   - interactive 3D view (open in a browser)
"""

import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
from stiffness_map import load_run, analyze_run, MATCH_TOL_MM

# A press contributes a boundary point only if the phantom's curve ends up
# clearly stiffer than the reference: the force excess at full depth must
# reach this many Newtons. Below that the "divergence depth" is noise.
MIN_EXCESS_N = 0.8
# The boundary point is taken where the force excess first reaches this
# fraction of its final value - early enough to sit near first ball contact,
# late enough to be above noise.
ONSET_FRACTION = 0.3
GRID_STEP_MM = 0.2
MIN_COMMON_DEPTH_MM = 4.0
MIN_POINTS_FOR_FIT = 8


def match_pairs(presses_p, presses_r):
    tree = cKDTree(np.array([p["tip"] for p in presses_r]))
    pairs = []
    for p in presses_p:
        dist, j = tree.query(p["tip"])
        if dist * 1000.0 <= MATCH_TOL_MM:
            pairs.append((p, presses_r[j]))
    return pairs


def _monotonic(d, f, tips):
    """Sort a loading curve by depth so it can be interpolated."""
    order = np.argsort(d)
    return d[order], f[order], tips[order]


def boundary_point(pa, pb):
    """Ball-boundary 3D point from one matched press pair, or None.

    pa = phantom press, pb = reference (empty) press at the same location.
    """
    da, fa, ta = _monotonic(*pa["curve"], pa["load_tip"])
    db, fb, _ = _monotonic(*pb["curve"], pb["load_tip"])
    d_max = min(da.max(), db.max())
    if d_max < MIN_COMMON_DEPTH_MM:
        return None

    grid = np.arange(0.0, d_max, GRID_STEP_MM)
    excess = np.interp(grid, da, fa) - np.interp(grid, db, fb)
    excess_end = excess[-3:].mean()
    if excess_end < MIN_EXCESS_N:
        return None   # no clear ball influence on this press

    idx = np.argmax(excess >= ONSET_FRACTION * excess_end)
    d_star = grid[idx]
    # tip position at the divergence depth, interpolated per coordinate
    point = np.array([np.interp(d_star, da, ta[:, i]) for i in range(3)])
    return point, float(d_star), float(excess_end)


def fit_sphere(P):
    """Algebraic least-squares sphere fit. Returns center, radius, rms."""
    A = np.hstack([2.0 * P, np.ones((len(P), 1))])
    b = (P ** 2).sum(axis=1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    c = sol[:3]
    r = float(np.sqrt(sol[3] + c @ c))
    rms = float(np.sqrt(np.mean((np.linalg.norm(P - c, axis=1) - r) ** 2)))
    return c, r, rms


def load_surface():
    path = paths.SURFACE_POINTS_BASE
    if not os.path.exists(path):
        return None
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return np.array([[float(r["x"]), float(r["y"]), float(r["z"])] for r in rows])


def locate_ball(phantom, presses_p, ref, presses_r, out_dir, tag=None):
    """Boundary point cloud + sphere fit for one phantom run against the
    empty-phantom reference. Runs come from load_run() + analyze_run().
    Saves <tag>_ball_cloud.png/.html into out_dir. Returns (center, radius,
    rms, n_points), or None when no ball influence is detected."""
    if tag is None:
        tag = phantom["name"]

    pairs = match_pairs(presses_p, presses_r)
    print(f"  matched {len(pairs)} presses")

    points, onsets, excesses = [], [], []
    for pa, pb in pairs:
        hit = boundary_point(pa, pb)
        if hit is not None:
            points.append(hit[0]); onsets.append(hit[1]); excesses.append(hit[2])
    points = np.array(points)
    print(f"  {len(points)} presses show clear ball influence "
          f"(force excess >= {MIN_EXCESS_N} N)")
    if len(points) < MIN_POINTS_FOR_FIT:
        print("  no ball detected - not enough boundary points for a sphere "
              "fit (inclusion absent, too soft, or too deep for the stroke).")
        return None

    center, radius, rms = fit_sphere(points)
    # axis center for the azimuth readout (vertical cone axis)
    all_tips = np.array([p["tip"] for p in presses_p])
    axis_xy = all_tips[:, :2].mean(axis=0)
    off_axis = np.linalg.norm(center[:2] - axis_xy) * 1000.0
    az = np.degrees(np.arctan2(center[1] - axis_xy[1], center[0] - axis_xy[0]))

    print(f"\nSphere fit over {len(points)} boundary points:")
    print(f"  center  = [{center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}] m")
    print(f"  height  = {center[2]*1000:.1f} mm   "
          f"off-axis = {off_axis:.1f} mm (toward az {az:.0f} deg)"
          if off_axis > 3 else
          f"  height  = {center[2]*1000:.1f} mm   on-axis (offset {off_axis:.1f} mm)")
    print(f"  radius  = {radius*1000:.1f} mm (biased high by shell compression)")
    print(f"  fit rms = {rms*1000:.1f} mm")

    surface = load_surface()

    # ---------------- static PNG ---------------- #
    fig = plt.figure(figsize=(9, 9))
    ax = fig.add_subplot(111, projection="3d")
    if surface is not None:
        S = surface[::4]
        ax.scatter(S[:, 0], S[:, 1], S[:, 2], c="dimgray", s=2, alpha=0.15,
                   label="Breast surface")
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c="red", s=25,
               label=f"Ball boundary points ({len(points)})")
    u, v = np.mgrid[0:2*np.pi:30j, 0:np.pi:20j]
    ax.plot_wireframe(center[0] + radius*np.cos(u)*np.sin(v),
                      center[1] + radius*np.sin(u)*np.sin(v),
                      center[2] + radius*np.cos(v),
                      color="royalblue", alpha=0.4, linewidth=0.6)
    ax.scatter(*center, c="royalblue", s=60, marker="x")
    ref_pts = surface if surface is not None else points
    centers = (ref_pts.max(axis=0) + ref_pts.min(axis=0)) / 2
    half = (ref_pts.max(axis=0) - ref_pts.min(axis=0)).max() / 2
    ax.set_xlim(centers[0]-half, centers[0]+half)
    ax.set_ylim(centers[1]-half, centers[1]+half)
    ax.set_zlim(centers[2]-half, centers[2]+half)
    ax.set_box_aspect((1, 1, 1))
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)"); ax.set_zlabel("Z (m)")
    ax.set_title(f"{tag}: fitted ball  r={radius*1000:.1f} mm  "
                 f"z={center[2]*1000:.1f} mm  (rms {rms*1000:.1f} mm)")
    ax.legend(loc="upper right", fontsize=8)
    png = os.path.join(out_dir, f"{tag}_ball_cloud.png")
    fig.savefig(png, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved {png}")

    # ---------------- interactive HTML ---------------- #
    try:
        import plotly.graph_objects as go
    except ImportError:
        print("  [warn] plotly not installed - skipping the interactive HTML.")
        return center, radius, rms, len(points)
    traces = []
    if surface is not None:
        S = surface[::3]
        traces.append(go.Scatter3d(x=S[:, 0], y=S[:, 1], z=S[:, 2],
                                   mode="markers",
                                   marker=dict(size=2, color="dimgray",
                                               opacity=0.35, symbol="circle"),
                                   name="Breast surface", hoverinfo="skip"))
    traces.append(go.Scatter3d(x=points[:, 0], y=points[:, 1], z=points[:, 2],
                               mode="markers",
                               marker=dict(size=4, color="red", symbol="circle"),
                               name="Ball boundary points"))
    traces.append(go.Surface(x=center[0] + radius*np.cos(u)*np.sin(v),
                             y=center[1] + radius*np.sin(u)*np.sin(v),
                             z=center[2] + radius*np.cos(v),
                             opacity=0.35, showscale=False,
                             colorscale=[[0, "royalblue"], [1, "royalblue"]],
                             name="Fitted ball"))
    figly = go.Figure(traces)
    figly.update_layout(
        title=f"{tag}: fitted ball  r={radius*1000:.1f} mm  "
              f"z={center[2]*1000:.1f} mm  (rms {rms*1000:.1f} mm)",
        scene=dict(xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
                   aspectmode="data", dragmode="orbit"),
    )
    html = os.path.join(out_dir, f"{tag}_ball_cloud.html")
    figly.write_html(html, include_plotlyjs="cdn")
    print(f"Saved {html}")

    return center, radius, rms, len(points)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    phantom_dir = sys.argv[1]
    if len(sys.argv) > 2:
        ref_dir = sys.argv[2]
    else:
        # default: the empty-phantom run stored next to the phantom folder
        ref_dir = os.path.join(os.path.dirname(os.path.normpath(phantom_dir)),
                               "empty")
        if not os.path.isdir(ref_dir):
            sys.exit(f"No reference folder at {ref_dir} - pass REFERENCE_DIR "
                     "explicitly.")
        print(f"Using reference: {ref_dir}")
    tag = os.path.basename(os.path.normpath(phantom_dir)).replace(" ", "_")

    print("Loading runs ...")
    phantom = load_run(phantom_dir)
    presses_p = analyze_run(phantom)
    ref = load_run(ref_dir)
    presses_r = analyze_run(ref)
    locate_ball(phantom, presses_p, ref, presses_r, out_dir=phantom_dir, tag=tag)


if __name__ == "__main__":
    main()
