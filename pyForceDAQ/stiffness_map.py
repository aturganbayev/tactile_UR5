#!/usr/bin/env python3
"""
Stiffness-based tumor localization from recorded press data (offline).

Instead of the peak force per press (a single scalar that mixes tissue
stiffness with calibration depth error), this extracts the full
force-vs-indentation curve of every press from a <stamp>_trajectory.csv and
maps three features over the cone surface:

  * k_full  - linear stiffness dF/dx over the whole loading phase (N/mm)
  * k_late  - stiffness over the upper half of the force range (N/mm); a hard
              inclusion stiffens the END of the curve most, so this is the
              more sensitive feature
  * relax   - fractional force decay during the hold at max depth; bulk
              silicone stress-relaxes differently than a press bearing on a
              hard ball

Given a second (reference) recording of an empty phantom pressed over the
same pose grid, it also plots the DIFFERENTIAL map phantom-minus-reference:
the mount, the axisymmetric baseline, and any calibration artifact are common
to both runs and cancel, so an embedded stiff inclusion should stand out as a
compact positive blob.

Usage:
    python3 stiffness_map.py PHANTOM_DIR [REFERENCE_DIR]

Each directory must contain one <stamp>_trajectory.csv (the matching
_presses.csv is not needed - presses are re-detected from the force signal).
Figures are written to the repo's figures/ directory.
"""

import glob
import os
import sys

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
from pose_utils import TOOL_TIP_OFFSET

# Press re-detection. Same hysteresis idea as record_cone_press.py but with no
# refractory window (offline we'd rather see every contact) - instead,
# non-press blips (retract rebounds, grazes) are dropped by the minimum peak /
# depth / duration filters below.
ON_N = 0.3
OFF_N = 0.15
DEBOUNCE_S = 0.5
MIN_PEAK_N = 0.35
MIN_DEPTH_MM = 2.0     # rebounds barely move the tip; real presses travel ~10 mm
MIN_DUR_S = 0.5

# Presses of the two runs are matched by contact position; the pose grid is
# identical between runs, so anything further apart than this is unmatched.
MATCH_TOL_MM = 6.0

HOLD_BAND_MM = 0.5     # samples within this of max depth count as the hold

# Same speed gate as record_cone_press.py: a real press contains the ~1 s
# stationary hold, while transit grazes / silicone-adhesion blips happen
# entirely on the move. An episode with no in-contact sample slower than this
# is discarded.
HOLD_SPEED_MS = 0.01


def find_trajectory(directory):
    hits = sorted(glob.glob(os.path.join(directory, "*_trajectory.csv")))
    if not hits:
        sys.exit(f"Error: no *_trajectory.csv in {directory}")
    if len(hits) > 1:
        print(f"  [warn] {len(hits)} trajectory files in {directory}, "
              f"using {os.path.basename(hits[-1])}")
    return hits[-1]


def load_run(directory):
    """Trajectory samples with the sensor tip position per sample (base frame)."""
    csv_path = find_trajectory(directory)
    df = pd.read_csv(csv_path)
    xyz = df[["x", "y", "z"]].to_numpy()
    rotvecs = df[["rx", "ry", "rz"]].to_numpy()
    # Vectorized tcp_pose_to_contact over the whole file
    tips = xyz + R.from_rotvec(rotvecs).apply(np.broadcast_to(TOOL_TIP_OFFSET, xyz.shape))
    return {
        "name": os.path.basename(os.path.normpath(directory)),
        "t": df["t"].to_numpy(),
        "tip": tips,
        "fmag": df["Fmag"].to_numpy(),
        "speed": df["speed"].to_numpy(),
    }


def detect_episodes(t, fmag):
    """Contact episodes as (i_start, i_end) index ranges, hysteresis+debounce."""
    episodes = []
    in_press, i0, off_since = False, 0, None
    for i in range(len(t)):
        if not in_press:
            if fmag[i] >= ON_N:
                in_press, i0, off_since = True, i, None
        else:
            if fmag[i] <= OFF_N:
                if off_since is None:
                    off_since = t[i]
                elif t[i] - off_since >= DEBOUNCE_S:
                    episodes.append((i0, i))
                    in_press, off_since = False, None
            else:
                off_since = None
    if in_press:
        episodes.append((i0, len(t) - 1))
    return episodes


def press_features(run, i0, i1):
    """Stiffness/relaxation features for one contact episode, or None."""
    tip = run["tip"][i0:i1 + 1]
    f = run["fmag"][i0:i1 + 1]
    t = run["t"][i0:i1 + 1]
    if t[-1] - t[0] < MIN_DUR_S or f.max() < MIN_PEAK_N:
        return None
    if not (run["speed"][i0:i1 + 1] <= HOLD_SPEED_MS).any():
        return None   # tool never stationary while in contact: transit blip

    # Indentation axis from the actual motion: onset -> peak-force position.
    # Signed projection onto it makes loading depth grow and retract shrink.
    p0 = tip[0]
    i_pk = int(np.argmax(f))
    axis = tip[i_pk] - p0
    if np.linalg.norm(axis) < 1e-4:
        return None
    axis = axis / np.linalg.norm(axis)
    depth = (tip - p0) @ axis * 1000.0   # mm

    d_max = depth.max()
    if d_max < MIN_DEPTH_MM:
        return None   # rebound blip or graze, not a press

    # Loading phase: onset up to first arrival at max depth.
    i_dmax = int(np.argmax(depth))
    load_d, load_f = depth[:i_dmax + 1], f[:i_dmax + 1]
    if len(load_d) < 5 or np.ptp(load_d) < 1e-6:
        return None
    k_full = float(np.polyfit(load_d, load_f, 1)[0])

    # Late-phase stiffness: upper half of the force range, where a hard
    # inclusion under the shell shows up strongest.
    hi = load_f >= 0.5 * load_f.max()
    k_late = (float(np.polyfit(load_d[hi], load_f[hi], 1)[0])
              if hi.sum() >= 4 and np.ptp(load_d[hi]) > 1e-6 else np.nan)

    # Relaxation over the hold at max depth.
    hold = depth >= d_max - HOLD_BAND_MM
    f_hold = f[hold]
    relax = (float((f_hold[0] - f_hold[-1]) / f_hold[0])
             if len(f_hold) >= 5 and f_hold[0] > 0 else np.nan)

    return {
        "t_peak": float(t[i_pk]),
        "tip": tip[i_pk],
        "f_peak": float(f.max()),
        "depth_max": float(d_max),
        "k_full": k_full,
        "k_late": k_late,
        "relax": relax,
        "curve": (load_d, load_f),      # for the sample-curve figure
        "load_tip": tip[:i_dmax + 1],   # tip positions along the loading phase
    }


def analyze_run(run):
    presses = []
    for i0, i1 in detect_episodes(run["t"], run["fmag"]):
        feat = press_features(run, i0, i1)
        if feat is not None:
            presses.append(feat)
    print(f"  {run['name']}: {len(presses)} presses with usable loading curves")
    return presses


def surface_coords(presses, center_xy):
    """(azimuth deg, height mm) of each press about the vertical cone axis."""
    P = np.array([p["tip"] for p in presses])
    az = np.degrees(np.arctan2(P[:, 1] - center_xy[1], P[:, 0] - center_xy[0]))
    h = P[:, 2] * 1000.0
    return az, h


def self_residual(h, k, row_tol_mm=1.5):
    """Per-press stiffness residual against the run's own axisymmetric baseline.

    Presses at the same generated height form tight rows; subtracting each
    row's median leaves only azimuth-localized deviations. This needs no
    reference phantom and cancels body-dimension differences entirely - but an
    inclusion CENTERED on the axis is axisymmetric and vanishes here (that
    case still needs the empty-phantom comparison).
    """
    resid = np.full(len(h), np.nan)
    order = np.argsort(h)
    row = [order[0]]
    rows = []
    for idx in order[1:]:
        if h[idx] - h[row[-1]] <= row_tol_mm:
            row.append(idx)
        else:
            rows.append(row)
            row = [idx]
    rows.append(row)
    for row in rows:
        idx = np.array(row)
        resid[idx] = k[idx] - np.nanmedian(k[idx])
    return resid


def make_figures(phantom, presses_p, ref=None, presses_r=None,
                 out_dir=None, tag=None):
    """Render the sample-curve and stiffness-map figures for one phantom run,
    optionally compared against a reference (empty-phantom) run. Runs come
    from load_run() + analyze_run(). Returns the saved figure paths."""
    if out_dir is None:
        out_dir = paths.FIGURES
    runs = [(phantom, presses_p)]
    ref_dir = ref is not None
    if ref_dir:
        runs.append((ref, presses_r))

    # One shared axis center so azimuths line up between runs (the calibration
    # pins the cone axis vertical, so the axis is just an (x, y) point).
    all_tips = np.array([p["tip"] for _, pr in runs for p in pr])
    center_xy = all_tips[:, :2].mean(axis=0)

    os.makedirs(out_dir, exist_ok=True)
    if tag is None:
        tag = phantom["name"] + (f"_vs_{ref['name']}" if ref_dir else "")

    # ---------------- sample loading curves ---------------- #
    fig, axes = plt.subplots(1, len(runs), figsize=(7 * len(runs), 5), squeeze=False)
    for ax, (run, presses) in zip(axes[0], runs):
        order = np.argsort([p["f_peak"] for p in presses])
        picks = [presses[order[j]] for j in
                 np.linspace(0, len(order) - 1, 6).astype(int)]
        for p in picks:
            d, f = p["curve"]
            ax.plot(d, f, lw=1.2,
                    label=f"z={p['tip'][2]*1000:.0f} mm  k={p['k_full']:.2f} N/mm")
        ax.set_xlabel("indentation (mm)"); ax.set_ylabel("|F| (N)")
        ax.set_title(f"{run['name']} - sample loading curves")
        ax.legend(fontsize=7)
        ax.grid(alpha=0.3)
    fig.tight_layout()
    curves_png = os.path.join(out_dir, f"{tag}_stiffness_curves.png")
    fig.savefig(curves_png, dpi=200)
    plt.close(fig)
    print(f"Saved {curves_png}")

    # ---------------- stiffness maps + residuals ---------------- #
    # Row 0: absolute stiffness per run. Row 1: self-baseline residual per run
    # (azimuth-localized anomalies, immune to body-dimension differences).
    # Row 2 (with a reference run): press-matched differential maps, which
    # also catch an inclusion centered on the axis.
    n_rows = 3 if ref_dir else 2
    fig, axes = plt.subplots(n_rows, 2, figsize=(15, 5.5 * n_rows), squeeze=False)

    for col, (run, presses) in enumerate(runs):
        az, h = surface_coords(presses, center_xy)
        k = np.array([p["k_full"] for p in presses])
        sc = axes[0][col].scatter(az, h, c=k, s=55, cmap="viridis")
        axes[0][col].set_title(f"{run['name']} - stiffness k_full (N/mm)")
        fig.colorbar(sc, ax=axes[0][col], label="N/mm")

        k_late = np.array([p["k_late"] for p in presses])
        resid = self_residual(h, k_late)
        lim = np.nanpercentile(np.abs(resid), 98)
        sc = axes[1][col].scatter(az, h, c=resid, s=55, cmap="RdBu_r",
                                  vmin=-lim, vmax=lim)
        axes[1][col].set_title(
            f"{run['name']} - k_late vs own height-row median")
        fig.colorbar(sc, ax=axes[1][col], label="Δk_late (N/mm)")

        top = np.argsort(-np.nan_to_num(resid, nan=-np.inf))[:3]
        print(f"\n  {run['name']} - top self-residual presses "
              "(stiffer than own row):")
        for j in top:
            print(f"    az={az[j]:7.1f} deg  z={h[j]:5.1f} mm  "
                  f"Δk_late={resid[j]:+.2f} N/mm")
    for ax in axes[:2].ravel():
        ax.set_xlabel("azimuth (deg)"); ax.set_ylabel("height z (mm)")

    if ref_dir:
        # Match presses of the two runs by contact position; the identical
        # pose grid makes nearest-neighbour matching unambiguous.
        tips_r = np.array([p["tip"] for p in presses_r])
        tree = cKDTree(tips_r)
        rows = []
        for p in presses_p:
            dist, j = tree.query(p["tip"])
            if dist * 1000.0 <= MATCH_TOL_MM:
                rows.append((p, presses_r[j]))
        print(f"  matched {len(rows)} of {len(presses_p)} presses to the reference")

        az, h = surface_coords([a for a, _ in rows], center_xy)
        for col, (feat, label) in enumerate([("k_full", "Δk_full (N/mm)"),
                                             ("k_late", "Δk_late (N/mm)")]):
            dk = np.array([a[feat] - b[feat] for a, b in rows])
            lim = np.nanpercentile(np.abs(dk), 98)
            sc = axes[2][col].scatter(az, h, c=dk, s=55, cmap="RdBu_r",
                                      vmin=-lim, vmax=lim)
            axes[2][col].set_title(
                f"{phantom['name']} - {ref['name']}:  {label}")
            axes[2][col].set_xlabel("azimuth (deg)")
            axes[2][col].set_ylabel("height z (mm)")
            fig.colorbar(sc, ax=axes[2][col], label=label)

        # Where does the residual point? Top hits by late-phase stiffness excess.
        dk_late = np.array([a["k_late"] - b["k_late"] for a, b in rows])
        order = np.argsort(-np.nan_to_num(dk_late, nan=-np.inf))
        print("\n  Top presses by Δk_late (stiffer than reference):")
        for j in order[:5]:
            a, _ = rows[j]
            print(f"    az={az[j]:7.1f} deg  z={h[j]:5.1f} mm  "
                  f"Δk_late={dk_late[j]:+.2f} N/mm  (k_late={a['k_late']:.2f})")

    fig.suptitle("Press stiffness over the cone surface", y=0.995)
    fig.tight_layout()
    map_png = os.path.join(out_dir, f"{tag}_stiffness_map.png")
    fig.savefig(map_png, dpi=200)
    plt.close(fig)
    print(f"Saved {map_png}")

    return [curves_png, map_png]


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    phantom_dir = sys.argv[1]
    ref_dir = sys.argv[2] if len(sys.argv) > 2 else None

    print("Loading runs ...")
    phantom = load_run(phantom_dir)
    presses_p = analyze_run(phantom)
    ref = presses_r = None
    if ref_dir:
        ref = load_run(ref_dir)
        presses_r = analyze_run(ref)
    make_figures(phantom, presses_p, ref, presses_r)


if __name__ == "__main__":
    main()
