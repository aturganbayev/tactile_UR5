#!/usr/bin/env python3
"""
Re-derive per-pose results from recorded palpation traces at ANY target total.

WHY THIS WORKS WITHOUT RE-RUNNING
Each press is logged continuously - all 48 raw channels plus the TCP pose at
60 Hz for the whole descent - so a run recorded with one stop criterion still
contains everything needed to answer "where would it have stopped at a
different one?". The online script stops at the first sample whose running-max
total crosses the target; this reads the same traces and finds the same
crossing offline.

WHY IT WAS NEEDED
The five specimens were collected with a target of 8000 total counts. That
turned out to be reachable on firm specimens (red_*, 7.7-8.9 mm) but to demand
14-17 mm on soft ones, where the UR5's own force protection engaged and ended
run 20260822_151442 partway through. A target only some specimens can reach is
not a valid comparison - one dataset had 23 poses, another 15.

Re-deriving at 5000 puts every specimen on all 23 poses at 5.2-9.8 mm, and
preserves the between-specimen spread (roughly 2x from firmest to softest),
so nothing is lost but the deep pressing.

WHAT IT CANNOT DO
Only targets BELOW what a pose actually reached can be re-derived - the trace
stops where the press stopped. Poses that ended on travel_cap or over_range
without crossing the requested total are reported as unreached, not guessed.

Usage:
    python3 rederive.py --target 5000
    python3 rederive.py --target 5000 --out rederived_5000.csv
    python3 rederive.py --target 5000 --specimens red_bot empty
"""

import argparse
import glob
import os
import sys

import numpy as np
import pandas as pd

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(_THIS_DIR, "data")
ZC = [f"raw_z{i}" for i in range(16)]
UV = np.array([[(i % 4) * 5.0, (i // 4) * 5.0] for i in range(16)])


def rederive_one(path, target):
    """Per-pose crossing of `target` total counts, from a recorded trace."""
    d = pd.read_csv(path)
    rows = []
    for p in sorted(d["pose"].unique()):
        g = d[d["pose"] == p]
        base = g[g["phase"] == "settle"][ZC].to_numpy(float)
        pr = g[g["phase"].isin(["press", "dwell"])]
        if len(base) < 5 or len(pr) < 5:
            continue
        # Same baseline convention as the online script: the settle window of
        # THIS pose, so the sensor's slow drift cancels per press.
        D = pr[ZC].to_numpy(float) - base.mean(axis=0)
        xyz = pr[["px", "py", "pz"]].astype(float).to_numpy()
        depth = np.linalg.norm(xyz - xyz[0], axis=1) * 1000.0
        total = np.maximum.accumulate(np.abs(D).sum(axis=1))
        peak = np.maximum.accumulate(D.max(axis=1))

        if (total >= target).any():
            k = int(np.argmax(total >= target))
            w = np.clip(D[k], 0, None)
            cen = ((w[:, None] * UV).sum(0) / w.sum()) if w.sum() > 0 else np.full(2, np.nan)
            rows.append(dict(pose=int(p), reached=True, depth_mm=depth[k],
                             peak_counts=peak[k], total_counts=total[k],
                             live=int((D[k] >= 100).sum()),
                             centroid_off_mm=float(np.hypot(*(cen - 7.5)))))
        else:
            rows.append(dict(pose=int(p), reached=False,
                             depth_mm=np.nan, peak_counts=peak[-1],
                             total_counts=total[-1], live=np.nan,
                             centroid_off_mm=np.nan))
    return pd.DataFrame(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target", type=float, default=5000.0,
                    help="total counts to re-derive at (default %(default)s)")
    ap.add_argument("--specimens", nargs="*",
                    help="trace basenames without .csv; default = every "
                         "trace that has a matching *_summary.csv")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()

    if a.specimens:
        names = a.specimens
    else:
        names = sorted(os.path.basename(f).replace("_summary.csv", "")
                       for f in glob.glob(os.path.join(DATA_DIR, "*_summary.csv")))
    frames = []
    print(f"re-deriving at target = {a.target:.0f} total counts\n")
    print("%-14s %6s %7s %7s %7s  %s" % ("specimen", "n_ok", "mean", "min",
                                         "max", "unreached"))
    for n in names:
        path = os.path.join(DATA_DIR, n + ".csv")
        if not os.path.exists(path):
            print(f"  (skip {n}: no trace)")
            continue
        r = rederive_one(path, a.target)
        if r.empty:
            print(f"  (skip {n}: no usable presses)")
            continue
        r.insert(0, "specimen", n)
        frames.append(r)
        ok = r[r["reached"]]
        miss = list(r[~r["reached"]]["pose"])
        print("%-14s %6d %7.2f %7.2f %7.2f  %s" % (
            n, len(ok), ok["depth_mm"].mean(), ok["depth_mm"].min(),
            ok["depth_mm"].max(), miss if miss else "-"))
    if not frames:
        sys.exit("nothing to do")
    out = pd.concat(frames, ignore_index=True)

    # Poses present and reached in EVERY specimen - the only fair basis for a
    # cross-specimen comparison.
    piv = out[out["reached"]].pivot_table(index="pose", columns="specimen",
                                          values="depth_mm")
    common = piv.dropna()
    print(f"\nposes reached by every specimen: {len(common)} of "
          f"{out['pose'].nunique()}")
    if len(common):
        print("\nmean depth over those common poses:")
        for s in common.columns:
            print(f"  {s:<14} {common[s].mean():6.2f} mm")
        sp = common.mean()
        print(f"\n  spread firmest -> softest: {sp.min():.2f} -> {sp.max():.2f} mm "
              f"({sp.max()/sp.min():.2f}x)")

    path = a.out or os.path.join(DATA_DIR, f"rederived_{int(a.target)}.csv")
    out.to_csv(path, index=False)
    print(f"\nwrote {len(out)} rows -> {path}")


if __name__ == "__main__":
    main()
