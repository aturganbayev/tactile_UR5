#!/usr/bin/env python3
"""Batch ball localization for recordings under this directory.

Walks every immediate subfolder here (e.g. red_mid/, ...), compares each
phantom's presses against the empty/ reference recording, and runs the
ball point-cloud analysis from the repo's pyForceDAQ/ball_point_cloud.py:
for every press whose force curve clearly diverges from the empty phantom's
at the same location, the tip position at the divergence depth gives a point
on the ball's boundary; a sphere fit turns the cloud into center + radius.

Outputs, saved into each phantom subfolder:
  <folder>_ball_cloud.png    - static 3D view (surface + points + fitted ball)
  <folder>_ball_cloud.html   - interactive 3D view (open in a browser)

Folders where no ball influence is found (including any without a clear
inclusion) are reported as "no ball detected" and skipped. The empty/
reference folder itself is skipped. New subfolders dropped in here are
picked up automatically the next time this script runs.

Usage:
  python3 ball_cloud_batch.py
"""

import glob
import os
import sys

# this folder lives inside the repo, so pyForceDAQ is a sibling directory
REPO_PYFORCEDAQ = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, "pyForceDAQ")
sys.path.insert(0, REPO_PYFORCEDAQ)
from stiffness_map import load_run, analyze_run
from ball_point_cloud import locate_ball

DATA_ROOT = os.path.dirname(os.path.abspath(__file__))
REFERENCE_FOLDER = "empty"


def has_recording(folder):
    return bool(glob.glob(os.path.join(folder, "*_trajectory.csv")))


def main():
    ref_dir = os.path.join(DATA_ROOT, REFERENCE_FOLDER)
    if not has_recording(ref_dir):
        raise FileNotFoundError(
            f"No reference recording in {ref_dir} - the ball localization "
            "needs the empty phantom run to compare against.")

    print(f"Loading reference '{REFERENCE_FOLDER}' ...")
    ref = load_run(ref_dir)
    presses_ref = analyze_run(ref)

    results = {}
    for folder in sorted(glob.glob(os.path.join(DATA_ROOT, "*"))):
        if (not os.path.isdir(folder) or not has_recording(folder)
                or os.path.normpath(folder) == os.path.normpath(ref_dir)):
            continue
        name = os.path.basename(folder).replace(" ", "_")
        print(f"\n{name}:")
        run = load_run(folder)
        presses = analyze_run(run)
        results[name] = locate_ball(run, presses, ref, presses_ref,
                                    out_dir=folder, tag=name)

    print("\n---- summary ----")
    for name, res in results.items():
        if res is None:
            print(f"  {name}: no ball detected")
        else:
            center, radius, rms, n = res
            print(f"  {name}: ball at z={center[2]*1000:.1f} mm, "
                  f"r={radius*1000:.1f} mm ({n} points, rms {rms*1000:.1f} mm)")
    print("Done.")


if __name__ == "__main__":
    main()
