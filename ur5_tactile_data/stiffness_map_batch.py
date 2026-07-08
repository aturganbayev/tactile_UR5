#!/usr/bin/env python3
"""Batch stiffness maps for recordings under this directory.

Walks every immediate subfolder here (e.g. empty/, red_mid/, ...), takes the
latest <stamp>_trajectory.csv in each, and runs the stiffness analysis from
the repo's pyForceDAQ/stiffness_map.py:

  <folder>_stiffness_curves.png  - sample force-vs-indentation loading curves
  <folder>_stiffness_map.png     - stiffness over the surface (azimuth x height):
                                   absolute k, self-baseline residual, and -
                                   for every folder except the reference -
                                   the press-matched differential vs empty/

The empty/ folder (same phantom body, no ball) is the reference: an embedded
ball shows up in the differential maps as a localized/ring-shaped positive
stiffness residual. Presses are re-detected from the force signal, so the
_presses.csv files are not used here.

Plots are saved back into the subfolder they came from. New subfolders
dropped in here are picked up automatically the next time this script runs.

Usage:
  python3 stiffness_map_batch.py
"""

import glob
import os
import sys

# this folder lives inside the repo, so pyForceDAQ is a sibling directory
REPO_PYFORCEDAQ = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, "pyForceDAQ")
sys.path.insert(0, REPO_PYFORCEDAQ)
from stiffness_map import load_run, analyze_run, make_figures

DATA_ROOT = os.path.dirname(os.path.abspath(__file__))
REFERENCE_FOLDER = "empty"


def has_recording(folder):
    return bool(glob.glob(os.path.join(folder, "*_trajectory.csv")))


def main():
    folders = sorted(
        d for d in glob.glob(os.path.join(DATA_ROOT, "*"))
        if os.path.isdir(d) and has_recording(d)
    )
    if not folders:
        raise FileNotFoundError(f"No subfolders with recordings in {DATA_ROOT}")

    # Load the reference (empty phantom) once and reuse it for every phantom.
    ref_dir = os.path.join(DATA_ROOT, REFERENCE_FOLDER)
    ref = presses_ref = None
    if has_recording(ref_dir):
        print(f"Loading reference '{REFERENCE_FOLDER}' ...")
        ref = load_run(ref_dir)
        presses_ref = analyze_run(ref)
    else:
        print(f"  [warn] no reference recording in {ref_dir} - only absolute "
              "and self-residual maps will be produced.")

    for folder in folders:
        name = os.path.basename(folder).replace(" ", "_")
        if os.path.normpath(folder) == os.path.normpath(ref_dir):
            # the reference itself: absolute + self-residual maps only
            make_figures(ref, presses_ref, out_dir=folder, tag=name)
            continue
        print(f"\n{name}:")
        run = load_run(folder)
        presses = analyze_run(run)
        make_figures(run, presses, ref, presses_ref, out_dir=folder, tag=name)

    print("\nDone.")


if __name__ == "__main__":
    main()
