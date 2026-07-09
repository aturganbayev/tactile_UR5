#!/usr/bin/env python3
"""Batch-plot recordings under this directory.

Walks every immediate subfolder here (e.g. empty/, yellow_low/,
"yellow mid/", ...), finds each <stamp>_trajectory.csv / <stamp>_presses.csv
pair, and produces all the plots from plot_cone_data.py:

  <folder>_force_vs_time.png         - Fz/|F| over time, press peaks marked
  <folder>_speed_vs_time.png         - TCP speed over time
  <folder>_trajectory_3d.png         - TCP path colored by |F| (static)
  <folder>_trajectory_3d.html        - same, but interactive (open in a browser)
  <folder>_peak_force_per_press.png  - peak Fz/|F| bar chart per press
  <folder>_press_force_on_cone.png   - peak force mapped onto the cone surface (static)
  <folder>_press_force_on_cone.html  - same, but interactive (open in a browser)

Each plot is saved back into the subfolder it came from (named after the
folder, not the recording timestamp), titled with the folder name plus the
max and average press force. Nothing is shown on screen - everything is
saved to disk; open the .html files in a browser for interactive 3D viewing.

New subfolders dropped in here later are picked up automatically the next
time this script is run - nothing needs to be added by hand. Recordings
whose plots already exist (and are newer than the CSVs) are skipped, so
re-running the script only plots what's new; delete a plot file or update
the recording to force a replot.

Usage:
  python3 plot_cone_data_batch.py
"""

import glob
import os
import sys

import pandas as pd
import matplotlib.pyplot as plt

# this folder lives inside the repo, so pyForceDAQ is a sibling directory
REPO_PYFORCEDAQ = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), os.pardir, "pyForceDAQ")
sys.path.insert(0, REPO_PYFORCEDAQ)
from plot_cone_data import (
    plot_force_vs_time,
    plot_speed_vs_time,
    plot_trajectory_3d,
    plot_peak_force_per_press,
    plot_press_force_on_cone,
    save_trajectory_3d_html,
    save_press_force_on_cone_html,
)

DATA_ROOT = os.path.dirname(os.path.abspath(__file__))


def find_stamps(folder):
    stamps = []
    for press_path in sorted(glob.glob(os.path.join(folder, "*_presses.csv"))):
        stamp = os.path.basename(press_path)[: -len("_presses.csv")]
        traj_path = os.path.join(folder, f"{stamp}_trajectory.csv")
        if os.path.exists(traj_path):
            stamps.append(stamp)
    return stamps


def outputs_up_to_date(folder, stamp, name):
    """True if every plot for this recording exists and is newer than its
    CSVs - then there is nothing to replot. The press-based plots are only
    expected when the recording actually has presses."""
    press_path = os.path.join(folder, f"{stamp}_presses.csv")
    traj_path = os.path.join(folder, f"{stamp}_trajectory.csv")
    with open(press_path) as f:
        has_presses = sum(1 for _ in f) > 1   # more than the header line
    outputs = [f"{name}_force_vs_time.png", f"{name}_speed_vs_time.png",
               f"{name}_trajectory_3d.png", f"{name}_trajectory_3d.html"]
    if has_presses:
        outputs += [f"{name}_peak_force_per_press.png",
                    f"{name}_press_force_on_cone.png",
                    f"{name}_press_force_on_cone.html"]
    src_mtime = max(os.path.getmtime(press_path), os.path.getmtime(traj_path))
    return all(
        os.path.exists(p) and os.path.getmtime(p) >= src_mtime
        for p in (os.path.join(folder, out) for out in outputs)
    )


def plot_recording(folder, stamp, name):
    traj = pd.read_csv(os.path.join(folder, f"{stamp}_trajectory.csv"))
    presses = pd.read_csv(os.path.join(folder, f"{stamp}_presses.csv"))
    print(f"{name}: plotting {stamp} ({len(traj)} samples, {len(presses)} presses) ...")

    plot_force_vs_time(traj, presses, os.path.join(folder, f"{name}_force_vs_time.png"))
    plt.close(plt.gcf())
    plot_speed_vs_time(traj, os.path.join(folder, f"{name}_speed_vs_time.png"))
    plt.close(plt.gcf())

    traj_title = (
        f"{name}: TCP approach paths, top 24 touches by force highlighted ({len(presses)} touches)"
        if len(presses) > 0 else f"{name}: TCP trajectory colored by |F|"
    )
    plot_trajectory_3d(
        traj,
        os.path.join(folder, f"{name}_trajectory_3d.png"),
        presses=presses,
        title=traj_title,
    )
    plt.close(plt.gcf())
    save_trajectory_3d_html(
        traj,
        os.path.join(folder, f"{name}_trajectory_3d.html"),
        presses=presses,
        title=traj_title,
    )

    if len(presses) > 0:
        plot_peak_force_per_press(presses, os.path.join(folder, f"{name}_peak_force_per_press.png"))
        plt.close(plt.gcf())

        max_fz = presses["peak_Fz"].max()
        max_fmag = presses["peak_Fmag"].max()
        mean_fz = presses["peak_Fz"].mean()
        mean_fmag = presses["peak_Fmag"].mean()
        press_title = (
            f"{name}: peak press force on cone, top 24 + next 24 highlighted "
            f"(max Fz={max_fz:.2f} N, max |F|={max_fmag:.2f} N, "
            f"avg Fz={mean_fz:.2f} N, avg |F|={mean_fmag:.2f} N)"
        )

        plot_press_force_on_cone(
            presses,
            os.path.join(folder, f"{name}_press_force_on_cone.png"),
            title=press_title,
        )
        plt.close(plt.gcf())
        save_press_force_on_cone_html(
            presses,
            os.path.join(folder, f"{name}_press_force_on_cone.html"),
            title=press_title,
        )
    else:
        print("    No presses recorded in this file; skipping press-based plots.")


def main():
    folders = sorted(
        d for d in glob.glob(os.path.join(DATA_ROOT, "*")) if os.path.isdir(d)
    )
    if not folders:
        raise FileNotFoundError(f"No subfolders found in {DATA_ROOT}")

    for folder in folders:
        stamps = find_stamps(folder)
        if not stamps:
            continue
        folder_name = os.path.basename(folder).replace(" ", "_")
        for i, stamp in enumerate(stamps):
            name = folder_name if len(stamps) == 1 else f"{folder_name}_{i + 1}"
            if outputs_up_to_date(folder, stamp, name):
                print(f"{name}: plots up to date - skipping.")
                continue
            plot_recording(folder, stamp, name)

    print("Done.")


if __name__ == "__main__":
    main()
