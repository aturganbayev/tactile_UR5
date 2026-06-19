#!/usr/bin/env python3
"""Plot a recording produced by record_cone_press.py.

Usage:
  python3 plot_cone_data.py [stamp]

  stamp - timestamp prefix of the recording, e.g. 2026-06-19_17-23-16
          (matches <stamp>_trajectory.csv / <stamp>_presses.csv in cone_data/).
          If omitted, the most recently modified recording is used.

Saves (to ../figures/) and shows:
  <stamp>_force_vs_time.png         - Fz/|F| over time, press peaks marked
  <stamp>_speed_vs_time.png         - TCP speed over time
  <stamp>_trajectory_3d.png         - 3D TCP path colored by |F|
  <stamp>_peak_force_per_press.png  - peak Fz/|F| bar chart per press
  <stamp>_press_force_on_cone.png   - peak force mapped onto the cone surface
"""

import glob
import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
from pose_utils import tcp_pose_to_contact

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "cone_data")


def find_stamp(stamp=None):
    if stamp:
        return stamp
    presses = sorted(glob.glob(os.path.join(DATA_DIR, "*_presses.csv")), key=os.path.getmtime)
    if not presses:
        raise FileNotFoundError(f"No *_presses.csv files found in {DATA_DIR}")
    name = os.path.basename(presses[-1])
    return name[: -len("_presses.csv")]


def set_equal_3d(ax, x, y, z):
    max_range = max(x.max() - x.min(), y.max() - y.min(), z.max() - z.min()) / 2
    mid_x, mid_y, mid_z = (x.max() + x.min()) / 2, (y.max() + y.min()) / 2, (z.max() + z.min()) / 2
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)


def plot_force_vs_time(traj, presses, out_path):
    fig, ax = plt.subplots(figsize=(11, 5))
    t0 = traj["t"].iloc[0]
    ax.plot(traj["t"] - t0, traj["Fz"], label="Fz", linewidth=0.8)
    ax.plot(traj["t"] - t0, traj["Fmag"], label="|F|", linewidth=0.8, alpha=0.7)
    for row in presses.itertuples():
        ax.axvline(row.t_peak - t0, color="red", linestyle="--", linewidth=0.6, alpha=0.6)
        ax.annotate(f"#{int(row.press)}", (row.t_peak - t0, row.peak_Fz),
                    fontsize=7, color="red", textcoords="offset points", xytext=(2, 2))
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (N)")
    ax.set_title("Force vs time (press peaks marked)")
    ax.legend()
    fig.savefig(out_path, dpi=200, bbox_inches="tight")


def plot_speed_vs_time(traj, out_path):
    fig, ax = plt.subplots(figsize=(11, 4))
    t0 = traj["t"].iloc[0]
    ax.plot(traj["t"] - t0, traj["speed"], linewidth=0.8, color="tab:green")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("TCP speed (m/s)")
    ax.set_title("TCP speed vs time")
    fig.savefig(out_path, dpi=200, bbox_inches="tight")


def plot_trajectory_3d(traj, out_path):
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection="3d")
    sc = ax.scatter(traj["x"], traj["y"], traj["z"], c=traj["Fmag"], cmap="inferno", s=3)
    fig.colorbar(sc, ax=ax, label="|F| (N)", shrink=0.6)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("TCP trajectory colored by force magnitude")
    set_equal_3d(ax, traj["x"], traj["y"], traj["z"])
    fig.savefig(out_path, dpi=200, bbox_inches="tight")


def plot_peak_force_per_press(presses, out_path):
    fig, ax = plt.subplots(figsize=(9, 4.5))
    ax.bar(presses["press"] - 0.15, presses["peak_Fz"], width=0.3, label="peak Fz")
    ax.bar(presses["press"] + 0.15, presses["peak_Fmag"], width=0.3, label="peak |F|")
    ax.set_xlabel("Press #")
    ax.set_ylabel("Force (N)")
    ax.set_title("Peak force per press")
    ax.legend()
    fig.savefig(out_path, dpi=200, bbox_inches="tight")


def plot_press_force_on_cone(presses, out_path):
    # Calibrated surface points (base frame, via ICP) so they line up with the
    # recorded TCP poses, which are also in the robot base frame.
    surface = pd.read_csv(paths.SURFACE_POINTS_BASE)

    # Convert TCP pose -> sensor tip position so points land on the surface
    # rather than ~9 cm above it (TOOL_TIP_OFFSET).
    tips = np.array([
        tcp_pose_to_contact([row.x, row.y, row.z], [row.rx, row.ry, row.rz])
        for row in presses.itertuples()
    ])

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(surface["x"], surface["y"], surface["z"],
               c="lightgray", s=2, alpha=0.3, label="Cone surface")
    sc = ax.scatter(tips[:, 0], tips[:, 1], tips[:, 2],
                     c=presses["peak_Fz"], cmap="inferno", s=60,
                     edgecolors="k", linewidths=0.5)
    fig.colorbar(sc, ax=ax, label="peak Fz (N)", shrink=0.6)
    for (x, y, z), p in zip(tips, presses["press"]):
        ax.text(x, y, z, str(int(p)), fontsize=6)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Peak press force mapped onto cone surface")
    all_x = np.concatenate([surface["x"].values, tips[:, 0]])
    all_y = np.concatenate([surface["y"].values, tips[:, 1]])
    all_z = np.concatenate([surface["z"].values, tips[:, 2]])
    set_equal_3d(ax, all_x, all_y, all_z)
    fig.savefig(out_path, dpi=200, bbox_inches="tight")


def main():
    stamp = sys.argv[1] if len(sys.argv) > 1 else None
    stamp = find_stamp(stamp)
    traj_path = os.path.join(DATA_DIR, f"{stamp}_trajectory.csv")
    press_path = os.path.join(DATA_DIR, f"{stamp}_presses.csv")
    traj = pd.read_csv(traj_path)
    presses = pd.read_csv(press_path)

    os.makedirs(paths.FIGURES, exist_ok=True)
    print(f"Plotting recording {stamp} ({len(traj)} samples, {len(presses)} presses) ...")

    plot_force_vs_time(traj, presses, os.path.join(paths.FIGURES, f"{stamp}_force_vs_time.png"))
    plot_speed_vs_time(traj, os.path.join(paths.FIGURES, f"{stamp}_speed_vs_time.png"))
    plot_trajectory_3d(traj, os.path.join(paths.FIGURES, f"{stamp}_trajectory_3d.png"))
    if len(presses) > 0:
        plot_peak_force_per_press(presses, os.path.join(paths.FIGURES, f"{stamp}_peak_force_per_press.png"))
        plot_press_force_on_cone(presses, os.path.join(paths.FIGURES, f"{stamp}_press_force_on_cone.png"))
    else:
        print("No presses recorded in this file; skipping press-based plots.")

    print(f"Saved figures to {paths.FIGURES}/{stamp}_*.png")
    plt.show()


if __name__ == "__main__":
    main()
