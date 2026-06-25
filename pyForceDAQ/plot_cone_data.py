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
import plotly.graph_objects as go

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


def filtered_trajectory(traj, contact_thresh=0.3, max_points=6000):
    t = traj
    # Drop the free-space return-to-start / inter-strip transit moves: those lift
    # the tool well above the cone (and carry no force), cluttering the plot.
    # Keep everything up to just above the highest in-contact height, so the
    # press cycles and their local approach/retract remain.
    contact = t[t["Fmag"] > contact_thresh]
    if len(contact):
        z_cap = contact["z"].max() + 0.01
        t = t[t["z"] <= z_cap]
    # Downsample so the 3D scatter stays responsive (the raw stream is ~125 Hz
    # over the whole run -> hundreds of thousands of points).
    if len(t) > max_points:
        t = t.iloc[:: max(1, len(t) // max_points)]
    return t


def _smooth(a, window):
    if len(a) < window:
        return a
    pad = window // 2
    kernel = np.ones(window) / window
    return np.convolve(np.pad(a, pad, mode="edge"), kernel, mode="valid")[: len(a)]


def trajectory_runs(traj, presses, contact_thresh=0.3, smooth_window=7, max_points_per_run=300):
    """Split the trajectory into one smoothed run per press: just the approach
    leading up to that press's contact peak. Drops the retreat/return-to-start
    travel after each press (and the final return home after the last press),
    since that's not informative and only clutters the plot."""
    contact = traj[traj["Fmag"] > contact_thresh]
    t = traj[traj["z"] <= contact["z"].max() + 0.01] if len(contact) else traj

    presses_sorted = presses.sort_values("t_peak").reset_index(drop=True)
    peak_times = presses_sorted["t_peak"].to_numpy()
    ts = t["t"].to_numpy()
    idx = np.searchsorted(peak_times, ts, side="left")

    has_next = idx < len(peak_times)
    next_t = np.where(has_next, peak_times[np.clip(idx, 0, len(peak_times) - 1)], np.inf)
    has_prev = idx > 0
    prev_t = np.where(has_prev, peak_times[np.clip(idx - 1, 0, len(peak_times) - 1)], -np.inf)
    approach = (next_t - ts) <= (ts - prev_t)

    t = t[approach]
    idx = idx[approach]

    runs = []
    for run_idx in np.unique(idx):
        run = t[idx == run_idx]
        if len(run) < 2:
            continue
        if len(run) > max_points_per_run:
            run = run.iloc[:: max(1, len(run) // max_points_per_run)]
        tips = pose_to_tips(run)
        x, y, z = _smooth(tips[:, 0], smooth_window), _smooth(tips[:, 1], smooth_window), _smooth(tips[:, 2], smooth_window)
        press_row = presses_sorted.iloc[run_idx]
        runs.append(dict(press=int(press_row["press"]), peak_Fz=press_row["peak_Fz"],
                          peak_Fmag=press_row["peak_Fmag"], x=x, y=y, z=z))
    return runs


def pose_to_tips(df):
    # Convert TCP pose -> sensor tip position so points land on the surface
    # rather than ~9 cm above it (TOOL_TIP_OFFSET). Needed for any TCP pose
    # (trajectory or presses) plotted alongside the cone surface, which is
    # captured in tip/contact coordinates.
    return np.array([
        tcp_pose_to_contact([row.x, row.y, row.z], [row.rx, row.ry, row.rz])
        for row in df.itertuples()
    ])


def cone_surface():
    # Calibrated surface points (base frame, via ICP) so they line up with the
    # recorded TCP poses, which are also in the robot base frame.
    return pd.read_csv(paths.SURFACE_POINTS_BASE)


def plot_trajectory_3d(traj, out_path=None, presses=None, contact_thresh=0.3, max_points=6000, ax=None, title=None, top_n=20):
    if ax is None:
        fig = plt.figure(figsize=(8, 8))
        ax = fig.add_subplot(111, projection="3d")
    else:
        fig = ax.figure

    if presses is not None and len(presses) > 0:
        runs = trajectory_runs(traj, presses, contact_thresh=contact_thresh)
        top_presses = set(presses.nlargest(top_n, "peak_Fz")["press"].astype(int))
        all_x, all_y, all_z = [], [], []
        for run in runs:
            is_top = run["press"] in top_presses
            x, y, z = run["x"][-1], run["y"][-1], run["z"][-1]
            label = f"#{run['press']} @ ({x:.3f}, {y:.3f}, {z:.3f}) m, Fz={run['peak_Fz']:.2f} N" if is_top else None
            ax.plot(run["x"], run["y"], run["z"], color="gold" if is_top else "black",
                    linewidth=2.6 if is_top else 1.0, alpha=1.0 if is_top else 0.5, label=label)
            if is_top:
                ax.text(x, y, z, f"#{run['press']}", fontsize=7, color="red", weight="bold")
            all_x.append(run["x"])
            all_y.append(run["y"])
            all_z.append(run["z"])
        ax.legend(loc="upper left", fontsize=6, bbox_to_anchor=(1.25, 1.0))
        all_x, all_y, all_z = np.concatenate(all_x), np.concatenate(all_y), np.concatenate(all_z)
        default_title = f"TCP approach paths, top {top_n} touches by force highlighted ({len(runs)} touches)"
    else:
        t = filtered_trajectory(traj, contact_thresh, max_points)
        tips = pose_to_tips(t)
        sc = ax.scatter(tips[:, 0], tips[:, 1], tips[:, 2], c=t["Fmag"], cmap="inferno", s=4)
        fig.colorbar(sc, ax=ax, label="|F| (N)", shrink=0.6, pad=0.15)
        all_x, all_y, all_z = tips[:, 0], tips[:, 1], tips[:, 2]
        default_title = f"TCP trajectory near the cone, colored by |F|  ({len(t)} pts)"

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.zaxis.labelpad = 10
    ax.set_title(title or default_title)
    set_equal_3d(ax, all_x, all_y, all_z)
    if out_path:
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


def plot_press_force_on_cone(presses, out_path=None, ax=None, title=None, top_n=20):
    surface = cone_surface()
    tips = pose_to_tips(presses)
    top_mask = (presses["peak_Fz"].rank(method="first", ascending=False) <= top_n).to_numpy()
    vmin, vmax = presses["peak_Fz"].min(), presses["peak_Fz"].max()

    if ax is None:
        fig = plt.figure(figsize=(9, 8))
        ax = fig.add_subplot(111, projection="3d")
    else:
        fig = ax.figure
    ax.scatter(surface["x"], surface["y"], surface["z"],
               c="lightgray", s=2, alpha=0.3, label="Cone surface")
    sc = ax.scatter(tips[~top_mask, 0], tips[~top_mask, 1], tips[~top_mask, 2],
                     c=presses.loc[~top_mask, "peak_Fz"], cmap="inferno", s=60,
                     vmin=vmin, vmax=vmax, edgecolors="k", linewidths=0.5)
    ax.scatter(tips[top_mask, 0], tips[top_mask, 1], tips[top_mask, 2],
               c=presses.loc[top_mask, "peak_Fz"], cmap="inferno", s=220,
               vmin=vmin, vmax=vmax, marker="*", edgecolors="red", linewidths=1.2,
               label=f"Top {top_n} Fz")
    fig.colorbar(sc, ax=ax, label="peak Fz (N)", shrink=0.6, pad=0.15)
    ax.legend(loc="upper left", fontsize=8)
    for (x, y, z), p in zip(tips, presses["press"]):
        ax.text(x, y, z, str(int(p)), fontsize=6)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.zaxis.labelpad = 10
    default_title = (
        f"Peak press force mapped onto cone surface "
        f"(max Fz={presses['peak_Fz'].max():.2f} N, max |F|={presses['peak_Fmag'].max():.2f} N, "
        f"avg Fz={presses['peak_Fz'].mean():.2f} N, avg |F|={presses['peak_Fmag'].mean():.2f} N)"
    )
    ax.set_title(title or default_title)
    all_x = np.concatenate([surface["x"].values, tips[:, 0]])
    all_y = np.concatenate([surface["y"].values, tips[:, 1]])
    all_z = np.concatenate([surface["z"].values, tips[:, 2]])
    set_equal_3d(ax, all_x, all_y, all_z)
    if out_path:
        fig.savefig(out_path, dpi=200, bbox_inches="tight")


_COLORBAR_LAYOUT = dict(len=0.7, y=0.4, yanchor="middle", x=1.02)


def save_trajectory_3d_html(traj, out_path, presses=None, contact_thresh=0.3, max_points=4000,
                             surface_max_points=1200, title=None, top_n=20):
    """WebGL-accelerated equivalent of plot_trajectory_3d - opens in any browser,
    stays interactive (rotate/zoom/pan) without rerunning Python."""
    surface = cone_surface()
    if len(surface) > surface_max_points:
        surface = surface.iloc[:: max(1, len(surface) // surface_max_points)]
    traces = [
        go.Scatter3d(
            x=surface["x"], y=surface["y"], z=surface["z"], mode="markers",
            marker=dict(size=2, color="dimgray", opacity=0.5), name="Cone surface",
            hoverinfo="skip",
        ),
    ]

    if presses is not None and len(presses) > 0:
        runs = trajectory_runs(traj, presses, contact_thresh=contact_thresh)
        top_presses = set(presses.nlargest(top_n, "peak_Fz")["press"].astype(int))
        for run in runs:
            is_top = run["press"] in top_presses
            x, y, z = run["x"][-1], run["y"][-1], run["z"][-1]
            customdata = np.tile([run["press"], run["peak_Fz"], run["peak_Fmag"]], (len(run["x"]), 1))
            name = (
                f"#{run['press']} @ ({x:.3f}, {y:.3f}, {z:.3f}) m, Fz={run['peak_Fz']:.2f} N"
                if is_top else "Approach path"
            )
            traces.append(go.Scatter3d(
                x=run["x"], y=run["y"], z=run["z"], mode="lines",
                line=dict(color="gold" if is_top else "black", width=6 if is_top else 2),
                name=name,
                showlegend=is_top, legendgroup="top" if is_top else "rest",
                opacity=1.0 if is_top else 0.5,
                customdata=customdata,
                hovertemplate=(
                    "Press #%{customdata[0]:.0f}<br>"
                    "X = %{x:.4f} m<br>Y = %{y:.4f} m<br>Z = %{z:.4f} m<br>"
                    "peak Fz = %{customdata[1]:.2f} N<br>"
                    "peak |F| = %{customdata[2]:.2f} N<extra></extra>"
                ),
            ))
            if is_top:
                traces.append(go.Scatter3d(
                    x=[run["x"][-1]], y=[run["y"][-1]], z=[run["z"][-1]], mode="markers+text",
                    marker=dict(size=5, color="red", symbol="diamond"),
                    text=[f"#{run['press']}"], textposition="top center",
                    textfont=dict(size=9, color="red"), showlegend=False, hoverinfo="skip",
                ))
        default_title = f"TCP approach paths, top {top_n} touches by force highlighted ({len(runs)} touches)"
    else:
        t = filtered_trajectory(traj, contact_thresh, max_points)
        tips = pose_to_tips(t)
        traces.append(go.Scatter3d(
            x=tips[:, 0], y=tips[:, 1], z=tips[:, 2], mode="markers", name="TCP trajectory",
            marker=dict(size=3, color=t["Fmag"], colorscale="Inferno",
                        colorbar=dict(title="|F| (N)", **_COLORBAR_LAYOUT), opacity=0.85),
            customdata=t["Fmag"],
            hovertemplate=(
                "X = %{x:.4f} m<br>Y = %{y:.4f} m<br>Z = %{z:.4f} m<br>"
                "|F| = %{customdata:.2f} N<extra></extra>"
            ),
        ))
        default_title = f"TCP trajectory near the cone, colored by |F|  ({len(t)} pts)"

    fig = go.Figure(traces)
    fig.update_layout(
        title=title or default_title,
        margin=dict(t=90),
        scene=dict(xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)", aspectmode="data"),
    )
    fig.write_html(out_path, include_plotlyjs="cdn")


def save_press_force_on_cone_html(presses, out_path, surface_max_points=1200, title=None, top_n=20):
    """WebGL-accelerated equivalent of plot_press_force_on_cone."""
    surface = cone_surface()
    if len(surface) > surface_max_points:
        surface = surface.iloc[:: max(1, len(surface) // surface_max_points)]
    tips = pose_to_tips(presses)
    top_mask = (presses["peak_Fz"].rank(method="first", ascending=False) <= top_n).to_numpy()
    vmin, vmax = presses["peak_Fz"].min(), presses["peak_Fz"].max()

    def press_trace(mask, name, symbol, size, line_width, showscale):
        sub = presses.loc[mask]
        sub_tips = tips[mask]
        customdata = sub[["press", "peak_Fz", "peak_Fmag"]].to_numpy()
        return go.Scatter3d(
            x=sub_tips[:, 0], y=sub_tips[:, 1], z=sub_tips[:, 2], mode="markers+text",
            marker=dict(size=size, color=sub["peak_Fz"], colorscale="Inferno",
                        cmin=vmin, cmax=vmax, symbol=symbol,
                        colorbar=dict(title="peak Fz (N)", **_COLORBAR_LAYOUT) if showscale else None,
                        showscale=showscale, line=dict(color="black", width=line_width)),
            text=[str(int(p)) for p in sub["press"]], textposition="top center",
            textfont=dict(size=8), name=name,
            customdata=customdata,
            hovertemplate=(
                "Press #%{customdata[0]:.0f}<br>"
                "X = %{x:.4f} m<br>Y = %{y:.4f} m<br>Z = %{z:.4f} m<br>"
                "peak Fz = %{customdata[1]:.2f} N<br>"
                "peak |F| = %{customdata[2]:.2f} N<extra></extra>"
            ),
        )

    fig = go.Figure([
        go.Scatter3d(
            x=surface["x"], y=surface["y"], z=surface["z"], mode="markers",
            marker=dict(size=2, color="dimgray", opacity=0.5), name="Cone surface",
            hoverinfo="skip",
        ),
        press_trace(~top_mask, "Presses", "circle", 6, 0.5, True),
        press_trace(top_mask, f"Top {top_n} Fz", "diamond", 10, 2, False),
    ])
    default_title = (
        f"Peak press force mapped onto cone surface "
        f"(max Fz={presses['peak_Fz'].max():.2f} N, max |F|={presses['peak_Fmag'].max():.2f} N, "
        f"avg Fz={presses['peak_Fz'].mean():.2f} N, avg |F|={presses['peak_Fmag'].mean():.2f} N)"
    )
    fig.update_layout(
        title=title or default_title,
        margin=dict(t=90),
        scene=dict(xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)", aspectmode="data"),
    )
    fig.write_html(out_path, include_plotlyjs="cdn")


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
    plot_trajectory_3d(traj, os.path.join(paths.FIGURES, f"{stamp}_trajectory_3d.png"), presses=presses)
    if len(presses) > 0:
        plot_peak_force_per_press(presses, os.path.join(paths.FIGURES, f"{stamp}_peak_force_per_press.png"))
        plot_press_force_on_cone(presses, os.path.join(paths.FIGURES, f"{stamp}_press_force_on_cone.png"))
    else:
        print("No presses recorded in this file; skipping press-based plots.")

    print(f"Saved figures to {paths.FIGURES}/{stamp}_*.png")
    plt.show()


if __name__ == "__main__":
    main()
