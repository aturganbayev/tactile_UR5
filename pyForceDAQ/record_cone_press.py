#!/usr/bin/env python3
"""
Synchronized cone-press recorder for the UR5 + ATI Nano17 (FT12876).

Run this ALONGSIDE a motion script (e.g. ../execution/run_side_strip_poses.py) that
drives the robot to press the cone. While it runs it:

  * reads the robot TCP Cartesian pose from the UR real-time stream (port 30003),
  * reads force from the Nano17 via pyForceDAQ,
  * auto-detects each press from the Fz signal, and
  * records the PEAK force of each press together with the TCP pose at that
    instant.

Outputs (in ./cone_data/):
  <name>_trajectory.csv  - t, x,y,z,rx,ry,rz, speed, Fx,Fy,Fz, |F|   (~LOOP_HZ)
  <name>_presses.csv     - press#, t_peak, peak_Fz, peak_|F|, Fx,Fy,Fz,
                           and TCP pose x,y,z,rx,ry,rz at the peak

Stop with Ctrl-C.

See COPYING file distributed along with the pyForceDAQ copyright and license terms.
"""

import csv
import math
import os
import socket
import struct
import sys
import threading
import time
from collections import deque
from time import strftime, localtime

from forceDAQ.force.data_recorder import DataRecorder
from forceDAQ.force.sensor import SensorSettings

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths
from pose_utils import SIM_HOST, REAL_HOST, tcp_pose_to_contact

# --------------------------------------------------------------------------- #
#                                  SETTINGS                                    #
# --------------------------------------------------------------------------- #

# Robot real-time interface
ROBOT_PORT = 30003
# On CB2/SW1.8 port 30003 is also the URScript command port, so a motion
# script writing to it can stall the state broadcast. If no packet arrives
# within this many seconds we treat the stream as stalled and reconnect.
ROBOT_STREAM_TIMEOUT_S = 1.0

# Nano17 sensor (FT12876 is the Nano17; see calibration/FT12876.cal)
SENSOR_NAME = "FT12876"
CALIBRATION_FOLDER = "calibration"
REVERSE_FZ = "Fz"          # press -> positive Fz, matches existing data
BIAS_SAMPLES = 500
# DAQ sample rate (Hz). The sensor runs in HW-timed single-point mode, so the
# host must service the device every sample; too high a rate overruns the DAQ
# buffer (NI error -200714: "could not transfer data fast enough"). We only log
# at LOOP_HZ (125) and presses last seconds, so a modest rate is plenty. Lower
# this further (e.g. 250) if the overrun recurs on a loaded machine.
SENSOR_RATE = 500

# Press detection on Fmag = |F| (Newtons). Hysteresis prevents flicker.
# Using the magnitude rather than signed Fz catches contact even where the
# local surface normal isn't aligned with the sensor's Z axis (e.g. near the
# cone's embedded bulge), which can load mostly Fx/Fy with Fz negative.
PRESS_ON_N = 0.5           # Fmag rising above this starts a press
PRESS_OFF_N = 0.3          # Fmag falling below this ends the press
MIN_PRESS_DURATION_S = 0.05  # ignore shorter blips as noise
# Some cones embed a small hard ball (tumor phantom) under the outer silicone
# shell. Pressing through the soft shell onto the ball gives a bimodal force
# curve: it dips momentarily between the shell contact and the ball contact,
# which would otherwise look like the press ending. Require Fmag to stay below
# PRESS_OFF_N for this long before the press is actually considered over, so
# the dip between the two bumps doesn't get recorded as two separate presses.
PRESS_OFF_DEBOUNCE_S = 0.5
# After a press ends, ignore new presses for this long. As the robot retracts
# it rebounds slightly, which can re-cross PRESS_ON and register a phantom
# second press; real presses are several seconds apart (move + settle between
# touch points), so this window removes rebounds without dropping real presses.
PRESS_REFRACTORY_S = 4.0

# Logging loop rate (the UR stream is ~125 Hz)
LOOP_HZ = 125

# Live view of the TCP path + detected presses over the cone surface, shown
# while recording. Rendered as a self-refreshing Plotly HTML file (open it in
# any browser) rather than a live matplotlib window: mplot3d redraws are slow
# enough to stall the 125 Hz DAQ loop they share a thread with (and a
# headless/no-display DAQ PC can't show a window at all). The HTML is built
# in a background thread that only ever takes a quick snapshot of the
# buffers, so a slow render just makes the file update less often instead of
# adding lag to the recording loop.
LIVE_PLOT = True
LIVE_PLOT_REFRESH_S = 1.0          # re-render (and browser auto-refresh) interval
LIVE_PLOT_DECIMATE = 4             # keep 1 of every N trajectory samples in the view
LIVE_PLOT_MAX_POINTS = 6000        # rolling window of recent trajectory points shown

# UR real-time packet layout (port 30003, big-endian).
#
# The packet is: int32 total-size, float64 time, then a stream of float64
# fields. We unpack it as ">I d <N>d" so that index 0 = size, index 1 = time,
# and index 2 is the first payload double.
#
# CRITICAL: the byte offset of the Cartesian "Tool vector" differs between
# controller generations. The total packet size tells us which layout to use:
#   * CB2 / software 1.x  -> 812-byte packet, tool vector at double index 74
#   * CB3 / e-Series 3.x+ -> 1044+ byte packet, tool vector at double index 56
# (The 812-byte v1.8 layout is what robot 192.168.0.110 actually streams; the
# old 636-byte / index-56 assumption read an unused zero region -> all-zero
# poses.)
_LAYOUT_V3 = {"pose": slice(56, 62), "speed": slice(62, 68)}   # CB3 / e-Series
_LAYOUTS = {
    812: {"pose": slice(74, 80), "speed": slice(80, 86)},      # CB2 / SW 1.8
}


def _layout_for(total_bytes):
    """Field slices for the given packet size (defaults to the v3.x layout)."""
    return _LAYOUTS.get(total_bytes, _LAYOUT_V3)

# --------------------------------------------------------------------------- #
#                              ROBOT POSE READER                              #
# --------------------------------------------------------------------------- #


class RobotPoseReader(threading.Thread):
    """Background thread holding the most recent TCP pose + linear speed."""

    def __init__(self, host, port=ROBOT_PORT):
        super().__init__(daemon=True)
        self._host = host
        self._port = port
        self._sock = None
        self._lock = threading.Lock()
        self._pose = None          # [x,y,z,rx,ry,rz]
        self._speed = 0.0          # linear TCP speed magnitude (m/s)
        # NB: do not name this "_stop" - that shadows threading.Thread._stop
        # and breaks _after_fork() when multiprocessing forks subprocesses.
        self._stop_event = threading.Event()
        self.error = None

    def connect(self):
        self._open_socket()

    def _open_socket(self):
        # short timeout for the connect itself ...
        self._sock = socket.create_connection((self._host, self._port), timeout=5.0)
        # ... then a per-recv timeout so a *silently* stalled stream (no bytes,
        # connection still open) is detected instead of blocking forever.
        self._sock.settimeout(ROBOT_STREAM_TIMEOUT_S)

    def _recv_exact(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("robot socket closed")
            buf += chunk
        return buf

    def _read_one_packet(self):
        size_bytes = self._recv_exact(4)
        total = struct.unpack(">I", size_bytes)[0]
        # int32 size + float64 time leave (total - 12) bytes of doubles
        if total < 12 or (total - 12) % 8 != 0:
            # drain unknown/short packet and skip
            if total > 4:
                self._recv_exact(total - 4)
            return
        rest = self._recv_exact(total - 4)
        packet = size_bytes + rest
        n_doubles = (total - 12) // 8
        vals = struct.unpack(f">Id{n_doubles}d", packet)
        layout = _layout_for(total)
        pose = list(vals[layout["pose"]])
        vx, vy, vz = vals[layout["speed"]][:3]
        speed = math.sqrt(vx * vx + vy * vy + vz * vz)
        with self._lock:
            self._pose = pose
            self._speed = speed

    def run(self):
        # Outer loop reconnects on a stalled/closed stream; only a stop() or a
        # persistent failure to reconnect surfaces an error to the main thread.
        while not self._stop_event.is_set():
            try:
                while not self._stop_event.is_set():
                    self._read_one_packet()
            except socket.timeout:
                if self._stop_event.is_set():
                    return
                print("  [warn] robot stream stalled - reconnecting ...")
            except OSError as e:
                if self._stop_event.is_set():
                    return
                print(f"  [warn] robot stream error ({e}) - reconnecting ...")
            # tear down and reconnect
            try:
                self._sock.close()
            except OSError:
                pass
            if not self._reconnect():
                return

    def _reconnect(self):
        """Re-open the stream; returns False (and sets error) if we give up."""
        for attempt in range(1, 11):
            if self._stop_event.is_set():
                return False
            try:
                self._open_socket()
                print("  [warn] robot stream reconnected.")
                return True
            except OSError:
                self._stop_event.wait(0.5)
        self.error = ConnectionError("robot stream lost; reconnect failed")
        return False

    def latest(self):
        with self._lock:
            return (list(self._pose) if self._pose is not None else None,
                    self._speed)

    def stop(self):
        self._stop_event.set()
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass


# --------------------------------------------------------------------------- #
#                                LIVE PLOT                                     #
# --------------------------------------------------------------------------- #


def _load_surface_points():
    path = paths.SURFACE_POINTS_BASE
    if not os.path.exists(path):
        return None
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return ([float(r["x"]) for r in rows],
            [float(r["y"]) for r in rows],
            [float(r["z"]) for r in rows])


class LiveHtmlPlot:
    """Live view of the TCP path + presses over the cone surface, rendered as
    a self-refreshing Plotly HTML file by a background thread.

    add_sample()/add_press() just append to lock-protected buffers from the
    DAQ loop - cheap, no rendering happens there. A background thread wakes
    up every LIVE_PLOT_REFRESH_S, takes a quick snapshot, and writes the HTML
    (to a temp file, then atomically renamed into place so a browser never
    reads a half-written file). Worst case a slow render just falls behind;
    it can never block or slow down the recording loop."""

    def __init__(self, out_path):
        import plotly.graph_objects as go
        self._go = go
        self.out_path = out_path
        self._lock = threading.Lock()
        self._traj_pts = deque(maxlen=LIVE_PLOT_MAX_POINTS)
        self._press_pts = []   # (x, y, z, press_num, peak_fz)
        self._sample_count = 0
        self._surface = _load_surface_points()
        self._stop_event = threading.Event()
        self._render()   # fail fast here (e.g. bad out_path) before starting the thread
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        self._open_in_browser()

    def _open_in_browser(self):
        # Open once, here, so the page is already loaded (and its own meta
        # refresh keeps it live) instead of relying on the user to find and
        # open the file manually.
        import webbrowser
        try:
            webbrowser.open(f"file://{os.path.abspath(self.out_path)}")
        except Exception as e:
            print(f"  [warn] could not auto-open live plot ({e}); open it manually:")
            print(f"    {os.path.abspath(self.out_path)}")

    def add_sample(self, tip_xyz):
        self._sample_count += 1
        if self._sample_count % LIVE_PLOT_DECIMATE == 0:
            with self._lock:
                self._traj_pts.append(tip_xyz)

    def add_press(self, tip_xyz, press_num, peak_fz):
        with self._lock:
            self._press_pts.append((*tip_xyz, press_num, peak_fz))

    def _snapshot(self):
        with self._lock:
            return list(self._traj_pts), list(self._press_pts)

    def _render(self):
        go = self._go
        traj, presses = self._snapshot()
        traces = []
        if self._surface is not None:
            traces.append(go.Scatter3d(
                x=self._surface[0], y=self._surface[1], z=self._surface[2],
                mode="markers", marker=dict(size=2, color="lightgray", opacity=0.4),
                name="Cone surface", hoverinfo="skip",
            ))
        if traj:
            xs, ys, zs = zip(*traj)
            traces.append(go.Scatter3d(
                x=xs, y=ys, z=zs, mode="markers",
                marker=dict(size=2, color="black", opacity=0.6),
                name="TCP path",
            ))
        if presses:
            px, py, pz, pn, pf = zip(*presses)
            traces.append(go.Scatter3d(
                x=px, y=py, z=pz, mode="markers+text",
                marker=dict(size=8, color="red", symbol="diamond",
                            line=dict(color="black", width=1)),
                text=[f"#{int(n)} {f:.1f}N" for n, f in zip(pn, pf)],
                textposition="top center", textfont=dict(size=9, color="red"),
                name="Press peak",
            ))
        fig = go.Figure(traces)
        fig.update_layout(
            title=f"Live TCP trajectory + presses ({len(presses)} press(es))",
            margin=dict(t=60),
            scene=dict(xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
                       aspectmode="data"),
        )
        tmp_path = self.out_path + ".tmp"
        fig.write_html(tmp_path, include_plotlyjs="cdn")
        with open(tmp_path) as f:
            html = f.read()
        # Auto-refresh the browser tab so you don't have to reload by hand.
        # Trade-off: this resets pan/zoom/rotation on every refresh.
        refresh_tag = f'<meta http-equiv="refresh" content="{LIVE_PLOT_REFRESH_S}">'
        html = html.replace("<head>", f"<head>{refresh_tag}", 1)
        with open(tmp_path, "w") as f:
            f.write(html)
        os.replace(tmp_path, self.out_path)

    def _run(self):
        while not self._stop_event.is_set():
            self._stop_event.wait(LIVE_PLOT_REFRESH_S)
            try:
                self._render()
            except Exception as e:
                print(f"  [warn] live plot render failed ({e})")

    def close(self):
        self._stop_event.set()
        self._thread.join(timeout=2.0)
        try:
            self._render()   # final snapshot with the last presses included
        except Exception:
            pass


# --------------------------------------------------------------------------- #
#                                    MAIN                                      #
# --------------------------------------------------------------------------- #


def select_host():
    while True:
        mode = input("Select mode ('sim' or 'real'): ").strip().lower()
        if mode == "sim":
            return SIM_HOST
        if mode == "real":
            return REAL_HOST
        print("Invalid input. Please type 'sim' or 'real'.")


def main():
    host = select_host()

    base_dir = os.path.dirname(os.path.abspath(__file__))
    out_dir = os.path.join(base_dir, "cone_data")
    os.makedirs(out_dir, exist_ok=True)
    stamp = strftime("%Y-%m-%d_%H-%M-%S", localtime())
    traj_path = os.path.join(out_dir, f"{stamp}_trajectory.csv")
    press_path = os.path.join(out_dir, f"{stamp}_presses.csv")

    # --- set up force sensor FIRST ------------------------------------------ #
    # DataRecorder forks the sensor subprocess; do this before starting any
    # reader thread so Python's _after_fork() has no extra threads to touch.
    sensor = SensorSettings(device_id="1",
                            calibration_folder=CALIBRATION_FOLDER,
                            sensor_name=SENSOR_NAME,
                            rate=SENSOR_RATE,
                            reverse_parameter_names=REVERSE_FZ)
    recorder = DataRecorder(force_sensor_settings=[sensor],
                            poll_udp_connection=False, polling_priority="normal")

    print("\nSetting bias - DO NOT TOUCH THE SENSOR ...")
    recorder.determine_biases(n_samples=BIAS_SAMPLES)
    print("Sensor calibrated (biased).")

    # Force is read live from shared memory (proc.get_Fxyz) and logged, synced
    # with the TCP pose, into <ts>_trajectory.csv. We deliberately do NOT open a
    # pyForceDAQ data file: the slow cone presses are fully captured at the
    # 125 Hz loop rate, and that buffered .gz did not reliably flush on Ctrl-C.
    recorder.start_recording()
    proc = recorder.force_sensor_processes()[0]

    # --- connect to robot (after all forking is done) ----------------------- #
    print(f"Connecting to robot at {host}:{ROBOT_PORT} ...")
    reader = RobotPoseReader(host)
    reader.connect()
    reader.start()
    # wait for the first decoded pose
    t0 = time.time()
    while reader.latest()[0] is None:
        if reader.error is not None:
            raise reader.error
        if time.time() - t0 > 5.0:
            raise TimeoutError("No pose received from robot real-time stream.")
        time.sleep(0.05)
    print("Robot pose stream OK.")

    traj_file = open(traj_path, "w", newline="")
    press_file = open(press_path, "w", newline="")
    traj_w = csv.writer(traj_file)
    press_w = csv.writer(press_file)
    traj_w.writerow(["t", "x", "y", "z", "rx", "ry", "rz", "speed",
                     "Fx", "Fy", "Fz", "Fmag"])
    press_w.writerow(["press", "t_peak", "peak_Fz", "peak_Fmag",
                      "Fx", "Fy", "Fz", "x", "y", "z", "rx", "ry", "rz"])

    live_plot = None
    if LIVE_PLOT:
        live_html_path = os.path.join(out_dir, "live_view.html")
        try:
            live_plot = LiveHtmlPlot(live_html_path)
            print(f"Live plot -> {live_html_path}  (open in a browser; "
                  f"auto-refreshes every {LIVE_PLOT_REFRESH_S:.0f}s)")
        except Exception as e:
            print(f"  [warn] live plot disabled ({e})")

    print("\nRecording. Start your motion script now.")
    print(f"  trajectory -> {traj_path}")
    print(f"  presses    -> {press_path}")
    print("Press Ctrl-C to stop.\n")

    period = 1.0 / LOOP_HZ
    in_press = False
    press_count = 0
    peak = None          # dict captured at max Fz of the current press
    press_start_t = 0.0
    last_press_end_t = 0.0   # for the refractory window
    off_since = None         # when Fz first dropped below PRESS_OFF_N (debounce)

    try:
        next_t = time.perf_counter()
        while True:
            fx, fy, fz = proc.get_Fxyz()
            fmag = math.sqrt(fx * fx + fy * fy + fz * fz)
            pose, speed = reader.latest()
            if reader.error is not None:
                raise reader.error
            now = time.time()

            if pose is not None:
                traj_w.writerow([f"{now:.6f}", *[f"{p:.6f}" for p in pose],
                                 f"{speed:.6f}",
                                 f"{fx:.4f}", f"{fy:.4f}", f"{fz:.4f}",
                                 f"{fmag:.4f}"])
                if live_plot is not None:
                    live_plot.add_sample(tcp_pose_to_contact(pose[:3], pose[3:]))

            # --- press state machine (on Fmag) ------------------------------ #
            # Thresholding on signed Fz alone misses real contact in regions
            # where the local surface normal isn't aligned with the sensor's Z
            # axis (e.g. near the cone's embedded bulge): a touch there can load
            # mostly Fx/Fy with Fz negative, never crossing a positive Fz
            # threshold even though |F| is well above it. Fmag is sign-agnostic
            # and catches contact regardless of which axis it loads.
            if not in_press:
                if fmag >= PRESS_ON_N and (now - last_press_end_t) >= PRESS_REFRACTORY_S:
                    in_press = True
                    press_start_t = now
                    off_since = None
                    peak = {"fz": fz, "f": (fx, fy, fz), "fmag": fmag,
                            "pose": pose, "t": now}
            else:
                if fmag > peak["fmag"]:
                    peak = {"fz": fz, "f": (fx, fy, fz), "fmag": fmag,
                            "pose": pose, "t": now}
                if fmag <= PRESS_OFF_N:
                    if off_since is None:
                        off_since = now
                    elif (now - off_since) >= PRESS_OFF_DEBOUNCE_S:
                        if (now - press_start_t) >= MIN_PRESS_DURATION_S:
                            press_count += 1
                            p = peak["pose"] if peak["pose"] is not None else [float("nan")] * 6
                            press_w.writerow([press_count, f"{peak['t']:.6f}",
                                              f"{peak['fz']:.4f}", f"{peak['fmag']:.4f}",
                                              *[f"{v:.4f}" for v in peak["f"]],
                                              *[f"{v:.6f}" for v in p]])
                            press_file.flush()
                            print(f"Press {press_count:>3}: peak Fz = {peak['fz']:.2f} N "
                                  f"(|F| = {peak['fmag']:.2f} N) at "
                                  f"TCP=[{p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f}]")
                            if live_plot is not None:
                                live_plot.add_press(tcp_pose_to_contact(p[:3], p[3:]),
                                                     press_count, peak["fz"])
                        in_press = False
                        last_press_end_t = now
                        peak = None
                        off_since = None
                else:
                    off_since = None  # Fmag back above PRESS_OFF_N - reset the debounce

            # pace the loop
            next_t += period
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()
    except KeyboardInterrupt:
        print("\nStopping ...")
    finally:
        traj_file.close()
        press_file.close()
        reader.stop()
        recorder.quit()
        print(f"\nDone. {press_count} press(es) recorded.")
        print(f"  {traj_path}")
        print(f"  {press_path}")
        if live_plot is not None:
            live_plot.close()
            print(f"  {live_plot.out_path} (final state)")


if __name__ == "__main__":
    main()
