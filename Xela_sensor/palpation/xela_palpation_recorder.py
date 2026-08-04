#!/usr/bin/env python3
"""
XELA palpation recorder.

RUNS ON THIS WORKSTATION (the machine with the CAN-USB adapter), NOT the DAQ
PC. Unlike the calibration session, everything needed is reachable from one
machine - the XELA websocket (localhost) and the robot's realtime stream
(192.168.0.110:30003) - so there is no cross-machine clock alignment here.

Setup: XELA sensor mounted on the UR5 end-effector (data/xela_sensor_mounted
CAD), phantom ("egg") on the platform. Run this alongside a motion script that
walks the pose grid, e.g.:

    # terminal 1 (here)
    python3 Xela_sensor/palpation/xela_palpation_recorder.py
    # terminal 2
    python3 execution/run_side_strip_poses.py     # choose 'real'

It records the tactile response synchronised with the TCP pose, auto-detects
each press, and logs the peak response per press.

*** UNITS - READ THIS ***
The XR1944 outputs RAW UNCALIBRATED counts. This logs raw counts plus a
summed-|dZ| response magnitude, NOT Newtons. Per-taxel force calibration was
found unachievable on this sensor (inherent cross-talk: above ~1N the peak
response migrates to a neighbouring taxel, and indenter changes did not help;
plus ~1000-count baseline drift over minutes, comparable to the signal). To
report Newtons, fit a TOTAL-force model (summed response -> N, which is immune
to cross-talk because it does not matter which taxel the load spreads to)
against the Nano17 and apply it to the `response` column afterwards.

BASELINE HANDLING
Because the baseline drifts, it is re-measured continuously whenever the tool
is in free-space transit (TCP speed above BASELINE_SPEED_MS - it cannot be in
contact while moving that fast). Every press is therefore referenced to a
baseline taken seconds earlier, not to a stale session-start value.

PREREQUISITES
  1. can0 up:            sudo ip link set up can0 type can bitrate 1000000
  2. xela_server running: cd Xela_sensor && ./xela_server -f xServ.ini --ip 0.0.0.0
  3. Sensor triggered if needed: cansend can0 203#07.00
  4. Robot powered on, in remote control, phantom mounted on the platform.

Stop with Ctrl-C (files are flushed on exit).
"""

import csv
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from collections import deque

import numpy as np
import websocket

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_XELA_DIR = os.path.dirname(_THIS_DIR)
REPO_ROOT = os.path.dirname(_XELA_DIR)
sys.path.insert(0, REPO_ROOT)

from pose_utils import REAL_HOST, SIM_HOST

DATA_DIR = os.path.join(_THIS_DIR, "data")
os.makedirs(DATA_DIR, exist_ok=True)

# --------------------------------------------------------------------------- #
#                                 SETTINGS                                    #
# --------------------------------------------------------------------------- #

XELA_WS_URL = "ws://127.0.0.1:5000"
ROBOT_PORT = 30003
ROBOT_STREAM_TIMEOUT_S = 1.0

LOOP_HZ = 60          # logging rate (XELA itself updates at ~120 Hz)
N_TAXELS = 16

# --- baseline (see module docstring) ---
BASELINE_SPEED_MS = 0.02   # TCP speed above this => free-space transit, safe
                            # to refresh the baseline
BASELINE_WINDOW = 30        # samples averaged into the rolling baseline

# --- press detection, on the summed |dZ| response (COUNTS, not Newtons) ---
# These are starting guesses: palpation response magnitudes on this phantom
# have not been characterised yet. The continuous trajectory log always
# contains everything, so presses can be re-extracted offline with better
# thresholds - a wrong threshold costs nothing but the live per-press summary.
# Run with --monitor first to see live values and tune these.
PRESS_ON_COUNTS = 3000
PRESS_OFF_COUNTS = 1500
MIN_PRESS_DURATION_S = 0.05
PRESS_OFF_DEBOUNCE_S = 0.5
PRESS_REFRACTORY_S = 1.5
# A real press contains a near-stationary hold; grazing contacts during
# transit do not. Same idea as pyForceDAQ/record_cone_press.py's speed gate.
PRESS_HOLD_SPEED_MS = 0.01

# --- OVERLOAD ABORT ---
# With the Nano17 out of the load path there is NO force measurement, so the
# XELA response is the only contact signal there is - it doubles as the
# overload guard. This matters because the ICP calibration error (~1.5-2.2 mm
# RMS) is comparable to the commanded press depth, so some poses will indent
# considerably deeper than intended and nothing else would notice.
#
# On trip, a stopl() program is streamed to the robot. That both decelerates
# the current move AND replaces the running program, so it halts the rest of
# the sweep (same mechanism as execution/stop_robot.py).
#
# TUNE THIS with --monitor before a real run: it must sit above the largest
# response a normal press produces but low enough to catch a runaway. The
# default is only a placeholder. Set 0 to disable (not recommended here).
ABORT_RESPONSE_COUNTS = 60000

# UR realtime packet layout (see pyForceDAQ/record_cone_press.py)
_LAYOUT_V3 = {"pose": slice(56, 62), "speed": slice(62, 68)}
_LAYOUTS = {812: {"pose": slice(74, 80), "speed": slice(80, 86)}}


def _layout_for(total_bytes):
    return _LAYOUTS.get(total_bytes, _LAYOUT_V3)


# --------------------------------------------------------------------------- #
#                                  READERS                                    #
# --------------------------------------------------------------------------- #

class XelaReader:
    """Background thread holding the latest parsed XELA sensor-1 sample."""

    def __init__(self, url=XELA_WS_URL, sensor_key="1"):
        self._url = url
        self._sensor_key = sensor_key
        self._lock = threading.Lock()
        self._latest = None      # (recv_time, np.array of 48 floats)
        self._ws = None
        self.error = None

    def _on_message(self, ws, message):
        recv_time = time.time()
        try:
            s = json.loads(message)[self._sensor_key]
            vals = np.array([int(v, 16) for v in s["data"].split(",")], dtype=float)
        except Exception:
            return
        if vals.size != 3 * N_TAXELS:
            return
        with self._lock:
            self._latest = (recv_time, vals)

    def _on_error(self, ws, error):
        self.error = error

    def start(self):
        self._ws = websocket.WebSocketApp(
            self._url, on_message=self._on_message, on_error=self._on_error)
        threading.Thread(target=self._ws.run_forever, daemon=True).start()

    def latest(self):
        with self._lock:
            return self._latest

    def wait_for_data(self, timeout=5.0):
        t0 = time.time()
        while self.latest() is None:
            if self.error is not None:
                raise RuntimeError(f"XELA websocket error: {self.error}")
            if time.time() - t0 > timeout:
                raise TimeoutError(
                    "No data from XELA server - is xela_server running "
                    "(--ip 0.0.0.0) and the sensor triggered "
                    "(cansend can0 203#07.00)?")
            time.sleep(0.05)

    def stop(self):
        if self._ws is not None:
            self._ws.close()


class RobotPoseReader(threading.Thread):
    """Background thread holding the most recent TCP pose + linear speed.

    Duplicated from pyForceDAQ/record_cone_press.py rather than imported: that
    module pulls in forceDAQ/nidaqmx, which only exist on the DAQ PC.
    """

    def __init__(self, host, port=ROBOT_PORT):
        super().__init__(daemon=True)
        self._host = host
        self._port = port
        self._sock = None
        self._lock = threading.Lock()
        self._pose = None
        self._speed = 0.0
        self._stop_event = threading.Event()
        self.error = None

    def connect(self):
        self._open_socket()

    def _open_socket(self):
        self._sock = socket.create_connection((self._host, self._port), timeout=5.0)
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
        if total < 12 or (total - 12) % 8 != 0:
            if total > 4:
                self._recv_exact(total - 4)
            return
        packet = size_bytes + self._recv_exact(total - 4)
        n_doubles = (total - 12) // 8
        vals = struct.unpack(f">Id{n_doubles}d", packet)
        layout = _layout_for(total)
        pose = list(vals[layout["pose"]])
        vx, vy, vz = vals[layout["speed"]][:3]
        with self._lock:
            self._pose = pose
            self._speed = math.sqrt(vx * vx + vy * vy + vz * vz)

    def run(self):
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
            try:
                self._sock.close()
            except OSError:
                pass
            if not self._reconnect():
                return

    def _reconnect(self):
        for _ in range(10):
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
#                               RESPONSE MATHS                                #
# --------------------------------------------------------------------------- #

def split_xyz(vals):
    """(48,) flat sample -> (x, y, z) arrays of length 16."""
    return vals[0::3], vals[1::3], vals[2::3]


def response_scalar(delta):
    """Uncalibrated total-normal-response proxy: sum |dZ| over all taxels."""
    return float(np.abs(delta[2::3]).sum())


def emergency_stop(host):
    """Stream a stopl() program, halting the current move and replacing the
    running sweep program (see ABORT_RESPONSE_COUNTS)."""
    try:
        s = socket.create_connection((host, ROBOT_PORT), timeout=2.0)
        s.sendall(b"def estop():\n  stopl(2.5)\nend\nestop()\n")
        time.sleep(0.5)
        s.close()
        return True
    except Exception as e:
        print(f"  [ERROR] could not send stop to the robot: {e}")
        return False


# --------------------------------------------------------------------------- #
#                                   MAIN                                      #
# --------------------------------------------------------------------------- #

def select_host():
    while True:
        mode = input("Select mode ('sim' or 'real'): ").strip().lower()
        if mode == "sim":
            return SIM_HOST
        if mode == "real":
            return REAL_HOST
        print("Invalid input. Please type 'sim' or 'real'.")


def monitor(xela, reader):
    """Print live response so PRESS_ON/OFF_COUNTS can be tuned."""
    print("\nMonitor mode - press the sensor by hand and watch the response.")
    print("Pick PRESS_ON_COUNTS comfortably above the idle noise.  Ctrl-C to exit.\n")
    baseline = None
    hist = deque(maxlen=BASELINE_WINDOW)
    try:
        while True:
            lx = xela.latest()
            pose, speed = reader.latest() if reader else (None, 0.0)
            if lx is not None:
                _, vals = lx
                if reader is None or speed > BASELINE_SPEED_MS or baseline is None:
                    hist.append(vals)
                    if len(hist) >= 5:
                        baseline = np.mean(hist, axis=0)
                if baseline is not None:
                    r = response_scalar(vals - baseline)
                    bar = "#" * min(60, int(r / 500))
                    print(f"\rresponse = {r:9.0f}  speed={speed:6.4f}  {bar:<60}",
                          end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nmonitor stopped.")


def main():
    args = [a for a in sys.argv[1:]]
    monitor_mode = "--monitor" in args
    args = [a for a in args if not a.startswith("--")]

    print("Connecting to XELA server ...")
    xela = XelaReader()
    xela.start()
    xela.wait_for_data()
    print("XELA OK.")

    host = select_host()
    print(f"Connecting to robot at {host}:{ROBOT_PORT} ...")
    reader = RobotPoseReader(host)
    reader.connect()
    reader.start()
    t0 = time.time()
    while reader.latest()[0] is None:
        if reader.error is not None:
            raise reader.error
        if time.time() - t0 > 5.0:
            raise TimeoutError("No pose received from robot realtime stream.")
        time.sleep(0.05)
    print("Robot pose stream OK.")

    if monitor_mode:
        try:
            monitor(xela, reader)
        finally:
            xela.stop()
            reader.stop()
        return

    egg = (args[0] if args else
           input("Egg / phantom name (subfolder, or 'none'): ").strip())
    out_dir = DATA_DIR if egg.lower() in ("", "none") else os.path.join(DATA_DIR, egg)
    os.makedirs(out_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    traj_path = os.path.join(out_dir, f"{stamp}_trajectory.csv")
    press_path = os.path.join(out_dir, f"{stamp}_presses.csv")

    # Interleaved (x0,y0,z0,x1,y1,z1,...) to match the order of `vals` and
    # `delta`, which are written straight out below.
    #
    # BUG FIX 2026-07-28: this was built axis-grouped (x0..x15, y0..y15,
    # z0..z15) while the values written are interleaved, so every column past
    # raw_x0 carried the wrong label - raw_x1 actually held taxel 0's Y, and
    # so on. The symptom was a "high" idle channel that rotated z,y,x,z,y,x...
    # across taxels in the logged CSVs, even though the live stream has Z high
    # on all 16. Values were never wrong, only their names; existing logs are
    # fully recoverable with fix_logged_channel_labels.py.
    ch = [f"{ax}{i}" for i in range(N_TAXELS) for ax in ("x", "y", "z")]
    traj_fields = (["t", "px", "py", "pz", "rx", "ry", "rz", "speed",
                    "response", "baseline_age_s"]
                   + [f"raw_{c}" for c in ch] + [f"d_{c}" for c in ch])
    press_fields = (["press", "t_peak", "peak_response", "px", "py", "pz",
                     "rx", "ry", "rz"] + [f"d_{c}" for c in ch])

    traj_f = open(traj_path, "w", newline="")
    press_f = open(press_path, "w", newline="")
    traj_w = csv.writer(traj_f)
    press_w = csv.writer(press_f)
    traj_w.writerow(traj_fields)
    press_w.writerow(press_fields)

    print(f"\nRecording (raw counts - NOT Newtons; see the header comment).")
    print(f"  trajectory -> {traj_path}")
    print(f"  presses    -> {press_path}")
    print(f"  press detect : on>={PRESS_ON_COUNTS}  off<={PRESS_OFF_COUNTS} counts")
    if ABORT_RESPONSE_COUNTS:
        print(f"  OVERLOAD ABORT: {ABORT_RESPONSE_COUNTS} counts -> stopl()"
              "   <-- tune with --monitor; this is the only contact guard")
    else:
        print("  OVERLOAD ABORT: DISABLED  <-- no contact guard at all")
    print("Start your motion script now. Ctrl-C to stop.\n")

    baseline = None
    baseline_t = 0.0
    hist = deque(maxlen=BASELINE_WINDOW)
    in_press = False
    overloaded = False
    press_count = 0
    peak = None
    press_start_t = 0.0
    last_press_end_t = 0.0
    off_since = None
    saw_hold = False
    period = 1.0 / LOOP_HZ
    next_t = time.perf_counter()

    try:
        while True:
            lx = xela.latest()
            pose, speed = reader.latest()
            now = time.time()

            if lx is not None and pose is not None:
                _, vals = lx

                # --- rolling baseline while in free-space transit ---
                if speed > BASELINE_SPEED_MS or baseline is None:
                    hist.append(vals)
                    if len(hist) >= 5:
                        baseline = np.mean(hist, axis=0)
                        baseline_t = now

                if baseline is not None:
                    delta = vals - baseline
                    r = response_scalar(delta)

                    # --- overload abort (only contact guard without a force
                    # sensor in the load path) ---
                    if (ABORT_RESPONSE_COUNTS and r >= ABORT_RESPONSE_COUNTS
                            and not overloaded):
                        overloaded = True
                        print(f"\n*** OVERLOAD: response {r:.0f} >= "
                              f"{ABORT_RESPONSE_COUNTS} counts - STOPPING ROBOT ***")
                        emergency_stop(host)
                        print("    Robot stop sent. Still logging; Ctrl-C when safe.")
                    traj_w.writerow(
                        [f"{now:.6f}"] + [f"{v:.6f}" for v in pose]
                        + [f"{speed:.6f}", f"{r:.1f}", f"{now - baseline_t:.2f}"]
                        + [f"{v:.0f}" for v in vals]
                        + [f"{v:.1f}" for v in delta])

                    # --- press detection ---
                    if not in_press:
                        if (r >= PRESS_ON_COUNTS
                                and (now - last_press_end_t) >= PRESS_REFRACTORY_S):
                            in_press = True
                            press_start_t = now
                            off_since = None
                            saw_hold = speed <= PRESS_HOLD_SPEED_MS
                            peak = {"r": r, "t": now, "pose": list(pose),
                                    "delta": delta.copy()}
                    else:
                        if speed <= PRESS_HOLD_SPEED_MS:
                            saw_hold = True
                        if r > peak["r"]:
                            peak = {"r": r, "t": now, "pose": list(pose),
                                    "delta": delta.copy()}
                        if r <= PRESS_OFF_COUNTS:
                            if off_since is None:
                                off_since = now
                            elif now - off_since >= PRESS_OFF_DEBOUNCE_S:
                                dur = off_since - press_start_t
                                if dur >= MIN_PRESS_DURATION_S and saw_hold:
                                    press_count += 1
                                    press_w.writerow(
                                        [press_count, f"{peak['t']:.6f}",
                                         f"{peak['r']:.1f}"]
                                        + [f"{v:.6f}" for v in peak["pose"]]
                                        + [f"{v:.1f}" for v in peak["delta"]])
                                    press_f.flush()
                                    print(f"Press {press_count:>3}: peak response "
                                          f"= {peak['r']:.0f} counts at TCP="
                                          f"[{peak['pose'][0]:.4f}, "
                                          f"{peak['pose'][1]:.4f}, "
                                          f"{peak['pose'][2]:.4f}]")
                                elif not saw_hold:
                                    print("  [info] ignored moving contact "
                                          f"(peak {peak['r']:.0f} counts)")
                                in_press = False
                                last_press_end_t = now
                                peak = None
                        else:
                            off_since = None

            next_t += period
            sleep_for = next_t - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_t = time.perf_counter()

    except KeyboardInterrupt:
        pass
    finally:
        traj_f.close()
        press_f.close()
        xela.stop()
        reader.stop()
        print(f"\nDone. {press_count} press(es) recorded.")
        print(f"  {traj_path}")
        print(f"  {press_path}")


if __name__ == "__main__":
    main()
