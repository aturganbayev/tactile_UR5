#!/usr/bin/env python3
"""
Synchronized cone-press recorder for the UR5 + ATI Nano17 (FT12876).

Run this ALONGSIDE a motion script (e.g. ../run_random_upper_poses.py) that
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
  <name>.csv.gz          - raw full-rate force from pyForceDAQ (backup)

Stop with Ctrl-C.

See COPYING file distributed along with the pyForceDAQ copyright and license terms.
"""

import csv
import math
import os
import socket
import struct
import threading
import time
from time import strftime, localtime

from forceDAQ.force.data_recorder import DataRecorder
from forceDAQ.force.sensor import SensorSettings

# --------------------------------------------------------------------------- #
#                                  SETTINGS                                    #
# --------------------------------------------------------------------------- #

# Robot real-time interface
SIM_HOST = "172.17.0.2"
REAL_HOST = "192.168.0.110"
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

# Press detection on Fz (Newtons). Hysteresis prevents flicker.
PRESS_ON_N = 0.5           # Fz rising above this starts a press
PRESS_OFF_N = 0.3          # Fz falling below this ends the press
MIN_PRESS_DURATION_S = 0.05  # ignore shorter blips as noise

# Logging loop rate (the UR stream is ~125 Hz)
LOOP_HZ = 125

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
                            reverse_parameter_names=REVERSE_FZ)
    recorder = DataRecorder(force_sensor_settings=[sensor],
                            poll_udp_connection=False, polling_priority="normal")

    print("\nSetting bias - DO NOT TOUCH THE SENSOR ...")
    recorder.determine_biases(n_samples=BIAS_SAMPLES)
    print("Sensor calibrated (biased).")

    recorder.open_data_file(filename=f"{stamp}.csv", subdirectory="cone_data",
                            zipped=True, time_stamp_filename=False, comment_line="")
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

    print("\nRecording. Start your motion script now.")
    print(f"  trajectory -> {traj_path}")
    print(f"  presses    -> {press_path}")
    print("Press Ctrl-C to stop.\n")

    period = 1.0 / LOOP_HZ
    in_press = False
    press_count = 0
    peak = None          # dict captured at max Fz of the current press
    press_start_t = 0.0

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

            # --- press state machine (on Fz) ------------------------------- #
            if not in_press:
                if fz >= PRESS_ON_N:
                    in_press = True
                    press_start_t = now
                    peak = {"fz": fz, "f": (fx, fy, fz), "fmag": fmag,
                            "pose": pose, "t": now}
            else:
                if fz > peak["fz"]:
                    peak = {"fz": fz, "f": (fx, fy, fz), "fmag": fmag,
                            "pose": pose, "t": now}
                if fz <= PRESS_OFF_N:
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
                    in_press = False
                    peak = None

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


if __name__ == "__main__":
    main()
