#!/usr/bin/env python3
"""
RUNS ON THE DAQ PC ONLY (where the NI-DAQmx driver + Nano17 hardware live,
and which already has network access to the robot - same machine
record_cone_press.py runs on).

Per-taxel closed-loop force-guided press: automatically visits all 16 taxel
positions (see taxel_geometry.py - derived from touch-probed measurements,
not nominal dimensions), and at each one steps the UR5 down in small
increments along the tool's local +Z axis, holding at several target Fmag
checkpoints. Logs a continuous trajectory and a per-checkpoint summary,
tagged by taxel index.

This does NOT read the XELA sensor at all - that lives on a different
machine (the one with the CAN-USB adapter). Run xela_session_logger.py
there, started before this script and stopped after it finishes, and align
the two logs afterward by wall-clock timestamp (fit step - see project
notes), using this session's *_checkpoints.csv start/end times plus the
taxel index to know which column of the XELA log corresponds to which row.

Only reads from pyForceDAQ (RobotPoseReader, DataRecorder, SensorSettings)
and taxel_geometry.py - does not modify anything in pyForceDAQ/.

PREREQUISITES:
  1. No other pyForceDAQ script is currently holding the Nano17 DAQ device.
  2. taxel_geometry.POINT0_EE_POSE_MM matches the CURRENT sensor mounting -
     re-measure and update it there if the sensor or platform ever moves.
  3. Clocks on this machine and the XELA-logging machine should be roughly
     in sync (a fraction of a second is fine for these quasi-static holds).

Safety (all per-taxel - one taxel aborting does not stop the whole sweep):
  * Hard abort + immediate retract-to-hover if Fmag ever exceeds MAX_SAFE_N.
  * Absolute travel cap (MAX_TOTAL_STEPS) independent of the force reading,
    so a stuck/disconnected Nano17 can't drive the robot forward forever.
  * Approach phase gives up (retract, no data for that taxel) if contact
    isn't detected within MAX_APPROACH_STEPS.
  * All motion uses the slow contact speed/accel already validated for the
    cone presses (pose_utils.A_approach_real / V_approach_real).
  * Supervise this run - it drives 16 automated presses in a row with only
    one confirmation prompt at the start.

Usage:
    python3 nano17_press_session.py [session_label] [taxel_index]

    session_label defaults to a timestamp. If taxel_index (0-15) is given,
    only that one taxel is visited (useful to sanity-check the geometry /
    safety limits before committing to the full 16-taxel sweep).
"""

import csv
import math
import os
import socket
import sys
import time

import numpy as np

# --------------------------------------------------------------------------- #
#                                   PATHS                                     #
# --------------------------------------------------------------------------- #
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))    # .../Xela_sensor/calibration
_XELA_DIR = os.path.dirname(_THIS_DIR)                     # .../Xela_sensor
REPO_ROOT = os.path.dirname(_XELA_DIR)                     # repo root
PYFORCEDAQ_DIR = os.path.join(REPO_ROOT, "pyForceDAQ")

sys.path.insert(0, REPO_ROOT)        # pose_utils, paths
sys.path.insert(0, PYFORCEDAQ_DIR)   # forceDAQ package, record_cone_press
sys.path.insert(0, _THIS_DIR)        # taxel_geometry

from pose_utils import REAL_HOST, A_approach_real, V_approach_real, pose_str
from forceDAQ.force.data_recorder import DataRecorder
from forceDAQ.force.sensor import SensorSettings
from record_cone_press import RobotPoseReader, ROBOT_PORT  # reused, not modified
from taxel_geometry import taxel_hover_pose, HOVER_CLEARANCE_M

DATA_DIR = os.path.join(_THIS_DIR, "data")
os.makedirs(DATA_DIR, exist_ok=True)

# --------------------------------------------------------------------------- #
#                                 CONSTANTS                                   #
# --------------------------------------------------------------------------- #

SENSOR_NAME = "FT12876"
CALIBRATION_FOLDER = os.path.join(PYFORCEDAQ_DIR, "calibration")
SENSOR_RATE = 500
BIAS_SAMPLES = 500
REVERSE_FZ = "Fz"     # press -> positive Fz, matches pyForceDAQ's convention

APPROACH_STEP_M = 0.0003        # 0.3 mm/step for the whole descent to first
                                 # contact - uniform (no position-based fine
                                 # zone: the sensor plane is slightly tilted so
                                 # a single measured contact height can't be
                                 # trusted per-taxel). Small enough for a gentle
                                 # contact, big enough to keep the descent quick.
STEP_M = 0.0001                 # 0.1 mm/step IN CONTACT - fine enough to land
                                 # on each low-force checkpoint without the
                                 # >1.5N per-step overshoot a coarse step gives
MAX_APPROACH_STEPS = 30         # descend up to ~9mm before giving up - enough
                                 # to reach contact across the tilted plane even
                                 # where a taxel sits several mm lower than the
                                 # measured reference point
MAX_TOTAL_STEPS = 120           # absolute cap on in-contact fine steps (12mm),
                                 # independent of Fmag - guards against a
                                 # stuck/dead force reading
SETTLE_TIMEOUT_S = 2.0
SETTLE_SPEED_MS = 0.01          # same threshold as PRESS_HOLD_SPEED_MS

# Per-taxel calibration is only valid at low force: above ~2N this sensor's
# mechanical cross-talk + the rounded tip's spreading contact make the peak
# response jump to a neighbour taxel (confirmed in the 2026-07-25 XELA log).
# So cap at 2N, and keep the safety ceiling just above it - the sensor's cell
# stress at higher force also risks damage.
CONTACT_ON_N = 0.15
CHECKPOINTS_N = [0.25, 0.5, 0.75, 1.0, 1.5, 2.0]
MAX_SAFE_N = 3.0
HOLD_S = 0.6
HOLD_POLL_HZ = 30

TRAJ_LOG_HZ = 60   # continuous trajectory logging rate during each press


# --------------------------------------------------------------------------- #
#                               MOTION HELPERS                                #
# --------------------------------------------------------------------------- #

def send_movel(host, pose, a=A_approach_real, v=V_approach_real):
    """Fire-and-forget a single movel via the secondary client port (30002)."""
    # NB: function name has NO leading underscore. The CB2 PolyScope 1.x
    # URScript parser rejects identifiers like `_step` (silently - the
    # program just never runs), even though newer e-series URSim accepts
    # them. Every working script here uses a plain-letter name.
    ur_script = (
        "def press_step():\n"
        f"  movel(p[{pose_str(pose)}], a={a}, v={v})\n"
        "end\npress_step()\n"
    )
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    s.connect((host, 30002))
    s.sendall(ur_script.encode("ascii"))
    # Give the CB2 controller time to latch + start the program before the
    # socket closes. This MUST be ~1s: at 0.5s the real controller silently
    # drops moves after a few steps and the robot freezes mid-descent (seen
    # 2026-07-25: runs 142322/143625 froze at ~1mm, while the 1.0s run 133348
    # pressed smoothly to 8N). The loop is still fast because wait_until_settled
    # no longer wastes a timeout per step.
    time.sleep(1.0)
    s.close()


def wait_until_settled(reader, timeout=SETTLE_TIMEOUT_S,
                        speed_threshold=SETTLE_SPEED_MS):
    """Confirm the robot is stopped after a commanded step.

    send_movel() already sleeps long enough for these small (<=0.5mm) moves to
    finish - they complete in well under 100ms. Do NOT try to detect motion
    'starting' here: a 0.1mm step peaks at ~4.5 mm/s, below SETTLE_SPEED_MS,
    so it would never be seen as moving and would waste the whole timeout on
    every step. Just verify we're stopped and return.
    """
    t0 = time.time()
    time.sleep(0.05)
    while True:
        _, speed = reader.latest()
        if speed <= speed_threshold:
            return True
        if time.time() - t0 > timeout:
            return False
        time.sleep(0.01)


def step_along_tool_z(reader, host, dz_m):
    """Move dz_m along the TOOL's current local +Z axis (base frame).

    Positive dz_m presses further INTO the surface (pose_utils' "TCP +Z
    points into the surface" convention).
    """
    from pose_utils import rotvec_to_matrix
    pose, _ = reader.latest()
    xyz = np.array(pose[:3])
    rotvec = pose[3:]
    z_axis = rotvec_to_matrix(rotvec)[:, 2]
    target = np.concatenate([xyz + dz_m * z_axis, rotvec])
    send_movel(host, target)
    wait_until_settled(reader)
    return target


def move_to_pose(reader, host, pose):
    send_movel(host, pose, a=A_approach_real, v=V_approach_real)
    wait_until_settled(reader, timeout=5.0)


# --------------------------------------------------------------------------- #
#                          PER-TAXEL PRESS ROUTINE                            #
# --------------------------------------------------------------------------- #

def press_one_taxel(taxel_index, reader, proc, traj_rows, cp_rows):
    """Run the approach + checkpoint-sweep for one taxel; always retracts to
    its hover pose before returning. Returns True if it aborted early."""

    hover_pose = taxel_hover_pose(taxel_index)
    print(f"\n=== Taxel {taxel_index:2d}: moving to hover pose ===")
    move_to_pose(reader, REAL_HOST, hover_pose)

    aborted = False
    contacted = False
    total_steps = 0
    next_checkpoint_idx = 0

    def log_traj():
        fx, fy, fz = proc.get_Fxyz()
        fmag = math.sqrt(fx * fx + fy * fy + fz * fz)
        pose, speed = reader.latest()
        traj_rows.append({"t": time.time(), "taxel": taxel_index,
                           "Fx": fx, "Fy": fy, "Fz": fz, "Fmag": fmag,
                           "speed": speed, "x": pose[0], "y": pose[1],
                           "z": pose[2]})
        return fx, fy, fz, fmag

    try:
        # --- approach phase: uniform descent until first contact ---
        for _ in range(MAX_APPROACH_STEPS):
            fx, fy, fz, fmag = log_traj()
            if fmag >= CONTACT_ON_N:
                contacted = True
                break
            step_along_tool_z(reader, REAL_HOST, APPROACH_STEP_M)

        if not contacted:
            print(f"  [abort] Taxel {taxel_index}: no contact within "
                  f"{MAX_APPROACH_STEPS * STEP_M * 1000:.1f} mm of travel "
                  f"(hover clearance was {HOVER_CLEARANCE_M * 1000:.1f} mm) - "
                  "skipping, no checkpoint data for this taxel.")
            aborted = True

        # --- checkpoint phase ---
        while not aborted and next_checkpoint_idx < len(CHECKPOINTS_N):
            if total_steps >= MAX_TOTAL_STEPS:
                print(f"  [abort] Taxel {taxel_index}: hit the absolute "
                      f"travel cap ({MAX_TOTAL_STEPS * STEP_M * 1000:.0f} mm) "
                      "without reaching all checkpoints.")
                aborted = True
                break

            fx, fy, fz, fmag = log_traj()

            if fmag >= MAX_SAFE_N:
                print(f"  [abort] Taxel {taxel_index}: Fmag={fmag:.2f} N hit "
                      f"the safety ceiling ({MAX_SAFE_N} N).")
                aborted = True
                break

            target_n = CHECKPOINTS_N[next_checkpoint_idx]
            if fmag >= target_n:
                print(f"  Taxel {taxel_index}: checkpoint {target_n} N "
                      f"reached (Fmag={fmag:.2f} N) - holding {HOLD_S}s ...")
                t_hold_start = time.time()
                hold_samples = []
                while time.time() - t_hold_start < HOLD_S:
                    fx, fy, fz, fmag_h = log_traj()
                    hold_samples.append((fx, fy, fz, fmag_h))
                    if fmag_h >= MAX_SAFE_N:
                        break
                    time.sleep(1.0 / HOLD_POLL_HZ)
                t_hold_end = time.time()

                cp_rows.append({
                    "taxel": taxel_index,
                    "target_N": target_n,
                    "t_hold_start": t_hold_start,
                    "t_hold_end": t_hold_end,
                    "n_samples": len(hold_samples),
                    "Fx": float(np.mean([h[0] for h in hold_samples])),
                    "Fy": float(np.mean([h[1] for h in hold_samples])),
                    "Fz": float(np.mean([h[2] for h in hold_samples])),
                    "Fmag": float(np.mean([h[3] for h in hold_samples])),
                })
                next_checkpoint_idx += 1
                continue   # re-check force before stepping again

            step_along_tool_z(reader, REAL_HOST, STEP_M)
            total_steps += 1
            time.sleep(max(0.0, 1.0 / TRAJ_LOG_HZ - 0.02))  # rough pacing

    finally:
        move_to_pose(reader, REAL_HOST, hover_pose)

    return aborted


# --------------------------------------------------------------------------- #
#                           MAIN CALIBRATION SESSION                          #
# --------------------------------------------------------------------------- #

def run_full_session(session_label, taxel_indices):
    traj_path = os.path.join(DATA_DIR, f"{session_label}_trajectory.csv")
    cp_path = os.path.join(DATA_DIR, f"{session_label}_checkpoints.csv")

    print("Setting up Nano17 (DO NOT TOUCH THE SENSOR) ...")
    sensor = SensorSettings(device_id="1",
                             calibration_folder=CALIBRATION_FOLDER,
                             sensor_name=SENSOR_NAME,
                             rate=SENSOR_RATE,
                             reverse_parameter_names=REVERSE_FZ)
    recorder = DataRecorder(force_sensor_settings=[sensor],
                             poll_udp_connection=False,
                             polling_priority="normal")
    recorder.determine_biases(n_samples=BIAS_SAMPLES)
    recorder.start_recording()
    proc = recorder.force_sensor_processes()[0]
    print("Nano17 OK (biased).")

    print(f"Connecting to robot at {REAL_HOST}:{ROBOT_PORT} ...")
    reader = RobotPoseReader(REAL_HOST)
    reader.connect()
    reader.start()
    t0 = time.time()
    while reader.latest()[0] is None:
        if reader.error is not None:
            raise reader.error
        if time.time() - t0 > 5.0:
            raise TimeoutError("No pose received from robot real-time stream.")
        time.sleep(0.05)
    print("Robot pose stream OK.")

    input(f"\nAbout to visit {len(taxel_indices)} taxel(s): {taxel_indices}. "
          "Make sure the XELA logger (xela_session_logger.py) is already "
          "running on the other machine, the sensor is clear of obstructions, "
          "and you're ready to supervise. Press Enter to start ...")

    traj_rows = []
    cp_rows = []
    aborted_taxels = []
    session_t0 = time.time()

    def save_csvs():
        # Written from the finally block so a Ctrl-C mid-run (e.g. you spot
        # the tool misbehaving and abort) still saves everything collected so
        # far, instead of losing the whole run.
        with open(traj_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=["t", "taxel", "Fx", "Fy", "Fz",
                                              "Fmag", "speed", "x", "y", "z"])
            w.writeheader()
            w.writerows(traj_rows)
        print(f"\nSaved trajectory ({len(traj_rows)} rows) -> {traj_path}")
        if cp_rows:
            with open(cp_path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=["taxel", "target_N",
                                                  "t_hold_start", "t_hold_end",
                                                  "n_samples", "Fx", "Fy", "Fz",
                                                  "Fmag"])
                w.writeheader()
                w.writerows(cp_rows)
            print(f"Saved {len(cp_rows)} checkpoint row(s) -> {cp_path}")
        else:
            print("No checkpoints reached this run.")

    try:
        for idx in taxel_indices:
            aborted = press_one_taxel(idx, reader, proc, traj_rows, cp_rows)
            if aborted:
                aborted_taxels.append(idx)
    finally:
        reader.stop()
        recorder.quit()
        save_csvs()

    if aborted_taxels:
        print(f"Taxels with incomplete data (aborted): {aborted_taxels}")

    print(f"\nSession duration: {time.time() - session_t0:.1f}s "
          "- make sure the XELA logger on the other machine covered this "
          "whole window before stopping it.")


def main():
    args = sys.argv[1:]
    session_label = args[0] if len(args) >= 1 else time.strftime("%Y%m%d_%H%M%S")
    if len(args) >= 2:
        taxel_indices = [int(args[1])]
    else:
        # Plain 0..15. With the corrected taxel_geometry axes (col = -X,
        # row = +Y), this already iterates X-first - taxels 0,1,2,3 run down
        # the -X column, then 4,5,6,7 the next column at +Y, etc. - AND the
        # index matches the XELA taxel that responds, so no relabelling needed.
        taxel_indices = list(range(16))
    run_full_session(session_label, taxel_indices)


if __name__ == "__main__":
    main()
