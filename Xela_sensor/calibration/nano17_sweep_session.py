#!/usr/bin/env python3
"""
RUNS ON THE DAQ PC ONLY (NI-DAQmx + Nano17 hardware, and network access to the
robot - the same machine nano17_press_session.py runs on).

GLOBAL FORCE-MAP CALIBRATION - the replacement for per-taxel calibration.
==========================================================================
nano17_press_session.py tried to fit 16 independent force curves, one per
taxel. That requires isolating load onto a single taxel, which this hardware
cannot do: the elastomer spreads contact, and in run 20260728_122448 the peak
response migrated to the +Y neighbour above ~1-1.5 N on 13 of 16 taxels.
Changing the indenter did not help. The isolation assumption is the problem,
not the tooling.

This script collects data for a different model: ONE map from all 48 raw
channels to the TOTAL contact force vector,

    [Fx, Fy, Fz] = W . delta_counts(48) + b

fitted by fit_force_model.py. Cross-talk stops being contamination and becomes
an input feature - it is precisely what tells you how much total load the pad
is carrying, regardless of which taxels it spread to. What you get is total
force in Newtons; what you do NOT get is per-taxel Newtons. That is the
trade, and per-taxel is the part that is unachievable here.

Four collection changes follow from that:

  1. RANDOMISED contact locations across the pad (including between taxel
     centres), not the 16 centres. Pressing only on centres leaves the design
     matrix aliased - every sample looks like one of 16 patterns, so the fit
     cannot generalise to the arbitrary contact locations palpation produces.
  2. CONTINUOUS ramp logging, not discrete force checkpoints. Every logged
     sample is a training row, so ~30 presses yields thousands of rows instead
     of 6 per taxel.
  3. The UNLOADING branch is recorded too (retract in the same steps), which
     captures the elastomer's hysteresis rather than only its loading curve.
  4. All three force axes are labels. The 21-30% parasitic tangential force
     that ruined per-taxel calibration is regressed ON here instead of being
     treated as error.

Baseline drift (~1000 counts over minutes) is handled in the fit, not here:
each press carries its own pre-contact window, and fit_force_model.py
references that press's counts to it.

This does NOT read the XELA sensor - that lives on the workstation with the
CAN adapter. Run xela_session_logger.py there with the SAME label, started
before this and stopped after, and the fit script aligns the two logs by
timestamp (it also estimates and removes any clock offset between the PCs).

Only reads from pyForceDAQ and taxel_geometry - modifies nothing in either.
nano17_press_session.py is left untouched and still works.

PREREQUISITES
  1. XELA on the optical breadboard (data/xela_sensor_breadboard_mount),
     Nano17 + indenter (data/tip_touch_li_0.7) on the UR5 end-effector.
  2. taxel_geometry.POINT0_EE_POSE_MM matches the CURRENT mounting. If the
     sensor was moved for palpation and remounted, RE-MEASURE IT - every press
     location here is derived from it. Sanity-check with xela_start_pose.py 0.
  3. No other pyForceDAQ script is holding the Nano17 device.
  4. Clocks on this PC and the workstation roughly in sync (the fit script
     corrects residual offset, but start within a few seconds of each other).

SAFETY (per press - one press aborting does not stop the sweep)
  * Hard abort + retract if Fmag exceeds MAX_SAFE_N.
  * Absolute travel cap independent of the force reading, so a stuck or
    disconnected Nano17 cannot drive the robot forward indefinitely.
  * Approach gives up (retract, no data) if contact isn't found in time.
  * Slow contact speed/accel already validated for the cone presses.
  * Supervise this - it drives ~30 automated presses with one prompt.

Usage:
    python3 nano17_sweep_session.py [session_label] [--n 30] [--fmax 3.0]
                                    [--shear] [--seed 0] [--dry-run]

    --dry-run  prints the press locations and exits without touching hardware.
    --shear    after each press, slide +/-0.5 mm laterally at constant depth to
               generate labelled tangential load. OFF by default: it drags the
               tip across the skin, which adds wear. Turn it on if you want the
               model's Fx/Fy predictions to be trustworthy, not just Fz.
"""

import argparse
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
from taxel_geometry import (pad_hover_pose, PITCH_M, SURFACE_NORMAL,
                            HOVER_CLEARANCE_M)

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

# Motion constants carried over unchanged from nano17_press_session.py - these
# were tuned against the real CB2 controller and are not free parameters:
#   0.3 mm approach step  - gentle contact, quick descent
#   0.2 mm in-contact step - 0.1 mm was too small; against a contact force the
#                            UR5 often did not move at all for such a tiny
#                            commanded increment, so force never built
APPROACH_STEP_M = 0.0003
STEP_M = 0.0002
MAX_APPROACH_STEPS = 30         # ~9 mm, enough to cross the tilted plane
MAX_TOTAL_STEPS = 60            # ~12 mm absolute cap, force-reading independent
SETTLE_TIMEOUT_S = 2.0
SETTLE_SPEED_MS = 0.01

CONTACT_ON_N = 0.15
RELEASE_OFF_N = 0.05            # unloading is done when force drops below this
DEFAULT_FMAX_N = 3.0            # top of the ramp
MAX_SAFE_N = 4.0                # hard ceiling, just above the ramp top

DWELL_S = 1.5                   # hold at peak - captures stress relaxation,
                                # which is exactly what a palpation hold does
TRAJ_LOG_HZ = 60

# Pre-contact baseline dwell. MEASURED, not guessed: in palpation run
# 20260728_151651 the summed true-Z counts settle ~35% of the press amplitude
# BELOW the pre-press level after release, and are still ~15% low a full
# minute later (median over 33 presses). The pad recovers viscoelastically,
# slowly, and it UNDERSHOOTS rather than creeping back from above.
#
# That is a direct threat to per-press referencing: a baseline taken too soon
# after the previous press is contaminated by that press's residual, which
# biases every delta in the press that follows.
#
# Waiting it out entirely is impractical (minutes per press, x30). The
# workable answer is to make the residual CONSTANT instead of small, so it is
# absorbed by the model's intercept rather than showing up as scatter:
#   * every press ramps to the same --fmax, so residuals are similar in size;
#   * this dwell is fixed, so each baseline is sampled at the same point on
#     the recovery curve;
#   * press locations are randomised, so any leftover bias is uncorrelated
#     with position and degrades to noise instead of a spatial artefact.
# 15 s puts the sample on the flatter part of the curve without adding more
# than ~8 min to a 30-press session.
BASELINE_DWELL_S = 15.0

# Randomised press locations, measured from taxel 0 along COL_AXIS / ROW_AXIS.
#
# V IS CLIPPED, AND THAT IS A MEASURED CORRECTION. In run test1 the response
# to a fixed 2 N collapsed in the v = 11-15 mm band (1598-4025 counts) while
# the v = 0-4 mm band was consistent (6378-7714, CV ~8%); corr(v, response)
# = -0.53 with no such trend in u. That band is the pad boundary: the indenter
# loads the FRAME, so the Nano17 registers force the sensing region never
# sees. Those presses are pure poison for a force model - labelled force with
# no corresponding signal - so the default window now stops short of it.
#
# The exact edge location relative to taxel 0 is not known precisely (this is
# inferred from response, not from a measurement of the pad outline), hence
# --vmax to adjust it if you re-probe and find the usable region is larger.
PAD_MIN_M = 0.0
PAD_MAX_M = 3 * PITCH_M          # 15 mm - full taxel-grid footprint, u only
PAD_V_MAX_M = 0.011              # 11 mm - stop short of the frame-loading band

SHEAR_SLIDE_M = 0.0005          # +/- lateral travel for --shear


# --------------------------------------------------------------------------- #
#                               MOTION HELPERS                                #
# --------------------------------------------------------------------------- #

def send_movel(host, pose, a=A_approach_real, v=V_approach_real, log_fn=None):
    """Fire-and-forget a single movel via the secondary client port (30002).

    log_fn, if given, is called repeatedly during the mandatory post-send wait
    so the ramp keeps being sampled instead of going blind for a full second.
    """
    # NB: function name has NO leading underscore. The CB2 PolyScope 1.x
    # URScript parser silently rejects identifiers like `_step` - the program
    # simply never runs. Every working script here uses a plain-letter name.
    ur_script = (
        "def press_step():\n"
        f"  movel(p[{pose_str(pose)}], a={a}, v={v})\n"
        "end\npress_step()\n"
    )
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    s.connect((host, 30002))
    s.sendall(ur_script.encode("ascii"))
    # MUST be ~1s: at 0.5s the real controller silently drops moves after a few
    # steps and the robot freezes mid-descent (seen 2026-07-25, runs
    # 142322/143625), while the 1.0s run 133348 pressed smoothly to 8 N.
    sleep_logging(1.0, log_fn)
    s.close()


def sleep_logging(duration_s, log_fn=None, hz=TRAJ_LOG_HZ):
    """time.sleep() that keeps logging. The 1s settle after every commanded
    step dominates the run, so sampling through it is where most of the
    training data comes from - and it is the most informative part, since the
    elastomer is relaxing under a held displacement."""
    if log_fn is None:
        time.sleep(duration_s)
        return
    t_end = time.time() + duration_s
    dt = 1.0 / hz
    while time.time() < t_end:
        log_fn()
        time.sleep(min(dt, max(0.0, t_end - time.time())))


def wait_until_settled(reader, timeout=SETTLE_TIMEOUT_S,
                       speed_threshold=SETTLE_SPEED_MS, log_fn=None):
    """Confirm the robot is stopped after a commanded step.

    Do NOT try to detect motion 'starting': a 0.1 mm step peaks at ~4.5 mm/s,
    below the threshold, so it would never register as moving and would burn
    the whole timeout on every step.
    """
    t0 = time.time()
    time.sleep(0.05)
    while True:
        if log_fn is not None:
            log_fn()
        _, speed = reader.latest()
        if speed <= speed_threshold:
            return True
        if time.time() - t0 > timeout:
            return False
        time.sleep(0.01)


def step_along_tool_z(reader, host, dz_m, log_fn=None):
    """Move dz_m along the TOOL's current local +Z axis (base frame).
    Positive dz_m presses further INTO the surface."""
    from pose_utils import rotvec_to_matrix
    pose, _ = reader.latest()
    xyz = np.array(pose[:3])
    rotvec = pose[3:]
    z_axis = rotvec_to_matrix(rotvec)[:, 2]
    target = np.concatenate([xyz + dz_m * z_axis, rotvec])
    send_movel(host, target, log_fn=log_fn)
    wait_until_settled(reader, log_fn=log_fn)
    return target


def step_in_plane(reader, host, d_m, axis_dir, log_fn=None):
    """Move d_m along a direction in the sensor plane, holding depth."""
    pose, _ = reader.latest()
    target = np.concatenate([np.array(pose[:3]) + d_m * axis_dir, pose[3:]])
    send_movel(host, target, log_fn=log_fn)
    wait_until_settled(reader, log_fn=log_fn)
    return target


def move_to_pose(reader, host, pose, log_fn=None):
    send_movel(host, pose, log_fn=log_fn)
    wait_until_settled(reader, timeout=5.0, log_fn=log_fn)


# --------------------------------------------------------------------------- #
#                             PRESS LOCATIONS                                 #
# --------------------------------------------------------------------------- #

def sample_locations(n, seed=0, vmax=PAD_V_MAX_M):
    """n randomised (u, v) pad locations in metres.

    Stratified (jittered grid), not uniform-random: with only ~30 presses,
    plain uniform sampling leaves clumps and bare patches, and a bare patch is
    a region the model was never taught. A jittered grid guarantees coverage
    while still landing off the taxel centres.
    """
    rng = np.random.default_rng(seed)
    side = int(math.ceil(math.sqrt(n)))
    cells = [(i, j) for i in range(side) for j in range(side)]
    rng.shuffle(cells)
    u_span = PAD_MAX_M - PAD_MIN_M
    v_span = vmax - PAD_MIN_M
    out = []
    for i, j in cells[:n]:
        u = PAD_MIN_M + u_span * (i + rng.uniform(0.15, 0.85)) / side
        v = PAD_MIN_M + v_span * (j + rng.uniform(0.15, 0.85)) / side
        out.append((u, v))
    return out


def repeat_locations(n, u_m, v_m):
    """The SAME location n times - the repeatability test.

    WHY THIS MATTERS MORE THAN MORE COVERAGE RIGHT NOW: in run test1 the
    response to a fixed 2 N varied 7.8x across the pad, but every press was at
    a different location, so that spread is unattributable. If pressing one
    spot repeatedly reproduces the same response, the variation is a real
    spatial gain field - learnable, but needing far more than 30 locations. If
    it does NOT reproduce, the variation is noise and no amount of data will
    produce a usable force model, so the honest ceiling is the ~0.67 N RMS the
    scalar sum|dZ| model already achieves.

    One cheap run answers which, and that answer decides whether to spend
    hours more on collection.
    """
    return [(u_m, v_m)] * n


# --------------------------------------------------------------------------- #
#                              ONE RAMP PRESS                                 #
# --------------------------------------------------------------------------- #

def press_one_location(press_idx, u_m, v_m, reader, proc, rows, args):
    """Approach -> ramp to fmax -> dwell -> (optional shear) -> unload.
    Always retracts to hover. Returns an abort reason or None."""

    hover_pose = pad_hover_pose(u_m, v_m)
    print(f"\n=== Press {press_idx}: pad (u={u_m*1000:5.2f}, "
          f"v={v_m*1000:5.2f}) mm ===")

    phase = {"name": "transit"}

    def log():
        fx, fy, fz = proc.get_Fxyz()
        pose, speed = reader.latest()
        rows.append({"t": time.time(), "press": press_idx,
                     "u_mm": u_m * 1000.0, "v_mm": v_m * 1000.0,
                     "phase": phase["name"],
                     "Fx": fx, "Fy": fy, "Fz": fz,
                     "Fmag": math.sqrt(fx * fx + fy * fy + fz * fz),
                     "speed": speed,
                     "x": pose[0], "y": pose[1], "z": pose[2]})

    def fmag():
        fx, fy, fz = proc.get_Fxyz()
        return math.sqrt(fx * fx + fy * fy + fz * fz)

    abort = None
    try:
        move_to_pose(reader, REAL_HOST, hover_pose)

        # --- pre-contact baseline window -------------------------------- #
        # The fit references this press's XELA counts to THIS window. The
        # dwell is long and FIXED so the previous press's viscoelastic
        # residual is sampled at a repeatable point on its recovery curve -
        # see the BASELINE_DWELL_S note above for the measurement behind it.
        phase["name"] = "baseline"
        sleep_logging(BASELINE_DWELL_S, log)

        # --- approach: uniform descent to first contact ------------------ #
        phase["name"] = "approach"
        contacted = False
        for _ in range(MAX_APPROACH_STEPS):
            log()
            if fmag() >= CONTACT_ON_N:
                contacted = True
                break
            step_along_tool_z(reader, REAL_HOST, APPROACH_STEP_M, log_fn=log)
        if not contacted:
            print(f"  [abort] press {press_idx}: no contact within "
                  f"{MAX_APPROACH_STEPS * APPROACH_STEP_M * 1000:.1f} mm "
                  f"(hover clearance {HOVER_CLEARANCE_M * 1000:.1f} mm).")
            return "no_contact"

        # --- load ramp ---------------------------------------------------- #
        # Stops on the PREDICTED force after the next step, not the current
        # one. Measured in run cal1: with a plain `f >= fmax` test the peaks
        # were 3.73 / 4.77 / 3.56 N against a 3.0 N target, and the 4.77 N
        # press blew through MAX_SAFE_N and aborted. Near full stiffness a
        # single 0.2 mm step adds ~1 N, and the step cannot be made smaller -
        # 0.1 mm often fails to move the arm at all against a contact force.
        # So overshoot cannot be trimmed by finer steps; the ramp has to stop
        # one step EARLY instead. Undershooting slightly is harmless (the
        # ramp is sampled continuously, so the data is there either way).
        phase["name"] = "load"
        steps = 0
        f_prev = fmag()
        df_step = 0.0
        while True:
            log()
            f = fmag()
            if f >= MAX_SAFE_N:
                print(f"  [abort] press {press_idx}: Fmag={f:.2f} N hit the "
                      f"safety ceiling ({MAX_SAFE_N} N).")
                abort = "over_force"
                break
            if f >= args.fmax or f + df_step >= args.fmax:
                print(f"  press {press_idx}: stopped at {f:.2f} N in "
                      f"{steps} steps (target {args.fmax} N, last step "
                      f"+{df_step:.2f} N).")
                break
            if steps >= MAX_TOTAL_STEPS:
                print(f"  [abort] press {press_idx}: travel cap "
                      f"({MAX_TOTAL_STEPS * STEP_M * 1000:.0f} mm) without "
                      f"reaching {args.fmax} N (Fmag={f:.2f} N).")
                abort = "travel_cap"
                break
            step_along_tool_z(reader, REAL_HOST, STEP_M, log_fn=log)
            steps += 1
            f_now = fmag()
            df_step = max(0.0, f_now - f_prev)
            f_prev = f_now

        if abort is None:
            # --- dwell at peak -------------------------------------------- #
            phase["name"] = "dwell"
            sleep_logging(DWELL_S, log)

            # --- optional shear ------------------------------------------- #
            if args.shear:
                phase["name"] = "shear"
                n = SURFACE_NORMAL / np.linalg.norm(SURFACE_NORMAL)
                for axis in (np.array([1.0, 0.0, 0.0]),
                             np.array([0.0, 1.0, 0.0])):
                    d = axis - np.dot(axis, n) * n      # keep depth constant
                    d /= np.linalg.norm(d)
                    for delta in (SHEAR_SLIDE_M, -2 * SHEAR_SLIDE_M,
                                  SHEAR_SLIDE_M):
                        if fmag() >= MAX_SAFE_N:
                            abort = "over_force"
                            break
                        step_in_plane(reader, REAL_HOST, delta, d, log_fn=log)
                    if abort:
                        break

        # --- unload ramp -------------------------------------------------- #
        # Stepping back out (rather than jumping to hover) records the
        # unloading branch, so the model sees the elastomer's hysteresis
        # instead of only its loading curve.
        phase["name"] = "unload"
        for _ in range(MAX_TOTAL_STEPS):
            log()
            if fmag() <= RELEASE_OFF_N:
                break
            step_along_tool_z(reader, REAL_HOST, -STEP_M, log_fn=log)

    finally:
        phase["name"] = "retract"
        move_to_pose(reader, REAL_HOST, hover_pose, log_fn=log)

    return abort


# --------------------------------------------------------------------------- #
#                                  SESSION                                    #
# --------------------------------------------------------------------------- #

FIELDNAMES = ["t", "press", "u_mm", "v_mm", "phase",
              "Fx", "Fy", "Fz", "Fmag", "speed", "x", "y", "z"]


def run_session(args):
    from forceDAQ.force.data_recorder import DataRecorder
    from forceDAQ.force.sensor import SensorSettings
    from record_cone_press import RobotPoseReader, ROBOT_PORT

    out_path = os.path.join(DATA_DIR, f"{args.label}_sweep.csv")
    locations = plan_locations(args)

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

    input(f"\nAbout to run {len(locations)} ramp presses to {args.fmax} N"
          f"{' with shear slides' if args.shear else ''}.\n"
          f"Confirm xela_session_logger.py is ALREADY running on the "
          f"workstation with label '{args.label}', the pad is clear, and you "
          f"are ready to supervise. Press Enter to start ...")

    rows = []
    aborted = {}
    t_session = time.time()

    def save():
        # Written from the finally block so a Ctrl-C mid-run still saves
        # everything collected so far instead of losing the whole session.
        with open(out_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=FIELDNAMES)
            w.writeheader()
            w.writerows(rows)
        print(f"\nSaved {len(rows)} row(s) -> {out_path}")

    try:
        for i, (u, v) in enumerate(locations):
            reason = press_one_location(i, u, v, reader, proc, rows, args)
            if reason:
                aborted[i] = reason
    finally:
        reader.stop()
        recorder.quit()
        save()

    if aborted:
        print(f"Presses with incomplete data: {aborted}")
    print(f"\nSession duration: {time.time() - t_session:.1f}s - make sure the "
          "XELA logger covered this whole window before stopping it.")
    print(f"\nNext (on the workstation, after copying both CSVs together):\n"
          f"  python3 Xela_sensor/calibration/fit_force_model.py {args.label}")


def plan_locations(args):
    """The press locations for this session: one spot repeated, or a
    randomised sweep."""
    if args.repeat:
        try:
            u, v = (float(x) / 1000.0 for x in args.repeat.split(","))
        except ValueError:
            raise SystemExit("--repeat wants U,V in mm, e.g. --repeat 7.5,5.0")
        return repeat_locations(args.n, u, v)
    return sample_locations(args.n, args.seed, args.vmax / 1000.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("label", nargs="?", default=time.strftime("%Y%m%d_%H%M%S"),
                    help="session label; MUST match xela_session_logger.py's")
    ap.add_argument("--n", type=int, default=30, help="number of press locations")
    ap.add_argument("--fmax", type=float, default=DEFAULT_FMAX_N,
                    help="top of the force ramp, N")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--repeat", metavar="U,V",
                    help="REPEATABILITY TEST: press this one pad location "
                         "(mm from taxel 0, e.g. --repeat 7.5,5.0) --n times "
                         "instead of sampling --n different locations")
    ap.add_argument("--vmax", type=float, default=PAD_V_MAX_M * 1000,
                    help="upper v limit for sampling, mm (default %(default)s "
                         "- stops short of the frame-loading band)")
    ap.add_argument("--shear", action="store_true",
                    help="add lateral slides at depth (labelled Fx/Fy data)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print planned locations and exit, no hardware")
    args = ap.parse_args()

    if args.fmax >= MAX_SAFE_N:
        ap.error(f"--fmax must stay below MAX_SAFE_N ({MAX_SAFE_N} N)")

    if args.dry_run:
        print(f"label={args.label}  n={args.n}  fmax={args.fmax} N  "
              f"shear={args.shear}  seed={args.seed}")
        if args.repeat:
            print(f"REPEATABILITY TEST: {args.n} presses at one location\n")
        else:
            print(f"pad sampling window: u in [{PAD_MIN_M*1000:.0f}, "
                  f"{PAD_MAX_M*1000:.0f}] mm, v in [{PAD_MIN_M*1000:.0f}, "
                  f"{args.vmax:.0f}] mm from taxel 0\n")
        for i, (u, v) in enumerate(plan_locations(args)):
            pose = pad_hover_pose(u, v)
            print(f"  press {i:2d}: u={u*1000:5.2f} v={v*1000:5.2f} mm  "
                  f"hover TCP = [{pose[0]*1000:7.2f}, {pose[1]*1000:7.2f}, "
                  f"{pose[2]*1000:7.2f}] mm")
        return

    run_session(args)


if __name__ == "__main__":
    main()
