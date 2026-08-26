#!/usr/bin/env python3
"""
XELA palpation by PRESSING TO A TACTILE COUNT TARGET - no force calibration.

RUNS ON THIS WORKSTATION. Both things it needs are reachable here: the XELA
websocket (localhost:5000) and the robot's realtime stream (30003). The
Nano17 is not involved.

THE IDEA
At every pose, press in until the strongest taxel reaches TARGET_PEAK_COUNTS,
then stop. Because the stopping condition is read from the sensor rather than
computed from geometry, registration error (ICP is 1.5-2.2 mm RMS here) and
the stale tool offset do not affect how hard each pose is loaded - only where
on the phantom it lands. That is the failure mode worth removing.

KNOWN TRADE-OFF, stated so it is not discovered later in the data: the tactile
response is being used as BOTH the control signal and the measurement. Every
pose that reaches target reports ~TARGET_PEAK_COUNTS by construction, so the
stiffness information does not live in the count - it lives in the DEPTH
required to get there, which is logged per pose as depth_mm in the summary.
Analyse that column, not the peak counts.

WHAT THE COUNT TARGET IS, AND WHAT IT IS NOT

It is NOT the sensor's saturation ceiling. An earlier version of this file
claimed the response "tops out near 1670 counts" based on session broad1,
where the strongest taxel climbed to 1670 at ~2.4 N and then declined. That
turnover is real but it is NOT saturation - it is the contact SPREADING. As
the indenter sinks, contact area grows and load redistributes to neighbours,
so the strongest taxel's share falls even while total load rises. The give-
away is that the total kept climbing throughout.

Taxels reach far higher than 1670. Across all recorded sessions every taxel
has exceeded 2400 counts, and taxel 12 reached 13,524 counts (sustained 9 s)
in session test1, where a 0.7 mm tip concentrated the whole load on it. Raw
values have spanned 18,147-41,757 against a 16-bit range, so the ADC is at
64% and is not the limit either.

The real constraint is the MODULE PRESSURE LIMIT. The 2019 hardware manual
gives 10 N / 25 kPa normal, and is explicit that it scales with contact area
("if you contact only half of the Sensor Module's surface, only apply 5N").
So:

    F_max = 10 N * contact_area / 672 mm^2

A peak-count target is a decent proxy for LOCAL pressure, which is the
quantity that limit is written in - one taxel reading high means that taxel
is loaded hard, regardless of how much total force the pad is carrying. That
is what makes this a usable stopping rule.

TARGET_PEAK_COUNTS defaults to 1400 because:
  * every pose of egg run 20260805_143703 that made real contact reached
    700-1850 counts, so it is achievable for most of the phantom;
  * scaling broad1's count/force distribution to the 1.05 N per-taxel
    pressure limit puts that limit near ~2700 counts, leaving ~2x margin;
  * MAX_PEAK_COUNTS aborts at 2200, below that estimate.

Those are estimates from indirect data, not a measured damage threshold.
Treat 1400 as a working value, not a validated one, and lower it rather than
raise it if the contact patch on your phantom looks small.

WHAT IT LOGS
Raw counts, all 48 channels, plus TCP pose, continuously. NOT Newtons.

A note on per-taxel uniformity, because an earlier version of this file got it
wrong: a single flat-plate press loads the taxels very unevenly, which looked
like a >100x sensitivity spread or even dead taxels. It is not. Pooling every
recorded session, EVERY taxel has exceeded 2400 counts and the highest maxima
are in the row that looked deadest. The unevenness is where contact landed,
not the sensor. Still treat a single frame's taxel map as qualitative - but do
not conclude a taxel is broken from one press.

SAFETY
  * Hard abort if any taxel exceeds MAX_PEAK_COUNTS. With the plateau stop
    removed this is the main guard against pushing a pose that cannot load,
    alongside the travel cap.
  * NO plateau stop. Every pose presses to the target or to the travel cap,
    so all poses receive the same stimulus and peak_counts is comparable
    across poses and specimens. A pose whose response stops growing is
    FLAGGED (went_flat / flat_depth_mm in the summary) but not stopped - see
    the PLATEAU block below for why stopping on it was removed.
  * Absolute travel caps that do not depend on the sensor at all.
  * STALL DETECTION. The XELA server has died mid-run before (20260805_143703,
    pose 8), and a cached last-sample reads exactly like "not touching", so
    the arm kept stepping forward blind for 172 s. A frozen stream now aborts.
  * Supervise this. Hand on the e-stop for the first poses.

PREREQUISITES
  1. can0 up:             sudo ip link set up can0 type can bitrate 1000000
  2. xela_server running: cd Xela_sensor && ./xela_server -f xServ.ini \
                              --ip 0.0.0.0 -l xela_server.log
  3. Sensor triggered:    cansend can0 203#07.00
  4. Robot powered, in remote control, phantom mounted.

Usage:
    python3 xela_palpation.py --check            offline reachability, no hardware
    python3 xela_palpation.py --monitor          live per-taxel readout, no robot
    python3 xela_palpation.py --motion-only      visit approach poses (URSim)
    python3 xela_palpation.py [--poses CSV] [--limit N] [--target 1400]
"""

import argparse
import csv
import json
import os
import socket
import struct
import sys
import threading
import time

import numpy as np
import pandas as pd
import websocket

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
sys.path.insert(0, REPO_ROOT)

import paths
from scipy.spatial.transform import Rotation as Rot

from pose_utils import (REAL_HOST, SIM_HOST, pose_str, rotvec_to_matrix,
                        TOOL_TIP_OFFSET,
                        A_approach_real, V_approach_real,
                        ur5_ik_near, UR5_IK_SEED)

DATA_DIR = os.path.join(_THIS_DIR, "data")
os.makedirs(DATA_DIR, exist_ok=True)

# --------------------------------------------------------------------------- #
#                                 CONSTANTS                                   #
# --------------------------------------------------------------------------- #

XELA_WS_URL = "ws://127.0.0.1:5000"
ROBOT_STATE_PORT = 30003     # realtime state stream (read-only)
ROBOT_CMD_PORT = 30002       # secondary client, for movel
N_TAXELS = 16

# --- press targets, all in raw counts relative to this pose's baseline ---
TARGET_PEAK_COUNTS = 1400    # see module docstring - set by the PRESSURE
                             # limit, not by any saturation ceiling
MAX_OVER_RANGE_POSES = 3     # consecutive poses hitting the peak ceiling
                             # before the run is stopped. One or two is normal
                             # under a total target (concentrated contact);
                             # three in a row means the target is unreachable
                             # on this phantom, not a local geometry quirk.
MAX_PEAK_COUNTS = 2200       # per-press stop. Above the 1847 the egg run reached,
                             # below the ~2700 that the per-taxel pressure
                             # limit is estimated to correspond to.
CONTACT_PEAK_COUNTS = 100    # first contact. Idle noise peaks at ~25 counts
                             # on a single taxel, so this is ~4x noise.
CONTACT_TOTAL_COUNTS = 350   # and the whole pad must show this much, which
                             # rejects drift smeared thinly over 16 taxels

# --- the press target: TOTAL, not peak ---
#
# Peak is a SINGLE-TAXEL reading, so any corner touching hard ends the press
# no matter how little of the pad is loaded. Run 20260822_135715 reached the
# 1400-peak target on all 23 poses, and at that moment the total load spanned
# 3187 to 11337 counts - a 96% spread:
#     pose  3:  2.7 mm  peak 1447  total  3187  live  6  <- one corner
#     pose  6: 17.1 mm  peak 1415  total  9092  live 16  <- whole pad
# Those are not comparable measurements despite both reporting "1400".
#
# The same run showed why: the pad lands on a corner and MIGRATES to central
# full contact as it presses in (contact offset 4-10 mm at first touch ->
# 0.5-2.5 mm at peak; live taxels 1-5 -> 11-16). Total keeps rising through
# that migration, so it cannot be satisfied by one saturated taxel and stops
# when the PAD AS A WHOLE has taken a set load.
#
# 5000, lowered from 8000 because 8000 is NOT REACHABLE ON SOFT SPECIMENS
# within a safe force. Depth needed to reach each target, measured across six
# specimens:
#            4000     5000     6000     8000
#   red_bot   4.2      5.2      6.1      7.8
#   red_top   4.4      5.4      6.4      7.9
#   red_mid   4.6      5.8      6.8      8.9
#   blue_bot  6.1      7.3      8.9     10.3
#   empty     7.0      8.3      9.4     11.4
#   softest   7.6      9.8     11.3     14.0
# At 8000 the softest specimen needed 14-17 mm and the UR5's own protective
# stop engaged partway through the run (20260822_151442, 15 of 23 poses). A
# target only some specimens can reach is not a valid comparison.
#
# 5000 is reached by every specimen on all 23 poses at 5.2-9.8 mm, and
# preserves the discrimination: firmest to softest still spans 1.95x, the same
# ratio 8000 gave. Nothing is lost but the deep pressing - which also keeps
# clear of the force limit and is gentler on both sensor and phantom.
TARGET_TOTAL_COUNTS = 5000

# --- plateau, measured on the RUNNING MAXIMUM of the peak taxel ---
#
# The instantaneous peak is NOT usable for this. On a curved phantom the pad
# rolls as it indents, contact migrates, and a different taxel takes over as
# the strongest - during that handover the max dips. Egg run 20260805_143703,
# pose 1: peak climbs to 1248 at 3.4 mm, sags to 1181 by 4.4 mm, then resumes
# hard to 1794 by 5.75 mm. A slope test on the instantaneous peak stops dead
# in that sag, cutting the press short by 2 mm and 550 counts. Pose 0 sags the
# same way.
#
# The running maximum cannot sag, so a genuine handover simply pauses its
# growth instead of reversing it, and only a real saturation keeps it flat.
# RELAXED after run 20260820_182045, where the detector was stopping presses
# that were still loading. The response does not climb smoothly - it stutters,
# with flat stretches of 1-2 mm followed by sharp jumps:
#     pose 0: flat at 779 counts from 0.5 to 2.5 mm, fired at 3.0 mm, then
#             immediately grew +246 counts/mm  <- stopped just as it began
#     pose 2: growth ran 344, 198, 40, 159, 68, 13, 58 counts/mm - the dips at
#             40 and 13 both recovered; it stopped on the next one
#     pose 3: flat for only 1.2 mm before firing
# Meanwhile poses that DID reach target in run 20260805 needed 9.3-10.0 mm of
# travel, so stopping at 3 mm was cutting them off less than a third of the way.
#
# These values were reasoned from those failures, not fitted: the recorded
# traces end where the detector fired, so no replay can show what a looser
# setting would have done. The intent is that only a genuine stall trips it -
# less than 40 counts gained across 4 mm, and never before 6 mm of travel.
PLATEAU_WINDOW_MM = 4.0
PLATEAU_MIN_GROWTH = 40
PLATEAU_MIN_DEPTH_MM = 6.0

# --- motion ---
APPROACH_STEP_M = 0.0005     # 0.5 mm until first contact
STEP_M = 0.0002              # 0.2 mm in contact. NOT tunable downward: against
                             # a contact force the UR5 often does not move at
                             # all for a 0.1 mm commanded step, so response
                             # never builds.
MAX_APPROACH_STEPS = 40      # ~20 mm, covers ICP error plus clearance
MAX_PRESS_STEPS = 90         # ~18 mm absolute cap once in contact. Raised
                             # from 12 mm: in run 20260820_184956 three poses
                             # hit the cap at 1181-1306 counts while still
                             # growing +50 to +111 counts/mm, needing only
                             # 1.6-4.3 mm more to reach target. The plateau
                             # detector (verified genuine on that run - every
                             # plateau gained <=31 counts over its final 4 mm)
                             # is what stops presses that cannot progress, so
                             # this cap only has to bound the ones that can.
SEND_SLEEP_S = 1.0           # MUST be ~1 s: at 0.5 s the CB2 controller
                             # silently drops moves after a few steps and the
                             # arm freezes mid-descent
SETTLE_TIMEOUT_S = 2.0
SETTLE_SPEED_MS = 0.01
SETTLE_S = 3.0               # baseline dwell at the approach pose. The pad is
                             # viscoelastic: after a press it sits ~35% of the
                             # press amplitude BELOW its pre-press level and
                             # is still ~15% low a minute later, so a baseline
                             # taken too soon is contaminated by the last pose.
DWELL_S = 0.5
LOG_HZ = 60

# --- transit clearance ---
#
# Approach poses sit only ~10 mm off the surface, and a movej between two of
# them takes a JOINT-space path that bulges - it does not stay on the line
# between the endpoints. In run 20260820_183618 the pad brushed the phantom
# while moving between poses. So the arm lifts clear along base +Z before
# every transit, and returns to a common overhead pose when changing rings,
# where the swing is largest. Same approach as
# execution/run_side_strip_poses.py, which lifts SAFE_LIFT_M for this reason.
TRANSIT_LIFT_M = 0.05        # straight up before transiting
RING_CHANGE_LIFT_M = 0.09    # bigger swing between rings, so lift further

# --- contact-centring adaptation: TESTED AND DISABLED --------------------- #
#
# The idea: an off-centre contact engages few taxels, so re-aim the pad after
# first contact to bring the touch toward its centre, then press.
#
# The observation behind it was real. Run 20260820_175138, 23 poses:
#     correlation(centroid offset, live taxels) = -0.90
#     reached target : offset 3.2 mm, 12.4 taxels live
#     plateaued      : offset 7.2 mm,  3.9 taxels live
#
# The MECHANISM works. Once the pad-axis mapping below was measured rather
# than assumed, run 20260820_182045 centred every pose by the predicted ~40%:
#     4.1 -> 2.9    4.7 -> 3.1    7.3 -> 4.0    7.2 -> 4.2 mm
#
# But the OUTCOMES got worse, same four poses of the same grid:
#     no adapt            mean peak 1218   2/4 reached target
#     adapt, wrong axis   mean peak 1162   2/4
#     adapt, correct axis mean peak  991   1/4  <- best centring, worst result
# Pose 2 was centred 7.3 -> 4.0 mm and its live taxels FELL from 8 to 3. Pose 3
# was centred 7.2 -> 4.2 mm and regressed from target to plateau.
#
# So the correlation was CONFOUNDED, not causal. Where the pad happens to sit
# flat against the surface it gets a central contact AND good loading - the
# flatness causes both. Tilting to force the contact central does not create
# flatness: it picks a different tangent point while aiming the press further
# off the surface normal, which loads less.
#
# Kept, off by default, because the experiment is worth not repeating.
# --adapt re-enables it. Do not turn it on expecting better contact.
# MEASURED 2026-08-20 with measure_pad_axes.py on a flat surface, tilting +5
# deg about each tool axis and watching the contact centroid:
#     +X tilt (drives the tool +Y side deeper) -> centroid moved +u
#     +Y tilt (lifts the tool +X side)         -> centroid moved -v
# so pad-u lies along tool +Y and pad-v along tool +X. The identity guess used
# until now had them SWAPPED, which is why the centring correction pushed the
# contact toward the pad centre on one pose, perpendicular on another and away
# on a third (13 / 81 / 123 deg from intended, run 20260820_180906).
PAD_U_IN_TOOL = np.array([0.0, 1.0, 0.0])
PAD_V_IN_TOOL = np.array([1.0, 0.0, 0.0])

ADAPT_ENABLED = False
ADAPT_MIN_OFFSET_MM = 3.0    # below this the contact is already central enough
# MEASURED with measure_pad_axes.py: +5 deg of tilt moved the contact centroid
# ~1.0 mm on a flat surface, i.e. ~0.2 mm per degree, so ~5 deg/mm would be
# unity gain. 2.0 is deliberately under-damped - roughly 40% of the offset
# corrected per attempt - because a single correction that overshoots leaves
# the pad worse aligned than it started, and there is only one attempt.
#
# That sensitivity was measured on a FLAT surface, where a compliant pad mostly
# redistributes pressure rather than moving contact. On a curved phantom the
# same tilt should move the contact further, so treat 2.0 as a starting point
# and check the "re-aimed: offset X -> Y" line to see what is actually achieved.
ADAPT_GAIN_DEG_PER_MM = 2.0
ADAPT_MAX_DEG = 12.0         # never re-aim more than this in one correction
ADAPT_BACKOFF_M = 0.004      # retract before re-orienting, so the pad is not
                             # dragged across the phantom while rotating

# --- stall detection ---
XELA_STALE_S = 2.0           # no websocket message at all
XELA_FROZEN_S = 5.0          # messages arriving but bit-identical values


class XelaStalled(RuntimeError):
    """The XELA stream stopped being trustworthy mid-run."""


# --------------------------------------------------------------------------- #
#                                  READERS                                    #
# --------------------------------------------------------------------------- #

class XelaReader:
    """Latest XELA sample as a flat (48,) array: x0,y0,z0,x1,y1,z1,..."""

    def __init__(self, url=XELA_WS_URL, sensor_key="1"):
        self._url, self._key = url, sensor_key
        self._lock = threading.Lock()
        self._latest = None          # (recv_time, np.array(48))
        self._ws = None
        self.error = None
        # stall bookkeeping
        self._seen = None
        self._t_change = time.time()

    def _on_message(self, ws, message):
        t = time.time()
        try:
            s = json.loads(message)[self._key]
            vals = np.array([int(v, 16) for v in s["data"].split(",")],
                            dtype=float)
        except Exception:
            return
        if vals.size != 3 * N_TAXELS:
            return
        with self._lock:
            self._latest = (t, vals)

    def _on_error(self, ws, error):
        self.error = error

    def start(self):
        self._ws = websocket.WebSocketApp(
            self._url, on_message=self._on_message, on_error=self._on_error)
        threading.Thread(target=self._ws.run_forever, daemon=True).start()

    def latest(self):
        with self._lock:
            return self._latest

    def sample(self):
        """Checked read. Raises XelaStalled if the stream died."""
        got = self.latest()
        if got is None:
            return None
        recv_t, vals = got
        now = time.time()
        if now - recv_t > XELA_STALE_S:
            raise XelaStalled(f"no XELA message for {now - recv_t:.1f}s "
                              "(server died?)")
        if self._seen is not None and np.array_equal(vals, self._seen):
            if now - self._t_change > XELA_FROZEN_S:
                raise XelaStalled(
                    f"XELA values frozen for {now - self._t_change:.1f}s")
        else:
            self._seen = vals.copy()
            self._t_change = now
        return vals

    def wait_for_data(self, timeout=5.0):
        t0 = time.time()
        while self.latest() is None:
            if self.error is not None:
                raise RuntimeError(f"XELA websocket error: {self.error}")
            if time.time() - t0 > timeout:
                raise TimeoutError(
                    "No data from XELA server - is it running with "
                    "--ip 0.0.0.0 and the sensor triggered "
                    "(cansend can0 203#07.00)?")
            time.sleep(0.05)

    def stop(self):
        if self._ws is not None:
            self._ws.close()


# The UR realtime packet layout DEPENDS ON PACKET SIZE, and getting it wrong
# is not a degraded reading - it is a wrong number that a motion command then
# gets built from.
#
# MEASURED against this robot 2026-08-06 (812-byte packet, 101 doubles):
#     [73] -0.000976   x        [79:85] all zero  <- TCP speed, stationary
#     [74] -0.514491   y        [86:91] 28..38    <- joint temperatures
#     [75] +0.241836   z
#     [76] -0.000051   rx
#     [77] +3.119979   ry
#     [78] +0.030041   rz
# which matches the poses in the recorded press logs (py ~ -0.510,
# pz ~ 0.213, rotvec ~ [0, 3.14, 0]).
#
# Two wrong values have been in this file: a hardcoded 56:62, which decoded
# ALL ZEROS on this controller - so step_along_tool_z commanded a movel to the
# robot's base origin and the arm left the workspace entirely - and an
# archived 74:80, which is this layout shifted by one.
# MEASURED per controller. Both entries sit one index EARLIER than the
# commonly-quoted offsets, and a one-place shift is not detectable by any
# sanity check - it still yields |xyz| < 2 m and |rotvec| <= pi. So guessing
# is not safe here, and an unknown packet size refuses rather than falling
# back to a plausible-looking wrong answer.
#   812  bytes / 101 doubles : real CB2 robot, verified 2026-08-06
#   1220 bytes / 152 doubles : URSim e-series, verified 2026-08-06
_LAYOUTS = {
    812: {"pose": slice(73, 79), "speed": slice(79, 85)},
    1220: {"pose": slice(55, 61), "speed": slice(61, 67)},
}


class UnknownPacketLayout(RuntimeError):
    """Realtime packet size not in the measured table."""


def _layout_for(total_bytes):
    try:
        return _LAYOUTS[total_bytes]
    except KeyError:
        raise UnknownPacketLayout(
            f"realtime packet is {total_bytes} bytes, which is not in the "
            f"measured layout table {sorted(_LAYOUTS)}. Guessing the offset is "
            "how the arm was once commanded to the robot's base origin - dump "
            "the packet, find the slice whose xyz looks like a TCP and whose "
            "next 6 doubles are ~0 while stationary, and add it to _LAYOUTS.")


class RobotPoseReader(threading.Thread):
    """TCP pose + linear speed from the 125 Hz realtime stream."""

    def __init__(self, host):
        super().__init__(daemon=True)
        self.host = host
        self._lock = threading.Lock()
        self._pose = None
        self._speed = 0.0
        self._sock = None
        self._stop = threading.Event()
        self._packet_size = None
        self.error = None

    def connect(self):
        self._sock = socket.create_connection((self.host, ROBOT_STATE_PORT),
                                              timeout=5.0)

    def run(self):
        buf = b""
        try:
            while not self._stop.is_set():
                chunk = self._sock.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while len(buf) >= 4:
                    size = struct.unpack("!i", buf[:4])[0]
                    if size <= 0 or len(buf) < size:
                        break
                    packet, buf = buf[:size], buf[size:]
                    n = (size - 4) // 8
                    vals = struct.unpack(f"!{n}d", packet[4:4 + n * 8])
                    try:
                        layout = _layout_for(size)
                    except UnknownPacketLayout as e:
                        self.error = e
                        break
                    if n < layout["speed"].stop:
                        continue
                    pose = np.array(vals[layout["pose"]])
                    v = np.array(vals[layout["speed"]])
                    # Refuse an implausible decode rather than let a motion
                    # command be built from it. A UR5 cannot reach 2 m, and a
                    # rotation vector cannot exceed pi in magnitude - either
                    # means the layout is wrong for this controller.
                    # An all-zero pose is the specific signature of a wrong
                    # layout on this controller, and it passes any naive
                    # range check - so reject it explicitly.
                    if (np.max(np.abs(pose[:3])) > 2.0
                            or np.linalg.norm(pose[3:]) > np.pi + 1e-3
                            or np.allclose(pose, 0.0)):
                        self.error = ValueError(
                            f"implausible TCP pose from a {size}-byte packet: "
                            f"{np.round(pose, 3)} - realtime layout is wrong "
                            "for this controller")
                        break
                    with self._lock:
                        self._pose = pose
                        self._speed = float(np.linalg.norm(v[:3]))
                        self._packet_size = size
        except Exception as e:
            self.error = e

    def latest(self):
        with self._lock:
            return (None if self._pose is None else self._pose.copy(),
                    self._speed)

    def stop_reading(self):
        self._stop.set()
        try:
            self._sock.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
#                               MOTION HELPERS                                #
# --------------------------------------------------------------------------- #

# --- continuous (non-stepped) motion --------------------------------------- #
#
# Stepping 0.2 mm at a time costs a mandatory ~1 s socket wait per step, so a
# 12 mm press took ~60 s and the arm visibly stuttered. Instead the descent is
# ONE slow movel that the sensor interrupts: poll at LOG_HZ and stream stopl()
# the moment the stop condition trips.
#
# Overshoot is then latency x speed, not the step size. At PRESS_SPEED_MS with
# ~0.2 s of detection latency that is ~0.4 mm; stopl's own deceleration adds
# only microns at these speeds.
APPROACH_SPEED_MS = 0.008    # 8 mm/s to find contact - covers the ~10 mm
                             # approach gap in about a second
PRESS_SPEED_MS = 0.002       # 2 mm/s once in contact, so the count target can
                             # be caught without large overshoot
MOVE_ACCEL = 0.1


def start_movel(host, pose, v, a=MOVE_ACCEL):
    """Begin a movel and return the OPEN socket.

    The socket is deliberately not closed here. Closing too early makes the
    CB2 controller silently drop the move - the reason the stepped version had
    to sleep ~1 s after every send. Here the caller holds it open for the
    duration of the motion, so the move always latches.
    """
    script = (f"def press_move():\n"
              f"  movel(p[{pose_str(pose)}], a={a}, v={v})\n"
              f"end\npress_move()\n")
    sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sk.settimeout(5.0)
    sk.connect((host, ROBOT_CMD_PORT))
    sk.sendall(script.encode("ascii"))
    return sk


def soft_stop(host, decel=0.5):
    """Decelerate the current move. Streaming a program to 30002 replaces
    whatever is running, which is what makes this able to interrupt a movel."""
    try:
        sk = socket.create_connection((host, ROBOT_CMD_PORT), timeout=2.0)
        sk.sendall(f"def hstop():\n  stopl({decel})\nend\nhstop()\n".encode())
        time.sleep(0.15)
        sk.close()
        return True
    except Exception as e:
        print(f"  [ERROR] could not stop: {e}")
        return False


def move_until(host, reader, target_pose, v, predicate, poll, timeout_s):
    """Run one movel, polling `predicate` throughout; stop the instant it
    fires. Returns (trigger, reached_end).

    predicate returns a truthy trigger value to stop, or None to continue.
    poll is called every cycle for logging.
    """
    sk = start_movel(host, target_pose, v)
    t0 = time.time()
    trigger, moved = None, False
    try:
        while True:
            poll()
            trig = predicate()
            if trig:
                trigger = trig
                soft_stop(host)
                break
            _, speed = reader.latest()
            if speed is not None:
                if speed > SETTLE_SPEED_MS:
                    moved = True
                elif moved and time.time() - t0 > 0.5:
                    break          # motion finished without the predicate
            if time.time() - t0 > timeout_s:
                soft_stop(host)
                break
            time.sleep(1.0 / LOG_HZ)
    finally:
        try:
            sk.close()
        except Exception:
            pass
    # Let the deceleration finish before the caller reads a pose.
    wait_until_settled(reader, timeout=2.0, poll=poll)
    return trigger, (trigger is None)


def send_movej(host, q, a=0.5, v=0.4, poll=None):
    """Transit to an explicit JOINT target.

    Cartesian movel leaves branch selection to the controller, which is free to
    pick a different IK solution than the one solved offline - so the joint
    limits verified by --check, and the wrist-3 minimisation baked into the
    pose file, describe a branch the robot may simply not use. On 2026-08-06
    the controller picked a solution that violated a joint limit on the second
    pose of a grid that checked clean.
    Commanding joints removes that freedom: what was checked is what runs.
    """
    script = (f"def transit():\n"
              f"  movej([{pose_str(q)}], a={a}, v={v})\n"
              f"end\ntransit()\n")
    sk = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sk.settimeout(5.0)
    sk.connect((host, ROBOT_CMD_PORT))
    sk.sendall(script.encode("ascii"))
    sleep_polling(SEND_SLEEP_S, poll)
    sk.close()


def send_movel(host, pose, poll=None):
    """Fire-and-forget one movel via the secondary client port."""
    # NB: the function name has NO leading underscore. The CB2 PolyScope 1.x
    # parser silently rejects identifiers like `_step` - the program just
    # never runs, with no error.
    script = (f"def press_step():\n"
              f"  movel(p[{pose_str(pose)}], a={A_approach_real}, "
              f"v={V_approach_real})\n"
              f"end\npress_step()\n")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5.0)
    s.connect((host, ROBOT_CMD_PORT))
    s.sendall(script.encode("ascii"))
    sleep_polling(SEND_SLEEP_S, poll)
    s.close()


def sleep_polling(duration_s, poll=None, hz=LOG_HZ):
    if poll is None:
        time.sleep(duration_s)
        return
    end = time.time() + duration_s
    while time.time() < end:
        poll()
        time.sleep(min(1.0 / hz, max(0.0, end - time.time())))


def wait_until_settled(reader, timeout=SETTLE_TIMEOUT_S, poll=None):
    t0 = time.time()
    time.sleep(0.05)
    while True:
        if poll is not None:
            poll()
        _, speed = reader.latest()
        if speed is not None and speed <= SETTLE_SPEED_MS:
            return True
        if time.time() - t0 > timeout:
            return False
        time.sleep(0.01)


def step_along_tool_z(reader, host, dz_m, poll=None):
    """Move dz_m along the tool's local +Z (into the surface when positive)."""
    pose, _ = reader.latest()
    z_axis = rotvec_to_matrix(pose[3:])[:, 2]
    target = np.concatenate([np.array(pose[:3]) + dz_m * z_axis, pose[3:]])
    send_movel(host, target, poll=poll)
    wait_until_settled(reader, poll=poll)


def emergency_stop(host):
    """Stream stopl(), which halts the current move AND replaces whatever
    program is running."""
    try:
        s = socket.create_connection((host, ROBOT_STATE_PORT), timeout=2.0)
        s.sendall(b"def estop():\n  stopl(2.5)\nend\nestop()\n")
        time.sleep(0.5)
        s.close()
        return True
    except Exception as e:
        print(f"  [ERROR] could not send stop: {e}")
        return False


# --------------------------------------------------------------------------- #
#                                  SESSION                                    #
# --------------------------------------------------------------------------- #

class Session:
    def __init__(self, xela, reader, host, args, rows):
        self.xela, self.reader, self.host = xela, reader, host
        self.args, self.rows = args, rows
        self.baseline = None
        self.pose_i = 0
        self.phase = "init"

    def read(self):
        """(counts48, delta48, peak, total) or Nones before a baseline."""
        vals = self.xela.sample()
        if vals is None:
            return None, None, None, None
        if self.baseline is None:
            return vals, None, None, None
        d = vals - self.baseline
        dz = d[2::3]
        return vals, d, float(dz.max()), float(np.abs(dz).sum())

    def log(self):
        vals, d, peak, total = self.read()
        if vals is None:
            return None, None
        pose, speed = self.reader.latest()
        if pose is None:
            return peak, total
        self.rows.append(
            [f"{time.time():.6f}", self.pose_i, self.phase,
             "" if peak is None else f"{peak:.0f}",
             "" if total is None else f"{total:.0f}",
             f"{speed:.6f}"]
            + [f"{v:.6f}" for v in pose]
            + [f"{v:.0f}" for v in vals])
        return peak, total

    def contact_centroid(self):
        """(u, v) mm of the contact on the pad, and its offset from centre."""
        dz = self.read()[1]
        if dz is None:
            return None, None
        w = np.clip(dz[2::3], 0.0, None)
        if w.sum() <= 0:
            return None, None
        uv = np.array([[(i % 4) * 5.0, (i // 4) * 5.0]
                       for i in range(N_TAXELS)])
        c = (w[:, None] * uv).sum(axis=0) / w.sum()
        return c, float(np.hypot(*(c - 7.5)))

    def recentre(self, pose_now):
        """Pose that tilts the pad to bring the contact toward its centre.

        The contact sits at offset d from the pad centre, so the pad is leaning
        on that side. Rotating about the in-plane axis PERPENDICULAR to d lifts
        that side and brings the opposite one down. Sign checked analytically:
        for axis a = [-d_v, d_u, 0]/|d| and contact at p = [d_u, d_v, 0],
        a x p = [0, 0, -|d|], i.e. a positive rotation moves the contact along
        -tool Z, away from the surface - which is the direction that lets the
        rest of the pad come down.

        The pad CENTRE is held fixed through the rotation, so re-aiming does
        not also translate the pad off the spot being measured.
        """
        c, off = self.contact_centroid()
        if c is None or off is None or off < ADAPT_MIN_OFFSET_MM:
            return None, off
        # Express the on-pad offset in TOOL axes before deriving the rotation
        # axis. Doing this with a hardcoded identity - as an earlier version
        # did - silently rotates about the wrong axis whenever the mapping is
        # not the identity, and here it is not.
        d_uv = (c - 7.5) / 1000.0
        d = d_uv[0] * PAD_U_IN_TOOL + d_uv[1] * PAD_V_IN_TOOL
        axis = np.array([-d[1], d[0], 0.0])
        na = np.linalg.norm(axis)
        if na < 1e-9:
            return None, off
        axis /= na
        theta = np.radians(min(ADAPT_GAIN_DEG_PER_MM * off, ADAPT_MAX_DEG))

        R_old = rotvec_to_matrix(pose_now[3:])
        R_new = R_old @ Rot.from_rotvec(theta * axis).as_matrix()
        # Hold the pad centre fixed: it is TCP + R*offset in both frames.
        pad = np.array(pose_now[:3]) + R_old @ np.asarray(TOOL_TIP_OFFSET)
        tcp_new = pad - R_new @ np.asarray(TOOL_TIP_OFFSET)
        return np.concatenate([tcp_new, Rot.from_matrix(R_new).as_rotvec()]), off

    def press_one(self, approach_pose, q_approach=None, ring_change=False):
        """Transit -> baseline -> approach -> indent to range -> retract.
        Returns (reason, peak, depth_mm, steps).

        q_approach is the offline-solved joint target. When present the transit
        is a movej to those joints, so the arm uses the branch that --check
        validated instead of whichever one the controller picks.
        """
        self.phase = "transit"
        # Lift straight up first. Without this the joint-space path to the next
        # approach pose can dip into the phantom - approach poses are only
        # ~10 mm off the surface, so there is very little room.
        here, _ = self.reader.latest()
        if here is not None:
            lift = RING_CHANGE_LIFT_M if ring_change else TRANSIT_LIFT_M
            up = np.array(here[:3]).copy()
            up[2] += lift
            send_movel(self.host, np.concatenate([up, here[3:]]), poll=self.log)
            wait_until_settled(self.reader, timeout=8.0, poll=self.log)
        if q_approach is not None:
            send_movej(self.host, q_approach, poll=self.log)
        else:
            send_movel(self.host, approach_pose, poll=self.log)
        wait_until_settled(self.reader, timeout=8.0, poll=self.log)

        # Baseline in free space, after letting the previous pose's
        # viscoelastic residual decay.
        self.phase = "settle"
        self.baseline = None
        sleep_polling(SETTLE_S, self.log)
        buf, end = [], time.time() + 0.5
        while time.time() < end:
            v = self.xela.sample()
            if v is not None:
                buf.append(v)
            time.sleep(0.005)
        if len(buf) < 5:
            return "no_xela_data", 0.0, 0.0, 0
        self.baseline = np.mean(buf, axis=0)

        reason, peak_seen, steps, depth = None, 0.0, 0, 0.0
        # Reset per-pose, or a pose that returns before the press phase (e.g.
        # no_contact) reports the PREVIOUS pose's total. Seen in run
        # 20260822_151442: three no_contact poses after a protective stop all
        # logged total_counts 7377, carried over from pose 17.
        self.last_total = 0.0
        self.last_flat = (False, 0.0)
        try:
            # --- approach: one continuous move until the sensor says touch --
            self.phase = "approach"
            approach_pose = np.asarray(approach_pose, dtype=float)
            R = rotvec_to_matrix(approach_pose[3:])
            z_axis = R[:, 2]
            far = np.concatenate([np.array(approach_pose[:3])
                                  + MAX_APPROACH_STEPS * APPROACH_STEP_M * z_axis,
                                  approach_pose[3:]])

            def contact_pred():
                peak, total = self.read()[2:]
                if peak is None:
                    return None
                return (peak, total) if (peak >= CONTACT_PEAK_COUNTS
                                         and total >= CONTACT_TOTAL_COUNTS) else None

            trig, _ = move_until(self.host, self.reader, far, APPROACH_SPEED_MS,
                                 contact_pred, self.log,
                                 timeout_s=MAX_APPROACH_STEPS
                                 * APPROACH_STEP_M / APPROACH_SPEED_MS + 8.0)
            if trig is None:
                return "no_contact", 0.0, 0.0, 0

            # --- re-aim so the contact lands nearer the pad centre -----------
            if self.args.adapt:
                new_pose, off = self.recentre(self.reader.latest()[0])
                if new_pose is not None:
                    self.phase = "adapt"
                    # Back off first: rotating while touching drags the pad
                    # across the phantom.
                    back = np.concatenate([
                        np.array(self.reader.latest()[0][:3])
                        - z_axis * ADAPT_BACKOFF_M, approach_pose[3:]])
                    send_movel(self.host, back, poll=self.log)
                    wait_until_settled(self.reader, timeout=6.0, poll=self.log)
                    send_movel(self.host, new_pose, poll=self.log)
                    wait_until_settled(self.reader, timeout=6.0, poll=self.log)
                    # Rotation changed the tool axis, so the descent direction
                    # must be re-read rather than reused.
                    z_axis = rotvec_to_matrix(
                        np.asarray(self.reader.latest()[0][3:]))[:, 2]
                    far2 = np.concatenate([
                        np.array(self.reader.latest()[0][:3])
                        + (ADAPT_BACKOFF_M + 0.004) * z_axis,
                        self.reader.latest()[0][3:]])
                    trig2, _ = move_until(self.host, self.reader, far2,
                                          APPROACH_SPEED_MS, contact_pred,
                                          self.log, timeout_s=12.0)
                    c2, off2 = self.contact_centroid()
                    print(f"    re-aimed: contact offset {off:.1f} -> "
                          f"{off2 if off2 is not None else float('nan'):.1f} mm")
                    if trig2 is None:
                        # Lost contact after re-aiming; press will find it.
                        pass

            # --- press: one slow continuous move until the count target ------
            self.phase = "press"
            z0 = np.array(self.reader.latest()[0][:3])
            deep = np.concatenate([z0 + MAX_PRESS_STEPS * STEP_M * z_axis,
                                   approach_pose[3:]])
            track = []
            state = {"reason": None, "peak": 0.0, "total": 0.0, "depth": 0.0,
                     "flat": False, "flat_depth": 0.0}

            def press_pred():
                _, _, peak, total = self.read()
                if peak is None:
                    return None
                state["peak"] = max(state["peak"], peak)
                state["total"] = max(state["total"], total)
                pose_now, _ = self.reader.latest()
                if pose_now is None:
                    return None
                depth = float(np.linalg.norm(
                    np.array(pose_now[:3]) - z0)) * 1000.0
                state["depth"] = depth
                # The plateau flag follows whichever signal is driving the
                # stop, so "stopped responding" means the same thing as
                # "stopped approaching the target".
                track.append((depth, state["total"] if self.args.use_total
                              else state["peak"]))
                # Safety stays on PEAK regardless of the stop criterion: the
                # module limit is about overloading an individual taxel, and a
                # total-based guard would let one corner be crushed while the
                # sum still looked modest.
                if peak >= MAX_PEAK_COUNTS:
                    state["reason"] = "over_range"
                    return "over_range"
                if self.args.use_total:
                    if state["total"] >= self.args.target_total:
                        return "target"
                elif state["peak"] >= self.args.target:
                    return "target"
                # Plateau is RECORDED, never acted on. Stopping on it made
                # marginal poses bistable: sitting on the decision boundary,
                # the same pose would plateau at ~6.3 mm in one run and press
                # on to 13 mm in the next, reporting 485 counts once and 1418
                # the next time for the same physical egg (poses 9/14/16/19,
                # runs empty_23 vs 20260820_204932). Those four contributed
                # 661 counts of mean difference against 111 for the other
                # nineteen - i.e. the stop condition, not the specimen,
                # dominated a between-egg comparison.
                #
                # Every pose now gets the SAME stimulus: press to target or to
                # the travel cap. peak_counts is then directly comparable
                # across poses and across specimens, which is what the
                # measurement is for. The flag still says which poses stopped
                # responding, so nothing is lost.
                if depth >= PLATEAU_MIN_DEPTH_MM and not state["flat"]:
                    old = [m for d, m in track
                           if d <= depth - PLATEAU_WINDOW_MM]
                    cur = state["total"] if self.args.use_total else state["peak"]
                    if old and cur - old[-1] < PLATEAU_MIN_GROWTH:
                        state["flat"] = True
                        state["flat_depth"] = depth
                return None

            trig, reached_end = move_until(
                self.host, self.reader, deep, PRESS_SPEED_MS, press_pred,
                self.log,
                timeout_s=MAX_PRESS_STEPS * STEP_M / PRESS_SPEED_MS + 8.0)
            reason = state["reason"]
            if trig is None and reached_end:
                reason = "travel_cap"
            peak_seen = state["peak"]
            depth = state["depth"]
            steps = len(track)
            self.last_flat = (state["flat"], state["flat_depth"])
            self.last_total = state["total"]

            if reason is None:
                self.phase = "dwell"
                sleep_polling(DWELL_S, self.log)
        finally:
            # over_range used to fire emergency_stop() and end the run. Under a
            # TOTAL target it is an expected per-pose outcome, not a fault: a
            # pose with concentrated corner contact drives one taxel to the
            # ceiling before the pad as a whole reaches the target. Run
            # 20260822_141003 ended on pose 2 for exactly this - peak 2207 at
            # total 6853. The press still stops immediately; the difference is
            # that the arm retracts normally and the sweep continues, with the
            # pose recorded as over_range. Repeated occurrences DO end the run
            # (see the caller), since that would mean something systematic.
            self.phase = "retract"
            send_movel(self.host, approach_pose, poll=self.log)
            wait_until_settled(self.reader, timeout=8.0, poll=self.log)
        return reason, peak_seen, depth, steps


# --------------------------------------------------------------------------- #
#                                    MAIN                                     #
# --------------------------------------------------------------------------- #

def check_poses(poses, press_mm=12.0):
    """Offline reachability check - no robot, no sim, no sensor.

    xela_palpation sends bare movel commands and has no idea whether a pose is
    solvable; an unreachable one just silently fails to move, and the press
    loop then reads "no contact" and blames the phantom. run_side_strip_poses
    solves IK up front and skips such poses, so this brings parity.

    Checks the approach pose AND the deepest point the press could reach, since
    a pose can be reachable at the approach height and not 12 mm further in.
    """
    # UR5 joints are +/-2*pi. ur5_ik_near does NOT enforce this - it only
    # checks position error, then wraps each joint to the 2*pi-equivalent
    # nearest the seed. Chaining seeds around a full ring lets that wrap
    # accumulate, so a grid can be "reachable" by that test while J6 winds
    # past the limit. On the raw 36-pose grid J6 ran to -1402 deg.
    JOINT_LIMIT_DEG = 360.0

    seed = UR5_IK_SEED
    bad_approach, bad_press, over_limit = [], [], []
    q_all = []
    for i, (_, row) in enumerate(poses.iterrows()):
        approach = np.array([row["approach_x"], row["approach_y"],
                             row["approach_z"], row["approach_rx"],
                             row["approach_ry"], row["approach_rz"]])
        q, ok = ur5_ik_near(approach, seed)
        if not ok:
            bad_approach.append(i)
            continue
        seed = q
        q_all.append(q)
        if np.max(np.abs(np.degrees(q))) > JOINT_LIMIT_DEG:
            over_limit.append(i)
        deep = approach.copy()
        deep[:3] = deep[:3] + rotvec_to_matrix(approach[3:])[:, 2] * (press_mm / 1000.0)
        _, ok2 = ur5_ik_near(deep, q)
        if not ok2:
            bad_press.append(i)
    print(f"\nreachability over {len(poses)} pose(s):")
    print(f"  approach poses unreachable : {len(bad_approach)}"
          + (f"  {bad_approach[:12]}{' ...' if len(bad_approach) > 12 else ''}"
             if bad_approach else ""))
    print(f"  unreachable {press_mm:.0f} mm deeper : {len(bad_press)}"
          + (f"  {bad_press[:12]}{' ...' if len(bad_press) > 12 else ''}"
             if bad_press else ""))
    print(f"  outside +/-{JOINT_LIMIT_DEG:.0f} deg joint limits : "
          f"{len(over_limit)}"
          + (f"  {over_limit[:12]}{' ...' if len(over_limit) > 12 else ''}"
             if over_limit else ""))
    if q_all:
        Q = np.degrees(np.array(q_all))
        travel = np.abs(np.diff(Q, axis=0)).sum(axis=0) if len(Q) > 1 else np.zeros(6)
        print("\n  joint    min      max     travel")
        for j in range(6):
            warn = "  <-- OVER LIMIT" if max(abs(Q[:, j].min()),
                                             abs(Q[:, j].max())) > JOINT_LIMIT_DEG else ""
            print(f"    J{j+1}  {Q[:, j].min():8.1f} {Q[:, j].max():8.1f} "
                  f"{travel[j]:9.1f}{warn}")
        # Cable wind-up is about NET rotation of the last wrist joint, not
        # about reachability - a sweep can be perfectly legal and still wrap
        # the sensor lead several turns.
        print(f"\n  wrist-3 net rotation: {Q[-1, 5] - Q[0, 5]:+.0f} deg "
              f"({abs(Q[-1, 5] - Q[0, 5]) / 360:.1f} turns of cable wind-up)")
        if abs(Q[-1, 5] - Q[0, 5]) > 180:
            print("    WARNING: the sensor cable will wrap. Regenerate with "
                  "make_xela_poses.py\n             (wrist minimisation is on "
                  "by default).")

    n_bad = len(set(bad_approach) | set(bad_press) | set(over_limit))
    if n_bad == 0:
        print("\n  -> all poses solvable, within joint limits, no cable "
              "wind-up.")
    else:
        print(f"\n  -> {n_bad} pose(s) would fail. The script sends bare movel "
              "commands, so those\n     will simply not move and then report "
              "no_contact.")
    return bad_approach, bad_press, over_limit


class SimXelaReader:
    """Stand-in for the real sensor so motion can be checked in URSim.

    Returns a constant baseline plus small noise. The noise is NOT cosmetic:
    the stall guard treats bit-identical frames as a dead stream, so a
    perfectly constant fake would trip it within 5 s.
    """

    def __init__(self):
        self._rng = np.random.default_rng(0)
        self._base = np.full(3 * N_TAXELS, 20000.0)
        self._base[2::3] = 28000.0

    def start(self):
        pass

    def wait_for_data(self, timeout=5.0):
        pass

    def latest(self):
        return (time.time(), self.sample())

    def sample(self):
        return self._base + self._rng.normal(0, 3, 3 * N_TAXELS)

    def stop(self):
        pass


def select_out_names(base_dir):
    """Ask what to call this recording, the way record_cone_press.py does.

    Named runs are written flat as <name>.csv / <name>_summary.csv, which is
    the layout rederive.py globs for and the naming the specimen comparisons
    already use. Blank falls back to a timestamp.

    An existing name is never overwritten - repeats get _2, _3 and so on, so
    running a specimen twice for repeatability cannot silently destroy the
    first run. That matters more than it sounds: the trajectory file is the
    only thing that allows a lower target to be re-derived later without
    going back to the robot.
    """
    name = input("Specimen name for this recording "
                 "(blank for a timestamp): ").strip().replace(" ", "_")
    if not name:
        stamp = time.strftime("%Y%m%d_%H%M%S")
        return (os.path.join(base_dir, f"{stamp}_palpation.csv"),
                os.path.join(base_dir, f"{stamp}_palpation_summary.csv"))
    traj = os.path.join(base_dir, f"{name}.csv")
    if os.path.exists(traj):
        n = 2
        while os.path.exists(os.path.join(base_dir, f"{name}_{n}.csv")):
            n += 1
        name = f"{name}_{n}"
        print(f"  that name exists - recording as '{name}' instead")
    return (os.path.join(base_dir, f"{name}.csv"),
            os.path.join(base_dir, f"{name}_summary.csv"))


def select_host():
    while True:
        m = input("Select mode ('sim' or 'real'): ").strip().lower()
        if m == "sim":
            return SIM_HOST
        if m == "real":
            return REAL_HOST
        print("Invalid input. Please type 'sim' or 'real'.")


def monitor(xela):
    """Live per-taxel readout with running maxima, for finding each taxel's
    ceiling by hand.

    Hand pressing is the RIGHT tool for this, not the robot with an indenter.
    A fingertip spreads load over ~100+ mm2, and the module's limit scales
    with contact area (10 N over the full 24x28 mm pad, so ~1.5 N over a
    fingertip patch) - so a firm finger press stays near spec, while a small
    rigid indenter at the same force does not. Press with the pad of a finger,
    not a nail or a tool.

    Shows delta from baseline AND absolute raw counts. They answer different
    questions: the delta is the response ceiling (where the magnet stops
    moving usefully), while the absolute value against the 16-bit range says
    whether the ADC itself is anywhere near clipping.
    """
    print("Press each taxel in turn with a FINGER PAD. Ctrl-C for a summary.\n")
    buf, end = [], time.time() + 1.5
    while time.time() < end:
        v = xela.sample()
        if v is not None:
            buf.append(v)
        time.sleep(0.005)
    if len(buf) < 5:
        print("no XELA data")
        return
    base = np.mean(buf, axis=0)
    print(f"baseline from {len(buf)} samples\n")

    running = np.zeros(N_TAXELS)
    raw_lo = np.full(N_TAXELS, np.inf)
    raw_hi = np.zeros(N_TAXELS)
    try:
        while True:
            v = xela.sample()
            if v is not None:
                z = v[2::3]
                dz = z - base[2::3]
                running = np.maximum(running, dz)
                raw_lo = np.minimum(raw_lo, z)
                raw_hi = np.maximum(raw_hi, z)
                lines = [f"  peak now {dz.max():6.0f}   max seen "
                         f"{running.max():6.0f}   total {np.abs(dz).sum():7.0f}",
                         "  now:                    max seen:"]
                for r in range(4):
                    a = " ".join(f"{dz[4 * r + c]:6.0f}" for c in range(4))
                    b = " ".join(f"{running[4 * r + c]:6.0f}" for c in range(4))
                    lines.append(f"   {a}    {b}")
                sys.stdout.write("\033[H\033[J" + "\n".join(lines) + "\n")
                sys.stdout.flush()
            time.sleep(0.08)
    except KeyboardInterrupt:
        pass

    print("\n" + "=" * 62)
    print("PER-TAXEL MAXIMUM RESPONSE (delta from baseline, counts)")
    for r in range(4):
        print("   " + " ".join(f"{running[4 * r + c]:6.0f}" for c in range(4)))
    print(f"\n  highest taxel : {running.max():.0f} counts "
          f"(taxel {int(running.argmax())})")
    print(f"  lowest taxel  : {running.min():.0f} counts "
          f"(taxel {int(running.argmin())})")
    if running.min() > 0:
        print(f"  spread        : {running.max() / running.min():.1f}x")
    weak = [i for i in range(N_TAXELS) if running[i] < 0.2 * running.max()]
    if weak:
        print(f"  UNDER 20% OF THE BEST: taxels {weak}")
        print("  If those were pressed as hard as the rest, they are dead or "
              "badly degraded.")
    print(f"\n  absolute raw range seen: {raw_lo.min():.0f} .. {raw_hi.max():.0f}"
          f"  ({100 * raw_hi.max() / 65535:.0f}% of 16-bit full scale)")
    print("  ADC clipping would show as a hard stop near 65535; well below "
          "that means\n  any ceiling is mechanical, not electronic.")
    print("=" * 62)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--poses",
                    default=os.path.join(paths.DATA, "xela_poses_nano17reg.csv"),
                    help="pose grid. Default is the PAD-SIZED grid from "
                         "make_xela_poses.py; the old cone_touch/ "
                         "xela_palpation grids are point-tip spacing and "
                         "oversample this sensor ~20x.")
    ap.add_argument("--limit", type=int)
    ap.add_argument("--target-total", type=float, default=TARGET_TOTAL_COUNTS,
                    help="press until the TOTAL response over all 16 taxels "
                         "reaches this (default %(default)s). This is the "
                         "default criterion - see the TARGET_TOTAL_COUNTS "
                         "comment for why peak alone is not comparable.")
    ap.add_argument("--target-peak", dest="use_total", action="store_false",
                    help="stop on the strongest single taxel instead of the "
                         "total. Legacy behaviour; not comparable across poses.")
    ap.set_defaults(use_total=True)
    ap.add_argument("--target", type=float, default=TARGET_PEAK_COUNTS,
                    help="peak-taxel counts to press to (default %(default)s). "
                         "Bounded by the module PRESSURE limit, not by sensor "
                         "saturation - see the module docstring.")
    ap.add_argument("--monitor", action="store_true",
                    help="live readout only, no robot motion")
    ap.add_argument("--adapt", dest="adapt", action="store_true",
                    help="re-aim the pad after contact to centre the touch. "
                         "OFF by default: it centres reliably but MADE RESULTS "
                         "WORSE (mean peak 1218 -> 991 on the same four poses) "
                         "- see the comment block above ADAPT_ENABLED.")
    ap.add_argument("--no-adapt", dest="adapt", action="store_false")
    ap.set_defaults(adapt=ADAPT_ENABLED)
    ap.add_argument("--check", action="store_true",
                    help="offline reachability check over the pose file. No "
                         "robot, no sim, no sensor - run this before any sweep.")
    ap.add_argument("--motion-only", action="store_true",
                    help="visit each APPROACH pose and stop - no descent, no "
                         "press. For verifying transit motion in URSim, where "
                         "there is no contact for the press loop to detect.")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if args.target >= MAX_PEAK_COUNTS:
        ap.error(f"--target must stay below MAX_PEAK_COUNTS ({MAX_PEAK_COUNTS})")
    if args.target > 1850:
        print(f"WARNING: target {args.target:.0f} counts is above anything the "
              "egg run reached (1847) and approaches the estimated per-taxel "
              "pressure limit (~2700). Small contact patches will hit the "
              "module limit before reaching it.")

    if args.monitor:
        xela = XelaReader()
        xela.start()
        xela.wait_for_data()
        monitor(xela)
        return

    poses = pd.read_csv(args.poses)
    if args.limit:
        poses = poses.head(args.limit)

    if args.check:
        check_poses(poses, MAX_PRESS_STEPS * STEP_M * 1000)
        return
    print(f"poses  : {len(poses)} from {args.poses}")
    if args.use_total:
        print(f"target : {args.target_total:.0f} TOTAL counts   "
              f"abort {MAX_PEAK_COUNTS} peak")
    else:
        print(f"target : {args.target:.0f} peak counts   abort {MAX_PEAK_COUNTS}")
    print(f"caps   : {MAX_APPROACH_STEPS * APPROACH_STEP_M * 1000:.0f} mm "
          f"approach, {MAX_PRESS_STEPS * STEP_M * 1000:.0f} mm press")
    # transit + settle + approach sweep + press sweep + dwell + retract
    per_pose = (2.0 + SETTLE_S
                + MAX_APPROACH_STEPS * APPROACH_STEP_M / APPROACH_SPEED_MS * 0.5
                + MAX_PRESS_STEPS * STEP_M / PRESS_SPEED_MS * 0.5
                + DWELL_S + 2.0)
    print(f"speeds  : approach {APPROACH_SPEED_MS*1000:.0f} mm/s, press "
          f"{PRESS_SPEED_MS*1000:.0f} mm/s (continuous, sensor-interrupted)")
    print(f"estimate: ~{len(poses) * per_pose / 60:.0f} min")
    if args.dry_run:
        print("\n--dry-run: nothing sent to the robot.")
        return

    host = select_host()
    xela = XelaReader()
    try:
        xela.start()
        xela.wait_for_data()
        print("XELA OK.")
    except (RuntimeError, TimeoutError) as e:
        # URSim has no phantom to touch, so a motion check does not need the
        # sensor at all. Outside that case, refuse: the tactile signal is the
        # only contact guard there is, and running without it is how the arm
        # ended up stepping forward blind in run 20260805_143703.
        if not args.motion_only:
            raise
        print(f"XELA unavailable ({e.__class__.__name__}) - using a synthetic "
              "reader for the motion check.")
        xela = SimXelaReader()
        xela.start()
    reader = RobotPoseReader(host)
    reader.connect()
    reader.start()
    t0 = time.time()
    while reader.latest()[0] is None:
        if reader.error is not None:
            raise reader.error
        if time.time() - t0 > 5.0:
            raise TimeoutError("no pose from the robot realtime stream")
        time.sleep(0.05)
    print("Robot pose stream OK.")

    # Solve the whole sequence offline and REFUSE to move if anything is out
    # of limits. The controller must not be left to choose IK branches - see
    # send_movej. Solving here also means the run uses exactly the joint path
    # that was validated, including the wrist-3 minimisation in the pose file.
    print("\nsolving joint targets for the whole sequence ...")
    seed = UR5_IK_SEED
    q_list, unsolved, over = [], [], []
    for _i, (_, _row) in enumerate(poses.iterrows()):
        _ap = np.array([_row["approach_x"], _row["approach_y"], _row["approach_z"],
                        _row["approach_rx"], _row["approach_ry"], _row["approach_rz"]])
        _q, _ok = ur5_ik_near(_ap, seed)
        if not _ok:
            unsolved.append(_i)
            q_list.append(None)
            continue
        seed = _q
        q_list.append(_q)
        if np.max(np.abs(np.degrees(_q))) > 360.0:
            over.append(_i)
    _Q = np.degrees(np.array([q for q in q_list if q is not None]))
    if len(_Q):
        print(f"  J6 range {_Q[:, 5].min():.1f}..{_Q[:, 5].max():.1f} deg, "
              f"net {_Q[-1, 5] - _Q[0, 5]:+.0f} deg")
        print("  per-joint max |angle|: "
              + "  ".join(f"J{j+1}={np.max(np.abs(_Q[:, j])):.0f}"
                          for j in range(6)))
    if unsolved:
        print(f"  {len(unsolved)} pose(s) have no IK solution: {unsolved[:10]}"
              "  (these will be SKIPPED)")
    if over:
        print(f"\n*** REFUSING TO RUN: {len(over)} pose(s) exceed +/-360 deg "
              f"joint limits: {over[:10]}")
        print("    Regenerate with make_xela_poses.py (wrist minimisation is "
              "on by default).")
        reader.stop_reading()
        xela.stop()
        return

    if args.motion_only:
        input(f"\nMOTION ONLY: visiting {len(poses)} approach pose(s). No "
              "descent, no press. Enter to start ...")
    else:
        crit = (f"{args.target_total:.0f} TOTAL counts" if args.use_total
                else f"{args.target:.0f} peak counts")
        input(f"\nAbout to indent {len(poses)} pose(s) to {crit}. "
              "Supervise, hand on the e-stop. Enter to start ...")

    traj, summ = select_out_names(DATA_DIR)
    rows, summary = [], []
    sess = Session(xela, reader, host, args, rows)

    def save():
        fields = (["t", "pose", "phase", "peak_counts", "total_counts",
                   "speed", "px", "py", "pz", "rx", "ry", "rz"]
                  + [f"raw_{ax}{i}" for i in range(N_TAXELS)
                     for ax in ("x", "y", "z")])
        with open(traj, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(fields)
            w.writerows(rows)
        with open(summ, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["pose", "strip", "reason", "peak_counts",
                        "total_counts", "depth_mm", "steps",
                        "went_flat", "flat_depth_mm", "px", "py", "pz"])
            w.writerows(summary)
        print(f"\nsaved {len(rows)} rows -> {traj}")
        print(f"saved {len(summary)} pose summaries -> {summ}")

    prev_ring = None
    n_over = 0
    try:
        for i, (_, row) in enumerate(poses.iterrows()):
            sess.pose_i = i
            cur_ring = int(row.get("strip", -1))
            approach = [row["approach_x"], row["approach_y"], row["approach_z"],
                        row["approach_rx"], row["approach_ry"],
                        row["approach_rz"]]
            q_i = q_list[i]
            if q_i is None:
                print(f"  pose {i+1}/{len(poses)}: no IK solution - skipped")
                summary.append([i, int(row.get("strip", -1)), "unsolvable",
                                "", "", "", 0, "", "", "", "", ""])
                continue
            try:
                if args.motion_only:
                    sess.phase = "transit"
                    send_movej(host, q_i, poll=sess.log)
                    wait_until_settled(reader, timeout=8.0, poll=sess.log)
                    reason, peak, depth, steps = "motion_only", 0.0, 0.0, 0
                else:
                    reason, peak, depth, steps = sess.press_one(
                        approach, q_i, ring_change=(prev_ring is not None
                                                    and cur_ring != prev_ring))
            except XelaStalled as e:
                print(f"\n*** XELA STREAM STALLED: {e}")
                print("    Stopping - without a tactile signal this would "
                      "drive blind.")
                emergency_stop(host)
                summary.append([i, int(row.get("strip", -1)), "xela_stalled",
                                "", "", "", 0, "", "", "", "", ""])
                break
            pose, _ = reader.latest()
            flat, flat_d = sess.last_flat
            summary.append([i, int(row.get("strip", -1)), reason or "ok",
                            f"{peak:.0f}", f"{sess.last_total:.0f}",
                            f"{depth:.2f}", steps,
                            int(flat), f"{flat_d:.2f}",
                            f"{pose[0]:.5f}", f"{pose[1]:.5f}",
                            f"{pose[2]:.5f}"])
            prev_ring = cur_ring
            tag = f"  [{reason}]" if reason else ""
            print(f"  pose {i+1}/{len(poses)}: total {sess.last_total:6.0f} "
                  f"peak {peak:5.0f} at {depth:5.1f} mm ({steps} samples){tag}")
            if reason == "over_range":
                n_over += 1
                print(f"       one taxel hit {MAX_PEAK_COUNTS} before the pad "
                      f"reached {args.target_total:.0f} total - concentrated "
                      "contact here.")
                if n_over >= MAX_OVER_RANGE_POSES:
                    print(f"\n*** {n_over} poses hit the peak ceiling - "
                          "stopping. That is systematic, not a property of a\n"
                          "    few spots: either the target total is too high "
                          "for this phantom, or the\n    pad is contacting far "
                          "more sharply than expected. ***")
                    emergency_stop(host)
                    break
            else:
                n_over = 0
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        reader.stop_reading()
        xela.stop()
        save()


if __name__ == "__main__":
    main()
