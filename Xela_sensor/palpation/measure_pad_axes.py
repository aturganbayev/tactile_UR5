#!/usr/bin/env python3
"""
Measure how the taxel grid's axes map onto the TOOL frame, by tilting a known
amount and watching which way the contact centroid moves.

WHY THIS EXISTS
xela_palpation's contact-centring correction rotates the pad about an axis
derived from the contact centroid, and that requires knowing which tool axis
the grid's u (taxel index +1) and v (index +4) directions correspond to. That
mapping was ASSUMED to be the identity - pad-u along tool X - and never
measured for the end-effector mount; the only measured value in the project
came from the breadboard mount, which is different hardware.

Run 20260820_180906 shows the assumption is wrong. The correction moved the
contact toward the pad centre on one pose, perpendicular on another and AWAY
on a third (13 / 81 / 123 degrees from the intended direction). A gain that is
merely too small fails consistently in ONE direction; scatter like that is a
wrong rotation axis.

WHAT IT DOES
Three light touches on a flat surface, holding the pad centre fixed between
them so the contact stays on the same spot:
    1. as parked                        -> baseline centroid
    2. tilted +ANGLE about TOOL X       -> centroid moves ...
    3. tilted +ANGLE about TOOL Y       -> centroid moves ...

PREDICTION UNDER THE CURRENT ASSUMPTION (pad-u = tool X, pad-v = tool Y)
Tool +Z points INTO the surface, so for a rotation about tool X by +theta a
point at +Y moves to z = +y*sin(theta), i.e. deeper - that side takes the load
and the centroid moves toward +v. About tool Y, a point at +X moves to
z = -x*sin(theta), i.e. away - so the centroid moves toward -u.

    tilt +X  ->  centroid moves +v
    tilt +Y  ->  centroid moves -u

If the measured moves are swapped, u and v are swapped. If one is reversed,
that axis needs a sign flip. Either way the answer goes straight into
PAD_U_IN_TOOL / PAD_V_IN_TOOL.

SURFACE
Anything flat and rigid works - the test only needs the contact to move
predictably. Prefer something NON-MAGNETIC: this is a magnetometer sensor, and
a steel plate can perturb it before touching. Wood, acrylic or aluminium is
safer than a steel breadboard.

SAFETY
  * Touches lightly and stops - it does not press to any target.
  * Per-taxel ceiling, travel cap, and stall detection all apply.
  * Supervise. Hand on the e-stop.

Usage:
    python3 measure_pad_axes.py [--angle 5] [--max-descent 25]
"""

import argparse
import os
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation as Rot

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(_THIS_DIR))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, _THIS_DIR)

from pose_utils import rotvec_to_matrix, TOOL_TIP_OFFSET
from xela_palpation import (XelaReader, RobotPoseReader, XelaStalled,
                            send_movel, wait_until_settled, move_until,
                            select_host, N_TAXELS, APPROACH_SPEED_MS,
                            CONTACT_PEAK_COUNTS, CONTACT_TOTAL_COUNTS)

PITCH_MM = 5.0
CENTRE_UV = np.array([7.5, 7.5])
LIFT_M = 0.010
TOUCH_CEILING = 800          # stop well short of a real press - this only
                             # needs the contact patch, not load


def centroid(dz):
    w = np.clip(dz, 0.0, None)
    if w.sum() <= 0:
        return None
    uv = np.array([[(i % 4) * PITCH_MM, (i // 4) * PITCH_MM]
                   for i in range(N_TAXELS)])
    return (w[:, None] * uv).sum(axis=0) / w.sum()


def touch(xela, reader, host, base_pose, R, max_descent_m):
    """Move to base_pose with orientation R, descend to light contact,
    return the centroid, then lift clear."""
    rv = Rot.from_matrix(R).as_rotvec()
    pose = np.concatenate([base_pose[:3], rv])
    send_movel(host, pose)
    wait_until_settled(reader, timeout=10.0)

    buf, end = [], time.time() + 1.5
    while time.time() < end:
        v = xela.sample()
        if v is not None:
            buf.append(v)
        time.sleep(0.005)
    if len(buf) < 5:
        print("  no XELA baseline")
        return None
    b = np.mean(buf, axis=0)

    z = R[:, 2]
    far = np.concatenate([np.array(pose[:3]) + max_descent_m * z, rv])
    got = {}

    def pred():
        v = xela.sample()
        if v is None:
            return None
        dz = (v - b)[2::3]
        if dz.max() >= TOUCH_CEILING:
            got["dz"] = dz
            return "ceiling"
        if dz.max() >= CONTACT_PEAK_COUNTS and np.abs(dz).sum() >= CONTACT_TOTAL_COUNTS:
            got["dz"] = dz
            return "contact"
        return None

    trig, _ = move_until(host, reader, far, APPROACH_SPEED_MS, pred,
                         lambda: None,
                         timeout_s=max_descent_m / APPROACH_SPEED_MS + 8.0)
    c = centroid(got["dz"]) if "dz" in got else None
    if trig is None:
        print(f"  no contact within {max_descent_m*1000:.0f} mm")

    p, _ = reader.latest()
    send_movel(host, np.concatenate([np.array(p[:3]) - z * LIFT_M, rv]))
    wait_until_settled(reader, timeout=10.0)
    return c


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--angle", type=float, default=5.0,
                    help="tilt applied about each tool axis, deg (default %(default)s)")
    ap.add_argument("--max-descent", type=float, default=25.0)
    a = ap.parse_args()

    print("=== XELA pad-axis mapping test ===")
    print("Park the pad a few mm above a FLAT, RIGID, ideally NON-MAGNETIC")
    print("surface, with the pad roughly parallel to it. Three light touches")
    print("follow; nothing is pressed hard.\n")

    host = select_host()
    xela = XelaReader()
    xela.start()
    xela.wait_for_data()
    reader = RobotPoseReader(host)
    reader.connect()
    reader.start()
    t0 = time.time()
    while reader.latest()[0] is None:
        if reader.error is not None:
            raise reader.error
        if time.time() - t0 > 5.0:
            raise TimeoutError("no pose from the robot")
        time.sleep(0.05)

    pose0, _ = reader.latest()
    R0 = rotvec_to_matrix(pose0[3:])
    # Hold the PAD CENTRE fixed across all three touches, so the contact stays
    # on the same patch of surface and the only thing changing is the tilt.
    pad = np.array(pose0[:3]) + R0 @ np.asarray(TOOL_TIP_OFFSET)
    print(f"TCP  [{pose0[0]*1000:8.2f}, {pose0[1]*1000:8.2f}, {pose0[2]*1000:8.2f}] mm")
    print(f"tool +Z [{R0[0,2]:+.3f}, {R0[1,2]:+.3f}, {R0[2,2]:+.3f}]")
    if R0[2, 2] > -0.5:
        print("WARNING: tool +Z not pointing down; park the pad facing the "
              "surface.")
    input("\nEnter to run the three touches ...")

    th = np.radians(a.angle)
    results = {}
    try:
        for name, R in (("flat", R0),
                        (f"+{a.angle:.0f} about tool X",
                         R0 @ Rot.from_rotvec(th * np.array([1., 0, 0])).as_matrix()),
                        (f"+{a.angle:.0f} about tool Y",
                         R0 @ Rot.from_rotvec(th * np.array([0, 1., 0])).as_matrix())):
            tcp = pad - R @ np.asarray(TOOL_TIP_OFFSET)
            print(f"\n--- {name} ---")
            c = touch(xela, reader, host, np.concatenate([tcp, [0, 0, 0]]), R,
                      a.max_descent / 1000.0)
            if c is None:
                print("  no centroid")
            else:
                print(f"  centroid (u={c[0]:.1f}, v={c[1]:.1f}) mm, "
                      f"{np.hypot(*(c - CENTRE_UV)):.1f} mm off centre")
            results[name] = c
    except XelaStalled as e:
        print(f"\n*** XELA STALLED: {e}")
    finally:
        reader.stop_reading()
        xela.stop()

    print("\n" + "=" * 62)
    base = results.get("flat")
    if base is None:
        print("No baseline touch - cannot interpret.")
        return
    print("RESULT")
    for name, c in results.items():
        if name == "flat" or c is None:
            continue
        d = c - base
        axis = "u" if abs(d[0]) > abs(d[1]) else "v"
        print(f"  {name}: centroid moved du={d[0]:+.1f}  dv={d[1]:+.1f} mm "
              f"-> dominated by {axis}")
    print()
    print("  EXPECTED if pad-u = tool X and pad-v = tool Y:")
    print("    +X tilt -> centroid moves +v")
    print("    +Y tilt -> centroid moves -u")
    print()
    print("  If the two are swapped, swap PAD_U_IN_TOOL and PAD_V_IN_TOOL.")
    print("  If a direction is reversed, negate that one.")
    print("  Those constants live in ur_calibration/record_icp_points_xela.py")
    print("  and are used by xela_palpation's contact-centring correction.")
    print("=" * 62)


if __name__ == "__main__":
    main()
