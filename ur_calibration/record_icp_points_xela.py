#!/usr/bin/env python3
"""
Record ICP calibration points using the XELA PAD as the probe.

WHY A SEPARATE RECORDER
record_icp_points.py asks you to touch an exact spot with a pointed tip and
type the TCP pose off the teach pendant. The XELA has no tip - it is a flat
24 x 28 mm face that touches the cone wherever it happens to meet it, and you
cannot see where. So contact is detected by the SENSOR and the pose is read
from the robot, instead of being judged by eye and transcribed.

WHY IT IS WORTH RE-PROBING AT ALL
Both ends of the pipeline apply the same constant:

    this script  :  contact = TCP     + R * TOOL_TIP_OFFSET
    pose gen     :  TCP     = contact - R * TOOL_TIP_OFFSET

Probe and press with the SAME tool and an error in that 86 mm CANCELS - the
fitted surface is displaced by exactly the amount the press is displaced back.
That is why the cone pipeline worked without anyone re-verifying the offset.
Today you probe with the Nano17 tip and press with the XELA pad, so it does
not cancel. Re-probing with the pad restores the cancellation WITHOUT anyone
needing to know the true offset.

Cancellation is exact only where press orientation matches probe orientation,
so spread the probe points over the same range of orientations you will press
at - not all clustered at the apex.

HOW A FLAT PAD GIVES A POINT
If the pad is not perfectly normal to the surface it touches on its leading
EDGE, not its centre - up to 12 mm off, which would be worse than the error
being fixed. The sensor solves this: the taxel map says where ON THE PAD the
contact landed, so the recorded point is corrected by the response-weighted
centroid. That drops the error to roughly half a taxel pitch (~2.5 mm) and
makes the procedure tolerant of jogging only roughly normal (~15 deg is fine).

The centroid is stored in the CSV as cu_mm/cv_mm, so if PAD_U_IN_TOOL /
PAD_V_IN_TOOL below turn out to be mapped wrong for this mount, the contact
points can be recomputed offline without re-probing anything.

PROCEDURE (repeat 10-15 times)
  1. Jog the robot so the pad hovers a few mm above the spot you want,
     roughly normal to the surface there.
  2. Press Enter.
  3. The script steps along TOOL +Z - the tool's own axis, which is the
     direction you just aimed - until the taxels register contact, records
     everything, and retracts.

SAFETY
  * Descent stops on contact, on a per-taxel count ceiling, or on a hard
    travel cap that does not depend on the sensor at all.
  * A stalled XELA stream aborts rather than descending blind.
  * Supervise. Hand on the e-stop.

Usage:
    python3 record_icp_points_xela.py [--out CSV] [--max-descent 25]
"""

import argparse
import csv
import os
import sys

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO_ROOT)
sys.path.insert(0, os.path.join(REPO_ROOT, "Xela_sensor", "palpation"))

import paths
from pose_utils import rotvec_to_matrix, TOOL_TIP_OFFSET
from xela_palpation import (XelaReader, RobotPoseReader, XelaStalled,
                            send_movel, wait_until_settled, step_along_tool_z,
                            select_host, N_TAXELS,
                            CONTACT_PEAK_COUNTS, CONTACT_TOTAL_COUNTS)

PITCH_MM = 5.0
PAD_CENTRE_UV = np.array([7.5, 7.5])   # centre of the 4x4 grid, mm from taxel 0

# Pad axes in TOOL axes. MEASURED 2026-08-20 with measure_pad_axes.py, not
# assumed: tilting +5 deg about tool X moved the centroid +u, and about tool Y
# moved it -v, which puts pad-u along tool +Y and pad-v along tool +X.
#
# The identity guess these replace was SWAPPED. Any points recorded before
# this date carry a lateral correction applied along the wrong axis - but the
# raw centroid is stored as cu_mm/cv_mm, so they can be recomputed rather than
# re-probed.
PAD_U_IN_TOOL = np.array([0.0, 1.0, 0.0])
PAD_V_IN_TOOL = np.array([1.0, 0.0, 0.0])

APPROACH_STEP_M = 0.0005
MAX_TAXEL_COUNTS = 1500      # ceiling during probing. Far below the palpation
                             # target - probing only needs first contact, so
                             # there is no reason to load the pad hard.
RETRACT_EXTRA_M = 0.005


def centroid_uv(dz):
    """Response-weighted contact centroid in pad coords (mm from taxel 0)."""
    w = np.clip(dz, 0.0, None)
    if w.sum() <= 0:
        return PAD_CENTRE_UV.copy()
    uv = np.array([[(i % 4) * PITCH_MM, (i // 4) * PITCH_MM]
                   for i in range(N_TAXELS)], dtype=float)
    return (w[:, None] * uv).sum(axis=0) / w.sum()


def report_aim(reader, surface_csv=None):
    """Where is the tool pointing, relative to the registered phantom?

    A tilted tool steps DIAGONALLY - at 43 deg from vertical a step is roughly
    equal parts sideways and down - which can look like the arm is moving away
    even while it closes on the surface. This turns that into numbers.
    """
    import pandas as pd
    pose, _ = reader.latest()
    R = rotvec_to_matrix(pose[3:])
    z = R[:, 2]
    pad = np.array(pose[:3]) + R @ np.asarray(TOOL_TIP_OFFSET)

    df = pd.read_csv(surface_csv or paths.SURFACE_POINTS_BASE)
    P = df[["x", "y", "z"]].to_numpy()
    Nrm = df[["nx", "ny", "nz"]].to_numpy()

    print(f"\nTCP        : [{pose[0]*1000:8.2f}, {pose[1]*1000:8.2f}, "
          f"{pose[2]*1000:8.2f}] mm")
    print(f"tool +Z    : [{z[0]:+.3f}, {z[1]:+.3f}, {z[2]:+.3f}]"
          f"   ({np.degrees(np.arccos(-z[2])):.0f} deg from straight down)")
    print(f"0.5 mm step: dX {0.5*z[0]:+.3f}  dY {0.5*z[1]:+.3f}  "
          f"dZ {0.5*z[2]:+.3f} mm")
    print(f"pad centre : [{pad[0]*1000:8.2f}, {pad[1]*1000:8.2f}, "
          f"{pad[2]*1000:8.2f}] mm  (TCP + {TOOL_TIP_OFFSET[2]*1000:.0f} mm along +Z)")

    i = int(np.argmin(np.linalg.norm(P - pad, axis=1)))
    side = float(np.dot(pad - P[i], Nrm[i]))
    print(f"\npad is {abs(side)*1000:.1f} mm "
          f"{'OUTSIDE' if side > 0 else 'INSIDE (!)'} the registered surface")

    v = P - pad
    t = v @ z
    perp = np.linalg.norm(v - np.outer(t, z), axis=1)
    ahead = t > 0
    if not ahead.any():
        print("ray along tool +Z: NOTHING AHEAD - the tool is aimed AWAY "
              "from the phantom.")
        return
    # NEAREST point ahead, not the global minimum perpendicular distance.
    # Minimising perp alone picks whichever point the axis happens to shave
    # closest anywhere along its length, which can be far past the phantom -
    # on 2026-08-06 it reported "hits 61 mm ahead" for a pose whose actual
    # surface was 3.3 mm in front, and that misdiagnosis cost a debugging pass.
    idx_ahead = np.where(ahead)[0]
    k = int(idx_ahead[int(np.argmin(np.linalg.norm(v[idx_ahead], axis=1)))])
    print(f"ray along tool +Z: closest approach {perp[k]*1000:.1f} mm, "
          f"{t[k]*1000:.1f} mm ahead")

    # "Does the ray hit" is NOT sufficient. A tool aimed almost tangentially
    # still hits eventually, but the pad scrapes ALONG the surface instead of
    # pressing into it - the arm fights the contact, motion goes jerky, and no
    # clean first-touch is ever registered. Seen 2026-08-06: a pose sitting
    # 3.2 mm off the surface whose ray only converged 61 mm ahead failed to
    # record, while one 7.0 mm off with a hit 10 mm ahead worked.
    #
    # The discriminator is the angle between the descent direction and the
    # local inward normal, plus the ratio of travel-to-hit against how close
    # the pad already is.
    incidence = np.degrees(np.arccos(np.clip(np.dot(z, -Nrm[i]), -1.0, 1.0)))
    print(f"incidence vs local surface normal: {incidence:.0f} deg "
          "(0 = straight in, 90 = tangential)")

    bad = False
    if perp[k] >= 0.012:
        print("  -> would MISS by more than the pad half-width. Re-aim.")
        bad = True
    if incidence > 45.0:
        print(f"  -> GRAZING: {incidence:.0f} deg off the normal. The pad will "
              "scrape along the\n     surface rather than press into it. "
              "Re-aim closer to normal.")
        bad = True
    if abs(side) > 1e-9 and t[k] > 4.0 * abs(side) and t[k] > 0.02:
        print(f"  -> TANGENTIAL: the pad is only {abs(side)*1000:.1f} mm from "
              f"the surface but the axis\n     does not reach it for "
              f"{t[k]*1000:.0f} mm. Re-aim.")
        bad = True
    if not bad:
        print("  -> AIMED AT THE PHANTOM. Contact expected after about "
              f"{t[k]*1000:.0f} mm of stepping.")


def expected_contact_m(reader, surface_csv=None):
    """Distance to the nearest surface point ahead, from the registered
    phantom. Used only as a sanity bound on the descent - the registration may
    be stale, so it is deliberately generous."""
    try:
        import pandas as pd
        pose, _ = reader.latest()
        R = rotvec_to_matrix(pose[3:])
        pad = np.array(pose[:3]) + R @ np.asarray(TOOL_TIP_OFFSET)
        P = pd.read_csv(surface_csv or paths.SURFACE_POINTS_BASE)[
            ["x", "y", "z"]].to_numpy()
        v = P - pad
        ahead = (v @ R[:, 2]) > 0
        if not ahead.any():
            return None
        return float(np.linalg.norm(v[ahead], axis=1).min())
    except Exception:
        return None


def probe_once(xela, reader, host, max_descent_m):
    """Descend along tool +Z until contact. Returns a dict or None."""
    pose0, _ = reader.latest()
    if pose0 is None:
        print("  no robot pose")
        return None

    buf, t_end = [], __import__("time").time() + 1.5
    import time as _t
    while _t.time() < t_end:
        v = xela.sample()
        if v is not None:
            buf.append(v)
        _t.sleep(0.005)
    if len(buf) < 5:
        print("  no XELA data for a baseline")
        return None
    base = np.mean(buf, axis=0)

    # Refuse to start if the pad is ALREADY loaded. The baseline is taken here,
    # so an existing contact gets absorbed into it - detection then needs load
    # ON TOP of what is already there, and if the phantom yields instead of
    # loading further, contact is never registered and the descent runs to the
    # travel cap while shoving the phantom. That is the failure seen at pose
    # [64.8, -514.5, 181.7] on 2026-08-06.
    spread = float(np.ptp(np.array(buf)[:, 2::3], axis=0).max())
    if spread > 200:
        print(f"  [refuse] the Z channels are moving by {spread:.0f} counts "
              "while stationary -\n           the pad is probably already "
              "touching. Back off and retry.")
        return None

    # Bound the descent by what the registration says is ahead, so a failure
    # of contact detection cannot drive the pad deep into the phantom. x3 is
    # slack for a stale registration; the CLI cap still applies on top.
    exp = expected_contact_m(reader)
    if exp is not None:
        bound = min(max_descent_m, max(0.008, 3.0 * exp))
        if bound < max_descent_m:
            print(f"  (surface expected ~{exp*1000:.1f} mm ahead; limiting "
                  f"this descent to {bound*1000:.1f} mm)")
        max_descent_m = bound

    n_steps = int(max_descent_m / APPROACH_STEP_M)
    hit = None
    for k in range(n_steps):
        v = xela.sample()
        if v is not None:
            dz = (v - base)[2::3]
            if dz.max() >= MAX_TAXEL_COUNTS:
                print(f"  [stop] taxel ceiling {dz.max():.0f} counts")
                hit = (v, dz)
                break
            if dz.max() >= CONTACT_PEAK_COUNTS and np.abs(dz).sum() >= CONTACT_TOTAL_COUNTS:
                hit = (v, dz)
                break
        step_along_tool_z(reader, host, APPROACH_STEP_M)
    if hit is None:
        print(f"  [abort] no contact within {max_descent_m*1000:.0f} mm")
        pose, _ = reader.latest()
        back = np.concatenate([pose0[:3], pose0[3:]])
        send_movel(host, back)
        wait_until_settled(reader, timeout=10.0)
        return None

    v, dz = hit
    pose, _ = reader.latest()
    R = rotvec_to_matrix(pose[3:])

    cu, cv = centroid_uv(dz)
    d_uv = np.array([cu, cv]) - PAD_CENTRE_UV          # mm, in pad axes
    lateral_tool = (d_uv[0] * PAD_U_IN_TOOL + d_uv[1] * PAD_V_IN_TOOL) / 1000.0
    # Contact = pad centre along the tool axis, PLUS where on the pad it
    # actually landed. Without the second term an edge touch is recorded as if
    # it happened dead centre.
    contact = np.array(pose[:3]) + R @ (np.asarray(TOOL_TIP_OFFSET) + lateral_tool)

    descent = float(np.linalg.norm(np.array(pose[:3]) - np.array(pose0[:3])))
    n_live = int((dz >= CONTACT_PEAK_COUNTS).sum())

    # Retract to where we started, plus margin.
    lift = np.concatenate([np.array(pose0[:3]) - R[:, 2] * RETRACT_EXTRA_M,
                           pose0[3:]])
    send_movel(host, lift)
    wait_until_settled(reader, timeout=10.0)

    return {
        "x_tcp": pose[0], "y_tcp": pose[1], "z_tcp": pose[2],
        "rx": pose[3], "ry": pose[4], "rz": pose[5],
        "x": contact[0], "y": contact[1], "z": contact[2],
        "cu_mm": round(float(cu), 2), "cv_mm": round(float(cv), 2),
        "peak_counts": round(float(dz.max())),
        "total_counts": round(float(np.abs(dz).sum())),
        "n_taxels": n_live, "descent_mm": round(descent * 1000, 2),
    }


FIELDS = ["x_tcp", "y_tcp", "z_tcp", "rx", "ry", "rz", "x", "y", "z",
          "cu_mm", "cv_mm", "peak_counts", "total_counts", "n_taxels",
          "descent_mm"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(
        paths.CALIBRATION, "physical_points_xela.csv"))
    ap.add_argument("--max-descent", type=float, default=60.0,
                    help="mm of travel allowed before giving up (default "
                         "%(default)s). This is a BACKSTOP for contact "
                         "detection failing, not the expected travel - park "
                         "a few mm off and contact normally comes in ~10 mm. "
                         "Raise it freely; setting it absurdly high just "
                         "means a missed aim drives into the holder.")
    ap.add_argument("--aim", action="store_true",
                    help="report where the tool is currently aimed relative to "
                         "the registered phantom, then exit. Moves nothing.")
    a = ap.parse_args()

    print("=== XELA ICP point recorder ===")
    print("Jog the pad a few mm above a spot, roughly normal to the surface,")
    print("then press Enter. The script descends along TOOL +Z until the")
    print("taxels register contact, records, and retracts.")
    print("Spread the points over the orientations you will PRESS at.\n")

    host = select_host()
    xela = XelaReader()
    xela.start()
    xela.wait_for_data()
    print("XELA OK.")
    reader = RobotPoseReader(host)
    reader.connect()
    reader.start()
    import time as _t
    t0 = _t.time()
    while reader.latest()[0] is None:
        if reader.error is not None:
            raise reader.error
        if _t.time() - t0 > 5.0:
            raise TimeoutError("no pose from the robot realtime stream")
        _t.sleep(0.05)

    # Show the decoded pose and the direction a step will actually take,
    # BEFORE anything moves. A wrong realtime-packet layout decodes garbage
    # here, and the first step then commands a movel built from it - on
    # 2026-08-06 that sent the arm to the robot's base origin. If the pose or
    # the step direction below look wrong, stop; do not probe.
    pose, _ = reader.latest()
    zdir = rotvec_to_matrix(pose[3:])[:, 2]
    print("Robot pose stream OK.")
    print(f"  TCP now      : [{pose[0]*1000:8.2f}, {pose[1]*1000:8.2f}, "
          f"{pose[2]*1000:8.2f}] mm")
    print(f"  tool +Z      : [{zdir[0]:+.3f}, {zdir[1]:+.3f}, {zdir[2]:+.3f}]"
          "   <- probing steps travel THIS way")
    print(f"  0.5 mm step  : dX {0.5*zdir[0]:+.3f}  dY {0.5*zdir[1]:+.3f}  "
          f"dZ {0.5*zdir[2]:+.3f} mm")
    if zdir[2] > -0.5:
        print("  WARNING: tool +Z is not pointing downward. If the pad faces "
              "the phantom,\n           this should be roughly [0, 0, -1] at "
              "the apex. Check before probing.")
    print()

    if a.aim:
        report_aim(reader)
        reader.stop_reading(); xela.stop()
        return

    rows = []
    try:
        while True:
            n = len(rows) + 1
            hint = " (APEX - do this one first)" if n == 1 else ""
            cmd = input(f"Point {n}{hint}: jog into position, Enter to probe "
                        "(or 'done'): ").strip().lower()
            if cmd == "done":
                break
            try:
                rec = probe_once(xela, reader, host, a.max_descent / 1000.0)
            except XelaStalled as e:
                print(f"\n*** XELA STALLED: {e}\n    Stopping.")
                break
            if rec is None:
                continue
            rows.append(rec)
            off = np.hypot(rec["cu_mm"] - PAD_CENTRE_UV[0],
                           rec["cv_mm"] - PAD_CENTRE_UV[1])
            print(f"  contact [{rec['x']*1000:8.2f}, {rec['y']*1000:8.2f}, "
                  f"{rec['z']*1000:8.2f}] mm   after {rec['descent_mm']:.1f} mm")
            print(f"  on-pad centroid ({rec['cu_mm']:.1f}, {rec['cv_mm']:.1f}) "
                  f"= {off:.1f} mm off centre, {rec['n_taxels']} taxel(s) live, "
                  f"peak {rec['peak_counts']}")
            if off > 8.0:
                print("  NOTE: contact well off centre - the pad met the "
                      "surface on an edge.\n        Correction applied, but "
                      "jog closer to normal for the next one.")
            if rec["n_taxels"] <= 1:
                print("  NOTE: only one taxel responded - grazing contact, "
                      "centroid is least\n        reliable here.")
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        reader.stop_reading()
        xela.stop()
        if rows:
            with open(a.out, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=FIELDS)
                w.writeheader()
                w.writerows(rows)
            print(f"\nSaved {len(rows)} point(s) -> {a.out}")
            if len(rows) < 10:
                print("Fewer than 10 points - ICP works better with 10-15.")
            print("\nNext: python3 ur_calibration/calibrate_icp.py --xela")
        else:
            print("\nNo points recorded; nothing written.")


if __name__ == "__main__":
    main()
