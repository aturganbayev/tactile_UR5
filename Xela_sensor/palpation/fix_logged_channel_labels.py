#!/usr/bin/env python3
"""
Repair the channel LABELS in palpation CSVs written before 2026-07-28.

THE BUG
xela_palpation_recorder.py built its header axis-grouped:

    x0..x15, y0..y15, z0..z15

but wrote the sample rows interleaved, the order the XELA server actually
sends them:

    x0, y0, z0, x1, y1, z1, ...

So only raw_x0 was correctly named. raw_x1 actually held taxel 0's Y, raw_x2
held taxel 0's Z, and so on. The giveaway is that in an affected CSV the
highest idle channel rotates z, y, x, z, y, x ... across taxels, whereas the
live stream has Z high on all 16 (confirmed by reading a raw message: every
taxel's third value is ~27000 while the first two sit near 16400).

WHAT IS AND ISN'T AFFECTED
  * Only the header line is wrong. Every number in the file is correct and in
    the right column position - the columns are simply misnamed. Nothing was
    lost, and this script only rewrites line 1.
  * The `response` column and all press detection are UNAFFECTED: they were
    computed from the in-memory flat vector (`delta[2::3]`, which really is
    the Z channels), never from the CSV names.
  * xela_session_logger.py (the calibration logger) was always correct - it
    builds its header interleaved. Calibration data needs no repair.

Usage:
    python3 fix_logged_channel_labels.py                 # dry run, all files
    python3 fix_logged_channel_labels.py --apply         # rewrite headers
    python3 fix_logged_channel_labels.py FILE... --apply

Dry run by default: it edits data files in place (the trajectory CSVs are
~150 MB, so rewriting a whole copy is worse than a one-line edit), and that
is not something to do without looking first.
"""

import argparse
import glob
import os
import sys

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(_THIS_DIR, "data")
N_TAXELS = 16

WRONG = [f"{ax}{i}" for ax in ("x", "y", "z") for i in range(N_TAXELS)]
RIGHT = [f"{ax}{i}" for i in range(N_TAXELS) for ax in ("x", "y", "z")]


def fix_header(line):
    """Return (new_header, n_renamed) or (None, 0) if nothing to do."""
    cols = line.rstrip("\r\n").split(",")
    out, n = [], 0
    for c in cols:
        for prefix in ("raw_", "d_"):
            if c.startswith(prefix) and c[len(prefix):] in WRONG:
                new = prefix + RIGHT[WRONG.index(c[len(prefix):])]
                if new != c:
                    n += 1
                out.append(new)
                break
        else:
            out.append(c)
    return ",".join(out) + "\n", n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="*")
    ap.add_argument("--apply", action="store_true",
                    help="actually rewrite the header line in place")
    a = ap.parse_args()

    files = a.files or sorted(glob.glob(os.path.join(DATA_DIR, "*.csv")))
    if not files:
        sys.exit(f"no CSVs found in {DATA_DIR}")

    for path in files:
        with open(path, "r") as f:
            header = f.readline()
        new, n = fix_header(header)
        size_mb = os.path.getsize(path) / 1e6
        if n == 0:
            print(f"  ok       {os.path.basename(path)} ({size_mb:.1f} MB) "
                  "- already correct")
            continue
        if not a.apply:
            print(f"  WOULD FIX {os.path.basename(path)} ({size_mb:.1f} MB) "
                  f"- {n} column name(s)")
            continue
        # Rewrite in place. The new header is the same byte length as the old
        # one (same names, reordered), so a seek-and-overwrite is safe and
        # avoids copying 150 MB.
        if len(new.encode()) != len(header.encode()):
            sys.exit(f"header length changed for {path} - aborting, this "
                     "script only does an in-place same-length rewrite")
        with open(path, "r+") as f:
            f.seek(0)
            f.write(new)
        print(f"  FIXED    {os.path.basename(path)} - {n} column name(s)")

    if not a.apply:
        print("\nDry run. Re-run with --apply to rewrite the headers.")


if __name__ == "__main__":
    main()
