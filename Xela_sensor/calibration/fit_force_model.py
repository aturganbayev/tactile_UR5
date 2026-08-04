#!/usr/bin/env python3
"""
Fit the global XELA force map from a nano17_sweep_session.py run.

Runs anywhere both CSVs are present (normally the workstation, after copying
the DAQ PC's *_sweep.csv next to the *_xela.csv):

    Xela_sensor/calibration/data/<label>_sweep.csv   (Nano17 + robot pose)
    Xela_sensor/calibration/data/<label>_xela.csv    (48 raw channels)

THE MODEL
    [Fx, Fy, Fz] = W . phi(delta_counts) + b

`delta_counts` is the 48-vector of raw counts referenced to THAT PRESS's own
pre-contact baseline. That per-press referencing is what makes the sensor's
~1000-count drift harmless: the baseline only has to hold for the ~15 s of one
press, not for the whole session.

Unlike the abandoned per-taxel calibration, nothing here assumes load is
confined to one taxel. Cross-talk is an input: when the load spreads to
neighbours, those neighbours' counts are features the regression uses. That is
why this can work on hardware where per-taxel calibration cannot.

WHAT IT REPORTS
Held-out error only, from a GROUPED split - whole presses are held out, never
individual samples. A random per-sample split would be self-deceiving here:
consecutive samples within one press are nearly identical, so training and
test would share almost the same rows and the error would look far better than
it is. A naive `sum|dZ|` single-scalar model is fitted alongside as a baseline,
so you can see whether the 48-channel model actually earns its complexity.

Usage:
    python3 fit_force_model.py <label> [--features sqrt] [--alpha auto]
                               [--fit-on all|load] [--no-save]
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(_THIS_DIR, "data")

N_TAXELS = 16
CHANNELS = [f"{ax}{i}" for i in range(N_TAXELS) for ax in ("x", "y", "z")]

# Cross-correlation search window for the clock offset between the two PCs.
MAX_CLOCK_OFFSET_S = 5.0
XCORR_HZ = 20.0


# --------------------------------------------------------------------------- #
#                                  LOADING                                    #
# --------------------------------------------------------------------------- #

def load_logs(label):
    sweep_path = os.path.join(DATA_DIR, f"{label}_sweep.csv")
    xela_path = os.path.join(DATA_DIR, f"{label}_xela.csv")
    for p in (sweep_path, xela_path):
        if not os.path.exists(p):
            sys.exit(f"missing {p}")
    sweep = pd.read_csv(sweep_path).sort_values("t").reset_index(drop=True)
    xela = pd.read_csv(xela_path).sort_values("t").reset_index(drop=True)
    print(f"sweep: {len(sweep)} rows, {sweep['press'].nunique()} presses, "
          f"{sweep['t'].iloc[-1] - sweep['t'].iloc[0]:.1f}s")
    print(f"xela : {len(xela)} rows, "
          f"{xela['t'].iloc[-1] - xela['t'].iloc[0]:.1f}s")
    return sweep, xela


def _resample(t, y, grid):
    return np.interp(grid, t, y)


def _detrend(y, hz=XCORR_HZ, window_s=30.0):
    """Remove slow drift with a rolling median.

    Essential for the XELA proxy: the sensor's baseline wanders ~1000 counts
    over minutes, which is the same order as the press response. Left in, that
    drift dominates the correlation and makes even a perfect alignment score
    ~0.1, so the confidence check would cry wolf on every good run.
    """
    w = max(3, int(window_s * hz) | 1)
    s = pd.Series(y).rolling(w, center=True, min_periods=1).median()
    return y - s.to_numpy()


def estimate_clock_offset(sweep, xela):
    """Seconds to ADD to the XELA timestamps to line them up with the sweep.

    The two logs come from different PCs, so their clocks can differ by far
    more than the alignment tolerance even when both run NTP. Rather than
    trusting them, correlate a crude XELA response magnitude against Fmag -
    both spike at the same physical instant, so the lag that maximises their
    correlation is the offset.
    """
    xz = xela[[f"z{i}" for i in range(N_TAXELS)]].to_numpy(float)
    resp = np.abs(xz - np.median(xz, axis=0)).sum(axis=1)

    t0 = max(sweep["t"].iloc[0], xela["t"].iloc[0]) - MAX_CLOCK_OFFSET_S
    t1 = min(sweep["t"].iloc[-1], xela["t"].iloc[-1]) + MAX_CLOCK_OFFSET_S
    grid = np.arange(t0, t1, 1.0 / XCORR_HZ)
    f = _detrend(_resample(sweep["t"].to_numpy(), sweep["Fmag"].to_numpy(), grid))
    f = f - f.mean()

    best, best_r = 0.0, -np.inf
    max_lag = int(MAX_CLOCK_OFFSET_S * XCORR_HZ)
    for lag in range(-max_lag, max_lag + 1):
        shift = lag / XCORR_HZ
        r_i = _detrend(_resample(xela["t"].to_numpy() + shift, resp, grid))
        r_i = r_i - r_i.mean()
        denom = np.linalg.norm(f) * np.linalg.norm(r_i)
        if denom == 0:
            continue
        r = float(f @ r_i / denom)
        if r > best_r:
            best_r, best = r, shift
    print(f"clock offset: {best:+.3f} s applied to XELA "
          f"(peak correlation {best_r:.3f})")
    if best_r < 0.3:
        print("  WARNING: weak correlation - the two logs may not overlap, or "
              "the XELA logger missed the presses. Check before trusting the "
              "fit.")
    return best


def merge(sweep, xela, offset_s):
    """XELA channels interpolated onto the sweep timestamps."""
    xt = xela["t"].to_numpy(float) + offset_s
    out = sweep.copy()
    for c in CHANNELS:
        out[c] = np.interp(out["t"].to_numpy(float), xt,
                           xela[c].to_numpy(float))
    # Drop sweep samples outside the XELA log's span - np.interp would clamp
    # them to the endpoint value, silently inventing data.
    keep = (out["t"] >= xt[0]) & (out["t"] <= xt[-1])
    if (~keep).any():
        print(f"dropped {int((~keep).sum())} sweep row(s) outside the XELA log")
    return out[keep].reset_index(drop=True)


def add_deltas(df):
    """Reference each press's counts to its own pre-contact baseline."""
    out = []
    for press, g in df.groupby("press", sort=True):
        base_rows = g[g["phase"] == "baseline"]
        if len(base_rows) < 3:
            # Fall back to the quietest pre-load samples of this press.
            base_rows = g[g["Fmag"] < 0.05].head(30)
        if len(base_rows) < 3:
            print(f"  press {press}: no usable baseline window - skipped")
            continue
        base = base_rows[CHANNELS].median()
        g = g.copy()
        for c in CHANNELS:
            g["d_" + c] = g[c] - base[c]
        out.append(g)
    if not out:
        sys.exit("no press had a usable baseline window")
    return pd.concat(out, ignore_index=True)


# --------------------------------------------------------------------------- #
#                                 FEATURES                                    #
# --------------------------------------------------------------------------- #

def build_features(df, kind):
    d = df[["d_" + c for c in CHANNELS]].to_numpy(float)
    if kind == "scalar":
        # Total response only - no per-channel weights, no location.
        #
        # This is the model to deploy, and that is an empirical result, not a
        # simplification for its own sake. Measured on test1 + rep1:
        #   * over the whole pad, response at a fixed 2 N varies 7.8x with
        #     location, so any location-invariant model is capped around
        #     0.66 N RMS - and the learning curve from 6 to 21 presses is
        #     FLAT, so more presses do not lift that cap;
        #   * handing the model location explicitly (contact centroid, and
        #     centroid-modulated gain) did not beat plain total response;
        #   * but at ONE location, this same scalar reaches 0.148 N RMS
        #     (R2 0.973), 4.5x better.
        # The gain field is real and repeatable (CV ~6% at a fixed spot vs
        # 46% across the pad) - it just is not learnable from the sparse
        # pattern a small rigid indenter produces. So calibrate for the
        # contact geometry you will actually use rather than trying to be
        # location-invariant.
        tot = np.abs(d).sum(axis=1, keepdims=True)
        return np.hstack([tot, np.sqrt(tot)]), ["sum_abs_d", "sqrt_sum_abs_d"]
    if kind == "linear":
        return d, [f"d_{c}" for c in CHANNELS]
    if kind == "sqrt":
        # Hertzian contact makes force grow faster than displacement, so a
        # signed-sqrt companion term lets the fit bend without going quadratic
        # and doubling the variance.
        s = np.sign(d) * np.sqrt(np.abs(d))
        return (np.hstack([d, s]),
                [f"d_{c}" for c in CHANNELS] + [f"sq_{c}" for c in CHANNELS])
    if kind == "quad":
        return (np.hstack([d, d ** 2]),
                [f"d_{c}" for c in CHANNELS] + [f"sqr_{c}" for c in CHANNELS])
    raise ValueError(kind)


def ridge_fit(X, Y, alpha):
    """Standardised ridge, returned in original units as (W, b)."""
    mu, sd = X.mean(0), X.std(0)
    sd[sd < 1e-9] = 1.0
    Xs = (X - mu) / sd
    ym = Y.mean(0)
    A = Xs.T @ Xs + alpha * np.eye(Xs.shape[1])
    Wc = np.linalg.solve(A, Xs.T @ (Y - ym))       # (n_feat, 3)
    W = Wc / sd[:, None]
    b = ym - mu @ W
    return W, b


def grouped_cv(X, Y, groups, alpha, n_folds=5):
    """Predictions for every sample, each made by a model that never saw that
    sample's press."""
    uniq = np.unique(groups)
    rng = np.random.default_rng(0)
    rng.shuffle(uniq)
    folds = np.array_split(uniq, min(n_folds, len(uniq)))
    pred = np.zeros_like(Y)
    for test_g in folds:
        te = np.isin(groups, test_g)
        W, b = ridge_fit(X[~te], Y[~te], alpha)
        pred[te] = X[te] @ W + b
    return pred


def report(name, Y, P):
    err = P - Y
    rms = np.sqrt((err ** 2).mean(0))
    mag_t = np.linalg.norm(Y, axis=1)
    mag_p = np.linalg.norm(P, axis=1)
    mag_rms = float(np.sqrt(((mag_p - mag_t) ** 2).mean()))
    ss = ((Y - Y.mean(0)) ** 2).sum()
    r2 = 1.0 - (err ** 2).sum() / ss if ss > 0 else float("nan")
    print(f"  {name:<22} Fx {rms[0]:.3f}  Fy {rms[1]:.3f}  Fz {rms[2]:.3f}  "
          f"|F| {mag_rms:.3f} N   R2 {r2:.3f}")
    return mag_rms


# --------------------------------------------------------------------------- #
#                                    MAIN                                     #
# --------------------------------------------------------------------------- #

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("label")
    ap.add_argument("--features", default="scalar",
                    choices=["scalar", "linear", "sqrt", "quad"],
                    help="'scalar' (default) is the one that actually wins - "
                         "see build_features() for the measurements")
    ap.add_argument("--alpha", default="auto",
                    help="ridge strength, or 'auto' to sweep")
    ap.add_argument("--fit-on", default="all", choices=["all", "load"],
                    help="'load' excludes the unloading branch (hysteresis)")
    ap.add_argument("--drop-first-press", action="store_true",
                    help="discard press 0: on rested silicone the first press "
                         "reads ~30%% high (Mullins effect), measured in rep1")
    ap.add_argument("--no-save", action="store_true")
    a = ap.parse_args()

    sweep, xela = load_logs(a.label)
    merged = merge(sweep, xela, estimate_clock_offset(sweep, xela))
    df = add_deltas(merged)

    if a.fit_on == "load":
        df = df[df["phase"].isin(["load", "dwell", "shear"])]
    # Free-space samples carry no information about force and would let the
    # model score well by predicting ~0 most of the time.
    df = df[df["Fmag"] >= 0.05]
    if a.drop_first_press:
        first = df["press"].min()
        df = df[df["press"] != first]
        print(f"dropped press {first} (unconditioned - reads ~30% high)")
    df = df.reset_index(drop=True)
    print(f"\ntraining rows: {len(df)} over {df['press'].nunique()} presses, "
          f"|F| range {df['Fmag'].min():.2f}-{df['Fmag'].max():.2f} N")
    if df["press"].nunique() < 5:
        sys.exit("need at least 5 presses to validate honestly")

    X, feat_names = build_features(df, a.features)
    Y = df[["Fx", "Fy", "Fz"]].to_numpy(float)
    groups = df["press"].to_numpy()

    if a.alpha == "auto":
        best, best_err = None, np.inf
        # Extends to 1e7: on the test1 data the selection pinned at the old
        # 1e4 ceiling for every feature set, meaning the sweep was truncated
        # and the reported alpha was an artefact of the grid, not an optimum.
        for alpha in [1e-2, 1e-1, 1, 10, 100, 1000, 1e4, 1e5, 1e6, 1e7]:
            e = np.sqrt((((grouped_cv(X, Y, groups, alpha)) - Y) ** 2).mean())
            if e < best_err:
                best, best_err = alpha, e
        alpha = best
        print(f"selected ridge alpha = {alpha:g}")
    else:
        alpha = float(a.alpha)

    print("\nheld-out error (whole presses held out):")
    P = grouped_cv(X, Y, groups, alpha)
    label = ("scalar (1 feat)" if a.features == "scalar"
             else f"{a.features} (48ch)")
    full_rms = report(label, Y, P)

    # Baseline: the single-scalar model the palpation recorder currently
    # implies (response = sum|dZ|). If the 48-channel model is not clearly
    # better than this, it is not earning its complexity.
    dz = df[[f"d_z{i}" for i in range(N_TAXELS)]].to_numpy(float)
    Xn = np.abs(dz).sum(1, keepdims=True)
    base_rms = report("sum|dZ| baseline", Y, grouped_cv(Xn, Y, groups, 1e-6))
    if full_rms < base_rms:
        print(f"  -> {label} is {base_rms / full_rms:.1f}x better on |F|")
    else:
        print("  -> WARNING: no better than the scalar baseline. Suspect "
              "time misalignment or too few presses.")

    if not a.no_save:
        W, b = ridge_fit(X, Y, alpha)
        out = os.path.join(DATA_DIR, f"{a.label}_force_model.npz")
        np.savez(out, W=W, b=b, features=a.features, alpha=alpha,
                 channels=np.array(CHANNELS), feat_names=np.array(feat_names),
                 heldout_rms_N=full_rms, n_presses=df["press"].nunique(),
                 fmax_N=float(df["Fmag"].max()))
        print(f"\nsaved -> {out}")
        print("Apply it with xela_force_model.py (raw counts -> N).")


if __name__ == "__main__":
    main()
