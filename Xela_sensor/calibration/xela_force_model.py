#!/usr/bin/env python3
"""
Apply a force model fitted by fit_force_model.py: raw XELA counts -> Newtons.

Import this from the palpation recorder (or any analysis script) to convert
logged raw counts after the fact - no need to re-run anything on the robot,
since the palpation logs already store all 48 raw channels.

    from xela_force_model import ForceModel
    m = ForceModel.load("20260728_180000")      # or a full .npz path
    fxyz = m.predict(counts48, baseline48)      # -> np.array([Fx, Fy, Fz])

BOUNDS OF VALIDITY - the model is a fit, not a measurement:
  * Only trustworthy inside the force range it was trained on (`fmax_N`) and
    on the pad region that was pressed. Extrapolation past those is guesswork.
  * `baseline` must be a RECENT non-contact reading, not a session-start one.
    The sensor drifts ~1000 counts over minutes, comparable to the signal.
  * It was fitted with a rigid indenter. A compliant contact (an egg, a
    fingertip) spreads load differently, so treat the Newtons as a calibrated
    relative scale rather than a traceable absolute.
"""

import os

import numpy as np

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.join(_THIS_DIR, "data")

N_TAXELS = 16
CHANNELS = [f"{ax}{i}" for i in range(N_TAXELS) for ax in ("x", "y", "z")]


def _phi(d, kind):
    d = np.atleast_2d(np.asarray(d, dtype=float))
    if kind == "linear":
        return d
    if kind == "sqrt":
        return np.hstack([d, np.sign(d) * np.sqrt(np.abs(d))])
    if kind == "quad":
        return np.hstack([d, d ** 2])
    raise ValueError(f"unknown feature kind: {kind}")


class ForceModel:
    def __init__(self, W, b, features, meta=None):
        self.W = W
        self.b = b
        self.features = str(features)
        self.meta = meta or {}

    @classmethod
    def load(cls, label_or_path):
        path = label_or_path
        if not path.endswith(".npz"):
            path = os.path.join(DATA_DIR, f"{label_or_path}_force_model.npz")
        z = np.load(path, allow_pickle=False)
        meta = {k: z[k] for k in ("alpha", "heldout_rms_N", "n_presses",
                                  "fmax_N") if k in z.files}
        return cls(z["W"], z["b"], str(z["features"]), meta)

    def predict(self, counts, baseline):
        """counts and baseline are 48-vectors (or (n, 48) arrays) ordered
        x0,y0,z0, x1,y1,z1, ... - the same order as the logger's columns."""
        d = np.atleast_2d(np.asarray(counts, float)) - \
            np.atleast_2d(np.asarray(baseline, float))
        out = _phi(d, self.features) @ self.W + self.b
        return out[0] if out.shape[0] == 1 else out

    def magnitude(self, counts, baseline):
        p = np.atleast_2d(self.predict(counts, baseline))
        m = np.linalg.norm(p, axis=1)
        return float(m[0]) if m.size == 1 else m

    def __repr__(self):
        parts = [f"features={self.features}"]
        if "heldout_rms_N" in self.meta:
            parts.append(f"heldout|F|RMS={float(self.meta['heldout_rms_N']):.3f}N")
        if "fmax_N" in self.meta:
            parts.append(f"trained to {float(self.meta['fmax_N']):.1f}N")
        if "n_presses" in self.meta:
            parts.append(f"{int(self.meta['n_presses'])} presses")
        return "<ForceModel " + ", ".join(parts) + ">"


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        sys.exit("usage: python3 xela_force_model.py <label|path>")
    m = ForceModel.load(sys.argv[1])
    print(m)
    print(f"W shape {m.W.shape}, b {np.round(m.b, 4)}")
