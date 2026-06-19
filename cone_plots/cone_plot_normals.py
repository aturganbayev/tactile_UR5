import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths

df = pd.read_csv(paths.SURFACE_POINTS)

top_idx = df["z"].idxmax()
center_x = df.loc[top_idx, "x"]
center_y = df.loc[top_idx, "y"]

# Flip any inward-pointing normals to point outward from the cone axis
for idx, row in df.iterrows():
    n = np.array([row["nx"], row["ny"], row["nz"]])
    v_out = np.array([row["x"] - center_x, row["y"] - center_y, 0.0])
    if np.linalg.norm(v_out) > 1e-5:
        v_out = v_out / np.linalg.norm(v_out)
        if np.dot(n[:2], v_out[:2]) < 0:
            df.loc[idx, ["nx", "ny", "nz"]] = -n

fig = plt.figure(figsize=(10, 10))
ax = fig.add_subplot(111, projection='3d')

ax.scatter(df.x, df.y, df.z, s=1)

step = 20

ax.quiver(
    df.x[::step],
    df.y[::step],
    df.z[::step],
    df.nx[::step],
    df.ny[::step],
    df.nz[::step],
    length=2,
    normalize=True
)

os.makedirs(paths.FIGURES, exist_ok=True)
fig.savefig(paths.SURFACE_NORMALS_PLOT, dpi=300, bbox_inches="tight")
plt.show()
