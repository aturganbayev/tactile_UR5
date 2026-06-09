import numpy as np
import pandas as pd

df = pd.read_csv("surface_points.csv")

# Convert STL points from millimeters to meters
df["x"] = df["x"] / 1000.0
df["y"] = df["y"] / 1000.0
df["z"] = df["z"] / 1000.0

real_top = np.array([0.00250, -0.51353, 0.13350])

idx = df["z"].idxmax()
generated_top = df.loc[idx, ["x", "y", "z"]].to_numpy(dtype=float)

offset = real_top - generated_top

print("Generated top (m):", generated_top)
print("Real top (m):", real_top)
print("Offset (m):", offset)

df["x"] += offset[0]
df["y"] += offset[1]
df["z"] += offset[2]

df.to_csv("surface_points_base.csv", index=False)

print("Saved surface_points_base.csv")
print(df.loc[df["z"].idxmax()])