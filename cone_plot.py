import pandas as pd
import matplotlib.pyplot as plt

# Read CSV
df = pd.read_csv("surface_points.csv")

x = df["x"]
y = df["y"]
z = df["z"]

fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

ax.scatter(x, y, z, s=2)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")

# Equal axis scaling
max_range = max(
    x.max()-x.min(),
    y.max()-y.min(),
    z.max()-z.min()
) / 2

mid_x = (x.max()+x.min())/2
mid_y = (y.max()+y.min())/2
mid_z = (z.max()+z.min())/2 

ax.set_xlim(mid_x-max_range, mid_x+max_range)
ax.set_ylim(mid_y-max_range, mid_y+max_range)
ax.set_zlim(mid_z-max_range, mid_z+max_range)

fig.savefig("surface__points_cone_plot.png", dpi=300, bbox_inches="tight")
plt.show()