import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("surface_points.csv")

fig = plt.figure(figsize=(10,10))
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

plt.show()