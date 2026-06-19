import os
import sys

import trimesh
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import paths

mesh = trimesh.load(paths.CONE_STL)

points, face_indices = trimesh.sample.sample_surface(mesh, 3000)

normals = mesh.face_normals[face_indices]

data = []

for p, n in zip(points, normals):
    data.append([
        p[0], p[1], p[2],
        n[0], n[1], n[2]
    ])

df = pd.DataFrame(
    data,
    columns=[
        "x","y","z",
        "nx","ny","nz"
    ]
)

df.to_csv(paths.SURFACE_POINTS, index=False)


# print(mesh)

