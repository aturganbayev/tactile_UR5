import trimesh
import pandas as pd

mesh = trimesh.load("cone.STL")

points, face_indices = trimesh.sample.sample_surface(mesh, 1000)

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

df.to_csv("surface_points.csv", index=False)


# print(mesh)

