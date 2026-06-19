"""Central path configuration for the tactile_UR5 project.

Single source of truth for where data files live, so scripts work regardless of
the current working directory and reorganizing files only means editing here.

Scripts in subfolders import this after adding the repo root to sys.path:

    import os, sys
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import paths
"""

import os

ROOT = os.path.dirname(os.path.abspath(__file__))

# Top-level directories
DATA = os.path.join(ROOT, "data")
FIGURES = os.path.join(ROOT, "figures")
CALIBRATION = os.path.join(ROOT, "ur_calibration")

# Geometry / generated data (in data/)
CONE_STL = os.path.join(DATA, "cone.STL")
SURFACE_POINTS = os.path.join(DATA, "surface_points.csv")
TOUCH_POSES = os.path.join(DATA, "touch_poses.csv")
RANDOM_UPPER_TOUCH_POSES = os.path.join(DATA, "random_upper_touch_poses.csv")
CONE_TOUCH_POSES = os.path.join(DATA, "cone_touch_poses.csv")

# Calibration artifacts (in ur_calibration/)
PHYSICAL_POINTS = os.path.join(CALIBRATION, "physical_points.csv")
SURFACE_POINTS_BASE = os.path.join(CALIBRATION, "surface_points_base.csv")
ICP_MATRIX = os.path.join(CALIBRATION, "icp_transformation_matrix.txt")

# Figure outputs
SURFACE_POINTS_PLOT = os.path.join(FIGURES, "surface__points_cone_plot.png")
SURFACE_NORMALS_PLOT = os.path.join(FIGURES, "surface__points_cone_normals_plot.png")
RANDOM_UPPER_PLOT = os.path.join(FIGURES, "random_upper_points_plot.png")
SIDE_STRIP_PLOT = os.path.join(FIGURES, "cone_touch_poses.png")
