"cad_env"                        - virtual environment. activate it if some of the scripts are not running
"calibrate_icp.py"               - aligns a CAD model of the cone to the real robot's coordinate frame using ICP (Iterative Closest Point).
"cone_plot_normals.py"           - plot cone surface points normals with a step size "STEP"
"cone_plot.py"                   - plot cone surface points
"cone.STL"                       - STL file of the silicone cone"
"egg_experiment.py"              - egg pressing part. OLD APPROACH. CAN IGNORE
"extract_points.py"              - extract 1000 points (can be tuned) from the "cone.STL" file
"generate_random_upper_poses.py" - generates random upper poses for testing
"generate_touch_poses.py"        - get touch poses from the "surface_points_base.csv" file
"go_home.py"                     - get back to home pose
"home_start.py"                  - move robot from home to start pose: home pose -> pre pose -> start pose
"icp_transformation_matrix.txt"  - used in "validate_calibration.py" to identify whether "calibrate_icp.py" went good
"motion_data.csv"                - file with cartesian coordinates for robot to follow the spiral path (old approach)
"physical_points.csv"            - contains physical points gotten from the "record_icp_points.py"
"pose_utils.py"                  - a geometry helper module that handles the math between TCP poses and physical contact points.
"run_random_upper_poses.py"      - runs rundom upper poses generated from "generate_random_upper_poses.py"
"spiral_poinits.csv"             - file with spiral motion moves
"start_pose.py"                  - get to start_pose (right above the egg)
"stop_robot.py"                  - stop the robot through pc if needed
"surface_points_base.csv"        - surface points of the cone transformed to the UR5 base frame
"surface_points.csv"             - extracted points from the "cone.STL"
"test_aza.py"                    - main script containing the whole experiment. IN PROGRESS
"touch_poses"                    - generated touch poses
"transform_points_to_base.py"    - tranform points into the UR5 base frame
