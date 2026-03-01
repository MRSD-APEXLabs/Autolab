Notes by Karthik:


Source files:
Planning code: Autolab/robot/ros_ws/src/autonomy/global_planner/src
Robot urdf: Autolab/robot/ros_ws/src/base_urdf
Moveit_Config files: Autolab/robot/ros_ws/src/base_moveit_config_2

Motion planning using RViz (ROS2 Humble + Moveit)
Steps to run the code
1. cd Autolab - go into the folder
2. autolab up robot - just fire up the robot docker in case you want to run only motion planning in RViz and no isaac sim. In case you want to run IsaacSim as well, then just run autolab up
3. Go into the docker - autolab connect robot. It should go into the ros_ws folder inside the docker.
4. build the packages (ignore 2 packages that give errors but are irrelevant for now) - colcon build --packages-ignore robot_bringup rviz_behavior_tree_panel
5. source install/setup.bash - ignore the robot_bringup warning
6. ros2 launch global_planner move_and_rviz.launch.py - This will open RViz, load the robot and execute the motion planning algorithms for a hardcoded pose and environment.

Functions of the code

    1. add_collision_table: Injects a static 3D box into the MoveIt scene to represent the work surface and prevent base collisions.
    2. saveTrajectoryToFile: Exports time-stamped joint positions to a .txt file for external logging or simulation replay.
    3. make_ee_down_constraint:Defines an orientation constraint that forces the end-effector to point vertically downward during motion.
    4. make_path_marker:	Uses Forward Kinematics (FK) to convert joint-space trajectories into 3D line markers for RViz visualization.
    5. plan_and_publish_rrt:	Stage 1 (Global): Uses OMPL (RRT*) to find a collision-free geometric path to a Cartesian target.
    6. plan_and_publish_chomp:	Stage 2 (Local): Smooths the RRT* path using gradient descent to minimize jerky joint movements.
    7. add_hollow_point_cloud_obstacle: Procedurally generates a 5-sided "safety box" made of spheres if live camera data is unavailable.


Flow of the code

    Initialization: Sets up MoveGroupInterface for the arm and gripper, and launches a background thread for the ROS 2 executor.

    Static Environment Setup: Adds a fixed collision table to the MoveIt Planning Scene to prevent base/floor collisions.

    Perception Sync: Waits up to 5 seconds for a live /perception/obstacle_pointcloud.

    Safety Fallback: If no live cloud arrives, it procedurally generates a Hollow 5-sided Box (Point Cloud Obstacle) to define a safe workspace.

    Target Acquisition: Retrieves the goal pose from the /perception/target_pose topic or falls back to hard-coded defaults.

    Stage 1 - Global Planning (RRT):*

        Validates target reachability via Inverse Kinematics (IK).

        Generates a collision-free geometric path while maintaining a Z-down orientation constraint.

        Executes the arm motion and publishes a Green Path marker to RViz.

        Synchronization: Pauses for 500ms post-execution to allow the Planning Scene to update the robot's current state.

    Stage 2 - Local Optimization (CHOMP) (only runs for Move, not for grasp):

        Reuses the RRT* end-state as a joint-space goal.

        Optimizes the trajectory for smoothness, minimizing jerky movements.

        Publishes a Blue Path marker to RViz.

    Task Finalization:

        If "Move": Completes the smoothed trajectory.

        If "Grasp": Actuates the gripper sliders to the live_grasp_width using gripper_group.move().

        Data Logging: Saves the final time-stamped joint trajectories to .txt files for analysis
