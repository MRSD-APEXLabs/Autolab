Instructions for testing navigation integration:

1. Build just this package
colcon build --packages-select swerve_hardware_interface
source install/setup.bash

2. Wire one physical module (say FL: drive ID 10, steer ID 32... wait, steer 30, CANcoder 20) to the CANivore. Leave the other 3 unplugged — that's fine, configure() runs per-module independently; only the wired one's
Apply() calls will succeed. Everything is logged, so you'll see which module reports config failure and can ignore that.

3. Run the node directly, bypassing every launch file:
ros2 run swerve_hardware_interface swerve_hardware_interface_node \
--ros-args --params-file $(ros2 pkg prefix swerve_hardware_interface)/share/swerve_hardware_interface/config/swerve_modules.yaml
Watch startup log — confirms configure() per module and whether FeedEnable timer started (it won't if ANY module fails config — see the safety gate added in review). For single-module bench testing you may want to
temporarily comment out the other 3 default_module_configs() entries so all_configured can be true with just one module wired — otherwise FeedEnable never starts and nothing will move. (Cleanest: add a --ros-args -p
test_module_only:="FL" filter later if you do this often; for a one-off bench test, just edit default_module_configs() locally, don't commit it.)

4. Command it directly, no navigation_controller involved:
# spin the wheel at 1 wheel-rot/s
ros2 topic pub --rate 10 /robot_1/joints/joint_command sensor_msgs/msg/JointState \
"{name: ['Revolute_2'], velocity: [1.0], position: [0.0], effort: [0.0]}"

# steer to 90 degrees (pi/2 rad)
ros2 topic pub --rate 10 /robot_1/joints/joint_command sensor_msgs/msg/JointState \
"{name: ['Revolute_6'], position: [1.5708], velocity: [0.0], effort: [0.0]}"
(joint names for FL from the mapping table: drive=Revolute_2, azimuth=Revolute_6 — swap per the table in the plan/spec for whichever module you've wired.)

5. Watch feedback:
ros2 topic echo /robot_1/joints/joint_states

This is also your chance to resolve the parked velocity-units question — command a known low value (e.g. velocity: [1.0]) and physically count wheel rotations over a few seconds. Confirms whether
navigation_controller's output actually means 1.0 = 1 rot/s or 2 rot/s before you ever run the full stack.

Once one module checks out, wire the next and repeat — no need to touch navigation_bringup/autonomy_bringup at all until all 4 are individually verified.