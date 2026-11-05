# Proposed ROS hardware ownership fix — not applied

`ros_hardware_command_ownership.patch` changes two files in
`5_planning/xarm_ros2/xarm_controller`, outside the currently authorized edit folders.
Only this patch artifact has been created; the driver sources are unchanged.
`git apply --check` passes. This proposal has not been compiled or tested on hardware.

The current driver's `write()` sends `set_servo_angle_j` whenever joint targets change,
and at least once per second even when unchanged. It does not check whether any
controller owns the arm command interfaces. SDK yaw/ACT also uses servo mode (1), so
the driver regards the arm as ready while both clients send incompatible targets.
The driver's `_need_reset()` path can also reactivate stopped controllers.

The patch tracks command-interface ownership through `perform_command_mode_switch()`.
With no claimed arm interfaces, `write()` sends nothing, suppresses automatic controller
reactivation, and allows `read()` to keep commands synchronized to measured joints.
The intended follow-up is an explicit controller-manager handoff in `act_inference`:
stop and verify arm controllers before SDK motion, monitor ownership, and restore planning
only through a tested transition that starts from current measured joints. Failed runs
must not automatically clear faults or re-enable the arm.

The current `act_inference` guard deliberately blocks execution with active OR inactive
unpatched ROS hardware. Do not remove that guard merely after applying this patch: the
controller handoff and its tests are also needed before motion can resume.

Humble calls the hardware write method in both active and inactive lifecycle states:
[upstream System::write](https://github.com/ros-controls/ros2_control/blob/humble/hardware_interface/src/system.cpp).
Stopping a trajectory controller alone cannot silence the unpatched custom hardware.

Read-only investigation on 2026-11-02 found C31, state 4, mode 0. The report pose and
direct axis-angle query agreed to 0.001 mm and 0.001 degrees; TCP offset was
`[0, 0, 140, 0, 0, 0]` mm/degrees. No enable, motion, fault-clear, or controller-switch
requests were sent. ROS hardware-state queries could not be completed, so the exact
controller lifecycle state at the incident is not established by a service response.
Both a standalone and a bringup `run_act_node.py` process were present in ROS domain 0.
This proves neither was handling that particular service call, but duplicate nodes must
be removed to make service routing deterministic.
