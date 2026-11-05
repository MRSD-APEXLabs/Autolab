# Mode 1 driver/SDK handoff

The fix is implemented directly in `5_planning/xarm_ros2/xarm_controller` and
`act_inference`. The earlier proposed patch file has been removed.

Before this change, the ROS hardware driver's `write()` resent its last joint target
at least once per second, even with no controller claiming an arm command interface.
ACT also uses mode 1, so the driver considered the robot ready during SDK motion.

The driver now tracks position/velocity command-interface claims atomically. It sends
no motion command, and does not auto-reactivate controllers, when the arm is unclaimed.
An explicit SDK lease additionally prevents arm-controller activation during SDK work.
Readout stays active; after handback, joint command interfaces are synchronized with a
fresh measured joint sample before controller activation.

All services below use `std_srvs/Trigger`, under `/xarm/sdk_control` by default:

| Service | Effect |
|---|---|
| `status` | Read-only JSON: protocol, robot_ip, sdk_owned, claimed bitmask, resync_pending |
| `acquire` | Succeeds only when no arm command interface is claimed and no owner/resync exists |
| `release` | Requires fault-free, ready mode 1; blocks ROS writes until joint resync finishes |

ACT verifies the driver protocol and robot IP, records which configured arm controllers were active, stops
those controllers with a strict controller-manager switch, and obtains acknowledged
SDK ownership **before any yaw, base, or policy motion**. On success it finishes the
release/lift, prepares mode 1/state 0 without issuing a pose or clearing errors, releases
the SDK lease, waits for measured-joint resync, then restores only previously active
controllers. It reports `done` after this handback completes.

On cancellation, exceptions, or arm faults, no handback/ROS activation is attempted.
The driver lease remains held (`sdk_control_held: true` in ACT status) to prevent stale
ROS targets from being sent. The same ACT node can reuse its lease on a later explicitly
started run. If that node exits while holding control, restarting it does **not** silently
steal/release the old lease. Inspect the arm and recover through the robot's normal fault
recovery/restart procedure. These new services never clear faults or enable motion.

Node parameters: `driver_control_prefix` (`/xarm/sdk_control`), `arm_controllers`
(`[xarm6_traj_controller]`). Another claimed arm controller makes driver acquisition fail
rather than allowing concurrent commands; include all relevant arm controllers in that
parameter. Prediction-only mode never changes ownership or controller state. Standalone
`run_act_inference.py --execute` is blocked to prevent bypassing the handoff; use
the ROS node and target-specific start services for execution.

## Build and load

Stop the running bringup/controller-manager and any standalone ACT node before replacing
the live driver library. In the robot container:

```bash
cd /home/robot/AutoLab/robot/ros_ws
colcon build --packages-select xarm_controller
source install/setup.bash
```

Restart the normal bringup and run only one ACT node. Source edits alone do not replace
the already-loaded C++ driver. An old driver has no ownership services, so the updated
ACT node will refuse execution instead of falling back to unsafe concurrent control.

The driver was compiled in an isolated Humble build without loading the plugin into the
running robot. A C++ test covers acquisition/claim races and resynchronization gating;
Python tests cover service ordering, complete-run ownership, dry runs, and failure paths.
No physical motion or fault clearing was performed during validation.
