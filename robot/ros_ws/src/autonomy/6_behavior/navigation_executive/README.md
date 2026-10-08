# navigation_executive

Behavior tree executive bridging the BT / `routine_executor` to the `1_navigation` stack's
`nav_api` topic contract (see `autonomy/1_navigation/README.md`). Owns two BT condition/action
pairs:

| Condition | Action | Command topic (relative, under `behavior/`) | Command msg |
|---|---|---|---|
| `Go To Location Commanded` | `Go To Location` | `go_to_location_command` | `std_msgs/String` (saved place name) |
| `Navigate To Pose Commanded` | `Navigate To Pose` | `navigate_to_pose_command` | `geometry_msgs/PoseStamped` |

Activation publishes the goal to `nav_api` (`/nav/goal_location` or `/nav/goal_pose`, both
absolute — `nav_api` is not namespaced under `behavior/`) and watches `/nav/state` +
`/nav/result` to derive RUNNING / SUCCESS / FAILURE. See
`docs/superpowers/specs/2026-09-22-nav-bt-integration-design.md` for the full design.

No automated test harness exists for this package (matches the rest of `6_behavior` — no gtest
in `lab_machine_executive`/`behavior_executive` either). Verification is build success + manual
`ros2 topic pub`/`echo` smoke testing, below.

## Build

Inside the robot container:

```bash
colcon build --packages-select navigation_executive --symlink-install
```

Expected: builds cleanly, no warnings about undeclared dependencies.

## Test plan

### 1. Command handling + mutual exclusion (no BT engine, no nav stack needed)

```bash
source install/setup.bash
ros2 run navigation_executive navigation_executive &

# in another shell:
ros2 topic echo /go_to_location_commanded_success &
ros2 topic echo /navigate_to_pose_commanded_success &

ros2 topic pub -1 /go_to_location_command std_msgs/msg/String "{data: ot2}"
```

Expected: `/go_to_location_commanded_success` → `data: true`, `/navigate_to_pose_commanded_success` → `data: false`.

```bash
ros2 topic pub -1 /navigate_to_pose_command geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}}}"
```

Expected: `/navigate_to_pose_commanded_success` → `data: true`, `/go_to_location_commanded_success` → `data: false` (mutual exclusion holds both ways).

### 2. Full dispatch / state machine

Simulate the BT engine ticking the action active with `ros2 topic pub` on the `_active` topic
directly — this exercises the node completely on its own, without the real BT engine or the
real `1_navigation` stack running.

```bash
source install/setup.bash
ros2 run navigation_executive navigation_executive --ros-args -p nav_response_timeout:=2.0 &

ros2 topic echo /go_to_location_status &
```

**Happy path:**
```bash
ros2 topic pub -1 /go_to_location_command std_msgs/msg/String "{data: ot2}"
ros2 topic pub -1 /go_to_location_active behavior_tree_msgs/msg/Active "{active: true, id: 1}"
# expect: /nav/goal_location was published with data 'ot2'; /go_to_location_status is RUNNING (1)
ros2 topic pub -1 /nav/state std_msgs/msg/String "{data: NAVIGATING}"
# expect: still RUNNING
ros2 topic pub -1 /nav/state std_msgs/msg/String "{data: SUCCEEDED}"
# expect: /go_to_location_status -> SUCCESS (2)
```

**Rejection (unknown location) fails fast, not after the timeout:**
```bash
ros2 topic pub -1 /go_to_location_active behavior_tree_msgs/msg/Active "{active: false, id: 1}"
ros2 topic pub -1 /go_to_location_command std_msgs/msg/String "{data: nonexistent}"
ros2 topic pub -1 /go_to_location_active behavior_tree_msgs/msg/Active "{active: true, id: 2}"
ros2 topic pub -1 /nav/result std_msgs/msg/String "{data: 'REJECTED unknown location nonexistent'}"
# expect: /go_to_location_status -> FAILURE (0) immediately, well under 2s
```

**No response at all (nav stack not running) fails after the timeout:**
```bash
ros2 topic pub -1 /go_to_location_active behavior_tree_msgs/msg/Active "{active: false, id: 2}"
ros2 topic pub -1 /go_to_location_command std_msgs/msg/String "{data: ot2}"
ros2 topic pub -1 /go_to_location_active behavior_tree_msgs/msg/Active "{active: true, id: 3}"
# publish nothing on /nav/state or /nav/result; wait >2s
# expect: /go_to_location_status -> FAILURE (0) around 2s in, with an ERROR log about no response
```

**Auto-cancel on deactivation mid-flight:**
```bash
ros2 topic echo /nav/cancel &
ros2 topic pub -1 /go_to_location_active behavior_tree_msgs/msg/Active "{active: false, id: 3}"
ros2 topic pub -1 /go_to_location_command std_msgs/msg/String "{data: ot2}"
ros2 topic pub -1 /go_to_location_active behavior_tree_msgs/msg/Active "{active: true, id: 4}"
ros2 topic pub -1 /nav/state std_msgs/msg/String "{data: NAVIGATING}"
ros2 topic pub -1 /go_to_location_active behavior_tree_msgs/msg/Active "{active: false, id: 4}"
# expect: one message appears on /nav/cancel
```

**`/go_to_location_status` will keep reading `RUNNING` (1) after this last step — that's
expected, not a bug.** `tick_channel` doesn't reset `status` on deactivation (matching the same
convention `lab_machine_executive` already uses), and it doesn't need to: the real BT engine's
`ActionNode::callback()` (`behavior_tree_implementation.cpp`) only calls `set_status()` while
`is_active` is true on its own side — once it has ticked this branch inactive, it ignores every
`Status` message from us regardless of value. So a stale `RUNNING` here is only visible to a
human running `ros2 topic echo` directly against `navigation_executive`; it has no effect on the
BT. The only real pass/fail signal for this sub-test is the single message on `/nav/cancel`.

Repeat the happy-path check for `/navigate_to_pose_command` + `/navigate_to_pose_active` +
`/navigate_to_pose_status`, publishing a `geometry_msgs/msg/PoseStamped` instead of a `String`,
to confirm the second channel behaves identically.

### 3. `drone.tree` still parses (with `behavior_tree` built)

```bash
ros2 run behavior_tree behavior_tree_implementation --ros-args \
  -p config:="$(ros2 pkg prefix behavior_tree)/share/behavior_tree/config/drone.tree" \
  -p timeout:=1.0 &
sleep 2
ros2 topic list | grep -E "go_to_location|navigate_to_pose"
kill %1
```

Expected topics present: `/go_to_location_active`, `/go_to_location_status`,
`/go_to_location_commanded_success`, and the equivalent `navigate_to_pose_*` ones. No
crash/exception in the node's stdout.

### 4. Full behavior bringup launches

```bash
export ROBOT_NAME=robot_1   # matches .env; skip if already set in the container
ros2 launch behavior_bringup behavior.launch.xml
```

Expected: no launch errors; `ros2 node list` (in another shell) shows
`/robot_1/behavior/navigation_executive` alongside `/robot_1/behavior/behavior_executive`,
`/robot_1/behavior/lab_machine_executive`, `/robot_1/behavior/behavior_tree_implementation`.

### 5. End-to-end, driven the way `routine_executor` would

With the stack from step 4 still running:

```bash
ros2 topic echo /robot_1/behavior/go_to_location_status &
ros2 topic pub -1 /robot_1/behavior/go_to_location_command std_msgs/msg/String "{data: ot2}"
```

Expected: `RUNNING` (confirms the full path command topic → `navigation_executive` → BT
condition → `drone.tree` → BT engine → `Active` → `navigation_executive` → dispatch is wired).
With the real `1_navigation` stack not running, this ends in `FAILURE` after
`nav_response_timeout` (10 s default) — expected here; step 2 already proved the state
machine's correctness against simulated `/nav/state`/`/nav/result`.

```bash
ros2 topic pub -1 /robot_1/behavior/go_to_location_command std_msgs/msg/String "{data: home}"
```

Expected: same pattern — confirms `home` shares `go_to`'s dispatch path (it's sugar over the
same command topic, not a separate mechanism).

```bash
ros2 topic echo /robot_1/behavior/navigate_to_pose_status &
ros2 topic pub -1 /robot_1/behavior/navigate_to_pose_command geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Expected: same pattern on `/robot_1/behavior/navigate_to_pose_status`.

### 6. Against the real nav stack (manual, needs the host Jazzy environment)

Needs `1_navigation` actually running (`autonomy/1_navigation/run.sh` on the host — see its own
README). When convenient:

```bash
autonomy/1_navigation/build.sh
autonomy/1_navigation/run.sh   # dry run - phoenix6 simulator, cannot move
```

then in another host terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic pub -1 /nav/goal_location std_msgs/msg/String "{data: home}"
ros2 topic echo /nav/state
```

Expected: `/nav/state` progresses through `SENDING` → `NAVIGATING` → `SUCCEEDED`. The point of
this step is confirming `home` resolves at all, not confirming a specific trajectory.

### Still to verify

Confirmed working so far: command handling, mutual exclusion, dispatch/timeout, the
condition-clear fix below, `drone.tree` parsing, full `behavior_bringup` launch, and the `go_to`
end-to-end path. Not yet separately confirmed:

- `routine_executor`'s pytest suite (`colcon test --packages-select routine_executor
  --event-handlers=console_direct+`) — needs a sourced ROS env with `behavior_tree_msgs` /
  `geometry_msgs` on the Python path.
- `home` and `go_to_pose` end-to-end through the real BT engine (step 5) — same code path as
  `go_to`, but worth confirming directly since they weren't exercised by the bug below.
- The rejection (`REJECTED unknown location ...`) and happy-path `SUCCESS` transitions
  specifically through the *real* BT engine (step 5), not just the isolated manual-pub tests in
  step 2.
- Step 6 (driving `home` against the real `1_navigation` stack) — optional, needs the host Jazzy
  environment, do whenever convenient.

## Known-fixed gotcha: clear `commanded_condition` on every terminal transition

Every place `tick_channel` reaches a terminal outcome (`REJECTED` result, `SUCCEEDED`,
`FAILED`/`CANCELED`, or the response timeout) must call `channel.commanded_condition->set(false)`
in the same step it calls `action->set_success()`/`set_failure()` — not just clear
`goal_in_flight`. This was missed in the first pass and reproduced exactly like this:

1. `go_to_location_command` arrives → `Go To Location Commanded` set `true`.
2. `1_navigation` never responds → `nav_response_timeout` fires → `action->set_failure()` called,
   `goal_in_flight` cleared — but the condition was left `true`.
3. Because the condition never went `false`, the `-> (Go To Location Commanded) ? [Go To
   Location]` `Sequence` in `drone.tree` keeps succeeding forever, so the wrapping `Fallback`
   never stops ticking that branch active, and the BT engine never sends `Active(false)` back.
4. `navigation_executive`'s own `action->is_active()` therefore stays `true` forever. The very
   next tick falls into the `!channel.goal_in_flight` branch, which (before the fix) just called
   `action->set_running()` — silently overwriting the `FAILURE` back to `RUNNING`, permanently.
   `/go_to_location_status` would show `status: 1` forever, even minutes after the timeout ERROR
   had already printed in the log.
5. Worse: since the BT engine believed the branch was still continuously active, a *later*
   `go_to`/`home` command would never redispatch either — `active_has_changed()` can only fire
   again after a real `false`→`true` edge, which was never going to happen. The channel was
   permanently wedged after one failure.

`lab_machine_executive` avoids this by always clearing `command_lab_machine_condition_` /
`ot2_condition_` / `shaker_condition_` alongside every terminal `set_success()`/`set_failure()`
call. The fix here does the same via a small `finish(set_status)` helper inside `tick_channel`
that sets the action status, clears the condition, and clears `goal_in_flight` together, called
from all four terminal sites. If you're extending this node with a new terminal transition,
route it through `finish` — don't call `action->set_success()`/`set_failure()` directly.

## Troubleshooting: seeing stale behavior after an edit

C++ changes need both a rebuild **and** a process restart — `colcon build` alone does not
restart an already-running node, and `--symlink-install` only makes non-compiled files (launch,
config, Python) update live. If a fix doesn't seem to take effect, or you see it behaving like
long-fixed code:

```bash
pkill -f behavior_tree_implementation
pkill -f behavior_executive
pkill -f navigation_executive
pkill -f lab_machine_executive
pkill -f manipulation_executive
ros2 node list   # confirm nothing is left under /<ROBOT_NAME>/behavior/...

cd ~/AutoLab/robot/ros_ws
colcon build --packages-select navigation_executive behavior_executive --symlink-install
source install/setup.bash   # in every terminal you use, including the one that launches
```

Then relaunch (step 4) and retest. A giveaway that you're running a stale build: seeing topics
from code that's since been deleted (e.g. `navigate_to_pose_goal` was removed from
`behavior_executive` in the cleanup that shipped alongside this package — if it's still in
`ros2 topic list`, something old is still running). The `RCLCPP_INFO`/`RCLCPP_ERROR` lines this
node logs on every command/dispatch/timeout (see step 5's expected log lines) are the fastest way
to confirm you're actually running the current build rather than guessing from topic values
alone.
