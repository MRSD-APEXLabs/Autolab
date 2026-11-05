# led_status

Drives the robot's underglow LED strip from behavior tree (BT) action state. The node watches
the BT action topics on the robot and sends one status letter over USB serial to the LED Nano,
which runs the `led/robot_status/robot_status.ino` firmware (repo root `led/`).

The node reads only robot-side signals, not the GCS `routine_executor`. The LEDs therefore
react the same way whether an action comes from a routine or from a manual
`ros2 topic pub`. The trade-off is that routine-level states (paused, between steps, `wait`
steps) are not shown.

## State → LED

| Robot state | Letter | Pattern |
|---|---|---|
| Nothing active | `I` | Idle: slow teal breathing |
| `Go To Location` / `Navigate To Pose` active | `M` | Moving: cyan packets flow rear → front |
| `Pick Object` / `Place Object` / `Pick Base Object` / `Place Base Object` active | `A` | Arm working: purple halo, front ripples |
| `Execute OT2 Protocol` / `Execute Shaker Protocol` active | `W` | Waiting: amber double pulse |
| A watched action failed | `E` | Fault: red breathing perimeter, held until the next action starts |
| A watched action succeeded | `R` | Ready: green halo for 5 s, then idle |
| Node shut down | `O` | Off |

The firmware also has patterns for turning left/right (`L`/`T`) and charging (`C`). They are
not used yet because the robot publishes no signal for them.

### Rules

- **Priority:** if several actions are active at once, moving beats arm and arm beats waiting.
  A running action always beats a held fault or success.
- **Only active actions count.** Every executive publishes `<action>_status` at 20 Hz, and an
  idle action reports `FAILURE` by default. Status is ignored unless the BT says the action is
  active.
- **Ignoring status left over from the last run:** right after an action becomes active, its
  executive can still report the previous run's status for a tick. A `FAILURE` or `SUCCESS`
  only counts once that run has reported `RUNNING`, or after `grace_s` (0.5 s) of activity.
- **Clearing a fault:** red holds until any watched action is newly activated, or until the
  failed action reports `RUNNING` again (for example, a retry).
- **Stale BT:** the BT engine republishes `<action>_active` every tick. If no `_active`
  message arrives for `stale_s` (1 s), the action is treated as inactive, so the strip returns
  to idle if the BT dies.

## Topics

All topics are relative and resolve under `/$(ROBOT_NAME)/behavior/` when the node is started
from `behavior.launch.xml`. Each topic stem is the action label in lowercase, with spaces
replaced by `_`; this matches `bt::Action::get_*_topic_name`.

| Subscribed | Type |
|---|---|
| `<action>_active` | `behavior_tree_msgs/Active` (published by the BT engine) |
| `<action>_status` | `behavior_tree_msgs/Status` (published by the executive: `FAILURE=0 RUNNING=1 SUCCESS=2`) |

Watched actions:
- `go_to_location`, `navigate_to_pose`
- `pick_object`, `place_object`, `pick_base_object`, `place_base_object`
- `execute_ot2_protocol`, `execute_shaker_protocol`

The node publishes nothing. Its pattern changes appear in the log as `LED status -> X`.

## Parameters

| Name | Default | Meaning |
|---|---|---|
| `port` | `""` | Device path. If empty, the node tries `by_id_path` and then searches sysfs for a CH340 (`1a86:7523`) |
| `by_id_path` | `/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0` | Preferred udev path for the device |
| `tick_hz` | `10.0` | How often the pattern is evaluated and sent |
| `success_hold_s` | `5.0` | How long green shows after a success |
| `stale_s` | `1.0` | Time without `_active` messages after which an action counts as inactive |
| `grace_s` | `0.5` | How long after activation a status can be ignored as left over from the last run |
| `resend_s` | `3.0` | The current letter is resent this often, so the strip recovers if the Nano resets |
| `retry_s` | `5.0` | Delay between attempts to reopen the port when it is missing or fails |
| `settle_s` | `2.0` | Wait after opening the port; opening resets the Nano into its bootloader |

The launch file exposes only `port`: `ros2 launch led_status led_status.launch.xml port:=/dev/ttyUSB1`.

## Code layout

| File | Purpose |
|---|---|
| `led_status/logic.py` | `LedLogic`: pure pattern selection; takes `now` explicitly. Unit tested |
| `led_status/serial_link.py` | `find_port` (by-id path, then sysfs search), `SerialPort` (raw termios at 115200), `LedLink` (send on change, resend, reconnect). No ROS. Unit tested |
| `led_status/node.py` | Thin rclpy wrapper. `GROUPS` maps actions to letters, in priority order |
| `launch/led_status.launch.xml` | Starts the node; included by `behavior_bringup/launch/behavior.launch.xml` |

The logic and serial code don't use rclpy, so they can be tested on any machine with Python
and pytest.

## Testing

### 1. Unit tests (no ROS, runs anywhere)

```bash
cd robot/ros_ws/src/autonomy/6_behavior/led_status
python3 -m pytest -q test/
```

### 2. Build and colcon test (robot container)

```bash
colcon build --packages-select led_status && sws
colcon test --packages-select led_status --event-handlers=console_direct+
```

### 3. Node smoke test without the BT or executives

Run the node alone (root namespace). `_active` must be published continuously; otherwise it
goes stale after 1 s.

```bash
ros2 run led_status led_status --ros-args -p port:=/dev/ttyUSB1   # omit -p to auto-detect

# shell 2: navigation running → M
ros2 topic pub -r 10 /go_to_location_active behavior_tree_msgs/msg/Active "{active: true, id: 1}" &
ros2 topic pub -r 20 /go_to_location_status behavior_tree_msgs/msg/Status "{status: 1, id: 1}"
# Ctrl-C the status pub, then report failure → E (red holds)
ros2 topic pub -r 20 /go_to_location_status behavior_tree_msgs/msg/Status "{status: 0, id: 1}"
# Kill both → still E. Start any action again → its pattern, red cleared.
# Report success instead of failure → R for 5 s, then I.
```

Without a Nano attached, the node logs `LED Nano unavailable (...)` once and still logs each
`LED status -> X` change, so the logic can be checked on a desktop or in sim.

### 4. End to end on the robot

1. Bring up the robot stack. In the `led_status` log, look for `LED Nano opened on /dev/...`.
   The strip should go teal (idle) about 2 s later.
2. Send a `go_to` (routine or `behavior/go_to_location_command`): cyan while driving, then
   green for 5 s, then teal.
3. Send a `pick` / `place`: purple.
4. Run an OT2 or shaker step: amber.
5. Force a failure, for example by sending `go_to` with a location name that does not exist:
   red holds until the next command.

Check what the BT thinks is active: `ros2 topic echo /robot_1/behavior/active_actions`.

### Testing the strip by hand (bypassing ROS)

Stop the node first, because only one process can hold the port:

```bash
cd led && python3 robot_status/send_status.py M     # or no arg for interactive
```

## Hardware and deployment notes

- **LED Nano:** ATmega328P Nano with a CH340 USB-serial chip (`1a86:7523`). The strip is 73
  WS2812 LEDs on D6. Physical layout: LEDs 1–17 back, 18–36 right, 37–54 front, 55–73 left,
  counter-clockwise from back-left. See `led/robot_status/README.md`.
- **Thor (`labx@192.168.10.3`):**
  - The LED Nano usually shows up as `/dev/ttyUSB1`. `/dev/ttyUSB0` is the *other* Nano (FTDI
    `0403:6001`), which must not be written to. The sysfs search only accepts CH340 devices.
  - Thor's kernel (`6.8.12-tegra`) ships without the CH341 driver, so a module was built and
    installed by hand (see `led/THOR.md`). **A kernel upgrade needs that module rebuilt**;
    otherwise the Nano gets no `/dev/ttyUSB*` node.
- **Container access:** `robot-l4t` runs `privileged` with group 20 (`dialout`), so no compose
  changes were needed. Docker copies the host's device nodes when the container starts:
  - If the Nano is plugged in after the container is already up, it may not appear until the
    container restarts.
  - `/dev/serial/by-id` symlinks are often missing inside the container; the sysfs search
    covers that.
- **Port conflicts:** `send_status.py`, `arduino-cli upload`, and serial monitors all need
  exclusive access to the port. Stop the node (or the behavior stack) before using them.
- **Firmware:** single ASCII letter plus newline at 115200 baud; case-insensitive; it replies
  with an acknowledgement, which the node reads and discards. It starts with the LEDs off
  after a reset. Build and flash instructions are in `led/THOR.md`.

## Extending

- **Watch another action:** add its BT label to a group in `GROUPS` in `node.py`, or add a new
  `(letter, [labels])` tuple at the position that sets its priority. The label must match the
  `bt::Action` label exactly.
- **New pattern:**
  1. Add a command letter and an effect to `led/robot_status/robot_status.ino`, then reflash
     (`led/THOR.md`).
  2. Map an action group to that letter in `GROUPS`.
- **Turning (`L`/`T`):** needs a turn signal. `/cmd_vel` lives on nav's private ROS domain 42
  and is not bridged (`navigation_executive/config/nav_domain_bridge.yaml`). Either bridge a
  yaw-rate topic or have `navigation_executive` publish one, then add a moving sub-state in
  `LedLogic`.
- **Charging (`C`) / real faults (e-stop, hardware health):** nothing publishes these yet. A
  latched "override" input in `LedLogic` that beats action state is the natural place to add
  them.
- **Routine-level states (paused, between steps):** these exist only in the GCS
  `/routine_executor/status` JSON. Subscribing to it would add a dependency on the GCS; it
  was left out on purpose.
- **Logic changes:** write the case in `test/test_logic.py` first. `LedLogic` takes timestamps
  explicitly, so timing cases need no sleeps. Remember that real `_active` messages arrive
  every BT tick: a test with a gap of more than `stale_s` between `_active` calls is read as a
  new activation.
