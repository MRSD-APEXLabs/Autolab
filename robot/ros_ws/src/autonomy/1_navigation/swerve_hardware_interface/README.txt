Instructions for testing navigation integration:

0. Bring up the CAN bus first — NOT automatic, and easy to waste hours on
(see "Known issues" at the bottom for the full debugging history):

  a. `caniv -a -s` (NOT `canivore_setup` — that binary is a no-op stub,
     confirmed via strace, it does nothing but print its banner).
     Needs sudo unless the CANivore udev rule grants your user access.
  b. Wait ~45s after bring-up before starting the node. Skipping this
     caused persistent "CAN frame not received/too-stale" status-signal
     errors even once the bus itself was confirmed up (`caniv -a -i`
     showing CAN Bus Enable: true, `candump can0` showing real traffic).
  c. `can_bus_name` param must be the SocketCAN ifname (`can0` on the
     current hardware — check with `ls /sys/class/net/ | grep can` +
     `caniv -a -s` to confirm which one lights up), not the CANivore's
     display name ("AutoLab Canivore") — the latter fails to connect
     even though it's what `caniv -a -i` reports as the device's own name.
  d. If you see `error while loading shared libraries:
     librcl_interfaces__rosidl_typesupport_cpp.so` on startup: the
     robot-l4t container was built from an image older than the
     ld.so.conf fix in Dockerfile.robot (setcap'd binaries run in glibc
     secure-exec mode, which ignores LD_LIBRARY_PATH — ROS's core libs
     must be registered via ldconfig instead). Rebuild the image, don't
     hand-patch /etc/ld.so.conf.d in a running container — it won't
     survive the next recreate.
  e. docker-compose.yaml's robot-l4t command now runs `caniv -a -s` +
     the 45s wait automatically before launch. If you're running the
     node manually (step 3 below) outside that launch path, you still
     need to do (a)-(c) yourself first.

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

Known issues / open follow-ups (from the 2026-09-15/16 bench bring-up session):

- swerve_hardware_interface_node.cpp's configure() used to run once,
  synchronously, immediately after constructing the CANBus/device objects —
  racing Phoenix6's own async CANivore session bring-up (its "CANbus
  Connected"/"Network Up"/"Library initialization is complete" log lines
  observably printed AFTER our first configure() attempt already failed).
  Fixed by retrying the whole configure pass up to 10x, 500ms apart, instead
  of giving up after one attempt. If you still see "Hardware is DISABLED"
  on startup after that fix, the race window may need to be wider — bump
  the retry count/interval before assuming it's a wiring issue.
- Even with configure() succeeding and motors commandable, expect ongoing
  "CAN frame not received/too-stale" spam from check_watchdog()'s
  is_healthy() check — it's an instantaneous per-tick check with no
  debounce, and appears to trip more often than actual motion is affected
  (motors did respond to commands during testing despite continuous
  "unhealthy" log spam). Not yet root-caused whether this is bus-bandwidth-
  driven (12 devices/module, ~13% bus util observed) or a Phoenix6 default
  signal-update-frequency that's lower than the watchdog's tolerance. If
  this becomes an actual problem (not just noise), look at either debouncing
  is_healthy() (require N consecutive stale ticks before zeroing velocity)
  or explicitly raising signal update frequency via Phoenix6's
  SetUpdateFrequency() API — don't just silence the log.
- `robot` user in the robot-l4t container is NOT actually in the `dialout`
  group at runtime (confirmed via `id`), despite both Dockerfile.robot's
  `usermod -aG dialout robot` and docker-compose.yaml's `group_add: ["20"]`
  on robot-l4t. Turned out to be a dead end for the CAN bring-up issues
  (there's no /dev/can* character device gated by group permissions — this
  driver is pure-netdev, group membership doesn't gate it), so it wasn't
  chased further. Still a real gap if anything else in this image expects
  dialout membership (e.g. serial devices) — worth fixing but not urgent.
- The known velocity-units question flagged in swerve_hardware_interface_node.cpp
  (navigation_controller's hypot(...)/(wheel_radius*pi) vs true rot/s of
  hypot(...)/(2*pi*wheel_radius), possible 2x factor) was NOT resolved this
  session — still needs the "command a known low velocity, count real wheel
  rotations" experiment from step 5 above before trusting speed commands
  from the full navigation_controller stack.