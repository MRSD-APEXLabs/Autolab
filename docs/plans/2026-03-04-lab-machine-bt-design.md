# Lab Machine Behavior Tree Branch — Design

**Date:** 2026-03-04
**Branch:** `jmuerto/lmi-behavior-tree`

## Overview

Add a new `Command Lab Machine` branch to the drone behavior tree that enables the robot's autonomy stack to command remote lab machines (OT-2 Liquid Handler and Custom Shaker Module) via REST APIs. Execution is non-blocking: a dedicated ROS2 node fires HTTP calls asynchronously and reports status back to the BT engine.

---

## Architecture

```
GCS/Operator
    │
    ├─ publishes BehaviorTreeCommands ──→ behavior_executive
    │   (sets conditions: "Command Lab Machine",
    │    "OT-2 Liquid Handler", "Custom Shaker Module")
    │
    └─ publishes LabMachineCommand ──────→ lab_machine_executive  (new node)
        (device + action + JSON params)       │
                                              │  bt::Action — subscribes to <label>_active
                                              │  bt::Action — publishes <label>_status
                                              │
                                              └─ fires HTTP POST (std::async / libcurl)
                                                    ├─ OT-2 wrapper  (e.g. /commands/aspirate)
                                                    └─ Shaker API    (POST plaintext int)

behavior_tree node (existing, untouched)
    ── ticks drone.tree ──→ activates actions ──→ reads statuses
```

**Key constraints:**
- The BT engine (`behavior_tree` package) is untouched.
- `behavior_executive` gains 3 new `bt::Condition` objects only.
- The new `lab_machine_executive` node runs inside the existing `robot` container.
- OT-2 base URL and shaker base URL are ROS2 parameters set via a YAML config file.

---

## New Message Type

**File:** `common/ros_packages/behavior_tree_msgs/msg/LabMachineCommand.msg`

```
string device           # "ot2" | "shaker"
string action           # reserved for future single-step use; "protocol" for multi-step
string parameters_json  # JSON payload (see below)
```

**OT-2 protocol payload** (multi-step, arbitrary repetition supported):
```json
{
  "steps": [
    {"action": "pick_up_tips", "resource_name": "tips", "well_indices": [0]},
    {"action": "aspirate",     "resource_name": "plate", "well_indices": [0], "volumes": [100.0]},
    {"action": "dispense",     "resource_name": "plate", "well_indices": [8], "volumes": [100.0]},
    {"action": "aspirate",     "resource_name": "plate", "well_indices": [0], "volumes": [100.0]},
    {"action": "dispense",     "resource_name": "plate", "well_indices": [16], "volumes": [100.0]},
    {"action": "return_tips"}
  ]
}
```

**Shaker payload:**
```json
{"pwm": 128}
```

**Parameter storage:** `lab_machine_executive` holds the most-recently-received protocol per device in memory. The GCS publishes `LabMachineCommand` before (or simultaneously with) setting the BT condition that enables the branch.

---

## Behavior Tree Structure

`robot/ros_ws/src/autonomy/6_behavior/behavior_tree/config/drone.tree`:

```
?
	->
		(Command Lab Machine)
		?
			->
				(OT-2 Liquid Handler)
				[Execute OT-2 Protocol]
			->
				(Custom Shaker Module)
				[Execute Shaker Protocol]
	->
		(Pause Commanded)
		?
			[Pause]
	->
		(Rewind Commanded)
		?
			[Rewind]
	->
		(Fixed Trajectory Commanded)
		?
			[Follow Fixed Trajectory]
	->
		(Global Plan Commanded)
		?
			[Follow Global Plan]
	->
		(Arm Commanded)
		?
			(Armed)
			[Arm]
	->
		(Disarm Commanded)
		?
			<!>
				(Armed)
			[Disarm]
	->
		(Autonomously Explore Commanded)
		->
			<!>
				(State Estimate Timed Out)
			?
				(Armed)
				[Arm]
		?
			->
				(Stuck)
				[Rewind]
			[Follow Global Plan]
```

**Semantics of new branch:**
- Outer `->`: `Command Lab Machine` must be true AND a sub-branch must succeed.
- Inner `?`: try OT-2 branch first, then shaker branch.
- OT-2 `->`: `OT-2 Liquid Handler` must be true, then `[Execute OT-2 Protocol]` runs the full step list.
- Shaker `->`: `Custom Shaker Module` must be true, then `[Execute Shaker Protocol]` fires one PWM POST.

---

## `lab_machine_executive` Node

### Package location
`robot/ros_ws/src/autonomy/6_behavior/lab_machine_executive/`

### File structure
```
lab_machine_executive/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── lab_machines.yaml
├── launch/
│   └── lab_machine_executive.launch.xml
├── include/lab_machine_executive/
│   └── lab_machine_executive.hpp
└── src/
    └── lab_machine_executive.cpp
```

### ROS2 Parameters (`lab_machines.yaml`)
```yaml
lab_machine_executive:
  ros__parameters:
    ot2_base_url: "http://192.168.1.100:8000"
    shaker_base_url: "http://192.168.1.101:8080"
```

### Key members
| Member | Type | Purpose |
|---|---|---|
| `ot2_action` | `bt::Action*` | BT action for OT-2 protocol |
| `shaker_action` | `bt::Action*` | BT action for shaker protocol |
| `ot2_condition` | `bt::Condition*` | "OT-2 Liquid Handler" |
| `shaker_condition` | `bt::Condition*` | "Custom Shaker Module" |
| `cmd_lab_machine_condition` | `bt::Condition*` | "Command Lab Machine" |
| `ot2_protocol_json` | `std::string` | Latest OT-2 protocol params |
| `shaker_protocol_json` | `std::string` | Latest shaker params |
| `ot2_current_step` | `int` | Step index into parsed OT-2 steps |
| `pending_http` | `std::future<bool>` | In-flight HTTP call result |
| `active_device` | `enum {NONE, OT2, SHAKER}` | Which device is currently executing |

### Async HTTP pattern
Each HTTP call runs in `std::async(std::launch::async, ...)` using `libcurl`. The timer callback (20 Hz) checks `pending_http.wait_for(std::chrono::milliseconds(0))`. If ready, it reads the result, advances `ot2_current_step`, and dispatches the next step or reports SUCCESS/FAILURE to the BT.

### Step execution state machine
```
IDLE
  └─(action becomes active, params present)─→ STEP_PENDING
        │                                          │
        │    (future not ready)                    │
        └──────────────────────────────────────────┘
        │    (future ready, step ok, more steps)─→ STEP_PENDING (next step)
        │    (future ready, all steps done)──────→ SUCCESS → IDLE
        │    (future ready, step failed)─────────→ FAILURE → IDLE
        │    (no params)──────────────────────────→ FAILURE → IDLE
```

---

## Changes to Existing Files

| File | Change |
|---|---|
| `behavior_tree_msgs/CMakeLists.txt` | Add `LabMachineCommand.msg` |
| `behavior_tree_msgs/msg/LabMachineCommand.msg` | New file |
| `behavior_executive/src/behavior_executive.cpp` | Add 3 `bt::Condition` objects: `Command Lab Machine`, `OT-2 Liquid Handler`, `Custom Shaker Module` |
| `behavior_executive/include/behavior_executive/behavior_executive.hpp` | Add 3 condition members |
| `behavior_tree/config/drone.tree` | Prepend new branch |
| `behavior_bringup/launch/behavior.launch.xml` | Include `lab_machine_executive.launch.xml` |

---

## Error Handling

- **HTTP failure** (non-2xx or curl error): action reports FAILURE immediately. No automatic retry.
- **Mid-protocol failure**: protocol aborts at the failed step; OT-2 state is left as-is. Recovery is the GCS operator's responsibility.
- **No parameters stored**: FAILURE + log warning.
- **Malformed JSON**: FAILURE + log error.
- **Condition cleanup**: GCS is responsible for clearing `Command Lab Machine` after observing SUCCESS or FAILURE. The executive does not auto-clear conditions.
