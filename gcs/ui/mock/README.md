# Mock backend for the experiment UI

Run the APEX experiment workflow (start → progress → pause/resume → cancel → fail → retry →
clog check) in the UI **without the robot, ROS, or the GCS container**.

`mock_executor.py` stands in for everything behind the MQTT broker:

| Real component | What the mock does |
|---|---|
| `routine_executor` (GCS) | Same MQTT topics (`cmd/routine_executor/*` in, `ros2/routine_executor/status` out at 1 Hz, `{"data": "<json>"}` envelope). Uses the **real** `RoutineStateMachine`, and reads the step names / required params straight from `step_config.py`, so validation, pause, cancel and retry behave like prod. |
| Robot / behavior tree | Each step "runs" for `--step-time` s (`wait` uses its `time_s`), then succeeds or fails. Like prod, **pause doesn't stop the current step**, it holds the next one. |
| `manipulation_executive` | `pick` / `place` / `pick_base` / `place_base` publish `ros2/<robot>/behavior/manipulation_phase` (SELECTING_CAMERA → PLANNING → RUNNING_ACT → SERVOING → PLANNING_SAFE) and `ros2/planning_state` (SUCCESS / ERROR). |
| `lab_machine_executive` | Each `aspirate` in an `ot2` step attaches a clog-detection clip to `status.recordings`. |
| Clog-detection service (:8100) | Fake HTTP API. Each clip goes recording → queued → running → done over ~10 s. |

Not mocked: camera streams (:8002) and the camera hub, so the camera panel shows offline.

## Run it

```bash
# 1. MQTT broker (needs Docker; on this Mac: `colima start` first)
docker run --rm --name mock-mosquitto -p 1883:1883 -p 9001:9001 \
  -v "$PWD/gcs/docker/monitoring/mosquitto/mosquitto.conf:/mosquitto/config/mosquitto.conf:ro" \
  eclipse-mosquitto

# 2. Mock backend (once: pip3 install paho-mqtt)
python3 gcs/ui/mock/mock_executor.py

# 3. UI dev server
cd gcs/ui && npm start        # http://localhost:3000
```

Then click through the welcome screen, edit the routine if you like, and press Continue.

## Scenarios

| Want to see | Run with |
|---|---|
| Happy path, quick | `--step-time 1` |
| One failure, then auto-retry succeeds ("Retry 1 of 3") | `--fail-step pick:1` |
| Retries exhausted → Failed card → **Retry from step N** succeeds | `--fail-step pick:3` (matches `max_retries: 3`) |
| A step that never succeeds, even after UI retry | `--fail-step pick` |
| Random flakiness | `--fail-rate 0.3 --seed 1` |
| Clog check: first aspirate flagged | `--clog 1` (Nth clip of the session; repeatable) |
| Clog service unreachable banner | `--no-anomaly` |

While it runs, type in its terminal: `f` = fail the current step now, `s` = succeed it now,
`p` = print status, `q` = quit.

Rejected routines (unknown step, missing param, start while running) are logged as `ERROR`
in the mock's terminal and ignored, the same as the real executor. The UI just keeps showing
the previous state.

## Tests

```bash
cd gcs/ui/mock && python3 -m pytest -q
```
