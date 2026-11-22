# Mock backend for the experiment UI

Run the APEX experiment workflow (start → progress → pause/resume → cancel → fail → retry →
clog check) in the UI **without the robot, ROS, or the GCS container**.

`mock_executor.py` stands in for everything behind the MQTT broker:

| Real component | What the mock does |
|---|---|
| `routine_executor` (GCS) | Same MQTT topics (`cmd/routine_executor/*` in, `ros2/routine_executor/status` out at 1 Hz, `{"data": "<json>"}` envelope). Uses the **real** `RoutineStateMachine`, and reads the step names / required params / `retryable` straight from `step_config.py`, so validation, pause, cancel and retry behave like prod: a failed `ot2` step is never auto-retried, and a Retry of it needs `{"force": true}`. |
| Robot / behavior tree | Each step "runs" for `--step-time` s (`wait` uses its `time_s`), then succeeds or fails. Like prod, **pause doesn't stop the current step**, it holds the next one. |
| `manipulation_executive` | `pick` / `place` / `pick_base` / `place_base` publish `ros2/<robot>/behavior/manipulation_phase` (SELECTING_CAMERA → PLANNING → RUNNING_ACT → SERVOING → PLANNING_SAFE) and `ros2/planning_state` (SUCCESS / ERROR). |
| `lab_machine_executive` | An `ot2` step (`--ot2-time` s, default 20) walks through connecting → each command → homing (the step succeeds once homed; the real executive keeps the OT-2 session open for the next run) and reports it in `status.step_progress`, like the real `ot2_progress` topic. After each `aspirate` (except one with `"tip_check": false`, e.g. clear liquid) the tip check runs pass by pass (live `step_progress.tip_check`: pass, status, P(empty) so far), then its finished result (`type: "tip_check"`, five averaged checks) goes to `status.recordings`. A clog or a failed check never fails the step. |
| Tip-check service (:8110) | Fake HTTP API: `POST /check` (one check, finished at once), `GET /jobs`, `GET /health`, and placeholder images under `/runs/…png` for the Clog check card. |

Not mocked: camera streams and the camera hub, so the camera panel shows offline. The UI now
streams from `http://192.168.10.3:8002` unless `REACT_APP_CAMERA_STREAM_BASE` is set; off the
robot LAN that request just times out.

On cancel, the real executor now also stops the step on the robot (`ManipulationCommand(type='cancel')`
or `/nav/cancel`). The mock just drops the running step, which looks the same from the UI.

## Run it

```bash
# 1. MQTT broker (needs Docker; on this Mac: `colima start` first)
docker run --rm --name mock-mosquitto -p 1883:1883 -p 9001:9001 \
  -v "$PWD/gcs/docker/monitoring/mosquitto/mosquitto.conf:/mosquitto/config/mosquitto.conf:ro" \
  eclipse-mosquitto

# 2. Mock backend (once: pip3 install paho-mqtt)
python3 gcs/ui/mock/mock_executor.py

# 3. UI dev server, pointed at the fake tip-check service for images and the debug check
cd gcs/ui && REACT_APP_TIP_CHECK_URL=http://localhost:8110 npm start   # http://localhost:3000
```

Then click through the welcome screen, edit the routine if you like, and press Continue.

To try the builder's **Describe the experiment** box as well, run the language planner alongside
(`npm start` forwards `/api/nlp` to `localhost:8090`):

```bash
NLP_CACHE=/tmp/nlp_cache.json python3 gcs/nlp_proxy/server.py
```

With no Ollama on this machine it answers the three demo prompts (the chips) from saved answers,
marked RECORDED, so type → blocks → Start works end to end with the mock.

## Scenarios

| Want to see | Run with |
|---|---|
| Happy path, quick | `--step-time 1` |
| One failure, then auto-retry succeeds ("Retry 1 of 3") | `--fail-step pick:1` |
| Retries exhausted → Failed card → **Retry from step N** succeeds | `--fail-step pick:4` (first attempt + `max_retries: 3` re-dispatches) |
| A step that never succeeds, even after UI retry | `--fail-step pick` |
| Random flakiness | `--fail-rate 0.3 --seed 1` |
| Clog check: first aspirate flagged | `--clog 1` (Nth tip check of the session; repeatable). Pick the **Anomaly** ready-made experiment in the builder for the matching routine. |
| Tip check fails (service down) but the protocol goes on | `--tip-check-error 1`, or `--tip-check-down` for every check |
| OT-2 fails after the aspirate → not retried → forced Retry | `--fail-step ot2:1` |
| Debug check / card images unreachable | `--no-tip-check-server` |

While it runs, type in its terminal: `f` = fail the current step now, `s` = succeed it now,
`p` = print status, `q` = quit.

Rejected routines (unknown step, missing param, start while running) are logged as `ERROR`
in the mock's terminal and ignored, the same as the real executor. The UI just keeps showing
the previous state.

## Tests

```bash
cd gcs/ui/mock && python3 -m pytest -q
```
