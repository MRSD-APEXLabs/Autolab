# Web UI

**Purpose:** the browser front-end ("APEX Labs / Apex") for launching and monitoring routines. A Create-React-App + Tailwind + framer-motion SPA that talks **MQTT over WebSocket** directly from the browser — there is no REST backend.

Path: `gcs/ui/`.

## Running it

!!! tip "There is already one running"
    A dev server is live on the GCS laptop at **`http://gcs.autolab:3000`** (verified 2026-09-12). It is a *patched* copy — see [the deployed copy](#the-deployed-copy-on-the-gcs-laptop-verified-2026-09-12) — and it has camera tiles but **no camera-mode buttons**.

```bash
# dev server with hot reload → http://localhost:3000
cd gcs/ui && npm install && npm start
# …or containerized (requires the gcs include enabled in the root compose — see GCS page):
COMPOSE_PROFILES=ui-dev autolab up ui-dev

# production (static build served by nginx) → http://localhost:80
autolab up ui
```

`Dockerfile.ui` has three targets: `dev` (CRA server), `build`, `prod` (nginx serving `/app/build`). Ports come from `.env` (`UI_PORT=80`, `UI_DEV_PORT=3000`).

!!! warning "The image does not build ([#29](../known-issues.md))"
    `package-lock.json` is out of sync with `package.json` (`mqtt` was added without regenerating the lock), so `npm ci` in `Dockerfile.ui` fails. Until the lock is regenerated, run the dev server in a throwaway container with the source mounted **read-only** (nothing written to the repo):

    ```bash
    cd ~/coding/Autolab
    docker run -d --name apex-ui -p 3000:3000 -e HOST=0.0.0.0 -e CI=true -v "$PWD/gcs/ui:/src:ro" -w /app node:20-alpine \
      sh -c "cp -r /src/. /app && npm install --no-audit --no-fund && npm start"      # → http://localhost:3000
    ```

    For camera feeds only, the Xavier's own dashboard needs no build at all — see [Camera-Edge (Xavier)](xavier.md#the-dashboard-example_clienttesthtml).

For the UI to actually do anything, the GCS side must be up: Mosquitto, both MQTT bridges, and `routine_executor_node` — see [GCS](gcs.md) and [Running the System](../running.md#3-ground-control-station).

## Pages and files

| File | Role |
|---|---|
| `src/App.js` | Welcome page → **Begin** → routine JSON editor (pre-filled with `DEFAULT_ROUTINE`) → **Continue** publishes to MQTT |
| `src/LoaderPage.js` | 5-second spinner (a fixed `setTimeout`, not real readiness), then renders the experiment view |
| `src/Experiment.js` | Live progress: one dot per step, retry counts, error/success cards; Cancel / Pause / Resume buttons |
| `src/CameraPanel.js` | Floating, resizable overlay with 3 camera tiles fed by raw WebSockets |
| `src/index.js`, `public/index.html` | Entry point (`<title>Apex</title>`) |

Honest notes for operators: the "Vision Understanding / Intelligent Planning / Robotic Execution" cards and the "Objects detected: 3 items" banner are **static text**; the real control surface is the JSON textarea.

## Wire protocol

Broker: `ws://${window.location.hostname}:9001` (`App.js`, `Experiment.js`) — so browsing from another machine works, as long as port 9001 is reachable.

| Direction | MQTT topic | Payload |
|---|---|---|
| publish | `cmd/routine_executor/start_routine_cmd` | The routine JSON ([schema](../running.md#5-run-a-routine-from-the-browser)) |
| publish | `cmd/routine_executor/cancel` | — |
| publish | `cmd/routine_executor/pause`, `…/resume` | ⚠ **dropped by the bridge** — the buttons do nothing end-to-end ([known issue #1](../known-issues.md)) |
| subscribe | `ros2/routine_executor/status` | `{"data": "<status json>"}` envelope from `ros2_mqtt_bridge` |
| subscribe | `ros2/behavior/manipulation_phase`, `ros2/planning_status` | Per-step detail topics |

(`Experiment.js` claims its `STEP_DISPLAY_TOPICS` mirrors a `display_topics` key in `step_config.py` — no such key exists; [known issue #15](../known-issues.md).)

## Camera panel

`src/CameraPanel.js` opens three raw WebSockets expecting `{image_jpeg_b64: …}` JSON frames:

- **Active** tile → `ws://${HOST}:8766` (the camera-edge live stream)
- **Wrist** and **Base** tiles → `ws://${HOST}:8767`, subscribing with `{camera:'wrist'}` / `{camera:'base'}`

Two caveats: the server for these ports is the [Xavier](xavier.md), **not in this repo** ([known issue #9](../known-issues.md)); and in **this repo** `HOST` is hardcoded to `'localhost'` (unlike the MQTT code, which uses `window.location.hostname`), so a UI built from git shows *No signal* unless the browser's machine reaches those ports on itself — `ssh -N -L 8766:localhost:8766 -L 8767:localhost:8767 autolab@192.168.1.101`, or edit `CameraPanel.js` ([known issue #15](../known-issues.md), [#35](../known-issues.md)).

The panel opens **only 8766 and 8767 — never the control port 8765**. That is why it has camera tiles but **no Servo / Inspect / Idle buttons**. To switch camera-edge modes use the Xavier's own dashboard or the CLI client — see [Camera-Edge (Xavier)](xavier.md#modes-and-protocol) and `CLAUDE.md` §3.2.

### The deployed copy on the GCS laptop (verified 2026-09-12)

`http://gcs.autolab:3000` (= `192.168.10.4:3000`) serves a **running CRA dev server** (`X-Powered-By: Express`, `<title>Apex</title>`) on Juan's laptop, and its camera tiles do work. The reason is that the copy deployed there is **patched and not in git**: its bundle has

```js
const HOST = 'xavier.autolab';      // deployed on gcs.autolab
const HOST = 'localhost';           // what gcs/ui/src/CameraPanel.js says in this repo
```

So the browser connects directly to `ws://xavier.autolab:8766` / `:8767`, with no tunnel. Whatever machine runs the browser must therefore resolve `xavier.autolab` (router DNS → `192.168.10.7`) and reach those two ports. The MQTT side of that same page points at `ws://gcs.autolab:9001`, which is up; port `1883` is not exposed.

## Extending the step display

The step vocabulary shown in the editor comes from whatever JSON you type; validation happens server-side in `routine_executor` against `step_config.py`. If you add a new step type there, also add its display topic mapping in `Experiment.js` so the progress view can show per-step detail.
