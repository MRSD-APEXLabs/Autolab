# GCS (Ground Control Station)

Operator-side stack: React UI, ROS2 GCS node bringup (rviz/rqt), MQTT bridges connecting the UI to ROS2, and an ELK monitoring stack.

## Layout

```
gcs/
├── docker/                          # docker-compose services for the GCS
│   ├── docker-compose.yaml          # gcs, gcs-real, ui, ui-dev, elasticsearch, kibana, filebeat
│   ├── gcs-base-docker-compose.yaml # shared ROS2 container definition (gcs-base)
│   ├── Dockerfile.gcs
│   └── monitoring/
│       ├── mosquitto/mosquitto.conf # MQTT broker config (ports 1883 + 9001 websocket)
│       └── filebeat/filebeat.yml
├── ui/                               # React web UI (Create React App)
└── ros_ws/src/
    ├── gcs_bringup/                  # top-level launch (rviz, rqt, domain_bridge, monitoring bridges, routine_executor)
    ├── gcs_monitoring/                # ROS2 <-> MQTT bridge nodes
    ├── routine_executor/              # runs operator-triggered routines (state machine)
    ├── rqt_gcs/                       # main rqt panel plugin (trajectory UI)
    └── rqt_autolab_control_panel/     # rqt operator control panel plugin
```

## UI

`gcs/ui` is a Create React App (React 19) that connects directly to the MQTT broker over WebSockets — it does not talk to ROS2 directly.

- Broker URL: `ws://<gcs-host>:9001` (`ui/src/App.js`), no ROS2 client involved.
- `mqtt` npm package used for pub/sub in the browser.

**Where it runs:**
- `ui` service (docker/docker-compose.yaml) — production build served by nginx on port `80` (override with `UI_PORT`). Built via `ui/Dockerfile.ui` (`prod` target: CRA build → nginx).
- `ui-dev` service — CRA dev server with hot reload on port `3000` (override with `UI_DEV_PORT`), profile `ui-dev`. Mounts `ui/src` and `ui/public` for live editing.
- Both build from `ui/Dockerfile.ui`; `dev` stage runs `npm start`, `prod` stage runs `npm run build` then serves via nginx.

Local (no Docker): `cd gcs/ui && npm install && npm start` (needs a broker reachable at `ws://localhost:9001`, e.g. from the `gcs` container's exposed `9001:9001` port).

Test/debug the UI's MQTT wiring without ROS2 using `ui/test_mqtt_status.py` (publishes/subscribes on `ros2/routine_executor/status` via the websocket port — see its docstring for usage).

## Bridges

MQTT is the transport between the browser UI and ROS2. Mosquitto runs inside the `gcs`/`gcs-real` container (`docker/monitoring/mosquitto/mosquitto.conf`): port `1883` for native MQTT, `9001` for MQTT-over-websockets (what the UI uses). Two ROS2 nodes in `gcs_monitoring` bridge the two worlds, both launched by `gcs_bringup/launch/gcs.launch.xml`:

- **`ros2_mqtt_bridge`** (`gcs_monitoring/ros2_mqtt_bridge.py`) — ROS2 → MQTT. Auto-discovers ROS2 topics every `ROS2MQTT_DISCOVERY_INTERVAL` seconds (default 10s), subscribes, and republishes each message as JSON to `ros2/<topic>` (e.g. `/odom` → `ros2/odom`). Skips `/tf`, `/tf_static`, `stereo` topics, and a hardcoded list of high-frequency/binary types (images, point clouds, IMU, joint states, TF, trajectories) to avoid flooding MQTT.
- **`mqtt_ros2_bridge`** (`gcs_monitoring/mqtt_ros2_bridge.py`) — MQTT → ROS2. Subscribes to a fixed set of UI command topics and republishes them as `std_msgs/String` on ROS2:
  - `cmd/routine_executor/start_routine_cmd` → `/routine_executor/start_routine_cmd`
  - `cmd/routine_executor/cancel` → `/routine_executor/cancel`

Both bridges connect to the broker at `MQTT_HOST`:`MQTT_PORT` (default `localhost:1883`, i.e. native MQTT — not the websocket port used by the browser), retrying until Mosquitto is up.

There's also a **`domain_bridge`** node (not MQTT — a ROS2 `domain_bridge` instance) configured by `gcs_bringup/config/domain_bridge.yaml`, which relays specific topics between ROS domain 0 (robot) and domain 1 (GCS) — behavior tree commands, joint states, fixed-trajectory commands, and bag-recording status/control.

## GCS ROS2 bringup

`ros2 launch gcs_bringup gcs.launch.xml` starts, in one process group:

1. `rviz2` with `rviz/gcs.rviz`
2. `rqt_gui` with the `config/gcs.perspective` layout (hosts the `rqt_gcs` and `rqt_autolab_control_panel` plugins)
3. `domain_bridge` (domain 0 ↔ 1 relay, see above)
4. `ros2_mqtt_bridge` and `mqtt_ros2_bridge` (monitoring bridges, see above)
5. `routine_executor_node` (executes operator-triggered routines; commanded via the MQTT bridges above)

All nodes `respawn` on crash. This launch runs automatically inside the `gcs`/`gcs-real` container when `AUTOLAUNCH=true` (see root `.env`), or manually via `autolab connect gcs` then `bws && sws && ros2 launch gcs_bringup gcs.launch.xml`.

## Monitoring stack (ELK)

`elasticsearch`, `kibana`, and `filebeat` services (docker/docker-compose.yaml) ship container logs for the `sitl`/default/`simple` profiles only — `gcs-real` uses `network_mode: host` and isn't covered by this stack. Kibana UI on `5601`, Elasticsearch API on `9200`. Config: `docker/monitoring/filebeat/filebeat.yml`.