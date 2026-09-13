# Setup

## Prerequisites

- Linux host (the lab robot runs an NVIDIA Jetson, L4T r36.4 / JetPack 6; development also works on x86-64)
- Docker + docker compose (the `autolab install` command can install these)
- X11 if you want RViz/rqt GUIs out of the containers
- Node 20 / npm only if you want to run the web UI *outside* Docker

```bash
git clone git@github.com:MRSD-APEXLabs/Autolab.git
cd Autolab
./autolab setup      # installs an `autolab` shell function into your .bashrc/.zshrc
autolab install      # first time only: installs docker engine etc.
```

## The `autolab` CLI

`autolab.sh` (~830 lines of bash) is a thin wrapper around docker compose plus quality-of-life commands. Full reference: `.autolab/README.md`; commands are registered in `autolab.sh` (line ~714) and the plugin modules `.autolab/modules/config.sh` and `.autolab/modules/dev.sh`.

| Command | What it does |
|---|---|
| `autolab setup` | Install the `autolab` shell function |
| `autolab install` | Install system dependencies (docker, …) |
| `autolab up [service]` | `USER_ID=$(id -u) GROUP_ID=$(id -g) docker compose -f docker-compose.yaml up <args> -d` |
| `autolab up robot` | Start only the robot container |
| `autolab up --build` | Rebuild images first |
| `autolab down [service]` | Stop containers (`--profile '*'`) |
| `autolab connect <container>` | `docker exec -it` into a container (fuzzy-matches the name) |
| `autolab status` | Show container status |
| `autolab logs <container>` | Tail container logs |
| `autolab config` | Config module |
| `autolab test` / `lint` / `format` | ⚠ stubs — they only `echo` (see [known issue #16](known-issues.md)) |

!!! warning "`autolab up` does NOT start the GCS or UI"
    The root `docker-compose.yaml` `include:`s only `robot/docker/docker-compose.yaml`. The GCS/Mosquitto/UI (`gcs/docker/docker-compose.yaml`), Isaac Sim, and docs includes are **commented out**. To get them: **GCS/UI** — un-comment `- gcs/docker/docker-compose.yaml` under `include:` in the root `docker-compose.yaml`, then `autolab up gcs` / `autolab up ui` (after which plain `autolab up` starts them too). **Docs / Isaac Sim** — run standalone: `docker compose --env-file .env -f docs/docker/docker-compose.yaml up docs`. ([known issue #7](known-issues.md))

    **Why not just `docker compose -f gcs/docker/docker-compose.yaml`?** Two reasons. (1) Compose only auto-loads the `.env` *next to the compose file you pass with `-f`*, but every component file interpolates `PROJECT_NAME`, `DOCKER_IMAGE_TAG`, `UI_PORT`, `AUTOLAUNCH`, … from the **root** `.env` — without it you get `unable to get image '/:v_mkdocs': invalid reference format`. `--env-file .env` fixes that for docs and Isaac Sim. (2) The GCS file additionally attaches its services to `autolab_network`, which only the root compose defines, so standalone it fails with `refers to undefined network autolab_network` no matter what — it must be included from the root. Always run compose commands from the repo root.

## Compose services and profiles

| Compose file | Services | Profile | Purpose |
|---|---|---|---|
| `robot/docker/docker-compose.yaml` | `robot` | `""` / `sitl` | x86 dev/sim robot container |
| | `robot-l4t` | `hitl` / `deploy` | Jetson deployment (`network_mode: host`) |
| | `zed-l4t` | `hitl` / `deploy` | Dual-ZED driver container |
| | `robot-test` | `test` | Runs `colcon test` for `behaviour_executive` |
| `gcs/docker/docker-compose.yaml` | `gcs` | `""` / `sitl` | GCS container (Mosquitto auto-starts inside) |
| | `gcs-real` | `hitl` / `deploy` | GCS with host networking |
| | `elasticsearch`, `kibana`, `filebeat` | | Log monitoring stack |
| | `ui` | | Production UI (nginx, port 80) |
| | `ui-dev` | `ui-dev` | Hot-reload UI dev server (port 3000) |
| `simulation/isaac-sim/docker/docker-compose.yaml` | `isaac-sim`, `isaac-sim-gui` | | Isaac Sim |
| `docs/docker/docker-compose.yaml` | `docs` | | This documentation site (port 8000) |

With `AUTOLAUNCH=true` (the default in `.env`), the robot container starts a tmux session `robot_bringup` that runs `bws && sws && ros2 launch $ROBOT_LAUNCH_PACKAGE $ROBOT_LAUNCH_FILE`; the GCS container's equivalent session is `gcs_bringup` (see `gcs/docker/gcs-base-docker-compose.yaml`). Attach with `tmux attach -t robot_bringup` inside the container to see launch output.

## The `.env` file

The root `.env` is the single source of truth for compose interpolation *and* for which launch file each autonomy stage runs. Fields you may need to touch:

| Variable | Current value | Meaning |
|---|---|---|
| `PROJECT_DOCKER_REGISTRY` / `DOCKER_IMAGE_TAG` | `docker.io/arker123` / `0.1.3` | Image naming |
| `AUTOLAUNCH` | `true` | Auto-launch the stack in tmux on container start |
| `NUM_ROBOTS` | `1` | Compose replicas of the robot container |
| `USERNAME` | `"arnavkha"` | ⚠ hardcoded; used for Jetson bag-volume mounts |
| `ROBOT_LAUNCH_PACKAGE` / `ROBOT_LAUNCH_FILE` | `robot_bringup` / `robot.launch.xml` | What the robot container launches |
| `*_LAUNCH_PACKAGE` / `*_LAUNCH_FILE` (interface, navigation, mapping, perception, local, global, planning, behavior) | see file | Per-stage launch indirection used by `autonomy.launch.xml` |
| `GLOBAL_LAUNCH_PACKAGE` | `global_bringup` | ⚠ **this package does not exist** — breaks the full launch chain ([known issue #2](known-issues.md)) |
| `ROBOT_DESCRIPTION_PACKAGE` / `ROBOT_URDF_FILE` | `iris_with_sensors_description` / `iris_…urdf` | ⚠ leftovers from a drone project; not this robot |
| `UI_PORT` / `UI_DEV_PORT` | `80` / `3000` | UI ports |

There is a second env file, `gcs/docker/.env`: `MQTT_USERNAME`/`MQTT_PASSWORD` (both blank — the broker is anonymous), `ROS_DOMAIN_ID=1`, `CAMERA1_STREAM_IP`, `ELASTIC_VERSION`.

## Lab network map

!!! warning "Two subnets are checked in"
    Launch-file defaults use `192.168.1.x` while `lab_machines.yaml` uses `192.168.10.x`. Reconcile against the actual lab network before trusting any address below. ([known issue #5](known-issues.md))

| Host | Address | Role |
|---|---|---|
| xArm6 controller | `192.168.1.236` | Arm driver target (`planning.launch.xml`, `joy_cartesian_jog.py`) |
| Xavier (camera-edge) | `192.168.1.101` wired (`xavier.autolab` = 192.168.10.7 on WiFi), login `autolab@` | Perception server `~/Autolab-Camera-Edge` (external code) — WS :8765 control, :8766 active stream, :8767 wrist/base cams; dashboard :8080 when served. See [Camera-Edge (Xavier)](subsystems/xavier.md) |
| Juan's GCS laptop | `gcs.autolab` = `192.168.10.4`, login `jpuerto@` | Where the GCS/UI and the OT-2 wrapper (`:8000`) historically ran |
| OT-2 API server | `192.168.10.4:8000` (robot host `192.168.10.9`) | `lab_machine_executive` HTTP target |
| Shaker | `192.168.10.14` (`/pwm_control/`) | `lab_machine_executive` HTTP target |

### Ports on the GCS/host machine

| Port | Service |
|---|---|
| 1883 / **9001** | Mosquitto MQTT (tcp / **websockets — the UI uses this**) |
| 80 / 3000 | Web UI (prod nginx / dev server) |
| 8000 | This docs site |
| 5601 / 9200 | Kibana / Elasticsearch |
| 2222, 2223–2243 | ssh into GCS / robot containers |
| 30000 | ZED streamer (Isaac Sim) |

Mosquitto config: `gcs/docker/monitoring/mosquitto/mosquitto.conf` — both listeners `allow_anonymous true`. No credentials or API keys exist anywhere in the system (OT-2/shaker HTTP calls are unauthenticated too).

## ROS domain IDs

Robot container: `ROS_DOMAIN_ID=0`. GCS: `gcs/docker/.env` says `1` (compose sets 0 — see [known issue #6](known-issues.md)). The two are connected by `domain_bridge` running on both sides, with the forwarded-topic lists in `gcs/ros_ws/src/gcs_bringup/config/domain_bridge.yaml` and `robot/ros_ws/src/robot_bringup/params/domain_bridge.yaml`. If a topic isn't crossing over, add it there.

## Machine-local files (not in git)

This robot's host machine carries several essential files that are **not committed**: `pointcloud_lidar.py`, `pointcloud_lidar_accumulated.py`, `pc4.py`, `joy_cartesian_jog.py`, `mobility/transform_annotated.npy` (the ZED↔VLP-16 extrinsic), and the two Python venvs (`venv/`, `.venv_jog/`). A fresh clone will not have them — copy them from the robot host or commit them. Their docs: [Perception](subsystems/perception.md), [Teleop](subsystems/teleop.md).
