# Data Files

Formats, producers, and consumers of the binary/JSON data files scattered through the repo.

## `robot/prm_roadmap.bin` — the PRM roadmap

The taught-waypoint roadmap the planner searches. Current file: 686 nodes, 6 DOF (117 KB).

**Binary layout** (little-endian):

```
uint64  n            # number of nodes
uint64  dof          # joints per node (6)
repeat n times:
    float64[dof]     # joint values (radians)
    uint64  k        # neighbor count
    int32[k]         # neighbor node indices
```

| Written by | Read by |
|---|---|
| `build_prm_roadmap()` in `global_planner/src/crrt_prm.hpp` (C++, on first plan if missing) | `load_prm_roadmap()` in `crrt_plan.hpp` / `constrained_rrt.hpp` |
| `robot/add_waypoints_to_roadmap.py` (broken — known issue #4) | `global_planner/src/visualize_roadmap.py` |
| `global_planner/src/inject_waypoints_node.py` | |

⚠ The C++ code reads it at the **container** path `/home/robot/AutoLab/robot/prm_roadmap.bin`; `visualize_roadmap.py` at the **host** path `/home/labx/coding/Autolab/robot/prm_roadmap.bin`.

## `robot/manual_waypoints_0.json` — taught waypoints

686 hand-taught arm configurations. Schema per entry:

```json
{
  "index": 0,
  "angles_deg": [0, 0, 0, 0, 0, 0],
  "angles_rad": [0, 0, 0, 0, 0, 0],
  "tcp_mm": [x, y, z, r, p, y],
  "note": "optional operator note"
}
```

Produced by `record_waypoints.py` / `record_waypoints_ros2.py` (see [Planning](../subsystems/planning.md#the-taught-waypoint-roadmap-toolchain)); consumed by the injectors above and by `crrt_prm.hpp::load_manual_waypoints` (roadmap Phase 1 + RRT waypoint biasing).

## Point-cloud caches (repo root, ⚠ machine-local)

| File | Shape | Meaning |
|---|---|---|
| `pc3_accumulated.npy` | `(≈2900, 3) float32` | Accumulated ZED cloud (10–15 frames) |
| `vlp16_accumulated.npy` | `(≈15000, 3) float32` | Accumulated Velodyne cloud |

Written and re-read on startup by `pc3.py`/`pc4.py`/`pointcloud_lidar*.py` (`USE_PC_CACHE = True`) so the bridges don't need to re-accumulate after a restart. Safe to delete; they regenerate. `.bkp` copies sit beside them.

## `mobility/transform_annotated.npy` (⚠ machine-local, irreplaceable-ish)

The 4×4 float64 ZED→VLP-16 extrinsic used by `pointcloud_lidar*.py` (loaded via the **relative** path `./mobility/transform_annotated.npy` — run the scripts from the repo root). **No tool in this repo produces it**; it came from an external calibration. A `_bkp` copy exists — don't lose these.

## Historical / orphaned files

| File | Status |
|---|---|
| `robot/prm_roadmap_old.bin` (4.5 MB) | Old Halton-heavy roadmap; unused |
| `robot/manual_waypoints_{old,old2,huge,v1,0_old}.json` | Snapshots; unused |
| `robot/bottleneck_waypoints.json` (1852 wps) | Referenced by **no code** — orphaned |
| `pointcloud_lidar.py.bkp`, `*.npy.bkp` | Manual backups |

## Where the paths point

Three path conventions coexist — check which one your tool uses before wondering why a file "doesn't exist":

| Convention | Used by |
|---|---|
| `/home/robot/AutoLab/robot/...` (in-container) | C++ planner headers, `inject_waypoints_node.py`, `record_waypoints.py`, `add_waypoints_to_roadmap.py`, `robot.launch.xml` (domain_bridge params) |
| `/home/labx/coding/Autolab/...` (this host) | `visualize_roadmap.py` |
| `./...` relative to CWD | `pointcloud_lidar*.py` (`./mobility/...`, `./pc3_accumulated.npy`) |
