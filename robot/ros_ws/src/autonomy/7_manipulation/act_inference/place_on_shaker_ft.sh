#!/usr/bin/env bash
# Shaker placing policy with the same rollout settings as apple_server_manip/scripts/place_on_shaker_ft.sh.
# Start above the shaker clamp at yaw ~90 degrees, as /act_inference/start/shaker does after plan_april_22.
# Defaults to prediction only; pass --execute to move the arm. Extra arguments override defaults.
set -euo pipefail
cd "$(dirname "$0")"
OMP_NUM_THREADS=2 exec python3 run_act_inference.py \
  --ckpt-dir models/xarm6_shaker_act_ft_20261108c \
  --duration-sec 60 --translation-speed 0.01 --workspace-radius 0.15 \
  --enable-rotation --rotation-speed 0.15 \
  "$@"
