#!/usr/bin/env bash
# Base-deck placing policy with the same rollout settings as apple_server_manip/scripts/place_on_base_ft.sh.
# Start at the base start pose by hand (get_position 32.06, 271.17, 116.80 mm, RPY -179.68, -1.57, -0.74), where
# /act_inference/start/base moves first. Defaults to prediction only; pass --execute to move the arm. Extra
# arguments override defaults.
set -euo pipefail
cd "$(dirname "$0")"
OMP_NUM_THREADS=2 exec python3 run_act_inference.py \
  --ckpt-dir models/xarm6_base_act_ft_20261108b \
  --duration-sec 60 --translation-speed 0.01 --workspace-radius 0.15 \
  --enable-rotation --rotation-speed 0.15 \
  "$@"
