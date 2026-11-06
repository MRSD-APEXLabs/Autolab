#!/usr/bin/env bash
# Same checkpoint and rollout settings as apple_server_manip/scripts/place_on_opentrons_ft.sh.
# Defaults to prediction only; pass --execute to move the arm. Extra arguments override defaults.
set -euo pipefail
cd "$(dirname "$0")"
OMP_NUM_THREADS=2 exec python3 run_act_inference.py \
  --ckpt-dir models/xarm6_placing_act_ft_all_20261105b \
  --duration-sec 60 --translation-speed 0.01 --workspace-radius 0.15 \
  --enable-rotation --rotation-speed 0.15 \
  "$@"
