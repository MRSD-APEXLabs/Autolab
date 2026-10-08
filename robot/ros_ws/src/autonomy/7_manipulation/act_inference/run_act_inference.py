#!/usr/bin/env python3
"""Live ACT inference on the xArm6 from the ZED X Nano wrist camera.

    python run_act_inference.py --ckpt-dir data/act_ckpts/<run>              # prediction only
    python run_act_inference.py --ckpt-dir data/act_ckpts/<run> --execute    # moves the arm

See ../README.md. The __main__ guard matters: the neural depth backend starts its GPU worker
with multiprocessing's "spawn" method, which re-imports this script.
"""
from act_inference.cli import main

if __name__ == '__main__':
    main()
