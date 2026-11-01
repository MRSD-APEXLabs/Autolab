#!/usr/bin/env python3
"""ROS 2 node for ACT placing rollouts, driven by manipulation_executive through /act_inference/*.

    python run_act_node.py                                   # moves the arm when a run is started
    python run_act_node.py --ros-args -p execute:=false      # prediction only

Started by manipulation_bringup (manipulation.launch.xml). See ../README.md. The __main__ guard
matters: the neural depth backend starts its GPU worker with multiprocessing's "spawn" method,
which re-imports this script.
"""
from act_inference.node import main

if __name__ == '__main__':
    main()
