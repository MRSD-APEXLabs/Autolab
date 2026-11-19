#!/usr/bin/env python3
"""`ros2 launch` that Ctrl-C cannot leave hanging. Same arguments: ros2_launch.py <package> <launch file> [args].

Plain `ros2 launch` keeps Python's default SIGINT handler, so every Ctrl-C also raises KeyboardInterrupt wherever
launch's event loop happens to be. During shutdown the loop is busy recording 30 processes exiting, and a second
Ctrl-C (or a key repeat) can kill the task that logs one process's exit and marks it done. launch then waits for that
process forever: it never prints "process has died" for it, ignores further Ctrl-C, and has to be killed. launch
does not need the KeyboardInterrupt: it already handles SIGINT through the signal wakeup fd (AsyncSafeSignalManager).
So while its event loop runs, this handler does nothing; before and after the loop, Ctrl-C still interrupts as usual.
"""
import asyncio
import signal
import sys


def _on_sigint(signum, frame):
    try:
        asyncio.get_running_loop()
    except RuntimeError:            # no event loop running: interrupt startup / teardown as usual
        raise KeyboardInterrupt from None


signal.signal(signal.SIGINT, _on_sigint)

from ros2cli.cli import main  # noqa: E402

sys.exit(main(argv=['launch'] + sys.argv[1:]))
