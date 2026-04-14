#!/usr/bin/env python3
"""
CLI for sending a routine to the RoutineExecutorNode and monitoring progress.

Usage:
    python3 run_routine.py [--robot ROBOT] [--retries N] STEP [STEP ...]

Steps accept parameters using colon-separated key=value pairs:
    pick_base
    place:target_machine=ot2
    shaker:pwm=200
    ot2:parameters_json='{"steps":[]}'

Examples:
    python3 run_routine.py pick_base "place:target_machine=ot2" ot2 pick_up "shaker:pwm=200"
    python3 run_routine.py --robot robot_1 --retries 3 pick_base "place:target_machine=shaker"
"""

import argparse
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from routine_executor.step_config import STEPS, KNOWN_STEPS


def parse_step(arg: str) -> dict:
    """Parse 'name' or 'name:key=val,key2=val2' into a step dict."""
    if ':' not in arg:
        return {'name': arg}
    name, params_str = arg.split(':', 1)
    params = {'name': name}
    for pair in params_str.split(','):
        if '=' not in pair:
            raise ValueError(f"Invalid param '{pair}' in step arg '{arg}': expected key=value")
        k, v = pair.split('=', 1)
        k = k.strip()
        v = v.strip()
        # auto-cast to int or float when possible
        try:
            v = int(v)
        except ValueError:
            try:
                v = float(v)
            except ValueError:
                pass
        params[k] = v
    return params


def format_step(step: dict) -> str:
    """Format a step dict as a human-readable string."""
    name = step['name']
    extras = {k: v for k, v in step.items() if k != 'name'}
    if not extras:
        return name
    param_str = ','.join(f'{k}={v}' for k, v in extras.items())
    return f'{name}({param_str})'


class RoutineCLI(Node):
    def __init__(self, robot: str, steps: list, max_retries: int):
        super().__init__('routine_cli')
        self._steps = steps  # list[dict]
        self._done = False
        self._exit_code = 0

        self._last_step_idx = -1
        self._last_retry_count = 0
        self._last_state = None
        self._command_sent = False

        self._pub = self.create_publisher(String, '/routine_executor/start_routine_cmd', 10)
        self.create_subscription(String, '/routine_executor/status', self._on_status, 10)

        # Wait 500ms before publishing (allow subscription to connect)
        self._robot = robot
        self._max_retries = max_retries
        self._send_timer = self.create_timer(0.5, self._send_command)

    def _send_command(self) -> None:
        payload = json.dumps({
            'robot': self._robot,
            'steps': self._steps,
            'max_retries': self._max_retries,
        })
        msg = String()
        msg.data = payload
        self._pub.publish(msg)
        self._command_sent = True
        self.get_logger().debug(f'Sent routine: {payload}')
        self._send_timer.cancel()  # one-shot: don't re-publish

    def _on_status(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        if not self._command_sent:
            return

        state = d.get('state', 'idle')
        step_idx = d.get('current_step', 0)
        step_name = d.get('current_step_name')
        retry_count = d.get('retry_count', 0)
        max_retries = d.get('max_retries', 3)
        total = len(d.get('steps', []))
        error = d.get('error')

        if state == 'idle':
            return  # not started yet

        # Print a status line when step or state changes
        if state == 'running' and step_name:
            if step_idx != self._last_step_idx or retry_count != self._last_retry_count:
                label = f'[{step_idx + 1}/{total}] {step_name}'
                if retry_count > 0:
                    print(f'{label:<30} FAILURE (retry {retry_count}/{max_retries})')
                else:
                    print(f'{label:<30} RUNNING')
                self._last_step_idx = step_idx
                self._last_retry_count = retry_count

        elif state == 'success' and self._last_state != 'success':
            steps = d.get('steps', [])
            total_steps = len(steps)
            last_step = steps[-1].get('name', '?') if steps else '?'
            label = f'[{total_steps}/{total_steps}] {last_step}'
            print(f'{label:<30} SUCCESS')
            print('Routine complete.')
            self._done = True
            self._exit_code = 0

        elif state == 'failed' and self._last_state != 'failed':
            if error == 'cancelled':
                print('Routine cancelled.')
            else:
                print(f'Routine FAILED: {error}')
            self._done = True
            self._exit_code = 1

        self._last_state = state


def main():
    parser = argparse.ArgumentParser(
        description='Run a robot routine via the behavior tree',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Step format: name or name:key=value,key2=value2\n'
            f'Known steps: {KNOWN_STEPS}\n'
            'Examples:\n'
            '  pick_base "place:target_machine=ot2" "shaker:pwm=200" ot2\n'
        ),
    )
    parser.add_argument('steps', nargs='+', help='Ordered steps with optional params')
    parser.add_argument('--robot', default='robot_1', help='Robot namespace (default: robot_1)')
    parser.add_argument('--retries', type=int, default=3, help='Max retries per step (default: 3)')
    args = parser.parse_args()

    # Parse each step arg into a dict
    parsed_steps = []
    for arg in args.steps:
        try:
            parsed_steps.append(parse_step(arg))
        except ValueError as e:
            print(f'ERROR: {e}', file=sys.stderr)
            sys.exit(1)

    # Validate step names
    unknown = [s['name'] for s in parsed_steps if s['name'] not in KNOWN_STEPS]
    if unknown:
        print(f'ERROR: Unknown steps: {unknown}', file=sys.stderr)
        print(f'Known steps: {KNOWN_STEPS}', file=sys.stderr)
        sys.exit(1)

    # Validate required params before connecting to ROS2
    for step in parsed_steps:
        config = STEPS[step['name']]
        missing = [p for p in config['required_params'] if p not in step]
        if missing:
            print(f'ERROR: Step "{step["name"]}" missing required params: {missing}', file=sys.stderr)
            sys.exit(1)

    rclpy.init()
    node = RoutineCLI(robot=args.robot, steps=parsed_steps, max_retries=args.retries)

    step_summary = ' -> '.join(format_step(s) for s in parsed_steps)
    print(f'Running routine: {step_summary}')
    print(f'Robot: {args.robot}  |  Max retries: {args.retries}')
    print()

    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node._exit_code = 1
        print('\nInterrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(node._exit_code)


if __name__ == '__main__':
    main()
