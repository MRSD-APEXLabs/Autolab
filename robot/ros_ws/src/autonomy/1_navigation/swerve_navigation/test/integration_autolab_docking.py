#!/usr/bin/env python3
"""Exercise default Autolab navigation and /nav API in an isolated simulation.

Run inside a network-isolated Humble test container with the packages built,
the fixed tf2 library loaded, and SWERVE_NAV_FORCE_SIM=1. Never starts hardware.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--undock-only', action='store_true')
    args = parser.parse_args()
    assert os.environ.get('SWERVE_NAV_FORCE_SIM') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') in ('78', '79')
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from geometry_msgs.msg import PoseStamped, Twist
    from std_msgs.msg import Empty, String
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    import yaml

    output = Path(tempfile.mkdtemp(prefix='autolab_station_it_'))
    places = yaml.safe_load((Path(get_package_share_directory('navigation_bringup')) /
                             'maps/best.locations.yaml').read_text())['locations']
    rclpy.init()
    node = rclpy.create_node('autolab_station_regression')
    state = {'pose': None, 'api': '', 'mission': {}, 'parking_yaw': 0.}
    history, premature = [], []
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def mission(msg):
        state['mission'] = json.loads(msg.data)
        print(msg.data, flush=True)

    def api(msg):
        state['api'] = msg.data
        history.append(msg.data)
        if msg.data == 'SUCCEEDED' and state['pose'] is not None and state.get('expected'):
            p = places[state['expected']]
            if math.hypot(p['x']-state['pose'][0], p['y']-state['pose'][1]) > .25:
                premature.append(state['pose'])

    def pose(msg):
        state['pose'] = (msg.pose.position.x, msg.pose.position.y)

    def cmd(msg):
        if state['mission'].get('state') in ('dock', 'undock', 'stage_adjust'):
            state['parking_yaw'] = max(state['parking_yaw'], abs(msg.angular.z))

    node.create_subscription(String, '/station_docking/result', mission, latched)
    node.create_subscription(String, '/nav/state', api, latched)
    node.create_subscription(PoseStamped, '/sim_world/ground_truth', pose, 10)
    node.create_subscription(Twist, '/cmd_vel', cmd, 10)
    goals = node.create_publisher(String, '/nav/goal_location', 1)
    undock = node.create_publisher(Empty, '/station_undock', 1)
    process = None

    def wait(condition, timeout):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if condition():
                return True
            if process and process.poll() is not None:
                raise RuntimeError('simulation launch exited')
            rclpy.spin_once(node, timeout_sec=.02)
        return condition()

    report = {'passed': False, 'output': str(output)}
    try:
        wait(lambda: False, 2.)
        foreign = set(node.get_node_names())-{node.get_name()}
        assert not foreign, f'occupied simulation domain: {foreign}'
        p = places['OT-2']
        with (output/'launch.log').open('w') as log:
            process = subprocess.Popen([
                'ros2', 'launch', str(Path(__file__).with_name('kinematic.launch.py')),
                'hardware:=false', 'joy:=false', 'rviz:=false',
                f'x:={p["x"]}', f'y:={p["y"]}', f'yaw:={p["yaw"]}'],
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        assert wait(lambda: state['pose'] is not None and state['api'] == 'IDLE', 45), 'startup timeout'
        wait(lambda: False, 8.)
        for name in (() if args.undock_only else ('shaker', 'home', 'OT-2')):
            state['expected'] = name
            offset = len(history)
            goals.publish(String(data=name))
            assert wait(lambda: any(s in ('SUCCEEDED', 'FAILED', 'CANCELED')
                                    for s in history[offset:]), 120), f'{name} timed out'
            assert state['api'] == 'SUCCEEDED', f'{name}: {state}'
            wait(lambda: False, .2)
            assert state['mission'].get('docked') is True, state
            assert not premature, f'API reported staging as final arrival: {premature}'
            p = places[name]
            error = math.hypot(p['x']-state['pose'][0], p['y']-state['pose'][1])
            assert error < .20, f'{name}: truth error {error}'
            report[name] = {'truth_position_error_m': error}
        state['expected'] = None
        state['mission'] = {}
        undock.publish(Empty())
        assert wait(lambda: state['mission'].get('state') in ('undocked', 'stopped'), 45)
        assert state['mission'].get('success') is True
        assert wait(lambda: state['api'] == 'SUCCEEDED', 3.), 'undock API outcome not delivered'
        assert state['parking_yaw'] == 0., state
        report['passed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = str(exc)
    finally:
        if process and process.poll() is None:
            process.send_signal(signal.SIGINT)  # launch delivers one signal to each child
            try:
                process.wait(timeout=15.)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=8.)
        node.destroy_node()
        rclpy.try_shutdown()
        log = (output/'launch.log').read_text() if (output/'launch.log').exists() else ''
        report.update(collisions=log.count('COLLISION:'), api_states=history)
        if report['collisions']:
            report['passed'] = False
        (output/'report.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
