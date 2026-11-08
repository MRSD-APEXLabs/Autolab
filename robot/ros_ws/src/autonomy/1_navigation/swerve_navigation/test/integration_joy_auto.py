#!/usr/bin/env python3
"""Check the SDL reader on private topics in a network-isolated test container.

Requires the controller input device to be mapped into the container. Starts
only joy_auto: no teleop, velocity mux, hardware bridge or navigation nodes.
"""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '79'
    assert os.environ.get('SWERVE_NAV_FORCE_SIM') == '1'
    import rclpy
    from sensor_msgs.msg import Joy
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy
    output = Path(tempfile.mkdtemp(prefix='wireless_reader_it_'))
    rclpy.init()
    node = rclpy.create_node('wireless_reader_probe')
    samples, devices = [], []
    node.create_subscription(Joy, '/wireless_probe/joy',
                             lambda msg: samples.append((time.monotonic(), msg)), 20)
    node.create_subscription(String, '/wireless_probe/selected_device',
                             lambda msg: devices.append(msg.data),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    with (output/'reader.log').open('w') as log:
        process = subprocess.Popen([
            sys.executable, '-m', 'swerve_navigation.joy_auto', '--ros-args',
            '-r', 'joy:=/wireless_probe/joy',
            '-r', 'joy/selected_device:=/wireless_probe/selected_device'],
            stdout=log, stderr=subprocess.STDOUT)
    try:
        deadline = time.monotonic()+10.
        while time.monotonic() < deadline and (len(samples) < 20 or not devices):
            rclpy.spin_once(node, timeout_sec=.05)
            assert process.poll() is None, (output/'reader.log').read_text()
        assert devices[-1:] == ['Xbox One S Controller'], devices
        assert len(samples) >= 20
        assert all(len(msg.axes) == 8 and len(msg.buttons) == 11 for _, msg in samples)
        hz = (len(samples)-1)/(samples[-1][0]-samples[0][0])
        assert 10. <= hz <= 30., hz
        report = dict(passed=True, controller=devices[-1], samples=len(samples),
                      rate_hz=round(hz, 2), output=str(output))
        (output/'report.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report), flush=True)
    finally:
        process.send_signal(signal.SIGINT)
        process.wait(timeout=5.)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
