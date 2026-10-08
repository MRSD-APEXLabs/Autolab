"""ROS 2 bridge to the Xavier camera hub, which streams the ZED X or the ZED X Nano, one at a time.

Node `camera_hub` (namespace / in the launch file):

  ~/status          std_msgs/String   the hub's GET /status JSON plus "reachable", "poll_error" and
                                      "hub_url"; published after every poll and every switch (reliable,
                                      transient local: a late subscriber gets the latest one)
  ~/select_<camera> std_srvs/Trigger  one per entry of `cameras` (select_zedx, select_zedx_nano)
  ~/release         std_srvs/Trigger  stop all cameras

A switch is a POST /select that blocks until the camera streams, fails or `select_timeout`
expires. The services run concurrently, so a release or another selection reaches the hub while
a switch still waits (the hub answers the superseded one with 409), and polling runs on its own
thread, so the status keeps flowing meanwhile. The camera nodes need no switching: each one waits
quietly while its camera is not the active one and resumes when it is selected.
"""
from functools import partial
import json
import re
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .stream import CameraHubClient

STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def describe(status):
    """One-line summary of a hub status, e.g. 'zedx streaming' or 'zedx_nano error: sensor stalled'."""
    text = f'{status.get("active") or "no camera"} {status.get("state")}'
    return f'{text}: {status["error"]}' if status.get('error') else text


class CameraHubNode(Node):
    def __init__(self, **kwargs):
        # kwargs go to rclpy Node (namespace, parameter_overrides, context, ...) for embedding and tests
        super().__init__('camera_hub', **kwargs)
        host = self.declare_parameter('host', '192.168.1.101').value
        port = int(self.declare_parameter('port', 8090).value)
        self.cameras = list(self.declare_parameter('cameras', ['zedx', 'zedx_nano']).value)
        self.poll_period = float(self.declare_parameter('poll_period', 1.0).value)
        timeout = float(self.declare_parameter('timeout', 3.0).value)
        self.select_timeout = float(self.declare_parameter('select_timeout', 30.0).value)
        for camera in self.cameras:
            if not re.fullmatch(r'\w+', camera):
                raise ValueError(f'cameras must be hub camera names such as zedx or zedx_nano, got {camera!r}')

        self.client = CameraHubClient(host, port, timeout)
        self.pub_status = self.create_publisher(String, '~/status', STATUS_QOS)
        group = ReentrantCallbackGroup()   # with main's MultiThreadedExecutor, a release need not wait for a switch
        for camera in self.cameras:
            self.create_service(Trigger, f'~/select_{camera}', partial(self._on_select, camera), callback_group=group)
        self.create_service(Trigger, '~/release', partial(self._on_select, None), callback_group=group)
        self._lock = threading.Lock()   # one poll at a time, so a switch's status is published after older polls
        self._logged = object()         # equals no key, so the first poll is always logged (even unreachable)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name='camera-hub-poll', daemon=True)
        self._thread.start()

    # -- status -------------------------------------------------------------------------------
    def _run(self):
        while not self._stop.is_set():
            try:
                self.poll()
            except Exception as exc:
                if self._stop.is_set() or not self.context.ok():
                    break   # shutting down (Ctrl-C shuts the context down before stop() runs)
                self.get_logger().error(f'Publishing the hub status failed: {exc}', throttle_duration_sec=30.0)
            self._stop.wait(self.poll_period)

    def poll(self):
        """GET /status and publish it on ~/status; returns the published dict."""
        with self._lock:
            try:
                status = dict(self.client.status(), reachable=True, poll_error=None)
            except Exception as exc:
                status = {'reachable': False, 'poll_error': str(exc)}
            status['hub_url'] = self.client.url('/')
            self.pub_status.publish(String(data=json.dumps(status)))
            self._log_change(status)
        return status

    def _log_change(self, status):
        """Log when the hub becomes (un)reachable or its active camera, state or error changes."""
        key = (status.get('active'), status.get('state'), status.get('error')) if status['reachable'] else None
        if key == self._logged:
            return
        self._logged = key
        if key is None:
            self.get_logger().warning(f'Camera hub {status["hub_url"]} unreachable: {status["poll_error"]}; retrying')
        elif status.get('state') == 'error':
            self.get_logger().warning(f'Camera hub: {describe(status)}')
        else:
            self.get_logger().info(f'Camera hub: {describe(status)}')

    # -- switching ----------------------------------------------------------------------------
    def _on_select(self, camera, request, response):
        self.get_logger().info(f'Selecting {camera}' if camera else 'Releasing all cameras')
        reply = self._select(camera)
        if reply is None:
            response.success, response.message = False, 'camera_hub node is shutting down'
            return response
        if isinstance(reply, Exception):
            response.success, response.message = False, f'Camera hub {self.client.url("/")}: {reply}'
        else:
            response.success = bool(reply.get('ok'))
            if response.success:
                response.message = describe(reply.get('status') or {})
            else:
                response.message = reply.get('error') or 'select failed'
        if not response.success:
            self.get_logger().error(f'{"Selecting " + camera if camera else "Release"} failed: {response.message}')
        self.poll()   # the new state is published before the caller gets the reply
        return response

    def _select(self, camera):
        """POST /select; returns the reply, the exception it raised, or None if the node stops first.

        The request runs on a daemon thread: a switch can block for `select_timeout`, and the
        executor's own threads are joined at exit, so Ctrl-C would otherwise wait for the hub.
        """
        result = []

        def post():
            try:
                result.append(self.client.select(camera, self.select_timeout))
            except Exception as exc:
                result.append(exc)
        worker = threading.Thread(target=post, name='camera-hub-select', daemon=True)
        worker.start()
        while worker.is_alive() and not self._stop.is_set():
            worker.join(0.1)
        return result[0] if result else None

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=max(2.0, self.client.timeout + 1.0))


def main(args=None):
    rclpy.init(args=args)
    node = CameraHubNode()
    executor = MultiThreadedExecutor()   # the switch services block, each on an executor thread
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()                      # also ends the services still waiting for the hub
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
