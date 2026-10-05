"""Keep Nav2's FollowPath action in the local-costmap frame.

Nav2's planner correctly works in ``map``.  The Humble controller_server on this
robot can, however, stop updating the AMCL ``map -> odom`` edge in its private TF
buffer.  A map-framed plan then fails before a velocity is produced even though
the transform is healthy everywhere else.

This node is a deliberately small action proxy.  It receives the map-framed
FollowPath goal from bt_navigator, transforms one copy with its own TF listener,
and forwards an odom-framed goal to controller_server.  Planning, path validity
checks, and RViz remain in map; only the controller-facing copy changes frame.
"""

from __future__ import annotations

import copy
from threading import Lock
from typing import Tuple

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


Quaternion = Tuple[float, float, float, float]


def _multiply(a: Quaternion, b: Quaternion) -> Quaternion:
    """Hamilton product, with ROS quaternion ordering (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate(q: Quaternion, point: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Rotate a vector by q without depending on tf_transformations."""
    x, y, z, w = q
    px, py, pz = point
    # q * p * conjugate(q), expanded to avoid temporary message objects.
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    return (
        px + w * tx + y * tz - z * ty,
        py + w * ty + z * tx - x * tz,
        pz + w * tz + x * ty - y * tx,
    )


def transform_path(path, transform, target_frame: str):
    """Return a deep-copied Path transformed by one coherent latest transform."""
    result = copy.deepcopy(path)
    translation = transform.transform.translation
    rotation_msg = transform.transform.rotation
    rotation = (rotation_msg.x, rotation_msg.y, rotation_msg.z, rotation_msg.w)

    result.header.frame_id = target_frame
    for pose in result.poses:
        point = pose.pose.position
        rx, ry, rz = _rotate(rotation, (point.x, point.y, point.z))
        point.x = rx + translation.x
        point.y = ry + translation.y
        point.z = rz + translation.z

        orientation = pose.pose.orientation
        ox, oy, oz, ow = _multiply(
            rotation, (orientation.x, orientation.y, orientation.z, orientation.w))
        orientation.x, orientation.y, orientation.z, orientation.w = ox, oy, oz, ow
        pose.header.frame_id = target_frame
        # Controller plugins ask TF at each control-cycle time.  A route is not a
        # historical observation, so zero means "latest" and avoids extrapolation.
        pose.header.stamp.sec = 0
        pose.header.stamp.nanosec = 0
    return result


class FollowPathAdapter(Node):
    def __init__(self) -> None:
        super().__init__('follow_path_adapter')
        self._target_frame = self.declare_parameter('target_frame', 'odom').value
        self._source_action = self.declare_parameter('source_action', '/follow_path').value
        self._target_action = self.declare_parameter(
            'target_action', '/follow_path_internal').value

        group = ReentrantCallbackGroup()
        self._backend_goals = {}
        self._backend_goals_lock = Lock()
        self._tf = Buffer(cache_time=Duration(seconds=30.0), node=self)
        # This node already uses a multithreaded executor.  The listener's
        # reentrant callback group is therefore serviced without a second
        # executor trying to own the same node.
        self._listener = TransformListener(self._tf, self, spin_thread=False)
        self._client = ActionClient(
            self, FollowPath, self._target_action, callback_group=group)
        self._server = ActionServer(
            self, FollowPath, self._source_action,
            execute_callback=self._execute,
            goal_callback=self._accept_goal,
            cancel_callback=self._cancel_goal,
            callback_group=group)
        self.get_logger().info(
            f'adapting {self._source_action} -> {self._target_action}; '
            f'controller plans use frame {self._target_frame!r}')

    @staticmethod
    def _goal_key(goal_handle) -> bytes:
        return bytes(goal_handle.goal_id.uuid)

    def _accept_goal(self, _request):
        # Do not accept work that cannot be forwarded. In particular, never park an
        # executor worker waiting synchronously for the controller action to appear.
        if not self._client.server_is_ready():
            self.get_logger().error(
                f'controller action {self._target_action!r} is not available')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_goal(self, goal_handle):
        # execute_callback awaits ROS futures and therefore yields its executor worker.
        # Forward cancellation from this independent callback as soon as a backend goal
        # exists. If acceptance is still in flight, _execute observes the cancel flag
        # immediately after it receives the backend handle.
        with self._backend_goals_lock:
            backend = self._backend_goals.get(self._goal_key(goal_handle))
        if backend is not None:
            backend.cancel_goal_async()
        return CancelResponse.ACCEPT

    def _controller_path(self, path):
        source_frame = path.header.frame_id
        if not source_frame and path.poses:
            source_frame = path.poses[0].header.frame_id
        if not source_frame:
            raise TransformException('FollowPath goal has no path frame')

        for pose in path.poses:
            pose_frame = pose.header.frame_id or source_frame
            if pose_frame != source_frame:
                raise TransformException(
                    f'FollowPath mixes frames {source_frame!r} and {pose_frame!r}')

        if source_frame == self._target_frame:
            transformed = copy.deepcopy(path)
            transformed.header.frame_id = self._target_frame
            for pose in transformed.poses:
                pose.header.frame_id = self._target_frame
                pose.header.stamp.sec = 0
                pose.header.stamp.nanosec = 0
            return transformed

        transform = self._tf.lookup_transform(
            self._target_frame, source_frame, Time(), timeout=Duration(seconds=0.5))
        return transform_path(path, transform, self._target_frame)

    async def _execute(self, goal_handle):
        result = FollowPath.Result()
        try:
            path = self._controller_path(goal_handle.request.path)
        except TransformException as exc:
            self.get_logger().error(f'cannot adapt FollowPath goal: {exc}')
            if hasattr(result, 'error_code'):
                result.error_code = FollowPath.Result.TF_ERROR
                result.error_msg = str(exc)
            goal_handle.abort()
            return result

        forwarded = FollowPath.Goal()
        forwarded.path = path
        for field in ('controller_id', 'goal_checker_id', 'progress_checker_id'):
            if hasattr(forwarded, field) and hasattr(goal_handle.request, field):
                setattr(forwarded, field, getattr(goal_handle.request, field))

        def feedback(message) -> None:
            if goal_handle.is_active:
                goal_handle.publish_feedback(message.feedback)

        # Awaiting an rclpy Future yields the executor worker. The old polling loops
        # slept inside each callback; three overlapping Nav2 recovery goals occupied
        # all three workers and prevented these same futures from ever completing.
        backend = await self._client.send_goal_async(
            forwarded, feedback_callback=feedback)
        if not backend.accepted:
            self.get_logger().error('controller_server rejected the adapted FollowPath goal')
            goal_handle.abort()
            return result

        key = self._goal_key(goal_handle)
        with self._backend_goals_lock:
            self._backend_goals[key] = backend
        try:
            if goal_handle.is_cancel_requested:
                backend.cancel_goal_async()
            response = await backend.get_result_async()
        finally:
            with self._backend_goals_lock:
                self._backend_goals.pop(key, None)
        if goal_handle.is_cancel_requested or response.status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
        elif response.status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return response.result

    def destroy_node(self) -> None:
        self._server.destroy()
        self._client.destroy()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowPathAdapter()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
