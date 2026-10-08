"""point_to_goal: RViz "Publish Point" clicks (/clicked_point) -> Nav2 NavigateToPose goals.

RViz's "2D Goal Pose" tool already reaches bt_navigator through /goal_pose; this node adds the
simpler "click a point" workflow: the clicked position becomes the goal and the goal heading is
either the robot's current heading (goal_heading:=keep) or the direction from the robot to the
clicked point (goal_heading:=travel). Keeping the starting heading can require a final turn after
forward-only travel; navigation.launch.py selects the heading mode for the motion profile.

/nav_cancel (joystick X button) cancels EVERY goal on the action server, not only ours: the
operator's stop must also stop a goal that RViz or another client started. For the same reason it
also cancels the Nav2 servers that FEED NavigateToPose (follow_waypoints, navigate_through_poses,
started from the RViz Nav2 panel): with only its current NavigateToPose sub-goal cancelled,
waypoint_follower (stop_on_failure false) simply drives on to the next waypoint.

The node only talks to Nav2; it never publishes velocity commands itself.
"""
from functools import partial
import math
from typing import Callable, List, Optional, Sequence, Tuple

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PointStamped, PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker

HEADING_MODES = ('keep', 'travel')
STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}


# --------------------------------------------------------------------------------------------
# ROS-free helpers
# --------------------------------------------------------------------------------------------

def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Quaternion (x, y, z, w) of a rotation about +z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about +z, ZYX convention) of a unit quaternion, in (-pi, pi]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def transform_point(translation: Sequence[float], rotation: Sequence[float],
                    point: Sequence[float]) -> Tuple[float, float, float]:
    """Apply a TF transform (translation xyz, unit quaternion xyzw) to a point."""
    qx, qy, qz, qw = rotation
    px, py, pz = point
    # v' = v + 2w (q x v) + 2 q x (q x v)
    cx, cy, cz = qy * pz - qz * py, qz * px - qx * pz, qx * py - qy * px
    ccx, ccy, ccz = qy * cz - qz * cy, qz * cx - qx * cz, qx * cy - qy * cx
    return (px + 2.0 * (qw * cx + ccx) + translation[0],
            py + 2.0 * (qw * cy + ccy) + translation[1],
            pz + 2.0 * (qw * cz + ccz) + translation[2])


def goal_yaw(mode: str, robot_xy: Sequence[float], robot_yaw: float,
             goal_xy: Sequence[float]) -> float:
    """Heading of the goal pose: 'keep' the robot's heading or face the direction of 'travel'."""
    if mode not in HEADING_MODES:
        raise ValueError(f"goal_heading must be one of {HEADING_MODES}, got '{mode}'")
    dx, dy = goal_xy[0] - robot_xy[0], goal_xy[1] - robot_xy[1]
    # a click (almost) on the robot has no direction of travel: keep the heading
    if mode == 'keep' or math.hypot(dx, dy) < 1e-3:
        return robot_yaw
    return math.atan2(dy, dx)


class GoalSequencer:
    """Decides WHEN a goal may be sent so that replacing a goal never loses the new one.

    Nav2's bt_navigator answers a cancel with SimpleActionServer::terminate_all(), which also
    kills a goal that is already waiting in its pending slot. A new goal sent right behind the
    cancel request of the old one would therefore die together with it. So goals are strictly
    sequenced: cancel the old goal, wait until it has terminated, only then send the new one.

    `send(goal)` and `cancel()` are the two side effects; the owner reports what happened to the
    goal that was sent last through the on_* methods.
    """

    IDLE, SENDING, ACTIVE, CANCELLING = 'idle', 'sending', 'active', 'cancelling'

    def __init__(self, send: Callable[[object], None], cancel: Callable[[], None]) -> None:
        self.state = self.IDLE
        self._send = send
        self._cancel = cancel
        self._queued = None             # newest goal waiting for the old one to terminate
        self._abandoned = False         # /nav_cancel arrived while the goal was being accepted

    def request(self, goal) -> None:
        """Handle a new goal from the operator: it replaces whatever is in progress."""
        if self.state == self.IDLE:
            self._start(goal)
            return
        self._queued = goal
        if self.state == self.ACTIVE:
            self._begin_cancel()
        # SENDING: cancelled as soon as it is accepted. CANCELLING: already on its way out.

    def abandon(self) -> None:
        """Forget everything (/nav_cancel). The owner cancels all goals on the server itself."""
        self._queued = None
        # The cancel-all request may overtake a goal request that is still in flight.
        self._abandoned = self.state == self.SENDING

    def on_accepted(self) -> None:
        if self._queued is not None or self._abandoned:
            self._begin_cancel()
        else:
            self.state = self.ACTIVE

    def on_rejected(self) -> None:
        self._finish()

    def on_terminated(self) -> None:
        self._finish()

    def expire(self) -> None:
        """Give up waiting for the server (no answer in time) so later goals are not blocked."""
        self._finish()

    def _start(self, goal) -> None:
        self.state = self.SENDING
        self._send(goal)

    def _begin_cancel(self) -> None:
        self.state = self.CANCELLING
        self._cancel()

    def _finish(self) -> None:
        self.state = self.IDLE
        self._abandoned = False
        if self._queued is not None:
            goal, self._queued = self._queued, None
            self._start(goal)


def cancel_service_names(action_name: str, extra_actions: Sequence[str]) -> List[str]:
    """Cancel services that /nav_cancel calls, in calling order: the feeders first.

    A feeder (waypoint_follower) that is still alive when its NavigateToPose sub-goal dies would
    send the next one, so `action_name` itself always comes LAST. Empty names and duplicates
    are dropped.
    """
    ordered = [name for name in dict.fromkeys(extra_actions) if name and name != action_name]
    return [f'{name}/_action/cancel_goal' for name in (*ordered, action_name)]


def import_navigate_to_pose():
    """Import nav2_msgs lazily so that a machine without Nav2 gets an actionable error."""
    try:
        from nav2_msgs.action import NavigateToPose
    except ImportError as exc:
        raise RuntimeError(
            'point_to_goal needs the nav2_msgs package, which is not installed in this '
            'environment. Install Nav2 (sudo apt install ros-jazzy-navigation2) and source '
            '/opt/ros/jazzy/setup.bash again.') from exc
    return NavigateToPose


# --------------------------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------------------------

class PointToGoal(Node):
    WATCHDOG_PERIOD_S = 1.0
    WATCHDOG_TICKS = 3          # no answer from the action server for this many periods -> give up

    def __init__(self) -> None:
        self._NavigateToPose = import_navigate_to_pose()    # fail before the node exists
        super().__init__('point_to_goal')

        def param(name, default, description):
            return self.declare_parameter(
                name, default, ParameterDescriptor(description=description)).value

        self._global_frame = param('global_frame', 'map', 'frame the goals are sent in')
        self._base_frame = param('base_frame', 'base_footprint', 'robot base frame')
        self._goal_heading = param(
            'goal_heading', 'keep',
            "goal orientation: 'keep' = current robot yaw in global_frame (from TF) | "
            "'travel' = face the direction from the robot to the clicked point")
        action_name = param('action_name', 'navigate_to_pose', 'NavigateToPose action server')
        extra_cancel_actions = param(
            'extra_cancel_actions', ['follow_waypoints', 'navigate_through_poses'],
            '/nav_cancel also cancels ALL goals on these action servers (the Nav2 servers that '
            "send NavigateToPose goals themselves); [''] = none")
        if self._goal_heading not in HEADING_MODES:
            raise ValueError(
                f"goal_heading must be one of {HEADING_MODES}, got '{self._goal_heading}'")

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._action_client = ActionClient(self, self._NavigateToPose, action_name)
        # The action client can only cancel goals it sent itself; "cancel all" is a request with
        # a zero goal id and zero stamp on the action's cancel service (action_msgs/CancelGoal).
        self._cancel_all_clients = [
            self.create_client(CancelGoal, service)
            for service in cancel_service_names(action_name, extra_cancel_actions)]

        self._sequencer = GoalSequencer(self._send_goal, self._cancel_goal)
        # Number of the goal sent last. Callbacks of older goals (given up by the watchdog) are
        # recognised by their number and must not drive the sequencer.
        self._seq = 0
        self._goal_handle = None
        self._watch = (None, 0)         # (sequencer state, seq) seen at the last watchdog tick
        self._stuck_ticks = 0

        # latched, so an RViz started after the click still shows the goal
        marker_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._marker_pub = self.create_publisher(Marker, '~/goal_marker', marker_qos)
        self.create_subscription(PointStamped, 'clicked_point', self._on_clicked_point, 5)
        self.create_subscription(Empty, 'nav_cancel', self._on_nav_cancel, 1)
        self.create_timer(self.WATCHDOG_PERIOD_S, self._watchdog)

        self.get_logger().info(
            f"point_to_goal ready: /clicked_point -> '{action_name}' goals in "
            f"'{self._global_frame}', goal_heading={self._goal_heading}; /nav_cancel cancels "
            f'all goals on {[c.srv_name for c in self._cancel_all_clients]}')

    # ---- clicked point -> goal pose ----

    def _lookup(self, source_frame: str):
        """Latest transform global_frame <- source_frame, or None (with a warning)."""
        try:
            return self._tf_buffer.lookup_transform(
                self._global_frame, source_frame, Time()).transform
        except TransformException as exc:
            self.get_logger().warn(
                f"click ignored: no transform '{self._global_frame}' <- '{source_frame}' "
                f'(is localisation running?): {exc}')
            return None

    def _point_in_global_frame(self, msg: PointStamped) -> Optional[Tuple[float, float]]:
        point = (msg.point.x, msg.point.y, msg.point.z)
        frame = msg.header.frame_id
        if frame and frame != self._global_frame:
            tf = self._lookup(frame)
            if tf is None:
                return None
            t, q = tf.translation, tf.rotation
            point = transform_point((t.x, t.y, t.z), (q.x, q.y, q.z, q.w), point)
        return point[0], point[1]

    def _on_clicked_point(self, msg: PointStamped) -> None:
        goal_xy = self._point_in_global_frame(msg)
        robot_tf = self._lookup(self._base_frame)
        if goal_xy is None or robot_tf is None:
            return
        if not (math.isfinite(goal_xy[0]) and math.isfinite(goal_xy[1])):
            self.get_logger().warn(f'click ignored: the point {goal_xy} is not finite')
            return
        if not self._action_client.server_is_ready():
            self.get_logger().warn('click ignored: the NavigateToPose action server is not '
                                   'available (is Nav2 active?)')
            return
        q = robot_tf.rotation
        yaw = goal_yaw(self._goal_heading,
                       (robot_tf.translation.x, robot_tf.translation.y),
                       quaternion_to_yaw(q.x, q.y, q.z, q.w), goal_xy)

        pose = PoseStamped()
        pose.header.frame_id = self._global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = goal_xy
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = yaw_to_quaternion(yaw)
        self._publish_marker(pose)
        self._sequencer.request(pose)

    # ---- goal life cycle (side effects and events of the GoalSequencer) ----

    def _send_goal(self, pose: PoseStamped) -> None:
        self._seq += 1
        p, q = pose.pose.position, pose.pose.orientation
        self.get_logger().info(
            f'goal #{self._seq}: ({p.x:.2f}, {p.y:.2f}) yaw '
            f'{math.degrees(quaternion_to_yaw(q.x, q.y, q.z, q.w)):.0f} deg')
        future = self._action_client.send_goal_async(
            self._NavigateToPose.Goal(pose=pose), feedback_callback=self._on_feedback)
        future.add_done_callback(partial(self._on_goal_response, self._seq))

    def _cancel_goal(self) -> None:
        self.get_logger().info(f'cancelling goal #{self._seq}')
        self._goal_handle.cancel_goal_async()

    def _on_goal_response(self, seq: int, future) -> None:
        goal_handle = future.result()
        if seq != self._seq:
            if goal_handle.accepted:
                self.get_logger().warn(f'goal #{seq} was accepted too late: cancelling it')
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().warn(f'goal #{seq} REJECTED by Nav2')
            self._clear_marker()
            self._sequencer.on_rejected()
            return
        self.get_logger().info(f'goal #{seq} accepted')
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(partial(self._on_result, seq))
        self._sequencer.on_accepted()

    def _on_feedback(self, feedback_msg) -> None:
        self.get_logger().info(
            f'distance remaining {feedback_msg.feedback.distance_remaining:.2f} m',
            throttle_duration_sec=2.0)

    def _on_result(self, seq: int, future) -> None:
        wrapped = future.result()
        status = STATUS_NAMES.get(wrapped.status, f'status {wrapped.status}')
        detail = ''
        if wrapped.status == GoalStatus.STATUS_ABORTED:
            # error_code / error_msg only exist in newer nav2_msgs (not Humble): a missing field
            # must not kill the node
            detail = (f" (error {getattr(wrapped.result, 'error_code', '?')} "
                      f"{getattr(wrapped.result, 'error_msg', '')})")
        self.get_logger().info(f'goal #{seq} finished: {status}{detail}')
        if seq != self._seq:
            return
        self._goal_handle = None
        if self._sequencer.state != GoalSequencer.CANCELLING:   # else the new marker is up
            self._clear_marker()
        self._sequencer.on_terminated()

    def _watchdog(self) -> None:
        """Unblock the sequencer if the action server stops answering (crashed / deactivated)."""
        watch = (self._sequencer.state, self._seq)
        waiting = watch[0] in (GoalSequencer.SENDING, GoalSequencer.CANCELLING)
        self._stuck_ticks = self._stuck_ticks + 1 if waiting and watch == self._watch else 0
        self._watch = watch
        if self._stuck_ticks >= self.WATCHDOG_TICKS:
            self.get_logger().warn(f'no answer from Nav2 while {watch[0]} goal #{self._seq}: '
                                   'giving up on it')
            self._seq += 1              # late callbacks of this goal are now ignored
            self._goal_handle = None
            self._stuck_ticks = 0
            self._sequencer.expire()

    # ---- cancel all ----

    def _on_nav_cancel(self, _msg: Empty) -> None:
        self._sequencer.abandon()
        self._clear_marker()
        # Servers that are not running (waypoint_follower is optional) have nothing to cancel.
        ready = [c for c in self._cancel_all_clients if c.service_is_ready()]
        if self._cancel_all_clients[-1] not in ready:
            self.get_logger().warn('/nav_cancel: no NavigateToPose action server to cancel on')
        if ready:
            self.get_logger().info('/nav_cancel: cancelling ALL navigation goals')
        for client in ready:
            future = client.call_async(CancelGoal.Request())
            future.add_done_callback(partial(self._on_cancel_all_response, client.srv_name))

    def _on_cancel_all_response(self, service: str, future) -> None:
        response = future.result()
        self.get_logger().info(
            f'/nav_cancel: {service}: {len(response.goals_canceling)} goal(s) cancelling '
            f'(return code {response.return_code})')

    # ---- RViz marker ----

    def _publish_marker(self, pose: PoseStamped) -> None:
        marker = Marker(header=pose.header, ns='point_to_goal', id=0, type=Marker.ARROW,
                        action=Marker.ADD, pose=pose.pose)
        marker.scale.x, marker.scale.y, marker.scale.z = 0.5, 0.08, 0.08
        marker.color.g = marker.color.a = 1.0
        self._marker_pub.publish(marker)

    def _clear_marker(self) -> None:
        marker = Marker(ns='point_to_goal', id=0, action=Marker.DELETE)
        marker.header.frame_id = self._global_frame
        self._marker_pub.publish(marker)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = PointToGoal()
    except (RuntimeError, ValueError) as exc:           # nav2_msgs missing / bad parameter
        get_logger('point_to_goal').fatal(str(exc))
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        # SIGINT/SIGTERM invalidate the context from rclpy's signal thread, possibly while a
        # callback is about to publish ("publisher's context is invalid"). That is a normal
        # shutdown, not a crash; with a live context it is a real error.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
