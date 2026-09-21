"""nav_api: the /nav topic interface of the navigation stack.

Everything the rest of the system needs in order to USE navigation is a plain ROS 2 topic under
/nav - no action client, no Nav2 headers, no knowledge of the internal topic names:

  send a goal      /nav/goal_pose        geometry_msgs/PoseStamped  (map frame, or any frame in TF)
                   /nav/goal_location    std_msgs/String            a name from the locations file
  stop             /nav/cancel           std_msgs/Empty             cancels every navigation goal
  what is it doing /nav/state            std_msgs/String   IDLE | SENDING | NAVIGATING |
                                                           SUCCEEDED | CANCELED | FAILED
                   /nav/busy             std_msgs/Bool     true while a goal is being driven
                   /nav/result           std_msgs/String   one message per finished goal
                   /nav/goal             geometry_msgs/PoseStamped  goal currently being driven
                   /nav/distance_remaining  std_msgs/Float32  [m]   (Nav2 feedback)
                   /nav/time_remaining      std_msgs/Float32  [s]   (Nav2 feedback)
  where is it      /nav/pose             geometry_msgs/PoseStamped  base_frame in map, from TF
  what can it      /nav/locations        std_msgs/String   JSON list of the saved places
                   (state, busy, goal and locations are latched: a node that starts late still
                    gets the current value)

HOW IT WORKS, and why it is built this way:

  * Goals are NOT sent to Nav2 directly.  They are published on the same topic RViz uses
    (/goal_pose_staged by default), so a goal from another computer takes exactly the path a
    goal clicked in RViz takes, staged approach (location_markers) included.  One goal path,
    one set of behaviours to trust.
  * Status is NOT taken from our own action client.  It is read off the NavigateToPose action's
    status and feedback topics, so /nav/state describes what the robot is ACTUALLY doing,
    whoever asked for it - this node, RViz, a saved-place click or the joystick's cancel.
  * A staged approach is two NavigateToPose goals in a row (staging leg, then straight in).  A
    terminal state is therefore only reported after settle_time with no goal running, so the gap
    between the two legs is not reported as "arrived".

This node commands no velocity and touches no hardware.  The joystick stays above everything:
LT overrides, B is the e-stop, X cancels.
"""

from __future__ import annotations

import json
import math
import os
from typing import Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, Float32, String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

# States published on /nav/state.
IDLE = 'IDLE'
SENDING = 'SENDING'             # goal published, no Nav2 goal running (yet)
NAVIGATING = 'NAVIGATING'
SUCCEEDED = 'SUCCEEDED'
CANCELED = 'CANCELED'
FAILED = 'FAILED'

ACTIVE_STATUSES = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
                   GoalStatus.STATUS_CANCELING)
TERMINAL_STATES = {
    GoalStatus.STATUS_SUCCEEDED: SUCCEEDED,
    GoalStatus.STATUS_CANCELED: CANCELED,
    GoalStatus.STATUS_ABORTED: FAILED,
}

Place = Tuple[float, float, float]      # x, y, yaw in the map frame


# --------------------------------------------------------------------------------------------
# ROS-free helpers
# --------------------------------------------------------------------------------------------

def yaw_of(x: float, y: float, z: float, w: float) -> float:
    """Yaw (rotation about +z) of a unit quaternion, in (-pi, pi]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_of(yaw: float) -> Tuple[float, float]:
    """(z, w) of a rotation about +z; x and y are always 0 for a planar robot."""
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def load_places(path: str) -> Dict[str, Place]:
    """Places of a locations file; malformed entries are skipped, a missing file is empty.

    Same format (and the same tolerance for junk) as location_markers.
    """
    if not path or not os.path.exists(path):
        return {}
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}
    places: Dict[str, Place] = {}
    for name, entry in (data.get('locations') or {}).items():
        try:
            place = (float(entry['x']), float(entry['y']), float(entry['yaw']))
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(value) for value in place):
            places[str(name)] = place
    return places


def places_json(places: Dict[str, Place]) -> str:
    """The saved places as a JSON list - what /nav/locations carries."""
    return json.dumps([{'name': name, 'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 5)}
                       for name, (x, y, yaw) in sorted(places.items())])


def match_place(places: Dict[str, Place], name: str) -> Optional[str]:
    """Key of `name` in `places`: exact first, then case/whitespace insensitive."""
    wanted = name.strip()
    if wanted in places:
        return wanted
    folded = wanted.casefold()
    for key in places:
        if key.casefold() == folded:
            return key
    return None


def active_goals(status_array: GoalStatusArray) -> List[bytes]:
    """Ids of the goals the action server is currently working on."""
    return [bytes(status.goal_info.goal_id.uuid) for status in status_array.status_list
            if status.status in ACTIVE_STATUSES]


def newest_terminal(status_array: GoalStatusArray) -> Optional[Tuple[bytes, int]]:
    """(goal id, status) of the most recently ACCEPTED goal that has finished, if any.

    The server keeps finished goals in the list for a while, so "the last one" is decided by the
    goal's own acceptance stamp, not by its position in the array.
    """
    finished = [status for status in status_array.status_list if status.status in TERMINAL_STATES]
    if not finished:
        return None
    newest = max(finished, key=lambda s: (s.goal_info.stamp.sec, s.goal_info.stamp.nanosec))
    return bytes(newest.goal_info.goal_id.uuid), newest.status


# --------------------------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------------------------

class NavApi(Node):

    LOCATIONS_PERIOD_S = 1.0        # re-read the locations file this often (it may be edited live)

    def __init__(self) -> None:
        super().__init__('nav_api')

        def param(name, default, description):
            return self.declare_parameter(
                name, default, ParameterDescriptor(description=description)).value

        self._global_frame = param('global_frame', 'map', 'frame goals and /nav/pose are in')
        self._base_frame = param('base_frame', 'base_footprint', 'robot base frame')
        self._locations_file = os.path.expanduser(param(
            'locations_file', '', 'maps/<map>.locations.yaml read for /nav/goal_location'))
        self._goal_topic = param(
            'goal_topic', '/goal_pose_staged',
            'where goals are published. /goal_pose_staged = staged approach (location_markers); '
            '/goal_pose = straight to the Nav2 bt_navigator')
        self._cancel_topic = param(
            'cancel_topic', '/nav_cancel',
            'topic that cancels every navigation goal (point_to_goal does the cancelling)')
        action_name = param('action_name', '/navigate_to_pose',
                            'NavigateToPose action watched for state and feedback')
        self._settle_time = float(param(
            'settle_time', 1.2,
            '[s] with no goal running before a terminal state is reported. Covers the gap '
            'between the two legs of a staged approach; raise it if a staged goal ever reports '
            'SUCCEEDED between the legs'))
        self._send_timeout = float(param(
            'send_timeout', 8.0,
            '[s] a published goal has to be picked up by Nav2 in, or /nav/state goes to FAILED'))
        pose_rate = float(param('pose_rate_hz', 10.0, '[Hz] rate of /nav/pose'))

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)

        # ---- the /nav API ----
        # The names are relative ('nav/...'), so the node's namespace decides where the API
        # appears: '/' (default) -> /nav/state, '/robot_1' -> /robot_1/nav/state. The topics of
        # the stack behind it stay absolute, wherever the API is published.
        self._state_pub = self.create_publisher(String, 'nav/state', latched)
        self._busy_pub = self.create_publisher(Bool, 'nav/busy', latched)
        self._goal_echo_pub = self.create_publisher(PoseStamped, 'nav/goal', latched)
        self._locations_pub = self.create_publisher(String, 'nav/locations', latched)
        self._result_pub = self.create_publisher(String, 'nav/result', 10)
        self._pose_pub = self.create_publisher(PoseStamped, 'nav/pose', 10)
        self._distance_pub = self.create_publisher(Float32, 'nav/distance_remaining', 10)
        self._time_pub = self.create_publisher(Float32, 'nav/time_remaining', 10)

        self.create_subscription(PoseStamped, 'nav/goal_pose', self._on_goal_pose, 1)
        self.create_subscription(String, 'nav/goal_location', self._on_goal_location, 1)
        self.create_subscription(Empty, 'nav/cancel', self._on_cancel, 1)

        # ---- the stack behind it ----
        self._goal_pub = self.create_publisher(PoseStamped, self._goal_topic, 1)
        self._nav_cancel_pub = self.create_publisher(Empty, self._cancel_topic, 1)

        # An action's status topic is latched by rcl (depth 1, transient local); its feedback
        # topic is not.  Both QoS profiles have to match or nothing is received.
        self.create_subscription(
            GoalStatusArray, f'{action_name}/_action/status', self._on_status,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(
            self._feedback_type(), f'{action_name}/_action/feedback', self._on_feedback, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._state: Optional[str] = None   # nothing published yet; IDLE is published below
        self._goal: Optional[PoseStamped] = None
        self._sent_at: Optional[float] = None       # monotonic-ish time the goal was published
        self._idle_since: Optional[float] = None    # first tick without a running Nav2 goal
        self._pending: Optional[Tuple[bytes, int]] = None   # terminal result waiting for settle
        self._reported: Optional[bytes] = None      # goal id whose result was already published
        self._places: Dict[str, Place] = {}
        self._places_mtime: Optional[float] = None
        self._places_loaded = False
        # The action status topic is latched: at start-up it replays the goals of the PREVIOUS
        # run. Their outcome is not ours to report, so terminal states only count once this node
        # has seen a goal actually running.
        self._seen_active = False

        self._publish_state(IDLE)
        self._reload_places()
        self.create_timer(self.LOCATIONS_PERIOD_S, self._reload_places)
        self.create_timer(1.0 / max(1.0, pose_rate), self._publish_pose)
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'nav_api ready: /nav/goal_pose + /nav/goal_location -> {self._goal_topic}, '
            f'/nav/cancel -> {self._cancel_topic}, state from {action_name}; '
            f'{len(self._places)} saved place(s) from {self._locations_file or "(none)"}')

    @staticmethod
    def _feedback_type():
        """NavigateToPose feedback message type; lazy so a machine without Nav2 gets an error."""
        try:
            from nav2_msgs.action import NavigateToPose
        except ImportError as exc:
            raise RuntimeError(
                'nav_api needs the nav2_msgs package, which is not installed in this '
                'environment. Install Nav2 (sudo apt install ros-jazzy-navigation2) and source '
                '/opt/ros/jazzy/setup.bash again.') from exc
        return NavigateToPose.Impl.FeedbackMessage

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # ---- goals in ----

    def _on_goal_pose(self, msg: PoseStamped) -> None:
        pose = self._in_global_frame(msg)
        if pose is None:
            return
        position = pose.pose.position
        if not (math.isfinite(position.x) and math.isfinite(position.y)):
            self.get_logger().warning('/nav/goal_pose ignored: the position is not finite')
            return
        self.get_logger().info(
            f'/nav/goal_pose -> ({position.x:.2f}, {position.y:.2f}) yaw '
            f'{math.degrees(yaw_of(*self._orientation(pose))):.0f} deg')
        self._dispatch(pose)

    def _on_goal_location(self, msg: String) -> None:
        key = match_place(self._places, msg.data)
        if key is None:
            known = ', '.join(sorted(self._places)) or 'none'
            self.get_logger().warning(
                f"/nav/goal_location '{msg.data}' is not a saved place (known: {known})")
            self._publish_result(f'REJECTED unknown location {msg.data}')
            return
        x, y, yaw = self._places[key]
        pose = PoseStamped()
        pose.header.frame_id = self._global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = quaternion_of(yaw)
        self.get_logger().info(f"/nav/goal_location -> '{key}' ({x:.2f}, {y:.2f})")
        self._dispatch(pose)

    def _dispatch(self, pose: PoseStamped) -> None:
        """Publish the goal where RViz publishes its own, and start waiting for Nav2."""
        pose.header.stamp = self.get_clock().now().to_msg()
        self._goal = pose
        self._sent_at = self._now()
        self._pending = None
        self._goal_echo_pub.publish(pose)
        self._goal_pub.publish(pose)
        self._publish_state(SENDING)

    def _on_cancel(self, _msg: Empty) -> None:
        self.get_logger().info('/nav/cancel: cancelling every navigation goal')
        self._sent_at = None
        self._pending = None
        self._nav_cancel_pub.publish(Empty())
        # The terminal state itself comes from the action status, like any other outcome.

    # ---- frames ----

    @staticmethod
    def _orientation(pose: PoseStamped) -> Tuple[float, float, float, float]:
        q = pose.pose.orientation
        return q.x, q.y, q.z, q.w

    def _in_global_frame(self, msg: PoseStamped) -> Optional[PoseStamped]:
        """The goal in global_frame; an empty frame_id is taken to mean it is already there."""
        frame = msg.header.frame_id
        if not frame or frame == self._global_frame:
            pose = PoseStamped(header=msg.header, pose=msg.pose)
            pose.header.frame_id = self._global_frame
            return pose
        try:
            from tf2_geometry_msgs import do_transform_pose
        except ImportError:
            self.get_logger().warning(
                f"/nav/goal_pose ignored: goal is in '{frame}', and tf2_geometry_msgs (needed to "
                f"convert it to '{self._global_frame}') is not installed. Send goals in "
                f"'{self._global_frame}'.")
            return None
        try:
            transform = self._tf_buffer.lookup_transform(self._global_frame, frame, Time())
        except TransformException as exc:
            self.get_logger().warning(
                f"/nav/goal_pose ignored: no transform '{self._global_frame}' <- '{frame}' "
                f'(is localisation running?): {exc}')
            return None
        pose = PoseStamped()
        pose.header.frame_id = self._global_frame
        pose.header.stamp = msg.header.stamp
        pose.pose = do_transform_pose(msg.pose, transform)
        return pose

    def _publish_pose(self) -> None:
        try:
            tf = self._tf_buffer.lookup_transform(
                self._global_frame, self._base_frame, Time()).transform
        except TransformException:
            return              # no localisation yet: publish nothing rather than a wrong pose
        pose = PoseStamped()
        pose.header.frame_id = self._global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tf.translation.x
        pose.pose.position.y = tf.translation.y
        pose.pose.orientation = tf.rotation
        self._pose_pub.publish(pose)

    # ---- what Nav2 is doing ----

    def _on_status(self, msg: GoalStatusArray) -> None:
        if active_goals(msg):
            self._sent_at = None
            self._pending = None
            self._idle_since = None
            self._seen_active = True
            self._publish_state(NAVIGATING)
            return
        if self._idle_since is None:
            self._idle_since = self._now()
        if not self._seen_active:
            return
        terminal = newest_terminal(msg)
        if terminal is not None and terminal[0] != self._reported:
            self._pending = terminal

    def _on_feedback(self, msg) -> None:
        feedback = msg.feedback
        self._distance_pub.publish(Float32(data=float(feedback.distance_remaining)))
        remaining = feedback.estimated_time_remaining
        self._time_pub.publish(
            Float32(data=float(remaining.sec) + float(remaining.nanosec) * 1e-9))

    def _tick(self) -> None:
        """Report a terminal state once Nav2 has been idle long enough to mean it."""
        now = self._now()
        if self._pending is not None and self._idle_since is not None:
            if now - self._idle_since >= self._settle_time:
                goal_id, status = self._pending
                self._pending = None
                self._reported = goal_id
                state = TERMINAL_STATES.get(status, FAILED)
                self._publish_state(state)
                self._publish_result(state)
                self._goal = None
            return
        if self._state == SENDING and self._sent_at is not None:
            if now - self._sent_at >= self._send_timeout:
                self._sent_at = None
                self.get_logger().warning(
                    f'no Nav2 goal started within {self._send_timeout:.0f} s of publishing the '
                    f'goal on {self._goal_topic} - is the stack up and localised?')
                self._publish_state(FAILED)
                self._publish_result('FAILED goal was never accepted by Nav2')
                self._goal = None

    # ---- outputs ----

    def _publish_state(self, state: str) -> None:
        if state == self._state:
            return
        self._state = state
        self._state_pub.publish(String(data=state))
        self._busy_pub.publish(Bool(data=state in (SENDING, NAVIGATING)))
        self.get_logger().info(f'state: {state}')

    def _publish_result(self, text: str) -> None:
        self._result_pub.publish(String(data=text))

    def _reload_places(self) -> None:
        try:
            mtime = os.path.getmtime(self._locations_file) if self._locations_file else None
        except OSError:
            mtime = None
        if mtime == self._places_mtime and self._places_loaded:
            return
        self._places_mtime = mtime
        try:
            places = load_places(self._locations_file)
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f'cannot read {self._locations_file}: {exc}')
            return
        if places != self._places or not self._places_loaded:
            self._places = places
            self._places_loaded = True
            self._locations_pub.publish(String(data=places_json(places)))


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = NavApi()
    except (RuntimeError, ValueError) as exc:           # nav2_msgs missing / bad parameter
        get_logger('nav_api').fatal(str(exc))
        rclpy.try_shutdown()
        raise SystemExit(1)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except rclpy_implementation.RCLError:
        # SIGINT invalidates the context from rclpy's signal thread, possibly while a callback is
        # about to publish. That is a normal shutdown, not a crash.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
