"""location_markers: saved places (checkpoints) as clickable arrows in RViz.

Reads maps/<map>.locations.yaml (written by scripts/locations.sh save <name>) and shows every
place as an interactive marker: an arrow with the saved heading plus its name.  Clicking the
arrow with RViz's "Interact" tool sends that place to Nav2 (NavigateToPose, staged approach
below), so the ROBOT DRIVES THERE.  X on the pad cancels, B is the e-stop.

The file is re-read whenever it changes, so a place saved while navigation runs appears within
a second.  Right-click an arrow for a menu with the same "Go" entry.

Staged approach (approach_distance > 0): a goal is driven to in two legs so the robot never turns
next to the obstacle it parks at (e.g. a table):
  1. a STAGING pose approach_distance behind the goal, already facing the goal heading - any
     turning happens there, in the open;
  2. straight in, along the goal heading, to the goal.
The same applies to RViz "2D Goal Pose", whose topic is goal_pose_staged in navigation.rviz.
The staging pose is checked against the global costmap and moved closer (or skipped) when it is
not free; it is also skipped when the robot already sits on the approach line.
"""

from functools import partial
import math
import os
from typing import Dict, List, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from interactive_markers import InteractiveMarkerServer, MenuHandler
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from visualization_msgs.msg import InteractiveMarkerFeedback
import yaml

Place = Tuple[float, float, float]      # x, y, yaw in the map frame

# Global costmap values (nav_msgs/OccupancyGrid scale): 99 = inscribed, 100 = lethal, -1 unknown.
# A staging pose whose centre is below INSCRIBED cannot put the footprint into an obstacle.
INSCRIBED = 99
# Fractions of approach_distance tried, in order, when the staging pose is not free.
STAGING_FRACTIONS = (1.0, 0.75, 0.5)
# Robot already on the approach line: within this lateral offset / heading error, go straight in.
ON_LINE_LATERAL = 0.12
ON_LINE_YAW = 0.30


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def staging_candidates(goal: Place, distance: float) -> List[Place]:
    """Poses behind the goal along its heading, farthest first, all facing the goal heading."""
    x, y, yaw = goal
    return [(x - f * distance * math.cos(yaw), y - f * distance * math.sin(yaw), yaw)
            for f in STAGING_FRACTIONS if f * distance > 0.0]


def on_approach_line(goal: Place, robot: Place, distance: float) -> bool:
    """True if the robot is already behind the goal, on its heading line, facing its heading."""
    x, y, yaw = goal
    dx, dy = robot[0] - x, robot[1] - y
    along = dx * math.cos(yaw) + dy * math.sin(yaw)          # < 0: behind the goal
    lateral = -dx * math.sin(yaw) + dy * math.cos(yaw)
    return (-(distance + 0.3) <= along <= 0.05 and abs(lateral) <= ON_LINE_LATERAL
            and abs(wrap(robot[2] - yaw)) <= ON_LINE_YAW)


def costmap_value(grid: OccupancyGrid, x: float, y: float) -> Optional[int]:
    """Cost of the cell under (x, y) (grid assumed axis aligned), None outside the grid."""
    info = grid.info
    col = math.floor((x - info.origin.position.x) / info.resolution)
    row = math.floor((y - info.origin.position.y) / info.resolution)
    if not (0 <= col < info.width and 0 <= row < info.height):
        return None
    return grid.data[row * info.width + col]


def choose_staging(goal: Place, distance: float, grid: Optional[OccupancyGrid]) -> Optional[Place]:
    """First free staging pose (no costmap: the farthest one is trusted), None if none is free."""
    for pose in staging_candidates(goal, distance):
        if grid is None:
            return pose
        cost = costmap_value(grid, pose[0], pose[1])
        if cost is not None and 0 <= cost < INSCRIBED:
            return pose
    return None


def load_places(path: str) -> Dict[str, Place]:
    """Places of a locations file; malformed entries are skipped, a missing file is empty."""
    if not os.path.exists(path):
        return {}
    with open(path) as handle:
        data = yaml.safe_load(handle) or {}
    places = {}
    for name, entry in (data.get('locations') or {}).items():
        try:
            place = (float(entry['x']), float(entry['y']), float(entry['yaw']))
        except (KeyError, TypeError, ValueError):
            continue
        if all(math.isfinite(v) for v in place):
            places[str(name)] = place
    return places


def place_of(pose: PoseStamped) -> Place:
    q = pose.pose.orientation
    return (pose.pose.position.x, pose.pose.position.y,
            math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


def goal_message(place: Place, frame: str, stamp) -> PoseStamped:
    x, y, yaw = place
    goal = PoseStamped()
    goal.header.frame_id = frame
    goal.header.stamp = stamp
    goal.pose.position.x, goal.pose.position.y = x, y
    goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    return goal


def place_marker(name: str, place: Place, frame: str) -> InteractiveMarker:
    x, y, yaw = place
    marker = InteractiveMarker()
    marker.header.frame_id = frame
    marker.name = name
    marker.description = name
    marker.scale = 0.6
    marker.pose.position.x, marker.pose.position.y = x, y
    marker.pose.orientation.z, marker.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)

    arrow = Marker(type=Marker.ARROW)
    arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.6, 0.12, 0.12
    arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.1, 0.8, 1.0, 0.9
    disc = Marker(type=Marker.CYLINDER)
    disc.scale.x = disc.scale.y = 0.3
    disc.scale.z = 0.02
    disc.color.r, disc.color.g, disc.color.b, disc.color.a = 0.1, 0.8, 1.0, 0.5

    control = InteractiveMarkerControl()
    control.name = 'go'
    control.interaction_mode = InteractiveMarkerControl.BUTTON
    control.always_visible = True
    control.description = f'Click: go to {name}'
    control.markers.extend([arrow, disc])
    marker.controls.append(control)
    return marker


class LocationMarkers(Node):

    def __init__(self) -> None:
        super().__init__('location_markers')
        self._path = os.path.expanduser(self.declare_parameter(
            'locations_file', '').value)
        self._frame = self.declare_parameter('global_frame', 'map').value
        self._base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        # 0 disables the staged approach: goals go straight to Nav2 on /goal_pose.
        self._approach = max(0.0, float(self.declare_parameter('approach_distance', 0.8).value))
        self._goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 1)

        from nav2_msgs.action import NavigateToPose     # lazy: tests of the helpers need no Nav2
        self._NavigateToPose = NavigateToPose
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._costmap: Optional[OccupancyGrid] = None
        self.create_subscription(
            OccupancyGrid, 'global_costmap/costmap', self._on_costmap,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE))
        # RViz "2D Goal Pose" publishes here (navigation.rviz) to get the staged approach too.
        self.create_subscription(PoseStamped, 'goal_pose_staged', self._on_staged_goal, 1)
        self._seq = 0                       # id of the current staged request
        self._final: Optional[Place] = None
        self._server = InteractiveMarkerServer(self, 'locations')
        self._menu = MenuHandler()
        self._menu.insert('Go here', callback=self._on_go)
        self._places: Dict[str, Place] = {}
        self._mtime: Optional[float] = None
        self._reload()
        self.create_timer(1.0, self._reload)
        self.get_logger().info(
            f'location_markers: {len(self._places)} place(s) from {self._path or "(none)"}; '
            'click an arrow with the RViz Interact tool to send the robot there')

    def _reload(self) -> None:
        try:
            mtime = os.path.getmtime(self._path) if self._path else None
        except OSError:
            mtime = None
        if mtime == self._mtime:
            return
        self._mtime = mtime
        try:
            places = load_places(self._path) if self._path else {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().warning(f'cannot read {self._path}: {exc}')
            return
        self._server.clear()
        for name, place in places.items():
            self._server.insert(place_marker(name, place, self._frame),
                                feedback_callback=self._on_feedback)
            self._menu.apply(self._server, name)
        self._server.applyChanges()
        if places != self._places:
            self.get_logger().info(f'places: {", ".join(places) or "none"}')
        self._places = places

    def _on_feedback(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
            self._go(feedback.marker_name)

    def _on_go(self, feedback: InteractiveMarkerFeedback) -> None:
        self._go(feedback.marker_name)

    def _go(self, name: str) -> None:
        place = self._places.get(name)
        if place is None:
            return
        self.get_logger().info(
            f"goal -> '{name}' (x={place[0]:.2f} y={place[1]:.2f} yaw={place[2]:.2f})")
        self._navigate(place)

    def _on_staged_goal(self, msg: PoseStamped) -> None:
        if msg.header.frame_id and msg.header.frame_id != self._frame:
            self.get_logger().warning(
                f"2D Goal Pose ignored: frame '{msg.header.frame_id}' is not '{self._frame}'")
            return
        self._navigate(place_of(msg))

    def _on_costmap(self, msg: OccupancyGrid) -> None:
        self._costmap = msg

    # ---- staged approach ----

    def _robot_place(self) -> Optional[Place]:
        try:
            tf = self._tf_buffer.lookup_transform(self._frame, self._base_frame, Time()).transform
        except TransformException:
            return None
        q = tf.rotation
        return (tf.translation.x, tf.translation.y,
                math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))

    def _navigate(self, goal: Place) -> None:
        self._seq += 1
        self._final = goal
        if self._approach <= 0.0 or not self._nav.server_is_ready():
            if self._approach > 0.0:
                self.get_logger().warning('navigate_to_pose not available: goal sent directly')
            self._goal_pub.publish(goal_message(goal, self._frame, self.get_clock().now().to_msg()))
            return
        robot = self._robot_place()
        if robot is not None and on_approach_line(goal, robot, self._approach):
            self.get_logger().info('already on the approach line: driving straight in')
            self._send(goal, 'final')
            return
        staging = choose_staging(goal, self._approach, self._costmap)
        if staging is None:
            self.get_logger().warning(
                'no free staging pose behind the goal: going to the goal directly')
            self._send(goal, 'final')
            return
        self.get_logger().info(
            f'staging at ({staging[0]:.2f}, {staging[1]:.2f}), '
            f'{math.hypot(staging[0] - goal[0], staging[1] - goal[1]):.2f} m behind the goal')
        self._send(staging, 'staging')

    def _send(self, place: Place, leg: str) -> None:
        goal = self._NavigateToPose.Goal(
            pose=goal_message(place, self._frame, self.get_clock().now().to_msg()))
        self._nav.send_goal_async(goal).add_done_callback(
            partial(self._on_accepted, self._seq, leg))

    def _on_accepted(self, seq: int, leg: str, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warning(f'{leg} leg REJECTED by Nav2')
            return
        handle.get_result_async().add_done_callback(partial(self._on_result, seq, leg))

    def _on_result(self, seq: int, leg: str, future) -> None:
        wrapped = future.result()
        if seq != self._seq:
            return                          # superseded by a newer click
        status = wrapped.status
        if leg == 'final':
            self.get_logger().info(
                'arrived' if status == GoalStatus.STATUS_SUCCEEDED
                else f'final leg ended with status {status}')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('staging reached: driving straight in')
            self._send(self._final, 'final')
        elif status == GoalStatus.STATUS_ABORTED and getattr(wrapped.result, 'error_code', 0) != 0:
            # a real failure (no path to the staging pose, ...): try the goal itself
            self.get_logger().warning(
                f'staging leg failed (error {wrapped.result.error_code}): '
                'going to the goal directly')
            self._send(self._final, 'final')
        else:
            # cancelled, or preempted by another goal (Nav2 aborts it with error 0): stop here.
            # Humble's result has no error_code, so there a failure cannot be told from a
            # preemption by another client and every abort stops the approach.
            self.get_logger().info(f'staging leg ended (status {status}): approach stopped')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocationMarkers()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl-C reaches the node as an external shutdown: a clean stop, not a crash.
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
