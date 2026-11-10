"""Station arrival/departure through Nav2 actions with bounded fine correction.

The saved pose faces into the station. NavigateToPose is used only at the open
staging pose. DockPath / BackUp request translation without an in-place turn.
The bridge counters yaw drift while translating. Small stopped heading turns
use a separate slow Spin action after checking the actual swept footprint.
No direct /cmd_vel publisher: every action uses the existing smoother, collision
monitor and teleop/e-stop mux. Configuration is opt-in per locations file.
"""

from dataclasses import dataclass
from functools import partial
import json
import math
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import BackUp, FollowPath, NavigateToPose, Spin
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, String, Float64
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
import yaml

from .location_markers import costmap_value, goal_message, wrap
from .yaw_bias import MAX_STATION_YAW_ERROR


@dataclass(frozen=True)
class DockConfig:
    approach_distance: float = 0.3
    staging_distance: float = 0.8  # turns and withdrawal remain away from the table
    speed: float = 0.10
    lateral_tolerance: float = 0.05
    yaw_tolerance: float = 0.12
    approach_yaw_tolerance: float = 0.12  # never accept a heading that final verification rejects
    position_tolerance: float = 0.12
    footprint_half_size: float = 0.35  # configured 0.34 m half-width + 0.01 m padding

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'docking {name} must be finite and positive')
        if not 0.25 <= self.approach_distance <= self.staging_distance <= 1.5:
            raise ValueError('docking distances must satisfy 0.25 <= approach <= staging <= 1.5 m')
        if self.staging_distance < 0.5:
            raise ValueError('docking staging_distance must be >= 0.5 m for turning clearance')
        if self.position_tolerance > 0.15:
            raise ValueError('docking position_tolerance must be <= 0.15 m')
        if self.speed > 0.10:
            raise ValueError('docking speed must be <= 0.10 m/s')
        if self.yaw_tolerance > 0.15 or self.lateral_tolerance > 0.10:
            raise ValueError('docking alignment tolerances are too large')
        if not self.yaw_tolerance <= self.approach_yaw_tolerance <= .12:
            raise ValueError('approach yaw tolerance must cover final yaw tolerance and be <= 0.12 rad')
        if self.footprint_half_size < 0.35:
            raise ValueError('docking footprint must cover the configured padded costmap footprint')


def load_docking_config(path):
    if not path:
        return None
    try:
        with open(path) as handle:
            settings = (yaml.safe_load(handle) or {}).get('docking', {})
    except FileNotFoundError:
        return None
    if not isinstance(settings, dict) or settings.get('enabled') is not True:
        return None
    return DockConfig(**{k: v for k, v in settings.items() if k != 'enabled'})


def staging_pose(station, distance):
    x, y, yaw = station
    return x - distance * math.cos(yaw), y - distance * math.sin(yaw), yaw


def line_errors(station, robot):
    dx, dy = robot[0] - station[0], robot[1] - station[1]
    c, s = math.cos(station[2]), math.sin(station[2])
    return dx * c + dy * s, -dx * s + dy * c, wrap(robot[2] - station[2])


def map_goal_in_odom(robot, odom, goal):
    yaw = odom[2] - robot[2]
    dx, dy = goal[0] - robot[0], goal[1] - robot[1]
    c, s = math.cos(yaw), math.sin(yaw)
    return (odom[0] + c*dx - s*dy, odom[1] + s*dx + c*dy,
            odom[2] - wrap(robot[2] - goal[2]))


def docking_marker(state, name, goal, robot, tolerance, held, reason, frame, stamp=None):
    """The full mission status, independent of Nav2's staging sub-goal status."""
    distance = math.hypot(robot[0]-goal[0], robot[1]-goal[1]) if robot and goal else None
    verified = state == 'arrived' and distance is not None and distance <= tolerance and held
    phases = {'idle': 'Ready', 'preparing': 'Preparing', 'staging': 'Moving to approach position',
              'dock_align': 'Fine heading correction', 'align': 'Aligning away from the table', 'stage_adjust': 'Moving to docking start',
              'dock': 'Parking', 'undock': 'Backing out', 'settling': 'Checking stopped pose',
              'stopped': 'STOPPED - parking incomplete', 'undocked': 'Undocked',
              'arrived': 'Outside verified parking pose'}
    title = 'DOCKED (verified)' if verified else 'NOT DOCKED'
    text = f'{name}: {title}\n{phases.get(state, state)}'
    if distance is not None:
        text += f'\n{distance:.2f} m from parked position'
    if state == 'stopped':
        text += '\n' + reason[:150]
    marker = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD)
    marker.header.frame_id = frame
    if stamp is not None:
        marker.header.stamp = stamp
    marker.ns, marker.id = 'station_docking_status', 0
    location = goal or robot or (0., 0., 0.)
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(location[0]), float(location[1]), 1.2
    marker.pose.orientation.w = 1.0
    marker.scale.z = 0.12
    color = (0.15, 1.0, 0.2) if verified else (1.0, .2, .15) if state == 'stopped' else (1.0, .8, .1)
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.0
    marker.text = text
    return marker


def departure_station(places, robot, config):
    """Recognize a parallel departure, not a precision docking centreline.

    A base offset within a station-width lane can still withdraw straight along
    its own heading. The caller must check that actual corridor and the swept
    footprint at its exit before sending any action. Tight docking tolerances
    and physical drift checks do not apply to this initial position offset.
    """
    candidates = []
    for station in places.values():
        along, lateral, yaw = line_errors(station, robot)
        if (-config.staging_distance - config.position_tolerance <= along <=
                config.position_tolerance and abs(lateral) <= config.footprint_half_size and
                abs(yaw) <= config.approach_yaw_tolerance):
            candidates.append((math.hypot(along, lateral), station))
    return min(candidates, key=lambda item: item[0])[1] if candidates else None


def footprint_clear(grid, pose, half_size, turning=False):
    """Conservative cell-area test against lethal/unknown cells, including footprint interior.

    Ignore soft inflation costs under the footprint: those already represent
    clearance of the robot CENTRE. A turning pose checks the full swept disk.
    Grid yaw is required to be zero, as in Nav2's axis-aligned costmaps.
    """
    if grid is None or grid.info.resolution <= 0:
        return False
    q = grid.info.origin.orientation
    if abs(q.x) + abs(q.y) + abs(q.z) > 1e-6:
        return False
    if costmap_value(grid, pose[0], pose[1]) not in range(99):
        return False
    step = grid.info.resolution
    radius = half_size * math.sqrt(2)
    margin = step / math.sqrt(2)
    ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
    left = math.floor((pose[0] - radius - margin - ox) / step)
    right = math.floor((pose[0] + radius + margin - ox) / step)
    bottom = math.floor((pose[1] - radius - margin - oy) / step)
    top = math.floor((pose[1] + radius + margin - oy) / step)
    c, s = math.cos(pose[2]), math.sin(pose[2])
    for row in range(bottom, top + 1):
        for col in range(left, right + 1):
            dx = ox + (col + 0.5) * step - pose[0]
            dy = oy + (row + 0.5) * step - pose[1]
            inside = (math.hypot(dx, dy) <= radius + margin if turning else
                      abs(dx * c + dy * s) <= half_size + margin and
                      abs(-dx * s + dy * c) <= half_size + margin)
            if inside and (not (0 <= row < grid.info.height and 0 <= col < grid.info.width)
                           or grid.data[row * grid.info.width + col] < 0
                           or grid.data[row * grid.info.width + col] >= 100):
                return False
    return True


def corridor_clear(grid, start, end, half_size):
    if grid is None or grid.info.resolution <= 0:
        return False
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    count = max(1, math.ceil(distance / (grid.info.resolution / 2)))
    return all(footprint_clear(grid, (start[0] + (end[0] - start[0]) * i / count,
                                     start[1] + (end[1] - start[1]) * i / count,
                                     start[2]), half_size) for i in range(count + 1))


class StationDocking:
    """One mission at a time; any cancellation invalidates all pending callbacks."""

    def __init__(self, node, config, behavior_tree, keep_transit_heading=False):
        self.node, self.config, self.tree = node, config, behavior_tree
        self.keep_transit_heading = keep_transit_heading
        self.clients = {
            'staging': ActionClient(node, NavigateToPose, 'navigate_to_pose'),
            'dock': ActionClient(node, FollowPath, 'follow_path'),
            'stage_adjust': ActionClient(node, FollowPath, 'follow_path'),
            'dock_align': ActionClient(node, Spin, 'dock_spin'),
            'align': ActionClient(node, Spin, 'spin'),
            'undock': ActionClient(node, BackUp, 'backup'),
        }
        self.cancel_clients = [node.create_client(CancelGoal, name + '/_action/cancel_goal')
                               for name in ('navigate_to_pose', 'follow_path', 'backup', 'spin', 'dock_spin')]
        self.seq = 0
        self.state = 'idle'
        self.handle = None
        self.pending = []
        self.parked_station = self.parked_odom = None
        self.goal = self.dock_goal_odom = self.odom_goal_yaw = None
        self.is_station = False
        self.status_message = 'Ready'
        self.estop = False
        self.local_grid = None
        self.global_grid = None
        self.local_time = self.global_time = self.odom_time = -math.inf
        self.stopped = False
        self.still_since = None
        self.after_settle = None
        self.settle_wait_actions = False
        self.close_since = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.status = node.create_publisher(String, 'station_docking/status', latched)
        self.result = node.create_publisher(String, 'station_docking/result', latched)
        self.status_marker = node.create_publisher(Marker, 'station_docking/marker', latched)
        node.create_timer(.2, self._publish_status_marker,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.rotation_lock = node.create_publisher(Bool, 'station_docking/rotation_locked', latched)
        self.heading_target_pub = node.create_publisher(Float64, 'station_docking/heading_target', latched)
        node.create_timer(0.1, self._publish_rotation_lock,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))
        node.create_subscription(OccupancyGrid, 'local_costmap/costmap',
                                 self._local_map, latched)
        node.create_subscription(OccupancyGrid, 'global_costmap/costmap',
                                 self._global_map, latched)
        node.create_subscription(Odometry, 'odom', self._odom, 10)
        node.create_subscription(Empty, 'nav_cancel', lambda _: self.cancel('cancelled'), 1)
        node.create_subscription(Empty, 'station_undock', lambda _: self.start(None), 1)
        node.create_subscription(Bool, 'e_stop', self._estop, latched)
        node.create_subscription(Twist, 'cmd_vel_teleop',
                                 lambda _: self.cancel('manual takeover'), 1)
        node.create_timer(0.05, self._tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self._report('idle')

    @property
    def active(self):
        return self.state not in ('idle', 'arrived', 'undocked', 'stopped')

    def _report(self, message):
        self.status_message = message
        self._publish_rotation_lock()
        self.status.publish(String(data=message))
        self._publish_status_marker()
        robot = self.node._robot_place()
        distance = math.hypot(robot[0]-self.goal[0], robot[1]-self.goal[1]) if robot and self.goal else None
        odom = self.node._robot_place('odom')
        held = (odom is not None and self.parked_odom is not None and
                math.hypot(odom[0]-self.parked_odom[0], odom[1]-self.parked_odom[1]) <= self.config.position_tolerance and
                abs(wrap(odom[2]-self.parked_odom[2])) <= self.config.yaw_tolerance)
        verified = (self.state == 'arrived' and self.is_station and held and
                    distance is not None and distance <= self.config.position_tolerance)
        success = verified or self.state == 'undocked' or (self.state == 'arrived' and not self.is_station)
        self.result.publish(String(data=json.dumps(dict(
            state=self.state, station=self._station_name(), target=self.goal, map_pose=robot,
            operation='undock' if self.goal is None else 'dock' if self.is_station else 'navigate',
            remaining_m=distance, success=success, docked=verified,
            done=self.state in ('arrived', 'stopped', 'undocked'), message=message))))
        self.node.get_logger().info(f'station docking: {message}')

    def _station_name(self):
        return next((name for name, p in self.node._places.items() if p == self.goal), 'Selected goal')

    def _publish_status_marker(self):
        robot, odom = self.node._robot_place(), self.node._robot_place('odom')
        held = (odom is not None and self.parked_odom is not None and
                math.hypot(odom[0]-self.parked_odom[0], odom[1]-self.parked_odom[1]) <= self.config.position_tolerance and
                abs(wrap(odom[2]-self.parked_odom[2])) <= self.config.yaw_tolerance)
        self.status_marker.publish(docking_marker(
            self.state, self._station_name(), self.goal, robot, self.config.position_tolerance,
            held, self.status_message, self.node._frame, self.node.get_clock().now().to_msg()))

    def _publish_rotation_lock(self):
        if hasattr(self, 'heading_target_pub'):
            target = None
            if self.state == 'dock':
                target = self.odom_goal_yaw
            elif self.state == 'stage_adjust':
                target = self.stage_goal_odom[2]
            elif self.state == 'undock':
                target = self.odom_endpoint[2]
            self.heading_target_pub.publish(Float64(data=float(target) if target is not None else math.nan))
        self.rotation_lock.publish(Bool(data=self.state not in ('idle', 'staging', 'align', 'dock_align', 'undocked')))

    def _local_map(self, msg):
        self.local_grid, self.local_time = msg, time.monotonic()

    def _global_map(self, msg):
        self.global_grid, self.global_time = msg, time.monotonic()

    def _odom(self, msg):
        v = msg.twist.twist
        self.stopped = (math.hypot(v.linear.x, v.linear.y) < 0.02 and abs(v.angular.z) < 0.03)
        self.odom_time = time.monotonic()

    def _estop(self, msg):
        self.estop = msg.data
        if msg.data:
            self.cancel('e-stop')

    def cancel(self, reason):
        if not self.active:
            return
        self.seq += 1
        self.state = 'stopped'
        self.after_settle = None
        if self.handle is not None:
            self.pending.append(self.handle.cancel_goal_async())
        self.handle = None
        self._report(f'stopped: {reason}; send a new request to continue')

    def _fresh(self):
        now = time.monotonic()
        return (now - self.local_time < 1.5 and now - self.global_time < 3.0 and
                now - self.odom_time < 0.5)

    def _clear_line(self, start, end):
        # The global costmap is in map, but the rolling local costmap is in odom.
        # Transform both endpoints before querying cells; never mix those coordinates.
        for grid in (self.global_grid, self.local_grid):
            a, b = self._in_grid(grid, start), self._in_grid(grid, end)
            if a is None or b is None or not corridor_clear(
                    grid, a, b, self.config.footprint_half_size):
                self._report(f'corridor check failed in {grid.header.frame_id or self.node._frame}')
                return False
        return True

    def _in_grid(self, grid, pose):
        frame = grid.header.frame_id
        if not frame or frame == self.node._frame:
            return pose
        from tf2_ros import TransformException
        try:
            tf = self.node._tf_buffer.lookup_transform(frame, self.node._frame, Time()).transform
        except TransformException:
            return None
        q = tf.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        return (tf.translation.x + c * pose[0] - s * pose[1],
                tf.translation.y + s * pose[0] + c * pose[1], wrap(pose[2] + yaw))

    def start(self, goal):
        if self.active:
            self._report('busy; cancel with X before sending another station request')
            return
        if self.estop or not self._fresh() or self.node._robot_place() is None:
            self._report('request refused: need released e-stop, fresh TF, odometry and both costmaps')
            return
        if not self.tree or not all(c.server_is_ready() for c in self.clients.values()):
            self._report('request refused: docking behavior tree or Nav2 action servers unavailable')
            return
        if not all(c.service_is_ready() for c in self.cancel_clients):
            self._report('request refused: action cancellation services unavailable')
            return
        if goal is not None and not all(math.isfinite(v) for v in goal):
            self._report('request refused: invalid station pose')
            return
        self.seq += 1
        self.recovery_attempts = 0
        self.align_attempts = 0
        self.correction_attempts = 0
        self.mission_deadline = time.monotonic() + 360.0
        # Quaternion round trips can change yaw by an ulp. Canonicalize named
        # goals so CLI/RViz requests reliably select docking, rather than transit.
        station = next((p for p in self.node._places.values() if goal is not None and
                        math.hypot(p[0] - goal[0], p[1] - goal[1]) < 1e-6 and
                        abs(wrap(p[2] - goal[2])) < 1e-6), None)
        self.goal = station if station is not None else goal
        self.is_station = station is not None
        self.dock_goal_odom = self.odom_goal_yaw = None
        self.state = 'preparing'
        self.deadline = time.monotonic() + 10.0
        self.still_since = None
        self.cancels = [c.call_async(CancelGoal.Request()) for c in self.cancel_clients]
        self._report('stopping previous navigation before station maneuver')

    def _settle(self, callback, wait_actions=False, delay=0.5):
        self.state = 'settling'
        self.after_settle = callback
        self.settle_wait_actions = wait_actions
        self.settle_delay = delay
        self.still_since = None
        self.deadline = time.monotonic() + 8.0

    def _begin(self):
        robot = self.node._robot_place()
        source = departure_station(self.node._places, robot, self.config)
        remembered = self._remembered_station()
        if source is None:
            source = remembered
        if self.is_station and math.hypot(robot[0] - self.goal[0], robot[1] - self.goal[1]) <= self.config.position_tolerance:
            odom = self.node._robot_place('odom')
            held = (remembered == self.goal and self.parked_odom is not None and odom is not None and
                    math.hypot(odom[0] - self.parked_odom[0], odom[1] - self.parked_odom[1]) <=
                    self.config.position_tolerance)
            if abs(wrap(robot[2] - self.goal[2])) <= self.config.yaw_tolerance or held:
                self._parked()
                return
        if source is not None:
            along, lateral, yaw = line_errors(source, robot)
            if (self.is_station and source == self.goal and
                    abs(lateral) <= self.config.lateral_tolerance):
                if -along <= self.config.approach_distance + self.config.position_tolerance:
                    self._dock()
                else:
                    self._adjust_stage()
                return
            # Back out on the current body axis, preserving any initial lateral
            # offset. Reach the station's staging plane before allowing a turn.
            distance = (self.config.staging_distance + along) / math.cos(yaw)
            exit_pose = staging_pose(robot, max(0.0, distance))
            if distance > self.config.position_tolerance:
                if not self._clear_line(robot, exit_pose):
                    self.cancel('departure corridor blocked or outside observed costmaps')
                    return
                self.endpoint = exit_pose
                self.line = source
                self._straight('undock', distance)
                return
            if not footprint_clear(self.global_grid, robot, self.config.footprint_half_size, turning=True):
                self.cancel('departure has not reached an area with turning clearance')
                return
        elif self.goal is None:
            self.cancel('no aligned station departure corridor at the current pose')
            return
        elif not footprint_clear(self.global_grid, robot, self.config.footprint_half_size, turning=True):
            self.cancel('current pose lacks turning clearance and no parallel withdrawal is available')
            return
        self._travel()

    def _remembered_station(self):
        # Retain a verified physical departure corridor across AMCL updates and
        # cancelled partial withdrawals. A manual turn/move outside it invalidates it.
        if self.parked_station is None or self.parked_odom is None:
            return None
        odom = self.node._robot_place('odom')
        if odom is None:
            return None
        along, lateral, yaw = line_errors(self.parked_odom, odom)
        if (-self.config.staging_distance - self.config.position_tolerance <= along <=
                self.config.position_tolerance and abs(lateral) <= self.config.lateral_tolerance and
                abs(yaw) <= self.config.yaw_tolerance):
            return self.parked_station
        return None

    def _parked(self):
        robot = self.node._robot_place()
        odom = self.node._robot_place('odom')
        if robot is None or odom is None or self.goal is None or not self.is_station:
            self.cancel('final station pose unavailable; not docked')
            return
        if not self.stopped or not all(math.isfinite(v) for p in (robot, odom, self.goal) for v in p):
            self.cancel('base not stopped or invalid final pose; not docked')
            return
        if self.dock_goal_odom is not None and math.hypot(
                odom[0]-self.dock_goal_odom[0], odom[1]-self.dock_goal_odom[1]) > .15:
            self.cancel('map/odometry disagree about station arrival; not docked')
            return
        distance = math.hypot(robot[0]-self.goal[0], robot[1]-self.goal[1])
        if distance > self.config.position_tolerance:
            if distance <= .20:
                self._recover(f'final position {distance:.3f} m from station', self._dock)
            else:
                self.cancel(f'final station verification failed: {distance:.3f} m remaining; not docked')
            return
        target_yaw = self.odom_goal_yaw
        if target_yaw is None and self.parked_station == self.goal and self.parked_odom is not None:
            target_yaw = self.parked_odom[2]
        yaw_error = abs(wrap(odom[2]-target_yaw)) if target_yaw is not None else abs(wrap(robot[2]-self.goal[2]))
        if yaw_error > self.config.yaw_tolerance:
            error = wrap(target_yaw-odom[2]) if target_yaw is not None else wrap(self.goal[2]-robot[2])
            if not self._try_heading_correction(error, self._parked):
                self._recover(f'final heading error {yaw_error:.3f} rad', self._retreat_for_retry)
            return
        self.parked_station, self.parked_odom = self.goal, odom
        self.state = 'arrived'
        self._report('parked; final pose verified')

    def _travel(self):
        if self.goal is None:
            self.state = 'undocked'
            self._report('undocked; stopped in the open staging area')
            return
        target = (staging_pose(self.goal, self.config.staging_distance)
                  if self.is_station else self.goal)
        robot = self.node._robot_place()
        if robot is None:
            self.cancel('fresh transit pose unavailable')
            return
        goal = NavigateToPose.Goal(pose=goal_message(
            ((target[0], target[1], robot[2])
             if self.is_station and self.keep_transit_heading else target),
            self.node._frame, self.node.get_clock().now().to_msg()))
        goal.behavior_tree = self.tree if self.is_station else ''
        self.align_attempts = 0
        self._send('staging', goal)

    def _dock(self):
        robot = self.node._robot_place()
        along, lateral, yaw = line_errors(self.goal, robot)
        if abs(yaw) > self.config.approach_yaw_tolerance:
            stage = staging_pose(self.goal, self.config.staging_distance)
            if (math.hypot(robot[0]-stage[0], robot[1]-stage[1]) <= .18 and self.align_attempts < 3):
                self._align()  # localization changed heading while still in the checked open staging area
                return
        if abs(yaw) > self.config.approach_yaw_tolerance:
            if not self._try_heading_correction(-yaw, self._dock):
                self._recover(f'approach heading error {abs(yaw):.3f} rad', self._retreat_for_retry)
            return
        if math.hypot(along, lateral) <= self.config.position_tolerance:
            self._parked()
            return
        if not self._clear_line(robot, self.goal):
            self.cancel('docking corridor blocked; no direct-to-station fallback')
            return
        self.endpoint = self.goal
        self.line = self.goal
        self.odom_line = self.node._robot_place('odom')
        if self.odom_line is None:
            self.cancel('fresh odometry transform unavailable for docking')
            return
        self.dock_goal_odom = map_goal_in_odom(robot, self.odom_line, self.goal)
        self.odom_goal_yaw = self.dock_goal_odom[2]
        self._send('dock', self._path_goal(robot, self.goal))

    def _path_goal(self, start, end, checker='dock_position_checker'):
        # Zero stamps use the current map->odom transform throughout the approach.
        # This path bypasses the normal rotation shim and all recovery behaviors.
        path = Path()
        path.header.frame_id = self.node._frame
        count = max(1, math.ceil(math.hypot(end[0] - start[0], end[1] - start[1]) / 0.05))
        path.poses = [goal_message((start[0] + (end[0] - start[0]) * i / count,
                                   start[1] + (end[1] - start[1]) * i / count,
                                   end[2]), self.node._frame, None)
                      for i in range(count + 1)]
        return FollowPath.Goal(path=path, controller_id='DockPath',
                               goal_checker_id=checker)

    def _align(self):
        robot = self.node._robot_place()
        stage = staging_pose(self.goal, self.config.staging_distance)
        if (math.hypot(robot[0] - stage[0], robot[1] - stage[1]) > 0.18 or
                not footprint_clear(self.global_grid, robot, self.config.footprint_half_size,
                                    turning=True)):
            self.cancel('not in a clear staging area; refusing station heading turn')
            return
        error = wrap(self.goal[2] - robot[2])
        if abs(error) <= self.config.approach_yaw_tolerance:
            self._adjust_stage()
            return
        goal = Spin.Goal(target_yaw=error)
        goal.time_allowance.sec = 30
        self.align_attempts += 1
        self._send('align', goal)

    def _adjust_stage(self):
        robot = self.node._robot_place()
        yaw_error = abs(wrap(robot[2] - self.goal[2]))
        if yaw_error > self.config.approach_yaw_tolerance:
            stage = staging_pose(self.goal, self.config.staging_distance)
            if math.hypot(robot[0]-stage[0], robot[1]-stage[1]) <= .18:
                if self.align_attempts < 3:
                    self._align()  # retry only here, with the full turn disk checked again
                else:
                    self.cancel('staging heading did not converge after 3 alignment attempts')
                return
            if not self._try_heading_correction(wrap(self.goal[2]-robot[2]), self._adjust_stage):
                self._recover(f'approach heading error {yaw_error:.3f} rad', self._retreat_for_retry)
            return
        stage = staging_pose(self.goal, self.config.approach_distance)
        if math.hypot(robot[0] - stage[0], robot[1] - stage[1]) <= 0.04:
            self._dock()
            return
        if not self._clear_line(robot, stage):
            self.cancel('staging position correction blocked')
            return
        self.endpoint = stage
        self.odom_line = self.node._robot_place('odom')
        if self.odom_line is None:
            self.cancel('fresh odometry transform unavailable at staging')
            return
        self.stage_goal_odom = map_goal_in_odom(robot, self.odom_line, stage)
        self._send('stage_adjust', self._path_goal(robot, stage, 'dock_alignment_checker'))

    def _straight(self, leg, distance):
        self.odom_line = self.node._robot_place('odom')
        if self.odom_line is None:
            self.cancel('fresh odometry transform unavailable for straight leg')
            return
        self.odom_endpoint = staging_pose(self.odom_line, distance)
        goal = BackUp.Goal()
        goal.target.x = -distance
        goal.speed = self.config.speed
        goal.time_allowance.sec = math.ceil(distance / self.config.speed * 3.0 + 5.0)
        self._send(leg, goal)

    def _send(self, leg, goal):
        self.state = leg
        self.handle = None
        self.close_since = None
        self.progress_pose = self.node._robot_place('odom')
        self.progress_time = time.monotonic()
        self.deadline = time.monotonic() + (180.0 if leg == 'staging' else 60.0)
        message = ('navigate to open space' if leg == 'staging' else
                   'rotate only in the checked open staging area' if leg == 'align' else
                   'slow bounded heading correction in checked clearance' if leg == 'dock_align' else
                   'translate while holding heading; no in-place turn')
        self._report(f'{leg}: {message}')
        future = self.clients[leg].send_goal_async(goal)
        self.pending.append(future)
        future.add_done_callback(partial(self._accepted, self.seq, leg))

    def _accepted(self, seq, leg, future):
        try:
            handle = future.result()
        except Exception as exc:
            if seq == self.seq:
                self.cancel(f'action submission failed: {exc}')
            return
        if seq != self.seq:
            if handle.accepted:
                self.pending.append(handle.cancel_goal_async())
                self.pending.append(handle.get_result_async())
            return
        if not handle.accepted:
            self.cancel(f'{leg} rejected by Nav2')
            return
        self.handle = handle
        future = handle.get_result_async()
        self.pending.append(future)
        future.add_done_callback(partial(self._result, seq, leg))

    def _result(self, seq, leg, future):
        if seq != self.seq:
            return
        self.handle = None
        try:
            result = future.result()
            ok = (result.status == GoalStatus.STATUS_SUCCEEDED and getattr(result.result, "error_code", 0) == 0)
        except Exception as exc:
            self.cancel(f'{leg} result unavailable: {exc}')
            return
        if not ok:
            if result.status == GoalStatus.STATUS_ABORTED and leg == 'dock_align':
                self._recover('fine heading correction aborted', self._retreat_for_retry)
                return
            # A preemption/cancel is an external stop, never a reason to restart.
            # Humble has no action error_code; ABORTED is still distinguishable.
            if result.status == GoalStatus.STATUS_ABORTED and leg in ('staging', 'stage_adjust', 'dock'):
                if leg == 'dock':
                    robot = self.node._robot_place()
                    if robot is not None and math.hypot(robot[0]-self.goal[0], robot[1]-self.goal[1]) <= self.config.position_tolerance:
                        self._settle(self._parked)
                        return
                resume = self._travel if leg == 'staging' else self._adjust_stage if leg == 'stage_adjust' else self._dock
                self._recover(f'{leg} controller aborted', resume)
                return
            detail = ''
            if leg in ('dock', 'stage_adjust'):
                robot = self.node._robot_place()
                if robot is not None:
                    detail = f', position error {math.hypot(robot[0] - self.endpoint[0], robot[1] - self.endpoint[1]):.3f} m'
            self.cancel(f'{leg} failed/cancelled (status {result.status}, error {getattr(result.result, "error_code", 0)}{detail})')
            return
        self._settle(partial(self._after_leg, leg))

    def _recover(self, reason, callback):
        """Keep the same public mission alive, with two stopped retries total.

        Never recover operator cancellation, missing perception or an e-stop.
        Invalidate old callbacks and wait for terminal actions before resuming.
        """
        if self.estop or not self._fresh() or self.node._robot_place() is None:
            self.cancel(f'{reason}; recovery unavailable without fresh perception and released e-stop')
            return
        if getattr(self, 'recovery_attempts', 0) >= 2:
            self.cancel(f'{reason}; automatic recovery exhausted (2 retries)')
            return
        self.recovery_attempts = getattr(self, 'recovery_attempts', 0) + 1
        self.seq += 1
        if self.handle is not None:
            self.pending.append(self.handle.cancel_goal_async())
        self.handle = None
        self._settle(callback, wait_actions=True, delay=1.0)
        self._report(f'recovering {self.recovery_attempts}/2: {reason}; waiting stopped before retry')

    def _correction_clear(self, error):
        robot = self.node._robot_place()
        if robot is None:
            return False
        for grid in (self.global_grid, self.local_grid):
            pose = self._in_grid(grid, robot)
            if pose is None:
                return False
            count = max(1, math.ceil(abs(error)/.02))
            for i in range(count+1):
                if not footprint_clear(grid, (pose[0], pose[1], pose[2]+error*i/count),
                                       self.config.footprint_half_size):
                    return False
        return True

    def _try_heading_correction(self, error, callback):
        # A small swept-angle check permits fine correction where a complete
        # 360-degree turn would not fit. Native Spin collision checking and the
        # collision monitor remain in the velocity chain throughout the turn.
        if (not self.stopped or self.estop or not self._fresh() or
                not math.isfinite(error) or abs(error) > MAX_STATION_YAW_ERROR or
                getattr(self, 'correction_attempts', 0) >= 2 or not self._correction_clear(error)):
            return False
        odom = self.node._robot_place('odom')
        if odom is None:
            return False
        self.correction_attempts = getattr(self, 'correction_attempts', 0)+1
        self.correction_start_odom = odom[2]
        self.correction_target_odom = odom[2]+error
        self.after_correction = callback
        goal = Spin.Goal(target_yaw=error)
        goal.time_allowance.sec = 15
        self._send('dock_align', goal)
        return True

    def _retreat_for_retry(self):
        """Withdraw on the current body axis; realign only in checked open space."""
        robot = self.node._robot_place()
        along, _, yaw = line_errors(self.goal, robot)
        if abs(yaw) > MAX_STATION_YAW_ERROR:
            self.cancel(f'recovery heading error {abs(yaw):.3f} rad exceeds withdrawal range')
            return
        distance = max(0., (self.config.staging_distance + along) / math.cos(yaw))
        exit_pose = staging_pose(robot, distance)
        if (not self._clear_line(robot, exit_pose) or
                not footprint_clear(self.global_grid, exit_pose, self.config.footprint_half_size, turning=True)):
            self.cancel('recovery withdrawal corridor or turning area blocked')
            return
        if distance <= .02:
            self._travel()
            return
        self.endpoint, self.line = exit_pose, self.goal
        self._straight('undock', distance)

    def _after_leg(self, leg):
        robot = self.node._robot_place()
        # Nav2 already reports completion of intermediate actions. Only verify
        # that withdrawal reached room to turn; final parking is checked once
        # against the actual station by _parked, below.
        if leg == 'undock' and not footprint_clear(
                self.global_grid, robot, self.config.footprint_half_size, turning=True):
            self.cancel('withdrawal has not reached clear turning space')
            return
        if leg == 'dock_align':
            self.after_correction()
        elif leg == 'undock':
            self._travel()
        elif leg == 'staging' and self.is_station:
            self._align()
        elif leg == 'align':
            self._adjust_stage()
        elif leg == 'stage_adjust':
            self._dock()
        elif leg == 'dock':
            self._parked()
        else:
            self.state = 'arrived'
            self._report('parked; no terminal rotation' if self.is_station else 'arrived')

    def _tick(self):
        if not self.active:
            return
        now = time.monotonic()
        robot = self.node._robot_place()
        if self.estop or not self._fresh() or robot is None:
            self.cancel('e-stop, stale odometry/costmap or missing/freshness-invalid TF')
            return
        if now > getattr(self, 'mission_deadline', math.inf):
            self.cancel('mission recovery time limit reached')
            return
        if now > self.deadline:
            self.cancel('maneuver timed out')
            return
        if self.state in ('preparing', 'settling'):
            if self.state == 'settling' and self.settle_wait_actions:
                self.pending = [f for f in self.pending if not f.done()]
                if self.pending:
                    return
            if self.state == 'preparing':
                # An old goal can be accepted AFTER X/cancel. Wait for its late
                # cancellation and terminal result before starting a new mission.
                self.pending = [f for f in self.pending if not f.done()]
                if self.pending:
                    return
                if not all(f.done() for f in self.cancels):
                    return
                try:
                    if any(f.result().return_code != 0 for f in self.cancels):
                        self.cancel('previous navigation could not be cancelled')
                        return
                except Exception as exc:
                    self.cancel(f'cancellation failed: {exc}')
                    return
            if not self.stopped:
                self.still_since = None
            elif self.still_since is None:
                self.still_since = now
            elif now - self.still_since >= (getattr(self, 'settle_delay', .5) if self.state == 'settling' else .5):
                callback = self._begin if self.state == 'preparing' else self.after_settle
                callback()
        elif self.state == 'dock_align':
            odom = self.node._robot_place('odom')
            if odom is not None and abs(wrap(odom[2]-self.correction_start_odom)) > .30:
                self.cancel('fine correction exceeded its turn limit')
            elif odom is None or not self._correction_clear(wrap(self.correction_target_odom-odom[2])):
                self.cancel('fine correction clearance lost; stopping turn')
        elif self.state in ('dock', 'undock', 'stage_adjust'):
            # AMCL can update map->odom while driving. Detect physical drift in
            # continuous odom, rather than interpreting a localization update as
            # a turn of the base. Final arrival is still verified in map.
            odom = self.node._robot_place('odom')
            if odom is None:
                self.cancel('fresh odometry transform lost during straight leg')
                return
            heading_target = (self.odom_goal_yaw if self.state == 'dock' else
                              self.stage_goal_odom[2] if self.state == 'stage_adjust' else self.odom_endpoint[2])
            yaw_error = abs(wrap(odom[2]-heading_target))
            if yaw_error > MAX_STATION_YAW_ERROR:
                self.cancel(f'heading deviation {yaw_error:.3f} rad exceeds moving correction range; '
                            'no in-place correction')
                return
            if self.state in ('dock', 'stage_adjust'):
                previous = getattr(self, 'progress_pose', None)
                if previous is None or math.hypot(odom[0]-previous[0], odom[1]-previous[1]) >= .02:
                    self.progress_pose, self.progress_time = odom, now
                elif now - self.progress_time >= 8.0 and self.handle is not None:
                    # Recover before Nav2's 20 s progress deadline. An obstacle
                    # still prevents motion; retries never bypass the costmaps.
                    resume = self._dock if self.state == 'dock' else self._adjust_stage
                    self._recover('translation stalled for 8 s', resume)
                    return
            if self.state == 'dock':
                close = (self.stopped and
                         math.hypot(robot[0] - self.goal[0], robot[1] - self.goal[1]) <=
                         self.config.position_tolerance and
                         abs(wrap(odom[2] - self.odom_goal_yaw)) <= self.config.yaw_tolerance)
                if not close:
                    self.close_since = None
                elif self.close_since is None:
                    self.close_since = now
                elif now - self.close_since >= 1.0:
                    # Below the drive's useful minimum speed, FollowPath may
                    # stall outside the controller target but inside the station
                    # acceptance limit. Accept only a stopped base, after
                    # cancellation is acknowledged and the action is terminal.
                    self.seq += 1
                    if self.handle is not None:
                        self.pending.append(self.handle.cancel_goal_async())
                    self.handle = None
                    self._settle(partial(self._after_leg, 'dock'), wait_actions=True)
                    self._report('verifying stopped parking pose inside configured tolerance')

