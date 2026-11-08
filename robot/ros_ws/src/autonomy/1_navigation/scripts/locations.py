#!/usr/bin/python3
"""Named places in the map: save the robot's current pose, list them, drive to one.

Run through scripts/locations.sh (it sources the ROS environment):
  scripts/locations.sh save OT-2     store the current pose + heading as "OT-2"   (listens only)
  scripts/locations.sh list          show all saved places
  scripts/locations.sh goto OT-2     send "OT-2" to Nav2 as a goal - THE ROBOT DRIVES THERE
  scripts/locations.sh undock       withdraw straight from a station, then stop

Places are stored in maps/<map>.locations.yaml (default map: "best"; change with --map NAME).
The coordinates are in the frame of that map: they stay valid wherever the robot is started, as
long as the same map is loaded, and they are lost in a new map.
"""

import argparse
import datetime
import math
import os
import sys
import time

import yaml

MAPS = os.environ.get(
    'SWERVE_NAV_MAPS', os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'navigation_bringup', 'maps'))


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def load(path):
    if not os.path.exists(path):
        return {}
    with open(path) as handle:
        return (yaml.safe_load(handle) or {}).get('locations', {})


def store(path, map_name, locations):
    # Keep per-map docking settings when updating a saved pose.
    document = {}
    if os.path.exists(path):
        with open(path) as handle:
            document = yaml.safe_load(handle) or {}
    document.update(map=map_name, locations=locations)
    text = (f'# Named places in the frame of maps/{map_name}.yaml (x, y in metres; yaw in radians,\n'
            '# counter-clockwise from the map x axis). Written by scripts/locations.sh.\n'
            + yaml.safe_dump(document, sort_keys=False))
    tmp = path + '.tmp'
    with open(tmp, 'w') as handle:
        handle.write(text)
    os.replace(tmp, path)


def read_pose(seconds=6.0):
    """Listen (publishes nothing) for map -> base_footprint and the AMCL uncertainty."""
    from geometry_msgs.msg import PoseWithCovarianceStamped
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node('locations_reader')
    buffer = Buffer()
    TransformListener(buffer, node)
    amcl = []
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', amcl.append, latched)
    samples = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if buffer.can_transform('map', 'base_footprint', Time()):
            t = buffer.lookup_transform('map', 'base_footprint', Time()).transform
            samples.append((t.translation.x, t.translation.y, yaw_of(t.rotation)))
    node.destroy_node()
    rclpy.shutdown()
    if not samples:
        sys.exit('ERROR: no map -> base_footprint transform. Is scripts/3_navigate.sh running '
                 '(same ROS_DOMAIN_ID) and is the robot localised?')
    recent = samples[-20:]
    moved = max(math.hypot(a[0] - recent[0][0], a[1] - recent[0][1]) for a in recent)
    if moved > 0.02:
        sys.exit(f'ERROR: the robot is moving ({moved:.2f} m in the last 2 s). Stop it, then save.')
    std = None
    if amcl:
        c = amcl[-1].pose.covariance
        std = [round(math.sqrt(max(c[i], 0.0)), 3) for i in (0, 7, 35)]
    return samples[-1], std


def send_goal(x=0.0, y=0.0, yaw=0.0, undock=False):
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Empty
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node('locations_goal')
    topic = '/station_undock' if undock else '/goal_pose_staged'
    publisher = node.create_publisher(Empty if undock else PoseStamped, topic, 1)
    end = time.monotonic() + 5.0
    while publisher.get_subscription_count() == 0 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if publisher.get_subscription_count() == 0:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(f'ERROR: nobody listens on {topic}. Is navigation running with docking enabled?')
    goal = Empty() if undock else PoseStamped()
    if not undock:
        goal.header.frame_id = 'map'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x, goal.pose.position.y = float(x), float(y)
        goal.pose.orientation.z, goal.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    publisher.publish(goal)
    for _ in range(5):      # let the message leave before the node goes away
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('command', choices=['save', 'list', 'goto', 'undock'])
    parser.add_argument('name', nargs='?')
    parser.add_argument('--map', default='best', help='map the places belong to (default: best)')
    args = parser.parse_args()
    path = os.path.join(MAPS, f'{args.map}.locations.yaml')
    locations = load(path)

    if args.command == 'undock':
        print('Withdrawing straight from the current station. X cancels, B e-stops.')
        send_goal(undock=True)
        return

    if args.command == 'list':
        if not locations:
            print(f'no places saved yet in {path}')
        for name, p in locations.items():
            print(f"{name:20s} x={p['x']:.3f}  y={p['y']:.3f}  yaw={p['yaw']:.4f} rad "
                  f"({math.degrees(p['yaw']):.1f} deg)  saved {p.get('saved', '?')}")
        return
    if not args.name:
        parser.error(f'{args.command} needs a name')

    if args.command == 'save':
        (x, y, yaw), std = read_pose()
        locations[args.name] = {
            'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 4),
            'saved': datetime.datetime.now().isoformat(timespec='seconds'),
            'amcl_std_x_y_yaw': std}
        store(path, args.map, locations)
        print(f"saved '{args.name}': x={x:.3f} y={y:.3f} yaw={yaw:.4f} rad "
              f'({math.degrees(yaw):.1f} deg) -> {path}')
        if std and (std[0] > 0.15 or std[1] > 0.15 or std[2] > 0.15):
            print(f'WARNING: AMCL is unsure of this pose (std {std[0]} m, {std[1]} m, {std[2]} '
                  'rad). Check that the live scan lies on the map walls; if not, fix the pose '
                  'estimate, drive a little and save again.')
        return

    if args.name not in locations:
        sys.exit(f"ERROR: no place '{args.name}' in {path}. Known: {', '.join(locations) or '-'}")
    p = locations[args.name]
    print(f"Sending the robot to '{args.name}' (x={p['x']} y={p['y']} yaw={p['yaw']}). "
          'X on the pad cancels, B is the e-stop.')
    send_goal(p['x'], p['y'], p['yaw'])


if __name__ == '__main__':
    main()
