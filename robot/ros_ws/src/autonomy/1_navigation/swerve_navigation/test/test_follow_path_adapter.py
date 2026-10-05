import inspect
import math

import pytest
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path

from swerve_navigation.follow_path_adapter import FollowPathAdapter, transform_path


def test_execute_callback_is_async_so_action_futures_cannot_starve_executor():
    # FollowPath updates can overlap during replanning and recovery. A synchronous callback that
    # polls an ActionClient future occupies one executor worker per update; once all workers are
    # occupied, the same executor cannot complete those futures and the proxy wedges forever.
    assert inspect.iscoroutinefunction(FollowPathAdapter._execute)


def test_transform_path_uses_one_transform_and_leaves_input_unchanged():
    path = Path()
    path.header.frame_id = 'map'
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp.sec = 123
    pose.pose.position.x = 1.0
    pose.pose.orientation.w = 1.0
    path.poses.append(pose)

    transform = TransformStamped()
    transform.header.frame_id = 'odom'
    transform.child_frame_id = 'map'
    transform.transform.translation.x = 10.0
    transform.transform.translation.y = 20.0
    transform.transform.rotation.z = math.sqrt(0.5)
    transform.transform.rotation.w = math.sqrt(0.5)

    result = transform_path(path, transform, 'odom')

    assert result.header.frame_id == 'odom'
    assert result.poses[0].header.frame_id == 'odom'
    assert result.poses[0].header.stamp.sec == 0
    assert result.poses[0].pose.position.x == pytest.approx(10.0)
    assert result.poses[0].pose.position.y == pytest.approx(21.0)
    assert result.poses[0].pose.orientation.z == pytest.approx(math.sqrt(0.5))
    assert result.poses[0].pose.orientation.w == pytest.approx(math.sqrt(0.5))

    assert path.header.frame_id == 'map'
    assert path.poses[0].header.stamp.sec == 123
    assert path.poses[0].pose.position.x == 1.0
