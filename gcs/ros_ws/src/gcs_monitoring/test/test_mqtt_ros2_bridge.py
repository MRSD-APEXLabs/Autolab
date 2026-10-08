import json
import math

from gcs_monitoring.mqtt_ros2_bridge import (
    STRING_CMD_TOPIC_MAP,
    TYPED_CMD_TOPIC_MAP,
    _build_goal_pose,
    _build_manipulation_command,
    _build_ot2_command,
    _build_shaker_command,
)


def test_build_goal_pose_builds_map_frame_pose_stamped():
    msg = _build_goal_pose({'x': 1.5, 'y': -2.0, 'theta': 90})
    assert msg.header.frame_id == 'map'
    assert msg.pose.position.x == 1.5
    assert msg.pose.position.y == -2.0
    assert msg.pose.position.z == 0.0


def test_build_goal_pose_converts_theta_degrees_to_yaw_quaternion():
    msg = _build_goal_pose({'x': 0, 'y': 0, 'theta': 90})
    q = msg.pose.orientation
    assert q.x == 0.0
    assert q.y == 0.0
    assert math.isclose(q.z, math.sin(math.radians(90) / 2.0))
    assert math.isclose(q.w, math.cos(math.radians(90) / 2.0))


def test_build_goal_pose_zero_theta_is_identity_orientation():
    msg = _build_goal_pose({'x': 1, 'y': 2, 'theta': 0})
    assert msg.pose.orientation.z == 0.0
    assert msg.pose.orientation.w == 1.0


def test_goal_pose_routes_through_behavior_tree():
    ros2_topic, _msg_type, _builder = TYPED_CMD_TOPIC_MAP['cmd/navigation/goal_pose']
    assert ros2_topic == '/robot_1/behavior/navigate_to_pose_command'


def test_go_to_location_topic_routes_through_behavior_tree():
    assert (
        STRING_CMD_TOPIC_MAP['cmd/navigation/go_to_location']
        == '/robot_1/behavior/go_to_location_command'
    )


def test_planning_command_topic_is_registered():
    assert STRING_CMD_TOPIC_MAP['cmd/planning/command'] == '/planning_command'


def test_perception_camera_mode_cmd_topic_is_registered():
    assert (
        STRING_CMD_TOPIC_MAP['cmd/perception/camera_mode_cmd']
        == '/robot_1/behavior/perception/camera_mode_cmd'
    )


def test_build_manipulation_command_maps_all_fields():
    msg = _build_manipulation_command({
        'type': 'place', 'object_type': 'well_plate', 'target_machine': 'ot2',
    })
    assert msg.type == 'place'
    assert msg.object_type == 'well_plate'
    assert msg.target_machine == 'ot2'


def test_build_manipulation_command_defaults_target_machine_to_empty():
    msg = _build_manipulation_command({'type': 'pick_up', 'object_type': 'well_plate'})
    assert msg.target_machine == ''


def test_build_ot2_command_passes_through_string_parameters_json():
    payload = '{"steps": []}'
    msg = _build_ot2_command({'action': 'protocol', 'parameters_json': payload})
    assert msg.device == 'ot2'
    assert msg.action == 'protocol'
    assert msg.parameters_json == payload


def test_build_ot2_command_serializes_dict_parameters_json():
    msg = _build_ot2_command({'parameters_json': {'steps': []}})
    assert json.loads(msg.parameters_json) == {'steps': []}


def test_build_ot2_command_defaults_action_to_protocol():
    msg = _build_ot2_command({'parameters_json': '{}'})
    assert msg.action == 'protocol'


def test_build_shaker_command_wraps_pwm_and_wait_time():
    msg = _build_shaker_command({'pwm': 200, 'wait_time_s': 20})
    assert msg.device == 'shaker'
    assert msg.action == 'protocol'
    assert json.loads(msg.parameters_json) == {'pwm': 200, 'wait_time_s': 20}


def test_build_shaker_command_defaults_wait_time_to_zero():
    msg = _build_shaker_command({'pwm': 100})
    assert json.loads(msg.parameters_json)['wait_time_s'] == 0
