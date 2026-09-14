import json

from gcs_monitoring.mqtt_ros2_bridge import (
    STRING_CMD_TOPIC_MAP,
    _build_goal_pose,
    _build_manipulation_command,
    _build_ot2_command,
    _build_shaker_command,
)


def test_build_goal_pose_converts_all_fields():
    msg = _build_goal_pose({'x': 1.5, 'y': -2.0, 'theta': 90})
    assert msg.x == 1.5
    assert msg.y == -2.0
    assert msg.theta == 90.0


def test_build_goal_pose_coerces_ints_to_float():
    msg = _build_goal_pose({'x': 1, 'y': 2, 'theta': 3})
    assert isinstance(msg.x, float)
    assert isinstance(msg.y, float)
    assert isinstance(msg.theta, float)


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
