import json
import pytest
from routine_executor.step_config import STEPS, KNOWN_STEPS


def test_all_known_steps_have_entries():
    for step in KNOWN_STEPS:
        assert step in STEPS, f"Step '{step}' listed in KNOWN_STEPS but missing from STEPS"


def test_step_has_required_keys():
    required = {'publish_topic', 'msg_type', 'make_msg', 'watch_topic', 'required_params'}
    for name, config in STEPS.items():
        assert required <= config.keys(), f"Step '{name}' missing keys: {required - config.keys()}"


def test_msg_type_values():
    valid_types = {'manipulation', 'lab_machine'}
    for name, config in STEPS.items():
        assert config['msg_type'] in valid_types, f"Step '{name}' has invalid msg_type: {config['msg_type']}"


def test_required_params_is_list():
    for name, config in STEPS.items():
        assert isinstance(config['required_params'], list), f"Step '{name}' required_params must be a list"


def test_make_msg_returns_object_for_parameterless_steps():
    for name, config in STEPS.items():
        if not config['required_params']:
            msg = config['make_msg']({'name': name})
            assert msg is not None, f"Step '{name}' make_msg() returned None"


def test_make_msg_place_uses_target_machine():
    from behavior_tree_msgs.msg import ManipulationCommand
    msg = STEPS['place']['make_msg']({'name': 'place', 'target_machine': 'ot2'})
    assert isinstance(msg, ManipulationCommand)
    assert msg.target_machine == 'ot2'


def test_make_msg_place_shaker_target():
    from behavior_tree_msgs.msg import ManipulationCommand
    msg = STEPS['place']['make_msg']({'name': 'place', 'target_machine': 'shaker'})
    assert msg.target_machine == 'shaker'


def test_make_msg_shaker_uses_pwm():
    from behavior_tree_msgs.msg import LabMachineCommand
    msg = STEPS['shaker']['make_msg']({'name': 'shaker', 'pwm': 200})
    assert isinstance(msg, LabMachineCommand)
    data = json.loads(msg.parameters_json)
    assert data['pwm'] == 200


def test_make_msg_ot2_uses_parameters_json():
    from behavior_tree_msgs.msg import LabMachineCommand
    payload = '{"steps": []}'
    msg = STEPS['ot2']['make_msg']({'name': 'ot2', 'parameters_json': payload})
    assert isinstance(msg, LabMachineCommand)
    assert msg.parameters_json == payload


def test_watch_topic_format():
    for name, config in STEPS.items():
        topic = config['watch_topic']
        assert isinstance(topic, str) and len(topic) > 0
        assert not topic.startswith('/'), f"Step '{name}' watch_topic should be relative, not absolute"


def test_publish_topic_format():
    for name, config in STEPS.items():
        topic = config['publish_topic']
        assert isinstance(topic, str) and len(topic) > 0
        assert not topic.startswith('/'), f"Step '{name}' publish_topic should be relative, not absolute"


def test_pick_base_watch_topic():
    assert STEPS['pick_base']['watch_topic'] == 'behavior/pick_base_object_status'


def test_place_watch_topic():
    assert STEPS['place']['watch_topic'] == 'behavior/place_object_status'


def test_pick_up_watch_topic():
    assert STEPS['pick_up']['watch_topic'] == 'behavior/pick_up_object_status'


def test_ot2_watch_topic():
    assert STEPS['ot2']['watch_topic'] == 'behavior/execute_ot2_protocol_status'


def test_shaker_watch_topic():
    assert STEPS['shaker']['watch_topic'] == 'behavior/execute_shaker_protocol_status'


def test_place_ot2_and_place_shaker_removed():
    assert 'place_ot2' not in STEPS
    assert 'place_shaker' not in STEPS


def test_place_required_params():
    assert 'target_machine' in STEPS['place']['required_params']


def test_shaker_required_params():
    assert 'pwm' in STEPS['shaker']['required_params']


def test_ot2_required_params():
    assert 'parameters_json' in STEPS['ot2']['required_params']
