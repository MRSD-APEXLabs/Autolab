import pytest
from routine_executor.step_config import STEPS, KNOWN_STEPS


def test_all_known_steps_have_entries():
    for step in KNOWN_STEPS:
        assert step in STEPS, f"Step '{step}' listed in KNOWN_STEPS but missing from STEPS"


def test_step_has_required_keys():
    required = {'publish_topic', 'msg_type', 'make_msg', 'watch_topic'}
    for name, config in STEPS.items():
        assert required <= config.keys(), f"Step '{name}' missing keys: {required - config.keys()}"


def test_msg_type_values():
    valid_types = {'manipulation', 'lab_machine'}
    for name, config in STEPS.items():
        assert config['msg_type'] in valid_types, f"Step '{name}' has invalid msg_type: {config['msg_type']}"


def test_make_msg_returns_object():
    for name, config in STEPS.items():
        msg = config['make_msg']()
        assert msg is not None, f"Step '{name}' make_msg() returned None"


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
    assert STEPS['pick_base']['watch_topic'] == 'pick_base_object_status'


def test_place_watch_topic():
    assert STEPS['place']['watch_topic'] == 'place_object_status'


def test_pick_up_watch_topic():
    assert STEPS['pick_up']['watch_topic'] == 'pick_up_object_status'


def test_ot2_watch_topic():
    assert STEPS['ot2']['watch_topic'] == 'execute_ot2_protocol_status'


def test_shaker_watch_topic():
    assert STEPS['shaker']['watch_topic'] == 'execute_shaker_protocol_status'
