import sys
import os
from unittest.mock import MagicMock

# Stub out ROS2 modules so run_routine.py can be imported outside a ROS2 environment.
# Do NOT mock 'routine_executor' itself — that would break imports in other test files.
for mod in ('rclpy', 'rclpy.node', 'std_msgs', 'std_msgs.msg'):
    sys.modules.setdefault(mod, MagicMock())
# Mock step_config specifically so parse_step/format_step can be imported
if 'routine_executor.step_config' not in sys.modules:
    mock_sc = MagicMock()
    mock_sc.STEPS = {}
    mock_sc.KNOWN_STEPS = []
    sys.modules['routine_executor.step_config'] = mock_sc

import pytest

# parse_step and format_step have no ROS2 dependency — import directly from the script
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from run_routine import parse_step, format_step


# --- parse_step ---

def test_parse_step_name_only():
    assert parse_step('pick_base') == {'name': 'pick_base'}


def test_parse_step_single_string_param():
    assert parse_step('place:target_machine=ot2') == {'name': 'place', 'target_machine': 'ot2'}


def test_parse_step_int_cast():
    result = parse_step('shaker:pwm=200')
    assert result == {'name': 'shaker', 'pwm': 200}
    assert isinstance(result['pwm'], int)


def test_parse_step_float_cast():
    result = parse_step('shaker:pwm=1.5')
    assert result == {'name': 'shaker', 'pwm': 1.5}
    assert isinstance(result['pwm'], float)


def test_parse_step_string_stays_string():
    result = parse_step('place:target_machine=ot2')
    assert isinstance(result['target_machine'], str)


def test_parse_step_multiple_params():
    result = parse_step('step:a=1,b=hello')
    assert result == {'name': 'step', 'a': 1, 'b': 'hello'}


def test_parse_step_strips_whitespace_from_key_and_value():
    result = parse_step('shaker: pwm = 200')
    assert result == {'name': 'shaker', 'pwm': 200}


def test_parse_step_invalid_param_raises():
    with pytest.raises(ValueError):
        parse_step('shaker:badparam')


def test_parse_step_value_with_equals_sign():
    # value containing '=' should not be split further
    result = parse_step('ot2:parameters_json=a=b')
    assert result['parameters_json'] == 'a=b'


# --- format_step ---

def test_format_step_no_extras():
    assert format_step({'name': 'pick_base'}) == 'pick_base'


def test_format_step_with_params():
    assert format_step({'name': 'place', 'target_machine': 'ot2'}) == 'place(target_machine=ot2)'


def test_format_step_multiple_params():
    result = format_step({'name': 'step', 'a': 1, 'b': 'x'})
    assert result == 'step(a=1,b=x)'
