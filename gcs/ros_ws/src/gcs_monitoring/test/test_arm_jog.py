"""Arm-jog helpers of the MQTT→ROS2 bridge (no ROS)."""
import json

import pytest

from gcs_monitoring.arm_jog import (
    JOG_MISSING,
    JOG_RESULT_TOPIC,
    JOG_SERVICES,
    JOG_TOPIC,
    parse_jog_payload,
    result_payload,
)
from gcs_monitoring.camera_select import call_select


def test_topics_and_services():
    assert JOG_TOPIC == 'cmd/manipulation/jog'
    assert JOG_RESULT_TOPIC == 'manipulation/jog_result'
    assert JOG_SERVICES == {
        'x_plus': '/act_inference/jog/x_plus', 'x_minus': '/act_inference/jog/x_minus',
        'y_plus': '/act_inference/jog/y_plus', 'y_minus': '/act_inference/jog/y_minus',
        'z_plus': '/act_inference/jog/z_plus', 'z_minus': '/act_inference/jog/z_minus',
        'rz_plus': '/act_inference/jog/rz_plus', 'rz_minus': '/act_inference/jog/rz_minus',
        'home': '/act_inference/jog/home', 'clear_errors': '/act_inference/clear_errors'}


@pytest.mark.parametrize('jog', sorted(JOG_SERVICES))
def test_parse_jog_payload(jog):
    assert parse_jog_payload(json.dumps(jog).encode()) == jog


@pytest.mark.parametrize('payload', [
    b'x_plus', b'"x_up"', b'null', b'["x_plus"]', b'{}', b'42', b'\xff', b''])
def test_parse_jog_payload_rejects(payload):
    with pytest.raises(ValueError):
        parse_jog_payload(payload)


def test_result_payload():
    assert json.loads(result_payload('z_plus', 1, 'jog_z_plus: moved 10 mm')) == {
        'jog': 'z_plus', 'ok': True, 'message': 'jog_z_plus: moved 10 mm'}


class MissingClient:
    srv_name = '/act_inference/jog/x_plus'

    def wait_for_service(self, timeout_sec):
        return False


def test_missing_jog_service_message():
    assert call_select(MissingClient(), object(), missing=JOG_MISSING) == (
        False, 'act_inference node is not running')
