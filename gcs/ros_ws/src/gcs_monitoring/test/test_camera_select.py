"""Camera-hub switch helpers of the MQTT→ROS2 bridge (no ROS)."""
import json
import threading

import pytest

from gcs_monitoring.camera_select import (
    CAMERA_SERVICES,
    SELECT_CAMERA_TOPIC,
    SELECT_RESULT_TOPIC,
    call_select,
    parse_select_payload,
    result_payload,
)


def test_topics_and_services():
    assert SELECT_CAMERA_TOPIC == 'cmd/perception/select_camera'
    assert SELECT_RESULT_TOPIC == 'perception/select_camera_result'
    assert CAMERA_SERVICES == {'zedx': '/camera_hub/select_zedx',
                               'zedx_nano': '/camera_hub/select_zedx_nano',
                               None: '/camera_hub/release'}


@pytest.mark.parametrize('payload, camera', [
    (b'"zedx"', 'zedx'), (b'"zedx_nano"', 'zedx_nano'), (b'null', None)])
def test_parse_select_payload(payload, camera):
    assert parse_select_payload(payload) == camera


@pytest.mark.parametrize('payload', [
    b'zedx', b'"other"', b'["zedx"]', b'{}', b'42', b'\xff', b''])
def test_parse_select_payload_rejects(payload):
    with pytest.raises(ValueError):
        parse_select_payload(payload)


def test_result_payload():
    assert json.loads(result_payload('zedx', 1, 'ok')) == {
        'camera': 'zedx', 'ok': True, 'message': 'ok'}


class Future:
    def __init__(self):
        self._callbacks, self._result = [], None

    def add_done_callback(self, cb):
        self._callbacks.append(cb)

    def finish(self, result):
        self._result = result
        for cb in self._callbacks:
            cb(self)

    def done(self):
        return self._result is not None

    def result(self):
        return self._result


class Response:
    def __init__(self, success, message):
        self.success, self.message = success, message


class Client:
    srv_name = '/camera_hub/select_zedx'

    def __init__(self, available=True, response=None, delay=0.0):
        self.available, self.response, self.delay = available, response, delay
        self.removed = []

    def wait_for_service(self, timeout_sec):
        return self.available

    def call_async(self, request):
        future = Future()
        if self.response is not None:
            threading.Timer(self.delay, future.finish, args=(self.response,)).start()
        return future

    def remove_pending_request(self, future):
        self.removed.append(future)


def test_call_select_success_and_failure():
    assert call_select(Client(response=Response(True, 'zedx streaming')), object()) == (
        True, 'zedx streaming')
    assert call_select(Client(response=Response(False, 'busy')), object()) == (False, 'busy')


def test_call_select_service_missing():
    assert call_select(Client(available=False), object()) == (
        False, 'camera_hub node is not running')


def test_call_select_timeout_removes_pending_request():
    client = Client(response=None)
    ok, message = call_select(client, object(), timeout_s=0.1)
    assert not ok and message == 'no reply from /camera_hub/select_zedx within 0.1 s'
    assert len(client.removed) == 1
