"""Camera-hub switching for the MQTT→ROS2 bridge: payloads, service names and the Trigger call.

No ROS imports, so it is testable without ROS; the bridge passes in its rclpy service client.
"""
import json
import threading

SELECT_CAMERA_TOPIC = 'cmd/perception/select_camera'
SELECT_RESULT_TOPIC = 'perception/select_camera_result'
# camera_hub's std_srvs/Trigger services; None releases every camera
CAMERA_SERVICES = {
    'zedx': '/camera_hub/select_zedx',
    'zedx_nano': '/camera_hub/select_zedx_nano',
    None: '/camera_hub/release',
}
SERVICE_WAIT_S = 2.0
# The hub answers within its select_timeout (30 s) plus HTTP and status polls: 39 s worst case.
SELECT_TIMEOUT_S = 45.0


def parse_select_payload(payload: bytes):
    """Camera name, or None to release, from a JSON payload ("zedx", "zedx_nano" or null)."""
    try:
        camera = json.loads(payload.decode('utf-8'))
    except (UnicodeDecodeError, ValueError) as exc:
        raise ValueError(f'select payload is not JSON: {exc}') from None
    if not (camera is None or isinstance(camera, str)) or camera not in CAMERA_SERVICES:
        raise ValueError(f'unknown camera {camera!r}')
    return camera


def result_payload(camera, ok, message) -> str:
    return json.dumps({'camera': camera, 'ok': bool(ok), 'message': str(message)})


def call_select(client, request, service_wait_s=SERVICE_WAIT_S, timeout_s=SELECT_TIMEOUT_S):
    """Call a Trigger client and wait for its reply: (ok, message).

    Blocks; run it off the MQTT thread.
    """
    if not client.wait_for_service(timeout_sec=service_wait_s):
        return False, 'camera_hub node is not running'
    done = threading.Event()
    future = client.call_async(request)
    future.add_done_callback(lambda _: done.set())
    if not future.done() and not done.wait(timeout_s):
        client.remove_pending_request(future)
        return False, f'no reply from {client.srv_name} within {timeout_s:g} s'
    result = future.result()
    return bool(result.success), result.message
