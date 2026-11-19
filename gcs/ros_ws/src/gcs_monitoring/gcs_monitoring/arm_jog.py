"""Arm jogging for the MQTT→ROS2 bridge: payloads and act_inference's jog service names.

No ROS imports, so it is testable without ROS; the bridge passes its rclpy service client to
camera_select.call_select, which makes the Trigger call.
"""
import json

JOG_TOPIC = 'cmd/manipulation/jog'
JOG_RESULT_TOPIC = 'manipulation/jog_result'
# act_inference's std_srvs/Trigger services: x/y/z move the TCP jog_step_mm (10) along that
# xArm base-frame axis, holding orientation; rz turns it (and the wrist camera) jog_step_deg (10)
# about the base z axis, holding XYZ; home moves it straight to the home_offset pose (no planning);
# clear_errors clears the arm's errors and enables it again (no motion). Each answers once it has ended.
JOG_SERVICES = {
    f'{axis}_{direction}': f'/act_inference/jog/{axis}_{direction}'
    for axis in ('x', 'y', 'z', 'rz') for direction in ('plus', 'minus')
}
JOG_SERVICES['home'] = '/act_inference/jog/home'
JOG_SERVICES['clear_errors'] = '/act_inference/clear_errors'
JOG_MISSING = 'act_inference node is not running'
# Controller/SDK handoff, the motion (home: up to ~25 s at 40 mm/s) and the joint resync: under this.
JOG_TIMEOUT_S = 60.0


def parse_jog_payload(payload: bytes) -> str:
    """Jog name ("x_plus" … "z_minus", "rz_plus", "rz_minus", "home", "clear_errors") from a JSON string payload."""
    try:
        jog = json.loads(payload.decode('utf-8'))
    except (UnicodeDecodeError, ValueError) as exc:
        raise ValueError(f'jog payload is not JSON: {exc}') from None
    if not isinstance(jog, str) or jog not in JOG_SERVICES:
        raise ValueError(f'unknown jog {jog!r}')
    return jog


def result_payload(jog, ok, message) -> str:
    return json.dumps({'jog': jog, 'ok': bool(ok), 'message': str(message)})
