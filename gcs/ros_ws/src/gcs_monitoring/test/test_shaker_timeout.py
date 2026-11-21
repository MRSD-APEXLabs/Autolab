import pytest
from gcs_monitoring.shaker_timeout import ShakerTimeout


class FakeTimer:
    def __init__(self, delay, callback):
        self.delay, self.callback = delay, callback
        self.started = self.cancelled = False

    def start(self):
        self.started = True

    def cancel(self):
        self.cancelled = True


def make_controller():
    sent, timers = [], []
    def factory(delay, callback):
        timer = FakeTimer(delay, callback)
        timers.append(timer)
        return timer
    return ShakerTimeout(sent.append, factory), sent, timers


def test_timeout_sends_zero_after_selected_delay():
    controller, sent, timers = make_controller()
    controller.start(150, 20)
    assert sent == [150]
    assert timers[0].delay == 20 and timers[0].started
    timers[0].callback()
    assert sent == [150, 0]


def test_new_command_replaces_old_timeout_even_if_callback_was_queued():
    controller, sent, timers = make_controller()
    controller.start(150, 20)
    controller.start(200, 30)
    assert timers[0].cancelled
    timers[0].callback()
    assert sent == [150, 200]
    timers[1].callback()
    assert sent == [150, 200, 0]


def test_manual_stop_cancels_pending_timeout():
    controller, sent, timers = make_controller()
    controller.start(150, 20)
    controller.start(0, 20)
    timers[0].callback()
    assert sent == [150, 0]
    assert len(timers) == 1


def test_zero_duration_leaves_manual_operation():
    controller, sent, timers = make_controller()
    controller.start(150, 0)
    assert sent == [150] and not timers


@pytest.mark.parametrize('pwm,duration', [(-1, 20), (256, 20), (1.5, 20),
    (150, -1), (150, float('nan')), (float('inf'), 20), (150, 'oops')])
def test_invalid_input_sends_nothing_and_preserves_existing_stop(pwm, duration):
    controller, sent, timers = make_controller()
    controller.start(150, 20)
    with pytest.raises(ValueError):
        controller.start(pwm, duration)
    assert sent == [150]
    assert not timers[0].cancelled


def test_debug_bridge_sends_start_and_timed_stop_on_same_ros_topic():
    import json
    from types import SimpleNamespace
    from unittest.mock import Mock
    from gcs_monitoring.mqtt_ros2_bridge import MqttRos2Bridge
    publisher = Mock()
    logger = SimpleNamespace(info=lambda text: None, warning=lambda text: None)
    bridge = SimpleNamespace(
        _typed_pubs={'/robot_1/behavior/lab_machine_command': publisher},
        get_logger=lambda: logger)
    timers = []
    def factory(delay, callback):
        timer = FakeTimer(delay, callback)
        timers.append(timer)
        return timer
    bridge._shaker_timeout = ShakerTimeout(
        lambda pwm: MqttRos2Bridge._send_shaker_pwm(bridge, pwm), factory)
    MqttRos2Bridge._relay_typed(bridge, 'cmd/lab_machine/shaker_command',
                              b'{"pwm":150,"wait_time_s":20}')
    assert json.loads(publisher.publish.call_args.args[0].parameters_json)['pwm'] == 150
    assert publisher.publish.call_count == 1
    timers[0].callback()
    stop = publisher.publish.call_args.args[0]
    assert stop.device == 'shaker' and stop.action == 'protocol'
    assert json.loads(stop.parameters_json)['pwm'] == 0
    assert publisher.publish.call_count == 2
