"""Hardware-free regressions for F/T acquisition and policy conditioning."""
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from act_inference.config import MotionConfig
from act_inference.robot import XArm6Robot


class Sensor:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.mode = self.error = self.code = self.error_queries = 0
        self.data = np.arange(6.)
        self.calls = []

    def get_ft_sensor_config(self):
        return 0, [self.mode, int(self.enabled)]

    def set_ft_sensor_enable(self, enabled):
        self.enabled = bool(enabled)
        self.calls.append(enabled)
        return 0

    def get_ft_sensor_error(self):
        self.error_queries += 1
        return 0, self.error

    def get_ft_sensor_data(self, is_raw=False):
        assert not is_raw
        return self.code, self.data

    def disconnect(self):
        self.calls.append('disconnect')


def test_sensor_polling_enable_restore_and_faults():
    Robot = XArm6Robot
    api = Sensor(enabled=False)
    robot = Robot('unused', MotionConfig(), read_only=True)
    robot._arm = api
    with pytest.raises(RuntimeError, match='sampling is disabled'):
        robot.prepare_force_torque(enable=True)
    assert not api.calls
    robot.read_only = False
    robot.prepare_force_torque(enable=True)
    np.testing.assert_array_equal(robot.force_torque(), np.arange(6.))
    api.data = np.array([0., 1, 2, 3, 4, np.nan])
    with pytest.raises(RuntimeError, match='Invalid F/T'):
        robot.force_torque()
    api.code = 9
    with pytest.raises(RuntimeError, match='get_ft_sensor_data'):
        robot.force_torque()
    api.code, api.data = 0, np.arange(6.)
    queries = api.error_queries
    for _ in range(3):
        robot.force_torque()
    assert queries == 1 and api.error_queries == 1   # checked at prepare only: the query stops an enabled arm
    api.error = 1
    with pytest.raises(RuntimeError, match='sensor error'):
        robot.prepare_force_torque(enable=True)
    robot._enabled = True
    robot.prepare_force_torque(enable=True)            # never queried once the arm is enabled
    assert api.error_queries == 2
    robot._enabled = False
    api.enabled = False
    with pytest.raises(RuntimeError, match='sampling disabled'):
        robot.force_torque()
    robot.close()
    assert api.calls == [1, 0, 'disconnect']


def test_existing_sampling_is_never_disabled():
    robot = XArm6Robot('unused', MotionConfig(), read_only=True)
    api = robot._arm = Sensor()
    robot.prepare_force_torque()
    robot.close()
    assert api.enabled and api.calls == ['disconnect']



@pytest.mark.parametrize('execute', [False, True])
def test_live_runner_passes_current_sensor_readings_to_every_query(monkeypatch, execute):
    from test_inference_rollout import setup_fake
    import act_inference.rollout as module
    from act_inference.config import InferenceConfig
    seen, prepared, reads = [], [], []

    def predict(*args, force_torque=None):
        seen.append(force_torque.copy())
        return np.zeros((50, 6))

    calls = setup_fake(monkeypatch, predict)
    Base = module.XArm6Robot

    class Robot(Base):
        def prepare_force_torque(self, enable=False):
            prepared.append(enable)

        def force_torque(self):
            reads.append(len(reads) + 1)
            return np.full(6, reads[-1])

    monkeypatch.setattr(module, 'XArm6Robot', Robot)
    monkeypatch.setattr(module, 'load_policy', lambda p: (SimpleNamespace(use_force_torque=True), None, None))
    module.run_act_rollout(InferenceConfig(), module.RolloutConfig(Path('unused'), execute=execute,
                                                             duration_sec=.12, query_interval=.02,
                                                             settle_motion=0))
    assert prepared == [execute] and len(seen) >= 2
    np.testing.assert_array_equal([w[0] for w in seen], np.arange(1, len(seen) + 1))
    assert ('enable' in calls) == execute
    assert ('servo' in calls) == execute


