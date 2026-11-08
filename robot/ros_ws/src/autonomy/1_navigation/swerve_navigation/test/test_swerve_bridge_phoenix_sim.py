"""Unit tests for the phoenix6-free parts of swerve_navigation.phoenix_sim (no phoenix6 is imported)."""

import math
import random
import sys

import pytest

from swerve_navigation.phoenix_sim import apply_friction
from swerve_navigation.phoenix_sim import chassis_omega
from swerve_navigation.phoenix_sim import DCMotorModel
from swerve_navigation.phoenix_sim import PySimSwerve

# The user's drive module: inertia 0.01 kg m^2, 7.03125:1 (tuner_constants.py)
DRIVE_INERTIA, DRIVE_RATIO = 0.01, 7.03125
HALF = 0.2537
LOCATIONS = [(HALF, HALF), (HALF, -HALF), (-HALF, HALF), (-HALF, -HALF)]


def rigid_body_velocities(vx, vy, omega, locations=LOCATIONS):
    return [(vx - omega * y, vy + omega * x) for x, y in locations]


class TestDCMotorModel:

    def test_free_speed_at_nominal_voltage(self):
        motor = DCMotorModel(DRIVE_INERTIA, DRIVE_RATIO)
        # Kraken X60 FOC: 5800 rpm free speed at 12 V (minus the free-current drop), at the rotor.
        rotor_free = 5800.0 * 2.0 * math.pi / 60.0
        mechanism = motor.steady_state_velocity(12.0)
        assert mechanism * DRIVE_RATIO == pytest.approx(rotor_free, rel=0.01)

    def test_converges_to_steady_state_and_integrates_position(self):
        motor = DCMotorModel(DRIVE_INERTIA, DRIVE_RATIO)
        for _ in range(2000):
            motor.step(1.0, 0.004)
        assert motor.velocity == pytest.approx(motor.steady_state_velocity(1.0), rel=1e-6)
        assert 0.0 < motor.position < motor.steady_state_velocity(1.0) * 8.0

    def test_step_is_exact_so_step_size_does_not_matter(self):
        coarse, fine = (DCMotorModel(DRIVE_INERTIA, DRIVE_RATIO) for _ in range(2))
        coarse.step(3.0, 0.1)
        for _ in range(100):
            fine.step(3.0, 0.001)
        assert coarse.velocity == pytest.approx(fine.velocity, rel=1e-9)
        assert coarse.position == pytest.approx(fine.position, rel=1e-9)

    def test_zero_volts_coasts_down_and_sign_symmetry(self):
        forward, backward = (DCMotorModel(DRIVE_INERTIA, DRIVE_RATIO) for _ in range(2))
        forward.step(6.0, 0.5)
        backward.step(-6.0, 0.5)
        assert forward.velocity == pytest.approx(-backward.velocity)
        assert forward.position == pytest.approx(-backward.position)
        before = forward.velocity
        forward.step(0.0, 0.05)
        assert 0.0 <= forward.velocity < before

    def test_zero_dt_changes_nothing(self):
        motor = DCMotorModel(DRIVE_INERTIA, DRIVE_RATIO)
        motor.step(6.0, 0.1)
        state = (motor.position, motor.velocity)
        motor.step(12.0, 0.0)
        assert (motor.position, motor.velocity) == pytest.approx(state)


class TestFriction:

    def test_dead_zone(self):
        assert apply_friction(0.19, 0.2) == 0.0
        assert apply_friction(-0.19, 0.2) == 0.0
        assert apply_friction(0.0, 0.2) == 0.0

    def test_outside_dead_zone_is_reduced_towards_zero(self):
        assert apply_friction(1.0, 0.2) == pytest.approx(0.8)
        assert apply_friction(-1.0, 0.2) == pytest.approx(-0.8)


class TestChassisOmega:

    @pytest.mark.parametrize('vx,vy,omega', [(0.0, 0.0, 0.0), (0.3, 0.0, 0.0), (0.0, -0.3, 0.0),
                                             (0.0, 0.0, 1.0), (0.0, 0.0, -0.5), (0.4, -0.2, 0.7)])
    def test_recovers_rigid_body_rotation(self, vx, vy, omega):
        assert chassis_omega(LOCATIONS, rigid_body_velocities(vx, vy, omega)) \
            == pytest.approx(omega, abs=1e-12)

    def test_recovers_rotation_for_off_centre_module_layouts(self):
        rng = random.Random(3)
        for _ in range(200):
            locations = [(rng.uniform(-1, 1), rng.uniform(-1, 1)) for _ in range(4)]
            vx, vy, omega = (rng.uniform(-2, 2) for _ in range(3))
            assert chassis_omega(locations, rigid_body_velocities(vx, vy, omega, locations)) \
                == pytest.approx(omega, abs=1e-9)

    def test_degenerate_layouts_give_zero(self):
        assert chassis_omega([], []) == 0.0
        assert chassis_omega([(0.1, 0.1)] * 4, [(1.0, 0.0)] * 4) == 0.0


class TestGuard:

    @pytest.mark.parametrize('value', [None, 'Hardware', 'simulation', 'Replay', ''])
    def test_refuses_without_explicit_simulation_target(self, monkeypatch, value):
        if value is None:
            monkeypatch.delenv('CTR_TARGET', raising=False)
        else:
            monkeypatch.setenv('CTR_TARGET', value)
        with pytest.raises(RuntimeError, match='refused'):
            PySimSwerve(object(), [])
        # The refusal happens before phoenix6 is even looked for.
        assert not any(name.split('.')[0] == 'phoenix6' for name in sys.modules)
