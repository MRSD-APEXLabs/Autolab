"""Pure-Python physics stepper for a phoenix6 swerve drivetrain running on the SIMULATION target.

WHY this exists: ``SwerveDrivetrain.update_sim_state()`` is only defined when wpilib/wpimath are
importable, and they are not installed on this robot computer.  Without somebody feeding rotor
position/velocity back into the simulated devices, the simulated Talons output a voltage but
nothing ever moves.  ``PySimSwerve`` does what phoenix6's ``SimSwerveDrivetrain.update()`` does,
with WPILib's ``DCMotorSim(krakenX60FOC)`` replaced by an exactly discretised first-order model.

SAFETY: this module only ever touches ``sim_state`` objects, and ``PySimSwerve`` refuses to be
constructed unless the process is explicitly on the phoenix6 Simulation target.  phoenix6 is
imported lazily so that the model code below stays importable (and unit-testable) without it.
"""

from __future__ import annotations

import math
import os
import threading
import time
from typing import Optional, Sequence, Tuple

SIM_TARGET = 'Simulation'
DEFAULT_STEP_PERIOD_S = 0.004   # same period as the Tuner X generated project's sim Notifier
SUPPLY_VOLTAGE_V = 12.0


class DCMotorModel:
    """WPILib ``DCMotor.krakenX60FOC(1)`` driving an inertia through a gearbox (exact ZOH step)."""

    def __init__(self, inertia: float, gear_ratio: float) -> None:
        nominal_v, stall_torque, stall_current, free_current = 12.0, 9.37, 483.0, 2.0
        free_speed = 5800.0 * 2.0 * math.pi / 60.0
        resistance = nominal_v / stall_current
        k_v = free_speed / (nominal_v - resistance * free_current)
        k_t = stall_torque / stall_current
        self._a = -(gear_ratio ** 2) * k_t / (k_v * resistance * inertia)   # 1/s, negative
        self._b = gear_ratio * k_t / (resistance * inertia)                 # rad/s^2 per volt
        self.position = 0.0    # mechanism (output shaft) angle, rad
        self.velocity = 0.0    # mechanism speed, rad/s

    def steady_state_velocity(self, voltage: float) -> float:
        return -self._b * voltage / self._a

    def step(self, voltage: float, dt: float) -> None:
        w_ss = self.steady_state_velocity(voltage)
        decay = math.exp(self._a * dt)
        self.position += w_ss * dt + (self.velocity - w_ss) * (decay - 1.0) / self._a
        self.velocity = w_ss + (self.velocity - w_ss) * decay


def apply_friction(voltage: float, friction_voltage: float) -> float:
    """Apply static friction as a voltage dead zone (as phoenix6's SimSwerveDrivetrain does)."""
    if abs(voltage) < friction_voltage:
        return 0.0
    return voltage - friction_voltage if voltage > 0.0 else voltage + friction_voltage


def chassis_omega(locations: Sequence[Tuple[float, float]],
                  velocities: Sequence[Tuple[float, float]]) -> float:
    """Least-squares chassis yaw rate (rad/s) from module positions (m) and velocities (m/s).

    Rigid body: v_i = (vx - w*y_i, vy + w*x_i).  Solving the normal equations for w with the
    translation eliminated gives the closed form below; it is exact for consistent module states
    and the best fit otherwise (wheel slip while steering).
    """
    n = len(locations)
    sx = sy = srr = bx = by = bw = 0.0
    for (x, y), (vx, vy) in zip(locations, velocities):
        sx += x
        sy += y
        srr += x * x + y * y
        bx += vx
        by += vy
        bw += -y * vx + x * vy
    det = srr - (sx * sx + sy * sy) / n if n else 0.0
    if abs(det) < 1e-12:
        return 0.0
    return (bw + (sy * bx - sx * by) / n) / det


class PySimSwerve:
    """Steps the simulated Talons/CANcoders/Pigeon2 of a swerve drivetrain from a daemon thread."""

    def __init__(self, drivetrain, module_constants) -> None:
        # Environment first: it is checkable without importing phoenix6 at all.
        if os.environ.get('CTR_TARGET') != SIM_TARGET:
            raise RuntimeError('PySimSwerve refused: CTR_TARGET is not "Simulation"')
        from phoenix6 import utils
        from phoenix6.sim.chassis_reference import ChassisReference
        if not utils.is_simulation():
            raise RuntimeError('PySimSwerve refused: phoenix6 is not on the Simulation target')
        self._utils = utils
        self._cw = ChassisReference.CLOCKWISE_POSITIVE
        self._ccw = ChassisReference.COUNTER_CLOCKWISE_POSITIVE
        self._drivetrain = drivetrain
        self._constants = list(module_constants)
        self._drive = [DCMotorModel(c.drive_inertia, c.drive_motor_gear_ratio)
                       for c in self._constants]
        self._steer = [DCMotorModel(c.steer_inertia, c.steer_motor_gear_ratio)
                       for c in self._constants]
        self._locations = [(c.location_x, c.location_y) for c in self._constants]
        self._yaw = 0.0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.error: Optional[BaseException] = None

    def update(self, dt: float, supply_voltage: float = SUPPLY_VOLTAGE_V) -> None:
        two_pi = 2.0 * math.pi
        modules = self._drivetrain.modules
        for c, drive, steer, module in zip(self._constants, self._drive, self._steer, modules):
            d = module.drive_motor.sim_state      # TalonFXSimState
            s = module.steer_motor.sim_state      # TalonFXSimState
            e = module.encoder.sim_state          # CANcoderSimState
            d.orientation = self._cw if c.drive_motor_inverted else self._ccw
            s.orientation = self._cw if c.steer_motor_inverted else self._ccw
            e.orientation = self._cw if c.encoder_inverted else self._ccw
            e.sensor_offset = c.encoder_offset
            d.set_supply_voltage(supply_voltage)
            s.set_supply_voltage(supply_voltage)
            e.set_supply_voltage(supply_voltage)
            drive.step(apply_friction(d.motor_voltage, c.drive_friction_voltage), dt)
            steer.step(apply_friction(s.motor_voltage, c.steer_friction_voltage), dt)
            d.set_raw_rotor_position(drive.position / two_pi * c.drive_motor_gear_ratio)
            d.set_rotor_velocity(drive.velocity / two_pi * c.drive_motor_gear_ratio)
            s.set_raw_rotor_position(steer.position / two_pi * c.steer_motor_gear_ratio)
            s.set_rotor_velocity(steer.velocity / two_pi * c.steer_motor_gear_ratio)
            e.set_raw_position(steer.position / two_pi)   # the azimuth encoder sees the mechanism
            e.set_velocity(steer.velocity / two_pi)

        velocities = []
        for module in modules:
            state = module.get_current_state()
            velocities.append((state.speed * state.angle.cos(), state.speed * state.angle.sin()))
        omega = chassis_omega(self._locations, velocities)
        self._yaw += omega * dt
        pigeon = self._drivetrain.pigeon2.sim_state
        pigeon.set_raw_yaw(math.degrees(self._yaw))
        pigeon.set_angular_velocity_z(math.degrees(omega))

    def start(self, period: float = DEFAULT_STEP_PERIOD_S) -> None:
        if self._thread is not None:
            raise RuntimeError('PySimSwerve already started')

        def run() -> None:
            last = self._utils.get_current_time_seconds()
            while not self._stop.is_set():
                now = self._utils.get_current_time_seconds()
                try:
                    self.update(now - last)
                except Exception as exc:   # keep the reason; a dead sim only means "no motion"
                    self.error = exc
                    return
                last = now
                time.sleep(period)

        self._thread = threading.Thread(target=run, name='phoenix_sim_step', daemon=True)
        self._thread.start()

    def is_alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def stop(self) -> None:
        """Stop stepping.  MUST be called before ``drivetrain.close()``: the thread uses it."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
