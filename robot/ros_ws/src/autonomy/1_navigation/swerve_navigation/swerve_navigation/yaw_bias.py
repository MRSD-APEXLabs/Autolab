"""Bounded feedforward for measured translational-to-yaw coupling; no ROS/hardware."""
from dataclasses import dataclass
import math

# Shared gross-error boundary for moving feedback and the station coordinator.
# This is separate from the much tighter final parking heading tolerance.
MAX_STATION_YAW_ERROR = 0.25


@dataclass(frozen=True)
class YawBiasConfig:
    vx_gain: float = 0.0
    vy_gain: float = 0.0
    gain: float = 0.75
    max_correction: float = 0.07
    model_speed: float = 0.30
    min_speed: float = 0.10
    max_speed: float = 0.45
    settle_s: float = 0.5
    correction_accel: float = 0.20
    command_deadband: float = 0.05
    max_angular_speed: float = 1.5
    max_angular_accel: float = 3.0

    def __post_init__(self):
        if not all(math.isfinite(v) for v in vars(self).values()):
            raise ValueError('yaw bias parameters must be finite')
        if max(abs(self.vx_gain), abs(self.vy_gain)) > 1.0:
            raise ValueError('yaw bias coupling gain must be within +/-1 rad/m')
        if not 0 < self.gain <= 1 or not 0 < self.max_correction <= 0.15:
            raise ValueError('yaw bias gain/correction outside allowed bounds')
        if not 0.05 <= self.min_speed <= self.model_speed <= self.max_speed <= 0.6:
            raise ValueError('invalid yaw bias speed envelope')
        if not 0.1 <= self.settle_s <= 3 or not 0 < self.correction_accel <= 1:
            raise ValueError('invalid yaw bias settling/ramp limits')
        if not 0 <= self.command_deadband <= 0.2:
            raise ValueError('invalid yaw command deadband')
        if min(self.max_angular_speed, self.max_angular_accel) <= 0:
            raise ValueError('angular limits must be positive')


class YawBiasCompensator:
    def __init__(self, config):
        self.config = config
        self.reset()

    def reset(self):
        self.previous = None
        self.since = None
        self.last_time = None
        self.correction = 0.0
        self.output_wz = 0.0
        self.mode = 'idle'

    def _limited_output(self, target, dt):
        c = self.config
        target = max(-c.max_angular_speed, min(c.max_angular_speed, target))
        start = self.output_wz if target * self.output_wz > 0 else 0.0
        if abs(target) <= abs(start):
            self.output_wz = target
        else:
            self.output_wz = math.copysign(min(abs(target), abs(start) + c.max_angular_accel * dt), target)
        return self.output_wz

    def apply(self, vx, vy, wz, now, *, locked=False, force_zero=False, source_known=True):
        if not all(math.isfinite(v) for v in (vx, vy, wz, now)):
            raise ValueError('nonfinite yaw bias input')
        c = self.config
        dt = 0.0 if self.last_time is None else max(0.0, min(now - self.last_time, 0.05))
        previous = self.previous
        self.previous, self.last_time = (vx, vy), now
        speed = math.hypot(vx, vy)
        raw_wz = 0.0 if abs(wz) < c.command_deadband else wz
        if force_zero:
            self.since, self.correction, self.output_wz, self.mode = None, 0.0, 0.0, 'rotation locked'
            return 0.0
        eligible = (not locked and source_known and c.min_speed <= speed <= c.max_speed
                    and abs(wz) <= 0.05)
        if not eligible:
            self.since, self.correction = None, 0.0
            self.mode = 'rotation locked' if locked else 'outside fitted envelope'
            return self._limited_output(raw_wz, dt)
        changing = (previous is None or dt <= 0 or
                    math.hypot(vx - previous[0], vy - previous[1]) / dt > 0.25)
        if changing or self.since is None:
            self.since = now
        if changing:
            self.correction = 0.0  # transients were excluded from the fitted data
            self.mode = 'settling'
            return self._limited_output(raw_wz, dt)
        target = 0.0
        self.mode = 'settling'
        if now - self.since >= c.settle_s:
            scale = min(1.0, c.model_speed / speed)  # saturate instead of extrapolating
            prediction = scale * (c.vx_gain * vx + c.vy_gain * vy)
            target = max(-c.max_correction, min(c.max_correction, -c.gain * prediction))
            self.mode = 'compensating'
        step = min(c.correction_accel, c.max_angular_accel) * dt
        self.correction += max(-step, min(step, target - self.correction))
        return self._limited_output(raw_wz + self.correction, dt)


class StationHeadingHold:
    """Counter physical yaw drift during station translation; never spin at rest.

    Reference is continuous odometry, so AMCL corrections do not rotate the base.
    Commanded AND measured translation are required for any angular correction.
    """

    def __init__(self, config, kp=1.5, kd=0.15):
        if not math.isfinite(kp) or not 0 < kp <= 3 or not math.isfinite(kd) or not 0 <= kd <= .5:
            raise ValueError('invalid station heading feedback gains')
        self.config, self.kp, self.kd = config, kp, kd
        self.reset()

    def reset(self):
        self.reference = None
        self.last_time = None
        self.correction = 0.0

    def apply(self, vx, vy, yaw, measured_wz, measured_speed, now, target_yaw=None):
        if not all(math.isfinite(v) for v in (vx, vy, yaw, measured_wz, measured_speed, now)):
            raise ValueError('nonfinite station heading input')
        speed = math.hypot(vx, vy)
        if not .05 <= speed <= .18:
            self.reset()
            return 0.0
        if target_yaw is not None:
            if (not math.isfinite(target_yaw) or
                    abs(math.atan2(math.sin(target_yaw-yaw), math.cos(target_yaw-yaw))) > MAX_STATION_YAW_ERROR):
                self.reset()
                return 0.0
            self.reference = target_yaw
        if self.reference is None:
            self.reference = yaw
        dt = 0.0 if self.last_time is None else max(0., min(now - self.last_time, .05))
        self.last_time = now
        if measured_speed < .02:
            self.correction = 0.0
            return 0.0
        error = math.atan2(math.sin(yaw - self.reference), math.cos(yaw - self.reference))
        c = self.config
        ff = c.gain * (c.vx_gain * vx + c.vy_gain * vy)
        target = -ff - self.kp * error - self.kd * measured_wz
        cap = min(c.max_correction, c.max_angular_speed)
        target = max(-cap, min(cap, target))
        step = min(c.correction_accel, c.max_angular_accel) * dt
        self.correction += max(-step, min(step, target - self.correction))
        return self.correction
