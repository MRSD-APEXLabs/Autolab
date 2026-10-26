"""The node's parameters -> ServoConfig and Mount (no ROS imports: the tools and tests use them too)."""
from __future__ import annotations

import numpy as np

from .control import Gains
from .mount import Mount
from .servo import GripperConfig, ServoConfig


def build_config(param):
    """ServoConfig from the node's parameters (`param(name, default)`); defaults are the dataclass ones."""
    base, gains, grip = ServoConfig(), Gains(), GripperConfig()
    lengths = lambda values: tuple(float(v) for v in values)
    config = ServoConfig(
        dry_run=not bool(param('execute', False)),
        control_rate=float(param('servo.control_rate', base.control_rate)),
        command_duration=float(param('servo.command_duration', base.command_duration)),
        level_at_start=bool(param('servo.level_at_start', base.level_at_start)),
        level_speed=float(param('servo.level_speed', base.level_speed)),
        hover_height=float(param('servo.hover_height', base.hover_height)),
        grasp_offset=lengths(param('servo.grasp_offset', list(base.grasp_offset))),
        finger_axis=str(param('servo.finger_axis', base.finger_axis)),
        settle_count=int(param('servo.settle_count', base.settle_count)),
        max_image_age=float(param('servo.max_image_age', base.max_image_age)),
        hold_timeout=float(param('servo.hold_timeout', base.hold_timeout)),
        lost_timeout=float(param('servo.lost_timeout', base.lost_timeout)),
        align_timeout=float(param('servo.align_timeout', base.align_timeout)),
        max_travel=float(param('servo.max_travel', base.max_travel)),
        min_tip_z=float(param('servo.min_tip_z', base.min_tip_z)),
        plate_size=lengths(param('servo.plate_size', list(base.plate_size))),
        size_tolerance=float(param('servo.size_tolerance', base.size_tolerance)),
        min_conf=float(param('servo.min_conf', base.min_conf)),
        weak_size_tolerance=float(param('servo.weak_size_tolerance', base.weak_size_tolerance)),
        filter_alpha=float(param('servo.filter_alpha', base.filter_alpha)),
        filter_gate=float(param('servo.filter_gate', base.filter_gate)),
        grasp=bool(param('grasp.enable', base.grasp)),
        grasp_depth=float(param('grasp.depth', base.grasp_depth)),
        approach_speed=float(param('grasp.approach_speed', base.approach_speed)),
        final_segment=float(param('grasp.final_segment', base.final_segment)),
        final_speed=float(param('grasp.final_speed', base.final_speed)),
        lift=float(param('grasp.lift', base.lift)),
        lift_speed=float(param('grasp.lift_speed', base.lift_speed)),
        max_retries=int(param('grasp.max_retries', base.max_retries)),
        retry_lift=float(param('grasp.retry_lift', base.retry_lift)),
        return_to_start=bool(param('grasp.return_to_start', base.return_to_start)),
        return_speed=float(param('grasp.return_speed', base.return_speed)),
        gains=Gains(
            k_xy=float(param('gains.k_xy', gains.k_xy)), k_z=float(param('gains.k_z', gains.k_z)),
            k_yaw=float(param('gains.k_yaw', gains.k_yaw)), max_xy=float(param('gains.max_xy', gains.max_xy)),
            max_z=float(param('gains.max_z', gains.max_z)), max_yaw=float(param('gains.max_yaw', gains.max_yaw)),
            tol_xy=float(param('gains.tol_xy', gains.tol_xy)), tol_z=float(param('gains.tol_z', gains.tol_z)),
            tol_yaw=float(np.radians(float(param('gains.tol_yaw_deg', float(np.degrees(gains.tol_yaw))))))),
        gripper=GripperConfig(
            close_start=int(param('gripper.close_start', grip.close_start)), step=int(param('gripper.step', grip.step)),
            max_close=int(param('gripper.max_close', grip.max_close)), samples=int(param('gripper.samples', grip.samples)),
            rate=float(param('gripper.rate', grip.rate)),
            target_pressure=tuple(int(v) for v in param('gripper.target_pressure', list(grip.target_pressure))),
            stop_pressure=tuple(int(v) for v in param('gripper.stop_pressure', list(grip.stop_pressure))),
            final_squeeze=int(param('gripper.final_squeeze', grip.final_squeeze)),
            final_squeeze_settle=float(param('gripper.final_squeeze_settle', grip.final_squeeze_settle)),
            nudge_mm=float(param('gripper.nudge_mm', grip.nudge_mm)),
            nudge_speed=float(param('gripper.nudge_speed', grip.nudge_speed)),
            use_fsr=bool(param('gripper.use_fsr', grip.use_fsr)),
            close_offset=int(param('gripper.close_offset', grip.close_offset)),
            close_settle=float(param('gripper.close_settle', grip.close_settle)),
            tare_samples=int(param('gripper.tare_samples', grip.tare_samples)),
            fsr_polarity=tuple(int(v) for v in param('gripper.fsr_polarity', list(grip.fsr_polarity))),
            fsr_samples=int(param('gripper.fsr_samples', grip.fsr_samples)),
            fsr_sample_interval=float(param('gripper.fsr_sample_interval', grip.fsr_sample_interval)),
            fsr_nudge_sign=tuple(int(v) for v in param('gripper.fsr_nudge_sign', list(grip.fsr_nudge_sign))),
            max_nudge_mm=float(param('gripper.max_nudge_mm', grip.max_nudge_mm)),
            max_nudges=int(param('gripper.max_nudges', grip.max_nudges)),
            use_ft=bool(param('gripper.use_ft', grip.use_ft)),
            ft_samples=int(param('gripper.ft_samples', grip.ft_samples)),
            ft_finger_axis=int(param('gripper.ft_finger_axis', grip.ft_finger_axis)),
            ft_finger_sign=int(param('gripper.ft_finger_sign', grip.ft_finger_sign)),
            ft_lateral_deadband_n=float(param('gripper.ft_lateral_deadband_n', grip.ft_lateral_deadband_n)),
            ft_track_mm_per_step=float(param('gripper.ft_track_mm_per_step', grip.ft_track_mm_per_step)),
            ft_track_force_n=float(param('gripper.ft_track_force_n', grip.ft_track_force_n)),
            ft_track_gain_mm_per_n=float(param('gripper.ft_track_gain_mm_per_n', grip.ft_track_gain_mm_per_n)),
            ft_track_max_mm=float(param('gripper.ft_track_max_mm', grip.ft_track_max_mm)),
            ft_contact_n=float(param('gripper.ft_contact_n', grip.ft_contact_n)),
            ft_contact_backoff=float(param('gripper.ft_contact_backoff', grip.ft_contact_backoff)),
            ft_hold_min_n=float(param('gripper.ft_hold_min_n', grip.ft_hold_min_n))),
    )
    if config.finger_axis not in ('x', 'y'):
        raise ValueError(f"servo.finger_axis must be 'x' or 'y', got {config.finger_axis!r}")
    if len(config.grasp_offset) != 2 or len(config.plate_size) != 2:
        raise ValueError('servo.grasp_offset and servo.plate_size need 2 values')
    if config.control_rate <= 0.0:
        raise ValueError('servo.control_rate must be > 0')
    grip = config.gripper
    if len(grip.fsr_polarity) != 2 or any(v not in (-1, 1) for v in grip.fsr_polarity):
        raise ValueError('gripper.fsr_polarity needs two values, each +1 or -1')
    if len(grip.target_pressure) != 2 or len(grip.stop_pressure) != 2:
        raise ValueError('gripper.target_pressure and gripper.stop_pressure need 2 values')
    if grip.fsr_samples < 1 or grip.tare_samples < 0 or not np.isfinite(grip.fsr_sample_interval) or grip.fsr_sample_interval < 0:
        raise ValueError('gripper needs fsr_samples >= 1, tare_samples >= 0 and fsr_sample_interval >= 0')
    if grip.fsr_nudge_sign not in ((1, -1), (-1, 1)):
        raise ValueError('gripper.fsr_nudge_sign must be [1, -1] or [-1, 1]')
    if grip.max_nudges < 1 or not np.isfinite(grip.max_nudge_mm) or grip.max_nudge_mm <= 0:
        raise ValueError('gripper.max_nudges and gripper.max_nudge_mm must be > 0')
    if grip.rate <= 0 or grip.nudge_mm < 0:
        raise ValueError('gripper.rate must be > 0 and gripper.nudge_mm must be >= 0')
    if grip.final_squeeze < 0 or not np.isfinite(grip.final_squeeze_settle) or grip.final_squeeze_settle < 0:
        raise ValueError('gripper.final_squeeze and gripper.final_squeeze_settle must be >= 0')
    if grip.ft_samples < 1 or grip.ft_finger_axis not in (0, 1, 2) or grip.ft_finger_sign not in (-1, 1):
        raise ValueError('gripper needs ft_samples >= 1, ft_finger_axis 0, 1 or 2 and ft_finger_sign +1 or -1')
    ft_values = (grip.ft_lateral_deadband_n, grip.ft_contact_n, grip.ft_contact_backoff, grip.ft_hold_min_n,
                 grip.ft_track_mm_per_step, grip.ft_track_force_n, grip.ft_track_gain_mm_per_n, grip.ft_track_max_mm)
    if not all(np.isfinite(v) and v >= 0 for v in ft_values):
        raise ValueError('gripper F/T values (ft_lateral_deadband_n .. ft_track_max_mm) must be >= 0')
    return config


def build_mount(param):
    mount = Mount()
    values = {}
    for key, default in (('camera_offset_mm', list(mount.camera_offset_mm)), ('tilt_deg', mount.tilt_deg),
                         ('lens_offset_mm', list(mount.lens_offset_mm)),
                         ('correction_rpy_deg', list(mount.correction_rpy_deg)), ('tip_mm', list(mount.tip_mm)),
                         ('ee_below_flange_mm', mount.ee_below_flange_mm), ('tip_camera_mm', [0.0])):
        values[key] = param(f'mount.{key}', default)
    if list(values['tip_camera_mm']) == [0.0]:   # rclpy can't declare an empty list: [0.0] means "use the CAD tip"
        values['tip_camera_mm'] = []
    return Mount.from_params(values)
