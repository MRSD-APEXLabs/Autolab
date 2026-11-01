"""Shared contract for UFACTORY compensated external wrench observations."""
import numpy as np

FORCE_TORQUE_SPACE = 'ufactory_external_wrench_N_Nm_v1'
FORCE_TORQUE_KEY = 'observations/force_torque'


def validate_force_torque(h5, required=False):
    if FORCE_TORQUE_KEY not in h5:
        if required:
            raise ValueError(f'{h5.filename}: missing force/torque observations; collect new demos with --force-torque')
        return
    data = h5[FORCE_TORQUE_KEY]
    if (h5.attrs.get('force_torque_space') != FORCE_TORQUE_SPACE
            or data.shape != (len(h5['action']), 6)
            or not np.issubdtype(data.dtype, np.floating) or not np.isfinite(data[()]).all()):
        raise ValueError(f'{h5.filename}: invalid force/torque data or semantics')


def validate_force_torque_stats(stats):
    for name in ('force_torque_mean', 'force_torque_std'):
        value = np.asarray(stats.get(name, []), dtype=float)
        if value.shape != (6,) or not np.isfinite(value).all() or (name.endswith('std') and np.any(value <= 0)):
            raise ValueError(f'Invalid force/torque normalization statistics: {name}')


def normalize_force_torque(wrench, stats):
    validate_force_torque_stats(stats)
    if wrench is None:
        raise ValueError('This policy requires a force/torque observation')
    value = np.asarray(wrench, dtype=float)
    if value.ndim < 1 or value.shape[-1] != 6 or not np.isfinite(value).all():
        raise ValueError('Force/torque observation must have six finite components')
    return (value - stats['force_torque_mean']) / stats['force_torque_std']


def checkpoint_force_torque(config, stats):
    """Fail on inconsistent metadata before connecting any hardware."""
    policy = config['policy_config']
    use_ft = config.get('use_force_torque', False)
    if not isinstance(use_ft, bool) or policy.get('use_force_torque', False) != use_ft:
        raise ValueError('Inconsistent force/torque checkpoint metadata')
    expected = config['state_dim'] + (6 if use_ft else 0)
    if policy.get('observation_dim', config['state_dim']) != expected:
        raise ValueError('Invalid checkpoint observation dimension')
    if use_ft:
        if config.get('force_torque_space') != FORCE_TORQUE_SPACE:
            raise ValueError('Unsupported checkpoint force/torque semantics')
        validate_force_torque_stats(stats)
    return use_ft
