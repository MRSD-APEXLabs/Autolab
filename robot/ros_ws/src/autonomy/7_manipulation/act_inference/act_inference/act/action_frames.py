"""Action representations for xArm TCP trajectories.

Relative chunks are expressed in the observed TCP frame. Every waypoint in a
chunk has the same observation pose as its anchor, rather than the preceding
waypoint. This makes a chunk safe to index at any elapsed-time offset.
"""
import numpy as np
from scipy.spatial.transform import Rotation


ABSOLUTE_TCP = 'absolute_tcp'
RELATIVE_TCP_TOOL = 'relative_tcp_tool_v1'
ACTION_REPRESENTATIONS = (ABSOLUTE_TCP, RELATIVE_TCP_TOOL)

# What the policy sees of the measured TCP pose. 'tcp_pose' is the full normalized pose
# (all checkpoints before state_inputs existed). 'none' hides it: the state is all zeros,
# so a relative-action policy must find the target in the camera image, and it behaves the
# same wherever the mobile base parks instead of driving to remembered base-frame positions.
STATE_INPUTS = ('tcp_pose', 'none')


def checkpoint_state_inputs(config, state_space):
    """The checkpoint's state inputs. Pose-free checkpoints also declare state_space 'none', so a
    runner that predates state_inputs refuses them instead of feeding them the pose they never saw."""
    state_inputs = config.get('state_inputs', 'tcp_pose')
    if state_inputs not in STATE_INPUTS or config.get('state_space') != ('none' if state_inputs == 'none' else state_space):
        raise ValueError('Checkpoint must be a measured-TCP policy, or a pose-free one with state_space and '
                         f'state_inputs both "none"; got state_space={config.get("state_space")}, state_inputs={state_inputs}')
    return state_inputs


def normalize_state(pose, stats, state_inputs='tcp_pose', force_torque=None, use_force_torque=False):
    """Policy state for one pose (6,) or a batch (..., 6), normalized with the dataset statistics."""
    if state_inputs not in STATE_INPUTS:
        raise ValueError(f'Unknown state inputs: {state_inputs}')
    pose = np.asarray(pose, dtype=np.float64)
    if pose.shape[-1] != 6:
        raise ValueError('TCP poses must have six components')
    state = (pose - np.asarray(stats['qpos_mean'], dtype=np.float64)) / np.asarray(stats['qpos_std'], dtype=np.float64)
    state = np.zeros_like(state) if state_inputs == 'none' else state
    if use_force_torque:
        try:
            from .force_torque import normalize_force_torque
        except ImportError:
            from force_torque import normalize_force_torque
        wrench = normalize_force_torque(force_torque, stats)
        if wrench.shape != pose.shape:
            raise ValueError('Pose and force/torque batch shapes differ')
        state = np.concatenate((state, wrench), axis=-1)
    return state


def to_relative_tcp(anchor, targets):
    """Convert absolute TCP targets to translation and rotation in the anchor TCP frame."""
    anchor = np.asarray(anchor, dtype=np.float64)
    targets = np.asarray(targets, dtype=np.float64)
    if anchor.shape[-1] != 6 or targets.shape[-1] != 6:
        raise ValueError('TCP poses must have six components')
    origin = Rotation.from_rotvec(anchor[..., 3:])
    translation = origin.inv().apply(targets[..., :3] - anchor[..., :3])
    rotation = (origin.inv() * Rotation.from_rotvec(targets[..., 3:])).as_rotvec()
    return np.concatenate((translation, rotation), axis=-1).astype(np.float32)


def from_relative_tcp(anchor, relative):
    """Reconstruct absolute TCP targets using the pose captured with the policy query."""
    anchor = np.asarray(anchor, dtype=np.float64)
    relative = np.asarray(relative, dtype=np.float64)
    if anchor.shape != (6,) or relative.shape[-1] != 6:
        raise ValueError('Expected one six-component anchor and six-component waypoints')
    origin = Rotation.from_rotvec(anchor[3:])
    translation = anchor[:3] + origin.apply(relative[..., :3])
    rotation = (origin * Rotation.from_rotvec(relative[..., 3:])).as_rotvec()
    # Keep the SDK rotation-vector branch near the measured orientation.
    flat = np.asarray(rotation).reshape(-1, 3)
    for vector in flat:
        magnitude = np.linalg.norm(vector)
        if magnitude > 1e-10:
            axis = vector / magnitude
            turns = round(float(np.dot(anchor[3:] - vector, axis)) / (2 * np.pi))
            vector += turns * 2 * np.pi * axis
    result = np.concatenate((translation, rotation), axis=-1)
    return result


def interpolate_pose(times, poses, at):
    """TCP pose(s) at time(s) `at` from a pose history: linear in position, slerp in rotation, clamped to the
    history. Used to find where the arm was when a camera image was taken."""
    from scipy.spatial.transform import Slerp
    times = np.asarray(times, dtype=np.float64)
    poses = np.asarray(poses, dtype=np.float64)
    scalar = np.ndim(at) == 0
    at = np.clip(np.atleast_1d(np.asarray(at, dtype=np.float64)), times[0], times[-1])
    keep = np.concatenate([[True], np.diff(times) > 0])
    times, poses = times[keep], poses[keep]
    out = np.empty((len(at), 6))
    for axis in range(3):
        out[:, axis] = np.interp(at, times, poses[:, axis])
    rotation = (Slerp(times, Rotation.from_rotvec(poses[:, 3:]))(at).as_rotvec() if len(times) > 1
                else np.repeat(poses[:1, 3:], len(at), axis=0))
    # Stay on the rotation-vector branch of the recorded pose before each time (the tool points down,
    # near |pi|, where the SDK and scipy can pick opposite branches for the same orientation).
    reference = poses[np.clip(np.searchsorted(times, at, side='right') - 1, 0, len(times) - 1), 3:]
    norm = np.linalg.norm(rotation, axis=1, keepdims=True)
    axis = np.divide(rotation, norm, out=np.zeros_like(rotation), where=norm > 1e-10)
    turns = np.round(np.sum((reference - rotation) * axis, axis=1, keepdims=True) / (2 * np.pi))
    out[:, 3:] = rotation + turns * 2 * np.pi * axis
    return out[0] if scalar else out
