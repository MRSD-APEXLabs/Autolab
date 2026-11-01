"""Live single-camera xArm ACT rollout. Prediction-only unless --execute is explicit.

Same loop as teleop/act_rollout.py: 50 Hz control, asynchronous policy queries every
query_interval, the newest action chunk indexed by elapsed time, and every target limited
in speed and to workspace_radius around the start pose before it is sent.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import replace
from pathlib import Path
from .act.force_torque import checkpoint_force_torque

import pickle
import select
import sys
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from zedx_nano_depth.stereo import colorize_depth

from .cameras import make_camera
from .config import STATE_SPACE, RolloutConfig  # noqa: F401  (RolloutConfig re-exported)
from .robot import XArm6Robot, continuous_rotvec
from .act.action_frames import (ABSOLUTE_TCP, RELATIVE_TCP_TOOL, ACTION_REPRESENTATIONS,
                                checkpoint_state_inputs, from_relative_tcp, interpolate_pose, normalize_state)


class SettleMonitor:
    """Holds the arm and ends the rollout once the policy reports that it has arrived.

    A relative-action policy is re-anchored to the measured pose at every query, so a residual
    output never averages out: 0.25 mm per 0.2 s query adds up to 12 mm in 10 s. The demos end
    standing still, so a chunk whose last waypoint is less than `motion` from the pose it was
    predicted from means "arrived": the target is held while that lasts, and `queries` such
    chunks in a row end the rollout.
    """

    def __init__(self, motion=0.002, queries=3):
        self.motion, self.queries = motion, queries
        self.count = 0
        self.holding = False
        self.travel = float('nan')

    def update(self, actions, anchor):
        """Call once per new chunk with the pose it was predicted from; True ends the rollout."""
        if self.motion <= 0:
            return False
        self.travel = float(np.linalg.norm(np.asarray(actions)[-1, :3] - np.asarray(anchor)[:3]))
        self.holding = self.travel < self.motion
        self.count = self.count + 1 if self.holding else 0
        return self.count >= self.queries


class ContactMonitor:
    """Detects the arm pressing down on something during an executed rollout.

    The xArm reaches its servo target one 20 ms tick later, so while it moves freely the target is
    at most about 0.8 mm below the measured TCP (31 placing demos). A relative-action policy is
    re-anchored to the measured pose at every query: once the plate rests on a surface, any
    downward output keeps the target below the TCP, the arm presses or slides the plate up in the
    fingers and keeps descending, and the 30 mm tracking limit never fires. Contact is declared
    when the target has stayed more than `gap` below the TCP, and more than `max_lag` seconds of
    the TCP's own descent speed, for `hold` seconds.
    """

    def __init__(self, gap=0.001, hold=0.3, max_lag=0.1):
        self.gap, self.hold, self.max_lag = gap, hold, max_lag
        self._samples = deque()
        self._since = None
        self.depth = 0.0

    def update(self, now, target_z, measured_z):
        if self.gap <= 0:
            return False
        self._samples.append((now, measured_z))
        while len(self._samples) > 2 and now - self._samples[1][0] >= self.hold:
            self._samples.popleft()
        then, z_then = self._samples[0]
        speed = max(0.0, (z_then - measured_z) / (now - then)) if now - then > 1e-3 else 0.0
        self.depth = measured_z - target_z
        if self.depth <= max(self.gap, speed * self.max_lag):
            self._since = None
            return False
        if self._since is None:
            self._since = now
        return now - self._since >= self.hold


class ChunkFollower:
    """Turns the newest action chunks into one servo target per control tick. Both runners and the offline
    gate use it, so the replay executes exactly what the robot would.

    - A chunk is anchored at the pose it was predicted from, and indexed by the time since that pose.
    - Near the goal (the newest chunk predicts less than `approach_distance` of travel) translation slows to
      `approach_speed`. The camera image lags the arm by 0.1-0.2 s: at 10 mm/s the policy's last correction
      comes from where the arm was about a millimetre earlier, at 3 mm/s from 0.4 mm.
    - Near the goal the target for the current time is the mean over the last `ensemble` chunks, so a single
      noisy query cannot decide the final millimetre.
    - Once the policy reports arrival (SettleMonitor) the target is held.
    """

    def __init__(self, motion, initial, rollout):
        self.motion = motion
        self.slow = (replace(motion, translation_speed=min(motion.translation_speed, rollout.approach_speed))
                     if rollout.approach_speed > 0 else motion)
        self.hz = motion.loop_hz
        self.initial = np.asarray(initial, dtype=float).copy()
        self.enable_rotation, self.radius = rollout.enable_rotation, rollout.workspace_radius
        self.approach_distance, self.ensemble = rollout.approach_distance, int(rollout.ensemble)
        self.settle = SettleMonitor(rollout.settle_motion, rollout.settle_queries)
        self.chunks = deque(maxlen=max(1, self.ensemble))
        self.near = False
        self.travel = float('nan')

    def add(self, actions, anchor, chunk_time):
        """A chunk of absolute targets predicted from `anchor`, waypoint 0 at `chunk_time`. True ends the rollout."""
        actions = np.asarray(actions, dtype=float)
        self.chunks.append((chunk_time, actions))
        self.travel = float(np.linalg.norm(actions[-1, :3] - np.asarray(anchor, dtype=float)[:3]))
        self.near = self.approach_distance > 0 and self.travel < self.approach_distance
        return self.settle.update(actions, anchor)

    def proposed(self, now):
        newest_time, newest = self.chunks[-1]
        index = max(0, int((now - newest_time) * self.hz))
        if index >= len(newest):
            raise RuntimeError('No fresh policy action chunk; stopping')
        if not self.near or self.ensemble <= 1 or len(self.chunks) == 1:
            return newest[index]
        points = np.asarray([acts[i] for t, acts in self.chunks for i in [int((now - t) * self.hz)] if 0 <= i < len(acts)])
        rotation = Rotation.from_rotvec(points[:, 3:]).mean().as_rotvec()
        return np.concatenate([points[:, :3].mean(axis=0), continuous_rotvec(rotation, newest[index, 3:])])

    def step(self, target, now):
        """The next servo target from the current one, and the waypoint it moves toward."""
        proposed = self.proposed(now)
        if self.settle.holding:
            return target, proposed
        motion = self.slow if self.near else self.motion
        return limit_action(target, proposed, motion, 1 / self.hz, self.initial, self.enable_rotation, self.radius), proposed


def load_policy(directory):
    import torch
    directory = Path(directory)
    with open(directory / 'config.pkl', 'rb') as f:
        config = pickle.load(f)
    if (config.get('task_name') != 'xarm6_demo' or config.get('robot') != 'xarm6'
            or config.get('policy_class') not in ('ACT', 'Diffusion')
            or config.get('state_dim') != 6 or config.get('camera_names') != ['wrist_rgb']):
        raise ValueError('Checkpoint must be a six-dimensional, one-camera xArm policy')
    state_inputs = checkpoint_state_inputs(config, STATE_SPACE)
    action_representation = config.get('action_representation', ABSOLUTE_TCP)
    if action_representation not in ACTION_REPRESENTATIONS or (config['policy_class'] == 'Diffusion' and action_representation != RELATIVE_TCP_TOOL):
        raise ValueError(f'Unsupported checkpoint action representation: {action_representation}')
    with open(directory / 'dataset_stats.pkl', 'rb') as f:
        stats = pickle.load(f)
    use_ft = checkpoint_force_torque(config, stats)
    for key in ('qpos_mean', 'qpos_std', 'action_mean', 'action_std'):
        value = np.asarray(stats[key])
        if value.shape != (6,) or not np.isfinite(value).all() or (key.endswith('std') and np.any(value <= 0)):
            raise ValueError(f'Invalid normalization statistics: {key}')
    from .act.policy import ACTPolicy
    from .act.diffusion_policy import DiffusionPolicy
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    torch.backends.cudnn.enabled = False  # match the training environment
    policy = (ACTPolicy(config['policy_config']) if config['policy_class'] == 'ACT'
              else DiffusionPolicy(config['policy_config'], pretrained_backbone=False)).to(device)
    policy.load_state_dict(torch.load(directory / 'policy_best.ckpt', map_location=device, weights_only=True))
    policy.eval()
    policy.action_representation = action_representation
    policy.state_inputs = state_inputs
    policy.use_force_torque = use_ft
    # Later checkpoints record the image size they were trained at, and whether each chunk is anchored at
    # the pose the camera image was taken at rather than at the pose read when the query is sent.
    policy.image_size = tuple(int(v) for v in config['policy_config'].get('image_size', (320, 240)))
    policy.anchor_at_capture = bool(config.get('anchor_at_capture', False))
    return policy, stats, device


def predict_chunk(policy, stats, device, pose, rgb, record_width=480, depth=None, force_torque=None):
    import torch
    from .act.image_inputs import encode_image, resize_pair
    # Match recorder downscale followed by ACT conversion (including aspect change).
    if record_width and rgb.shape[1] > record_width:
        rgb, depth = resize_pair(rgb, depth, (record_width, round(rgb.shape[0] * record_width / rgb.shape[1])))
    rgb, depth = resize_pair(rgb, depth, tuple(getattr(policy, 'image_size', (320, 240))))
    encoded = encode_image(rgb, depth, policy.use_depth)
    q = normalize_state(pose, stats, getattr(policy, 'state_inputs', 'tcp_pose'), force_torque, getattr(policy, 'use_force_torque', False))
    image = torch.from_numpy(encoded.transpose(2, 0, 1).copy()).to(device)[None, None]
    state = torch.as_tensor(q, dtype=torch.float32, device=device)[None]
    with torch.inference_mode():
        actions = policy(state, image)[0].cpu().numpy() * stats['action_std'] + stats['action_mean']
    if actions.ndim != 2 or actions.shape[1] != 6 or len(actions) == 0 or not np.isfinite(actions).all():
        raise RuntimeError('Policy returned invalid actions')
    if getattr(policy, 'action_representation', ABSOLUTE_TCP) == RELATIVE_TCP_TOOL:
        actions = from_relative_tcp(pose, actions)
    return actions


def limit_action(current, proposed, motion, period, initial, enable_rotation, radius):
    proposed = np.asarray(proposed, dtype=float)
    if proposed.shape != (6,) or not np.isfinite(proposed).all():
        raise RuntimeError('Nonfinite or invalid policy target')
    if np.linalg.norm(proposed[:3] - initial[:3]) > radius:
        raise RuntimeError('Policy target outside the configured workspace radius')
    result = current.copy()
    delta = proposed[:3] - current[:3]
    result[:3] += delta * min(1, motion.translation_speed * period / max(np.linalg.norm(delta), 1e-12))
    if enable_rotation:
        relative = Rotation.from_rotvec(proposed[3:]) * Rotation.from_rotvec(current[3:]).inv()
        vector = relative.as_rotvec()
        vector *= min(1, motion.rotation_speed * period / max(np.linalg.norm(vector), 1e-12))
        result[3:] = continuous_rotvec((Rotation.from_rotvec(vector) * Rotation.from_rotvec(current[3:])).as_rotvec(), current[3:])
    else:
        result[3:] = initial[3:]
    return result


def move_final_offset(robot, target, offset, motion, initial, radius, contact, period, stop=None):
    """After the policy reported arrival: one straight move of the held target by `offset` (robot base frame),
    orientation kept, at `motion` speed. The policy is no longer queried, so the offset is applied exactly once.
    Enter or contact stops it. Returns the last target and whether the move completed."""
    goal = np.asarray(target, dtype=float).copy()
    goal[:3] += offset
    settled_ticks = 0
    while settled_ticks < int(round(0.3 / period)):      # then hold 0.3 s so the arm reaches the end
        tick = time.monotonic()
        if stop is not None and stop.is_set():
            return target, False
        if stop is None and sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
            sys.stdin.readline()
            return target, False
        pose, _ = robot.observation()
        if contact.update(tick, target[2], pose[2]):
            print(f'Contact during the final offset at measured_mm={np.round(pose[:3] * 1000, 2).tolist()}; stopping', flush=True)
            return target, False
        target = limit_action(target, goal, motion, period, initial, True, radius)
        settled_ticks = settled_ticks + 1 if np.linalg.norm(goal[:3] - target[:3]) < 1e-9 else 0
        robot.servo(target, period, measured=pose)
        time.sleep(max(0, period - (time.monotonic() - tick)))
    return target, True


def warm_up(policy, record_width=480):
    """Initialize the model before serving runs, without reading the robot or camera."""
    model, stats, device = policy
    kwargs = ({'force_torque': np.asarray(stats['force_torque_mean'])}
              if getattr(model, 'use_force_torque', False) else {})
    predict_chunk(model, stats, device, np.zeros(6), np.zeros((600, 960, 3), np.uint8), record_width,
                  np.full((600, 960), 500, np.uint16), **kwargs)


def run_act_rollout(config, rollout, policy=None, stop=None):
    """Return done on arrival or elapsed duration, stopped on cancellation or contact.

    The ROS node supplies a preloaded policy and a stop Event; the CLI loads once per run.
    """
    if abs(config.motion.loop_hz - 50) > 1e-6:
        raise ValueError('These policies use 50 Hz action chunks; use --control-hz 50')
    policy, stats, device = policy if policy is not None else load_policy(rollout.ckpt_dir)
    camera = make_camera(config)
    # Real state is read even in prediction-only mode. enable()/servo() are execute-only.
    robot = XArm6Robot(config.robot_ip, config.motion, dry_run=False, read_only=not rollout.execute)
    pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='act-inference')
    future = None
    result = 'done'
    try:
        if stop is not None and stop.is_set():
            return 'stopped'
        camera.start()
        if not camera.calibration.get('rectified'):
            raise RuntimeError('Expected rectified Nano RGB')
        if stop is not None and stop.is_set():
            return 'stopped'
        robot.connect()
        use_ft = getattr(policy, 'use_force_torque', False)
        if use_ft:
            robot.prepare_force_torque(enable=rollout.execute)
        initial = target = robot.initialize_target_pose()
        frame = camera.latest()
        # Warm-up and initial inference before enabling the arm; CUDA initialization can be slow.
        future = pool.submit(predict_chunk, policy, stats, device, initial.copy(), frame.image, config.record_image_width, getattr(frame, 'depth', None), **({'force_torque': robot.force_torque()} if use_ft else {}))
        deadline = time.monotonic() + 60
        while not future.done():
            if stop is not None and stop.is_set():
                return 'stopped'
            if time.monotonic() >= deadline:
                raise RuntimeError('Initial policy inference missed its deadline')
            time.sleep(0.01)
        actions = future.result()
        future = None
        camera.check_mode(force=True)
        camera.latest()
        initial = target = robot.initialize_target_pose()
        # Check initial action before enabling any motion.
        limit_action(target, actions[0], config.motion, 1 / 50, initial, rollout.enable_rotation, rollout.workspace_radius)
        if stop is not None and stop.is_set():
            return 'stopped'
        if rollout.execute:
            robot.enable()
        print('LIVE EXECUTION' if rollout.execute else 'PREDICTION ONLY: robot state reads, no motion commands', flush=True)
        print(f'Policy actions: {getattr(policy, "action_representation", ABSOLUTE_TCP)}', flush=True)
        if use_ft:
            print('Policy F/T inputs: Fx,Fy,Fz (N), Tx,Ty,Tz (Nm), compensated SDK-native wrench', flush=True)
        pose_input = ' + TCP pose' if getattr(policy, 'state_inputs', 'tcp_pose') == 'tcp_pose' else ' (no TCP pose)'
        print('Policy inputs: RGB + metric depth + validity' + pose_input if getattr(policy, 'use_depth', False)
              else 'Policy inputs: RGB' + pose_input + ' (depth is preview only)', flush=True)
        print(('Ctrl-C or Enter stops. ' if stop is None else '') +
              'Orientation ' + ('enabled.' if rollout.enable_rotation else 'held fixed.'), flush=True)
        anchor_at_capture = getattr(policy, 'anchor_at_capture', False)
        start = chunk_submitted = last_submit = time.monotonic()
        last_print = 0.0
        period = 1 / config.motion.loop_hz
        # Only an executed rollout can press on anything: without --execute the arm never follows the target.
        contact = ContactMonitor(rollout.contact_gap if rollout.execute else 0.0, rollout.contact_hold, rollout.contact_max_lag)
        follower = ChunkFollower(config.motion, initial, rollout)
        follower.add(actions, initial, start)
        history = deque(maxlen=int(2 * config.motion.loop_hz))   # measured poses of the last 2 s, for capture-time anchors
        settled = False
        while time.monotonic() - start < rollout.duration_sec:
            tick = time.monotonic()
            if stop is not None and stop.is_set():
                result = 'stopped'
                break
            if stop is None and sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                result = 'stopped'
                break
            camera.check_mode()
            frame = camera.latest()
            pose, _ = robot.observation()
            history.append((tick, pose.copy()))
            if contact.update(tick, target[2], pose[2]):
                print(f'Contact: the target stayed {contact.depth * 1000:.1f} mm below the TCP for '
                      f'{rollout.contact_hold:.1f} s at measured_mm={np.round(pose[:3] * 1000, 2).tolist()}; stopping the rollout', flush=True)
                result = 'stopped'
                break
            if future is not None:
                if tick - submitted_at > rollout.prediction_timeout:
                    raise RuntimeError('Policy inference missed its deadline')
                if future.done():
                    actions = future.result()
                    future = None
                    chunk_submitted = submitted_at
                    if follower.add(actions, submitted_anchor, submitted_time):
                        print(f'Settled: {rollout.settle_queries} chunks in a row predicted under '
                              f'{rollout.settle_motion * 1000:.1f} mm of travel; placement done at '
                              f'measured_mm={np.round(pose[:3] * 1000, 2).tolist()}', flush=True)
                        settled = True
                        break
            if future is None and tick - last_submit >= rollout.query_interval:
                submitted_at = last_submit = time.monotonic()
                if anchor_at_capture:   # where the arm was when this image was taken
                    times, poses = zip(*history)
                    submitted_time = frame.captured_monotonic
                    submitted_anchor = interpolate_pose(times, poses, submitted_time)
                    submitted_anchor[3:] = continuous_rotvec(submitted_anchor[3:], pose[3:])
                else:
                    submitted_time, submitted_anchor = submitted_at, pose.copy()
                future = pool.submit(predict_chunk, policy, stats, device, submitted_anchor.copy(), frame.image, config.record_image_width, getattr(frame, 'depth', None), **({'force_torque': robot.force_torque()} if use_ft else {}))
            now = time.monotonic()
            if now - chunk_submitted > rollout.prediction_timeout:
                raise RuntimeError('No fresh policy action chunk; stopping')
            target, proposed = follower.step(target, now)
            camera.latest()
            if rollout.execute:
                robot.servo(target, period, measured=pose)
            if now - last_print > .5:
                print(f't={now-start:.2f}s predicted_mm={np.round(proposed[:3] * 1000, 1).tolist()} '
                      f'measured_mm={np.round(pose[:3] * 1000, 1).tolist()}' + (' approach' if follower.near else '')
                      + f' camera_age_ms={(now-frame.captured_monotonic)*1000:.0f}', flush=True)
                last_print = now
            if config.preview:
                cv2.imshow('ACT wrist_rgb', cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR))
                cv2.imshow('ACT depth: 0.1-1.0 m, near=red far=blue invalid=black',
                           colorize_depth(frame.depth))
                if cv2.waitKey(1) in (27, ord('q')):
                    result = 'stopped'
                    break
            time.sleep(max(0, period - (time.monotonic() - tick)))
        if any(rollout.final_offset):
            offset = np.asarray(rollout.final_offset, dtype=float)
            where = f'{np.round(offset * 1000, 2).tolist()} mm (robot base frame)'
            if not settled:
                print(f'Final offset {where} not applied: the run did not end with Settled', flush=True)
            elif not rollout.execute:
                print(f'PREDICTION ONLY: an executed run would now move {where}', flush=True)
            else:
                print(f'Final offset: moving {where} at {follower.slow.translation_speed * 1000:.0f} mm/s', flush=True)
                target, done = move_final_offset(robot, target, offset, follower.slow, initial,
                                                 rollout.workspace_radius, contact, period, stop=stop)
                if not done:
                    result = 'stopped'
                pose, _ = robot.observation()
                print(f'Final offset {"done" if done else "stopped"} at measured_mm={np.round(pose[:3] * 1000, 2).tolist()}', flush=True)
    except KeyboardInterrupt:
        print('Stopping ACT rollout', flush=True)
        result = 'stopped'
    finally:
        try:
            robot.close()
        finally:
            try:
                camera.stop()
            finally:
                pool.shutdown(wait=False, cancel_futures=True)
                if config.preview:
                    cv2.destroyAllWindows()
    return result
