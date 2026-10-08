"""Live single-camera xArm ACT rollout. Prediction-only unless --execute is explicit.

Same loop as teleop/act_rollout.py: 50 Hz control, asynchronous policy queries every
query_interval, the newest action chunk indexed by elapsed time, and every target limited
in speed before it is sent.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
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


def load_policy(directory):
    import torch
    directory = Path(directory)
    with open(directory / 'config.pkl', 'rb') as f:
        config = pickle.load(f)
    if (config.get('task_name') != 'xarm6_demo' or config.get('robot') != 'xarm6'
            or config.get('state_space') != STATE_SPACE or config.get('policy_class') != 'ACT'
            or config.get('state_dim') != 6 or config.get('camera_names') != ['wrist_rgb']):
        raise ValueError('Checkpoint must be a measured-TCP, six-dimensional, one-camera xArm ACT policy')
    with open(directory / 'dataset_stats.pkl', 'rb') as f:
        stats = pickle.load(f)
    for key in ('qpos_mean', 'qpos_std', 'action_mean', 'action_std'):
        value = np.asarray(stats[key])
        if value.shape != (6,) or not np.isfinite(value).all() or (key.endswith('std') and np.any(value <= 0)):
            raise ValueError(f'Invalid normalization statistics: {key}')
    from .act.policy import ACTPolicy
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    torch.backends.cudnn.enabled = False  # match the training environment
    policy = ACTPolicy(config['policy_config']).to(device)
    policy.load_state_dict(torch.load(directory / 'policy_best.ckpt', map_location=device, weights_only=True))
    policy.eval()
    return policy, stats, device


def predict_chunk(policy, stats, device, pose, rgb, record_width=480, depth=None):
    import torch
    from .act.image_inputs import encode_image, resize_pair
    # Match recorder downscale followed by ACT conversion (including aspect change).
    if record_width and rgb.shape[1] > record_width:
        rgb, depth = resize_pair(rgb, depth, (record_width, round(rgb.shape[0] * record_width / rgb.shape[1])))
    rgb, depth = resize_pair(rgb, depth, (320, 240))
    encoded = encode_image(rgb, depth, policy.use_depth)
    q = (pose - stats['qpos_mean']) / stats['qpos_std']
    image = torch.from_numpy(encoded.transpose(2, 0, 1).copy()).to(device)[None, None]
    state = torch.as_tensor(q, dtype=torch.float32, device=device)[None]
    with torch.inference_mode():
        actions = policy(state, image)[0].cpu().numpy() * stats['action_std'] + stats['action_mean']
    if actions.ndim != 2 or actions.shape[1] != 6 or len(actions) == 0 or not np.isfinite(actions).all():
        raise RuntimeError('Policy returned invalid actions')
    return actions


def limit_action(current, proposed, motion, period, initial, enable_rotation, radius):
    proposed = np.asarray(proposed, dtype=float)
    if proposed.shape != (6,) or not np.isfinite(proposed).all():
        raise RuntimeError('Nonfinite or invalid policy target')
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


def run_act_rollout(config, rollout):
    if abs(config.motion.loop_hz - 50) > 1e-6:
        raise ValueError('These policies use 50 Hz action chunks; use --control-hz 50')
    policy, stats, device = load_policy(rollout.ckpt_dir)
    camera = make_camera(config)
    # Real state is read even in prediction-only mode. enable()/servo() are execute-only.
    robot = XArm6Robot(config.robot_ip, config.motion, dry_run=False, read_only=not rollout.execute)
    pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='act-inference')
    future = None
    try:
        camera.start()
        if not camera.calibration.get('rectified'):
            raise RuntimeError('Expected rectified Nano RGB')
        robot.connect()
        initial = target = robot.initialize_target_pose()
        frame = camera.latest()
        # Warm-up and initial inference before enabling the arm; CUDA initialization can be slow.
        future = pool.submit(predict_chunk, policy, stats, device, initial.copy(), frame.image, config.record_image_width, getattr(frame, 'depth', None))
        actions = future.result(timeout=60)
        future = None
        camera.check_mode(force=True)
        camera.latest()
        initial = target = robot.initialize_target_pose()
        # Check initial action before enabling any motion.
        limit_action(target, actions[0], config.motion, 1 / 50, initial, rollout.enable_rotation, rollout.workspace_radius)
        if rollout.execute:
            robot.enable()
        print('LIVE EXECUTION' if rollout.execute else 'PREDICTION ONLY: robot state reads, no motion commands', flush=True)
        print('Policy inputs: RGB + metric depth + validity + TCP pose' if getattr(policy, 'use_depth', False)
              else 'Policy inputs: RGB + TCP pose (depth is preview only)', flush=True)
        print('Ctrl-C or Enter stops. Orientation ' + ('enabled.' if rollout.enable_rotation else 'held fixed.'), flush=True)
        start = chunk_time = last_submit = time.monotonic()
        last_print = 0.0
        period = 1 / config.motion.loop_hz
        while time.monotonic() - start < rollout.duration_sec:
            tick = time.monotonic()
            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                break
            camera.check_mode()
            frame = camera.latest()
            pose, _ = robot.observation()
            if future is not None:
                if tick - submitted_at > rollout.prediction_timeout:
                    raise RuntimeError('Policy inference missed its deadline')
                if future.done():
                    actions = future.result()
                    chunk_time = submitted_at
                    future = None
            if future is None and tick - last_submit >= rollout.query_interval:
                submitted_at = last_submit = time.monotonic()
                future = pool.submit(predict_chunk, policy, stats, device, pose.copy(), frame.image, config.record_image_width, getattr(frame, 'depth', None))
            now = time.monotonic()
            index = int((now - chunk_time) * config.motion.loop_hz)
            if now - chunk_time > rollout.prediction_timeout or index >= len(actions):
                raise RuntimeError('No fresh policy action chunk; stopping')
            target = limit_action(target, actions[index], config.motion, period, initial,
                                  rollout.enable_rotation, rollout.workspace_radius)
            camera.latest()
            if rollout.execute:
                robot.servo(target, period, measured=pose)
            if now - last_print > .5:
                print(f't={now-start:.2f}s target={np.round(target, 5).tolist()} camera_age_ms={(now-frame.captured_monotonic)*1000:.0f}', flush=True)
                last_print = now
            if config.preview:
                cv2.imshow('ACT wrist_rgb', cv2.cvtColor(frame.image, cv2.COLOR_RGB2BGR))
                cv2.imshow('ACT depth: 0.1-1.0 m, near=red far=blue invalid=black',
                           colorize_depth(frame.depth))
                if cv2.waitKey(1) in (27, ord('q')):
                    break
            time.sleep(max(0, period - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        print('Stopping ACT rollout', flush=True)
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
