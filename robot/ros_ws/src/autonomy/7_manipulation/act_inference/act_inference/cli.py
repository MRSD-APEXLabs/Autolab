"""Command line of run_act_inference.py (the options of the teleop pipeline's `act-rollout` subcommand)."""
import argparse
from pathlib import Path

from .config import CAMERA_SOURCES, DEPTH_BACKENDS, InferenceConfig, MotionConfig, RolloutConfig


def build_parser():
    parser = argparse.ArgumentParser(
        description='Live ACT inference on the xArm6 from the ZED X Nano wrist camera. '
                    'Prediction-only (robot state is read, nothing is commanded) unless --execute is given.')
    parser.add_argument('--ckpt-dir', type=Path, required=True,
                        help='ACT checkpoint folder (policy_best.ckpt, config.pkl, dataset_stats.pkl)')
    parser.add_argument('--execute', action='store_true', help='Enable and command the physical arm')
    parser.add_argument('--duration-sec', type=float, default=10.0)
    parser.add_argument('--enable-rotation', action='store_true', help='Follow predicted orientation (default: hold the start orientation)')
    parser.add_argument('--query-interval', type=float, default=0.2, help='seconds between policy queries')
    parser.add_argument('--prediction-timeout', type=float, default=0.75, help='stop if a query or the current chunk is older than this, seconds')
    parser.add_argument('--workspace-radius', type=float, default=0.1, help='Maximum predicted translation distance from start, meters')
    parser.add_argument('--robot-ip', default='192.168.1.236')
    parser.add_argument('--camera-source', choices=CAMERA_SOURCES, default='nano-stream',
                        help="'nano-stream': stereo pairs straight from the Xavier, depth computed in this process (default); "
                             "'ros': rectified image and depth from a running zedx_nano_depth node")
    parser.add_argument('--camera-host', default='192.168.1.101', help='nano-stream: Xavier running the Nano stream server')
    parser.add_argument('--camera-port', type=int, default=8090, help='nano-stream server port')
    parser.add_argument('--ros-namespace', default='/zedx_nano', help='ros: namespace of the zedx_nano_depth topics')
    parser.add_argument('--record-image-width', type=int, default=480,
                        help='RGB width the training demos were recorded at (0 = native stream width); '
                             'frames are downscaled to it before the policy resize, as in recording')
    parser.add_argument('--depth-width', type=int, default=640, help='nano-stream: image width used for stereo matching')
    parser.add_argument('--depth-backend', choices=DEPTH_BACKENDS, default='neural',
                        help="nano-stream: 'neural' GPU stereo network (default) or 'sgbm' OpenCV on the CPU")
    parser.add_argument('--depth-model', default='raft-realtime',
                        help='neural backend: raft-realtime (default), raft-middlebury, fast-foundation or a .pth path')
    parser.add_argument('--depth-repo', default='', help='fast-foundation: path of the Fast-FoundationStereo checkout')
    parser.add_argument('--depth-models-dir', default='',
                        help='neural weights folder (default $ZEDX_NANO_DEPTH_MODELS or ~/.cache/zedx_nano_depth; missing RAFT weights are downloaded)')
    parser.add_argument('--max-frame-age', type=float, default=0.25, help='camera staleness budget, seconds')
    parser.add_argument('--preview', action='store_true', help='show the policy image and depth in OpenCV windows')
    parser.add_argument('--control-hz', type=float, default=50)
    parser.add_argument('--translation-speed', type=float, default=0.025, help='meters/second')
    parser.add_argument('--rotation-speed', type=float, default=0.1, help='radians/second')
    return parser


def configs_from_args(args):
    config = InferenceConfig(
        robot_ip=args.robot_ip, camera_source=args.camera_source, camera_host=args.camera_host,
        camera_port=args.camera_port, ros_namespace=args.ros_namespace, record_image_width=args.record_image_width,
        depth_width=args.depth_width, depth_backend=args.depth_backend, depth_model=args.depth_model,
        depth_repo=args.depth_repo, depth_models_dir=args.depth_models_dir, preview=args.preview,
        max_frame_age=args.max_frame_age,
        motion=MotionConfig(loop_hz=args.control_hz, translation_speed=args.translation_speed,
                            rotation_speed=args.rotation_speed))
    rollout = RolloutConfig(ckpt_dir=args.ckpt_dir, execute=args.execute, duration_sec=args.duration_sec,
                            enable_rotation=args.enable_rotation, query_interval=args.query_interval,
                            prediction_timeout=args.prediction_timeout, workspace_radius=args.workspace_radius)
    return config, rollout


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        config, rollout = configs_from_args(args)
        from .rollout import run_act_rollout
        return run_act_rollout(config, rollout)
    except (RuntimeError, ValueError, OSError) as exc:
        raise SystemExit(str(exc)) from exc
