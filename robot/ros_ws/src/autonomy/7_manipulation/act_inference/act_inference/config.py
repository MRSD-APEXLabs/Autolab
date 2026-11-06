"""Settings of a live ACT rollout: robot, camera source, depth backend and motion limits."""
from dataclasses import dataclass, field
from pathlib import Path
import math

CAMERA_SOURCES = ('nano-stream', 'ros')
DEPTH_BACKENDS = ('sgbm', 'neural')
# Robot state the policies were trained on: measured TCP position (m) and rotation vector (rad).
STATE_SPACE = 'measured_tcp_xyz_m_rotvec_rad'


@dataclass(frozen=True)
class MotionConfig:
    loop_hz: float = 50.0
    translation_speed: float = 0.025
    rotation_speed: float = 0.1
    max_tracking_error: float = 0.03

    def __post_init__(self):
        for name in ('loop_hz', 'translation_speed', 'rotation_speed', 'max_tracking_error'):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be finite and positive')


@dataclass(frozen=True)
class InferenceConfig:
    robot_ip: str = '192.168.1.236'
    camera_source: str = 'nano-stream'  # Xavier MJPEG server with depth computed here, or 'ros' (zedx_nano_depth topics)
    camera_host: str = '192.168.1.101'
    camera_port: int = 8090
    ros_namespace: str = '/zedx_nano'   # ros: namespace of the zedx_nano_depth node
    record_image_width: int = 480       # RGB width of the training demos (0 = native stream size)
    depth_width: int = 640              # nano-stream: width at which stereo matching runs
    depth_backend: str = 'neural'       # nano-stream: 'neural' (GPU stereo network) or 'sgbm' (OpenCV, CPU)
    depth_model: str = 'raft-realtime'  # neural: raft-realtime | raft-middlebury | fast-foundation | /path/to.pth
    depth_repo: str = ''                # fast-foundation: Fast-FoundationStereo checkout ('' = <models dir>/Fast-FoundationStereo)
    depth_models_dir: str = ''          # neural weights ('' = $ZEDX_NANO_DEPTH_MODELS or ~/.cache/zedx_nano_depth)
    preview: bool = False
    max_frame_age: float = 0.25
    motion: MotionConfig = field(default_factory=MotionConfig)

    def __post_init__(self):
        if self.camera_source not in CAMERA_SOURCES:
            raise ValueError(f'camera_source must be one of {CAMERA_SOURCES}')
        if not 0 < int(self.camera_port) < 65536:
            raise ValueError('camera_port must be a TCP port')
        if int(self.record_image_width) < 0:
            raise ValueError('record_image_width must be zero (native) or positive')
        if not 160 <= int(self.depth_width) <= 4096:
            raise ValueError('depth_width must be between 160 and 4096 pixels')
        if self.depth_backend not in DEPTH_BACKENDS:
            raise ValueError(f'depth_backend must be one of {DEPTH_BACKENDS}')
        if not math.isfinite(self.max_frame_age) or self.max_frame_age <= 0:
            raise ValueError('max_frame_age must be finite and positive')


@dataclass(frozen=True)
class RolloutConfig:
    ckpt_dir: Path
    execute: bool = False
    duration_sec: float = 10.0
    enable_rotation: bool = False
    query_interval: float = 0.2
    prediction_timeout: float = 0.75
    workspace_radius: float = 0.1
    contact_gap: float = 0.001       # m the target may sit below the TCP before it counts as pressing; 0 disables
    contact_hold: float = 0.3        # s of continuous pressing before the rollout stops
    contact_max_lag: float = 0.1     # s; a target this far ahead of the TCP's own descent is still free motion
    settle_motion: float = 0.002     # m of predicted travel over a whole chunk below which the policy has arrived; 0 disables
    settle_queries: int = 3          # consecutive arrived chunks before the rollout ends
    approach_speed: float = 0.003    # m/s once the policy predicts under approach_distance of travel; 0 keeps full speed
    approach_distance: float = 0.012 # m
    ensemble: int = 5                # near the goal, average the current waypoint of this many recent chunks; 1 disables
    final_offset: tuple = (0.0, 0.0, 0.0)  # m, robot base frame: one straight move after the policy reports arrival

    def __post_init__(self):
        for name in ('duration_sec', 'query_interval', 'prediction_timeout', 'workspace_radius', 'contact_hold'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) <= 0:
                raise ValueError(f'{name} must be finite and positive')
        for name in ('contact_gap', 'contact_max_lag', 'settle_motion', 'approach_speed', 'approach_distance'):
            if not math.isfinite(getattr(self, name)) or getattr(self, name) < 0:
                raise ValueError(f'{name} must be finite and non-negative')
        if int(self.settle_queries) != self.settle_queries or self.settle_queries < 1:
            raise ValueError('settle_queries must be a positive integer')
        if int(self.ensemble) != self.ensemble or self.ensemble < 1:
            raise ValueError('ensemble must be a positive integer')
        offset = tuple(float(v) for v in self.final_offset)
        if len(offset) != 3 or not all(math.isfinite(v) for v in offset) or math.hypot(*offset) > 0.01:
            raise ValueError('final_offset must be three finite values within 10 mm')
