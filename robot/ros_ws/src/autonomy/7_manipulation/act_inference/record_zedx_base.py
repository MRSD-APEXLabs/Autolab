#!/usr/bin/env python3
"""Save base ZED X JPEG frames and capture timestamps without switching cameras.

    python3 record_zedx_base.py --duration 30
    python3 record_zedx_base.py --count 100 --fps 5 --out /tmp/base_frames
    python3 record_zedx_base.py --source http --count 10
    python3 record_zedx_base.py --source ros --topic /zedx/left/image_raw/compressed

Auto mode tries ROS first, then reads the Xavier hub directly if no frames arrive.
HTTP frames have no ROS stamp: ros_stamp_ns is empty and capture_wall_ns is the
capture time mapped to this machine's wall clock. ROS frames retain their ROS stamp.
"""
import argparse
import csv
from datetime import datetime, timezone
import math
import os
from pathlib import Path
import sys
import time


class Recorder:
    def __init__(self, output, *, count=0, duration=0, fps=0):
        self.output = output
        self.count, self.duration, self.fps = count, duration, fps
        self.saved, self.started, self.last_saved = 0, None, float('-inf')
        self.manifest = None

    def done(self):
        return bool((self.count and self.saved >= self.count) or
                    (self.started is not None and self.duration and
                     time.monotonic() - self.started >= self.duration))

    def save(self, payload, stamp, frame_id, *, source='ros', capture_wall_ns=None):
        now = time.monotonic()
        if self.done() or (self.fps and now - self.last_saved < 1 / self.fps):
            return
        if not payload.startswith(b'\xff\xd8'):
            raise RuntimeError(f'{source} stream must contain JPEG frames')
        if self.manifest is None:
            self.output.mkdir(parents=True, exist_ok=True)
            self.manifest = (self.output / 'frames.csv').open('x', newline='')
            self.writer = csv.writer(self.manifest)
            self.writer.writerow(['filename', 'ros_stamp_ns', 'received_wall_ns', 'frame_id',
                                  'source', 'capture_wall_ns'])
        filename = f'{self.saved:06d}_{stamp if stamp is not None else capture_wall_ns}.jpg'
        with (self.output / filename).open('xb') as stream:
            stream.write(payload)
        self.writer.writerow([filename, stamp, time.time_ns(), frame_id, source, capture_wall_ns])
        self.manifest.flush()
        self.started = now if self.started is None else self.started
        self.last_saved = now
        self.saved += 1

    def close(self):
        if self.manifest is not None:
            self.manifest.close()


def record_ros(args, recorder):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    rclpy.init(args=[])
    node = rclpy.create_node('record_zedx_base')

    def frame(msg):
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        recorder.save(bytes(msg.data), stamp, msg.header.frame_id)

    try:
        node.create_subscription(CompressedImage, args.topic, frame, qos_profile_sensor_data)
        print(f'Waiting for {args.topic} (ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")})', flush=True)
        waiting_since = last_frame = time.monotonic()
        saved = recorder.saved
        while rclpy.ok() and not recorder.done():
            now = time.monotonic()
            if recorder.saved != saved:
                saved, last_frame = recorder.saved, now
            # FPS limiting can intentionally leave more than timeout seconds between saves.
            stall_timeout = max(args.timeout, 2 / args.fps) if args.fps else args.timeout
            if not recorder.saved and now - waiting_since >= args.timeout:
                if args.source == 'auto':
                    print('No ROS frames; trying the base camera HTTP stream.', flush=True)
                    return True
                raise RuntimeError(f'No frames on {args.topic}; check camera publishers, ROS_DOMAIN_ID '
                                   'and DDS discovery, or use --source http')
            if recorder.saved and now - last_frame >= stall_timeout:
                raise RuntimeError(f'Camera stream stalled on {args.topic}')
            rclpy.spin_once(node, timeout_sec=.1)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return False


def record_http(args, recorder):
    # Register the sibling camera package, just like the inference entry points.
    import act_inference  # noqa: F401
    from zedx_nano_camera.stream import (NanoStreamClient, captured_monotonic, frame_header,
                                         iter_mjpeg_parts, output_size)

    client = NanoStreamClient(args.host, args.port, args.timeout, camera='zedx')
    info = client.fetch_info('left')  # Raises if the base camera is inactive; never selects it.
    offset, _, mono_offset = client.synchronize_clock()
    width, height = output_size(info)
    print(f'Recording {client.url("/video_feed/left")}', flush=True)
    with client.open_feed('left') as response:
        for headers, payload in iter_mjpeg_parts(response):
            if recorder.done():
                break
            wall, mono = time.time_ns(), time.monotonic()
            metadata = frame_header(headers, width, height)
            captured = captured_monotonic(metadata, wall, mono, offset, mono_offset)
            capture_wall_ns = wall - round((mono - captured) * 1e9)
            recorder.save(payload, None, 'zedx_left_camera_optical_frame',
                          source='http', capture_wall_ns=capture_wall_ns)
            if recorder.done():
                break
        else:
            raise RuntimeError('Base camera HTTP stream ended before recording completed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', choices=('auto', 'ros', 'http'), default='auto')
    parser.add_argument('--topic', default=None, help='ROS topic (implies --source ros in auto mode)')
    parser.add_argument('--host', default='192.168.1.101', help='Xavier camera hub host')
    parser.add_argument('--port', type=int, default=8090, help='Xavier camera hub HTTP port')
    parser.add_argument('--out', type=Path, default=None)
    parser.add_argument('--duration', type=float, default=0, help='seconds from first saved frame; 0 = until Ctrl+C')
    parser.add_argument('--count', type=int, default=0, help='stop after this many saved frames; 0 = unlimited')
    parser.add_argument('--fps', type=float, default=0, help='maximum save rate; 0 = every received frame')
    parser.add_argument('--timeout', type=float, default=10, help='first-frame/stall/network timeout in seconds')
    args = parser.parse_args()
    if args.count < 0 or any(not math.isfinite(v) or v < 0 for v in (args.duration, args.fps)):
        parser.error('--count, --duration and --fps must be non-negative and finite')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    if not 1 <= args.port <= 65535:
        parser.error('--port must be between 1 and 65535')
    if args.topic is not None and args.source == 'auto':
        args.source = 'ros'
    args.topic = args.topic or '/zedx/left/image_raw/compressed'
    output = args.out or Path('data/zedx_base') / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    recorder = Recorder(output, count=args.count, duration=args.duration, fps=args.fps)
    print(f'Saving to {output.resolve()} (Ctrl+C stops)', flush=True)
    try:
        use_http = args.source == 'http'
        if not use_http:
            try:
                use_http = record_ros(args, recorder)
            except ImportError as exc:
                if args.source != 'auto':
                    raise RuntimeError('ROS Python packages unavailable; source the ROS environment '
                                       'or use --source http') from exc
                print(f'ROS unavailable ({exc}); trying HTTP.', flush=True)
                use_http = True
        if use_http:
            record_http(args, recorder)
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'Error: {exc}', file=sys.stderr, flush=True)
        return 1
    finally:
        recorder.close()
        print(f'Saved {recorder.saved} JPEG frames to {output.resolve()}', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
