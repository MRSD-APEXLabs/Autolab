#!/usr/bin/env python3
"""Save base ZED X JPEG frames and ROS timestamps; subscribe only, never switch cameras.

    python3 record_zedx_base.py --duration 30
    python3 record_zedx_base.py --count 100 --fps 5 --out /tmp/base_frames
"""
import argparse
import csv
from datetime import datetime, timezone
import math
from pathlib import Path
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/zedx/left/image_raw/compressed')
    parser.add_argument('--out', type=Path, default=None)
    parser.add_argument('--duration', type=float, default=0, help='seconds from first saved frame; 0 = until Ctrl+C')
    parser.add_argument('--count', type=int, default=0, help='stop after this many saved frames; 0 = unlimited')
    parser.add_argument('--fps', type=float, default=0, help='maximum save rate; 0 = every received frame')
    parser.add_argument('--timeout', type=float, default=10, help='seconds to wait for the first frame')
    args = parser.parse_args()
    if args.count < 0 or any(not math.isfinite(v) or v < 0 for v in (args.duration, args.fps)):
        parser.error('--count, --duration and --fps must be non-negative and finite')
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be positive and finite')
    output = args.out or Path('data/zedx_base') / datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage

    rclpy.init(args=[])
    node = rclpy.create_node('record_zedx_base')
    saved, started, last_saved = 0, None, float('-inf')
    manifest = None
    writer = None

    def frame(msg):
        nonlocal saved, started, last_saved, manifest, writer
        now = time.monotonic()
        if args.fps and now - last_saved < 1 / args.fps:
            return
        payload = bytes(msg.data)
        if not payload.startswith(b'\xff\xd8'):
            raise RuntimeError(f'{args.topic} must publish JPEG CompressedImage; format={msg.format!r}')
        if manifest is None:
            output.mkdir(parents=True, exist_ok=True)
            manifest = (output / 'frames.csv').open('x', newline='')
            writer = csv.writer(manifest)
            writer.writerow(['filename', 'ros_stamp_ns', 'received_wall_ns', 'frame_id'])
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        filename = f'{saved:06d}_{stamp}.jpg'
        with (output / filename).open('xb') as stream:
            stream.write(payload)
        writer.writerow([filename, stamp, time.time_ns(), msg.header.frame_id])
        manifest.flush()
        started = now if started is None else started
        last_saved = now
        saved += 1

    node.create_subscription(CompressedImage, args.topic, frame, qos_profile_sensor_data)
    print(f'Recording {args.topic} -> {output.resolve()} (Ctrl+C stops)', flush=True)
    waiting_since = time.monotonic()
    try:
        while rclpy.ok():
            if args.count and saved >= args.count:
                break
            if started is not None and args.duration and time.monotonic() - started >= args.duration:
                break
            if started is None and time.monotonic() - waiting_since > args.timeout:
                raise RuntimeError(f'No frames on {args.topic}; check the camera stream and ROS_DOMAIN_ID')
            rclpy.spin_once(node, timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        if manifest is not None:
            manifest.close()
        node.destroy_node()
        rclpy.try_shutdown()
        print(f'Saved {saved} JPEG frames to {output.resolve()}', flush=True)


if __name__ == '__main__':
    main()
