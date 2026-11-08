#!/usr/bin/env python3
"""Read-only SDL controller preflight. Publishes no ROS topics or motor commands."""
import argparse
from pathlib import Path
import sys
import yaml

NAVIGATION = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NAVIGATION/'swerve_navigation'))
from swerve_navigation.joy_auto import SDLControllers


def missing_gamepad_nodes(sys_root=Path('/sys/class/input'), dev_root=Path('/dev/input')):
    missing = []
    for event in sorted(sys_root.glob('event*')):
        try:
            name = (event/'device/name').read_text().strip()
        except OSError:
            continue
        if any(token in name.lower() for token in ('8bitdo', 'xbox', 'x-box')):
            path = dev_root/event.name
            if not path.exists():
                missing.append((str(path), name))
    return missing


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path,
                        default=NAVIGATION/'navigation_bringup/config/joy.yaml')
    args = parser.parse_args()
    config = yaml.safe_load(args.config.read_text())['joy_node']['ros__parameters']
    driver = None
    try:
        driver = SDLControllers()
        identity, name, _, _ = driver.read(config['preferred_device_name'],
                                          config['fallback_device_name'], config['deadzone'])
        if identity is not None:
            print(f'Controller ready: {name}. Release LT and centre sticks after startup.')
            return 0
        print('No accessible preferred Bluetooth controller or wired fallback.', file=sys.stderr)
        for path, name in missing_gamepad_nodes():
            print(f'  {name} exists in sysfs, but {path} is missing inside this container.', file=sys.stderr)
        print('Check pairing, input permissions and the live /dev/input mount. See '
              '1_navigation/README.md (Wireless controller).', file=sys.stderr)
        return 1
    except (OSError, RuntimeError) as exc:
        print(f'Controller check failed: {exc}', file=sys.stderr)
        return 1
    finally:
        if driver is not None:
            driver.close()


if __name__ == '__main__':
    raise SystemExit(main())
