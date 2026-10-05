"""visual_servo.launch.xml under ros2 launch with a stand-in executable that records its parameters (needs ROS 2)."""
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

pytest.importorskip('launch_ros')
yaml = pytest.importorskip('yaml')

PACKAGES = Path(__file__).resolve().parents[2]
EXECUTABLES = {'visual_servoing': ('visual_servo_node',), 'camera_ui': (), 'camera_perception': (),
               'zedx_nano_camera': ('camera_hub_node', 'zedx_nano_camera_node'),
               'zedx_nano_depth': ('zedx_nano_depth_node',)}


def source_of(package):
    """A package's source folder: next to visual_servoing, or in a sibling group folder (Autolab autonomy/N_area)."""
    candidates = [PACKAGES / package] + sorted(PACKAGES.parent.glob(f'*/{package}'))
    return next(path for path in candidates if (path / 'package.xml').is_file())
STAND_IN = '''#!/usr/bin/env python3
import json, os, sys
args = sys.argv[1:]
ros = args[args.index('--ros-args') + 1:] if '--ros-args' in args else []
pairs = [(ros[i], ros[i + 1]) for i in range(len(ros) - 1)]
name = [v for f, v in pairs if f == '-r' and v.startswith('__node:=')][0][8:]
record = {'params_files': [v for f, v in pairs if f == '--params-file'],
          'params': dict(v.split(':=', 1) for f, v in pairs if f == '-p')}
with open(os.path.join(os.environ['VISUAL_SERVO_TEST_OUT'], name + '.json'), 'w') as f:
    json.dump(record, f)
'''


@pytest.fixture
def launch(tmp_path):
    ros2 = shutil.which('ros2')
    if ros2 is None:
        pytest.skip('ros2 not on PATH')
    prefix, out = tmp_path / 'prefix', tmp_path / 'out'
    index = prefix / 'share' / 'ament_index' / 'resource_index' / 'packages'
    index.mkdir(parents=True)
    for package, executables in EXECUTABLES.items():
        (index / package).touch()
        (prefix / 'share' / package).symlink_to(source_of(package))
        (prefix / 'lib' / package).mkdir(parents=True)
        for executable in executables:
            path = prefix / 'lib' / package / executable
            path.write_text(STAND_IN)
            path.chmod(0o755)
    env = dict(os.environ, AMENT_PREFIX_PATH=os.pathsep.join([str(prefix), os.environ.get('AMENT_PREFIX_PATH', '')]),
               VISUAL_SERVO_TEST_OUT=str(out), TMPDIR=str(tmp_path), ROS_LOG_DIR=str(tmp_path / 'log'))

    def run(*args):
        shutil.rmtree(out, ignore_errors=True)
        out.mkdir()
        result = subprocess.run([ros2, 'launch', 'visual_servoing', 'visual_servo.launch.xml', *args], env=env,
                                capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stdout + result.stderr
        nodes = {}
        for path in out.glob('*.json'):
            record = json.loads(path.read_text())
            params = {}
            for params_file in record['params_files']:     # the YAML, then the launch file's own values
                for entry in yaml.safe_load(Path(params_file).read_text()).values():
                    stack = [('', entry['ros__parameters'])]
                    while stack:
                        prefix, values = stack.pop()
                        for key, value in values.items():
                            if isinstance(value, dict):
                                stack.append((f'{prefix}{key}.', value))
                            else:
                                params[prefix + key] = value
            params.update({k: yaml.safe_load(v) for k, v in record['params'].items()})
            record['params'] = params
            record['yamls'] = [os.path.realpath(f) for f in record['params_files'] if f.endswith('visual_servo.yaml')]
            nodes[path.stem] = record
        return nodes
    return run


def test_default_launch_is_a_dry_run(launch):
    nodes = launch()
    assert list(nodes) == ['visual_servo']
    node = nodes['visual_servo']
    assert node['yamls'] == [str(source_of('visual_servoing') / 'config' / 'visual_servo.yaml')]
    params = node['params']
    assert params['execute'] is False and params['preview'] is False and params['grasp.enable'] is True
    assert params['yolo.model'].endswith('/camera_perception/models/yolo/best_wrist.pt')   # shipped in the package
    assert (params['robot_ip'], params['control_port'], params['yolo.device']) == ('192.168.1.236', 8765, 'cuda:0')
    assert params['mount.tilt_deg'] == 20.0 and params['servo.hover_height'] == 0.08


def test_launch_arguments(launch):
    params = launch('execute:=true', 'grasp:=false', 'model:=best_top.pt', 'yolo_models_dir:=/models', 'control_port:=-1',
                    'robot_ip:=10.0.0.5', 'device:=cpu')['visual_servo']['params']
    assert params['execute'] is True and params['grasp.enable'] is False
    assert params['yolo.model'] == '/models/best_top.pt'
    assert (params['control_port'], params['robot_ip'], params['yolo.device']) == (-1, '10.0.0.5', 'cpu')
    nodes = launch('with_cameras:=true')
    assert sorted(nodes) == ['camera_hub', 'visual_servo', 'zedx_camera', 'zedx_depth', 'zedx_nano_camera',
                             'zedx_nano_depth']
