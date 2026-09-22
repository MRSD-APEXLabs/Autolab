"""The launch files under ros2 launch, with stand-in executables that record how each node was started (needs ROS 2).

`$(find-pkg-share ...)` resolves to the source packages through a temporary ament prefix (they
install launch/ and config/ unchanged), and each stand-in writes its node name, namespace and
parameters, as the files it was given resolve them, then exits.
"""
import json
import os
from pathlib import Path
import shutil
import subprocess

import pytest

pytest.importorskip('launch_ros')
yaml = pytest.importorskip('yaml')

PACKAGES = Path(__file__).resolve().parents[2]
EXECUTABLES = {'zedx_nano_camera': ('camera_hub_node', 'zedx_nano_camera_node'),
               'zedx_nano_depth': ('zedx_nano_depth_node',), 'camera_ui': ('camera_ui_node',)}
STAND_IN = '''#!/usr/bin/env python3
import json, os, sys
args = sys.argv[1:]
value = lambda flag, prefix: [a for i, a in enumerate(args[1:]) if args[i] == flag and a.startswith(prefix)]
record = {'executable': os.path.basename(sys.argv[0]), 'name': value('-r', '__node:=')[0][8:],
          'namespace': (value('-r', '__ns:=') or ['__ns:='])[0][6:], 'params_files': value('--params-file', '')}
with open(os.path.join(os.environ['CAMERA_UI_TEST_OUT'], record['name'] + '.json'), 'w') as f:
    json.dump(record, f)
'''


@pytest.fixture
def launch(tmp_path):
    """launch(file, *args) -> {node name: record} of the nodes that `ros2 launch camera_ui <file> <args>` starts."""
    ros2 = shutil.which('ros2')
    if ros2 is None:
        pytest.skip('ros2 not on PATH')
    prefix, out = tmp_path / 'prefix', tmp_path / 'out'
    index = prefix / 'share' / 'ament_index' / 'resource_index' / 'packages'
    index.mkdir(parents=True)
    for package, executables in EXECUTABLES.items():
        (index / package).touch()
        (prefix / 'share' / package).symlink_to(PACKAGES / package)
        (prefix / 'lib' / package).mkdir(parents=True)
        for executable in executables:
            path = prefix / 'lib' / package / executable
            path.write_text(STAND_IN)
            path.chmod(0o755)
    env = dict(os.environ, AMENT_PREFIX_PATH=os.pathsep.join([str(prefix), os.environ.get('AMENT_PREFIX_PATH', '')]),
               CAMERA_UI_TEST_OUT=str(out), TMPDIR=str(tmp_path), ROS_LOG_DIR=str(tmp_path / 'log'))

    def run(file, *args):
        shutil.rmtree(out, ignore_errors=True)
        out.mkdir()
        result = subprocess.run([ros2, 'launch', 'camera_ui', file, *args], env=env, capture_output=True, text=True,
                                timeout=60)
        assert result.returncode == 0, result.stdout + result.stderr
        nodes = {}
        for path in out.glob('*.json'):
            record = json.loads(path.read_text())
            record['params'] = {}
            for params_file in record['params_files']:   # later files win, as in rcl
                for pattern, entry in yaml.safe_load(Path(params_file).read_text()).items():
                    assert pattern in ('/**', f'{record["namespace"].rstrip("/")}/{record["name"]}'), pattern
                    record['params'].update(entry['ros__parameters'])
            # the package yamls; launch writes each <param name= value=> to a temporary launch_params_* file
            record['yamls'] = [os.path.realpath(f) for f in record['params_files'] if f.endswith('.yaml')]
            nodes[record['name']] = record
        return nodes
    return run


def config(package, name):
    return str(PACKAGES / package / 'config' / name)


def test_camera_ui_launch_starts_the_hub_both_cameras_and_the_ui(launch):
    nodes = launch('camera_ui.launch.xml')
    assert sorted(nodes) == ['camera_hub', 'camera_ui', 'zedx_camera', 'zedx_depth', 'zedx_nano_camera', 'zedx_nano_depth']

    hub = nodes['camera_hub']
    assert (hub['executable'], hub['namespace']) == ('camera_hub_node', '/')
    # its own yaml, not camera_ui's: an include sees the including file's launch arguments
    assert hub['yamls'] == [config('zedx_nano_camera', 'camera_hub.yaml')]
    assert hub['params']['host'] == '192.168.1.101' and hub['params']['port'] == 8090
    assert hub['params']['cameras'] == ['zedx', 'zedx_nano'] and 'labels' not in hub['params']

    for camera in ('zedx', 'zedx_nano'):
        node = nodes[f'{camera}_camera']
        assert (node['executable'], node['namespace']) == ('zedx_nano_camera_node', f'/{camera}')
        assert node['yamls'] == [config('zedx_nano_camera', 'zedx_nano_camera.yaml')]
        assert node['params']['camera'] == camera
        assert (node['params']['host'], node['params']['port']) == ('192.168.1.101', 8090)
        assert node['params'].get('left_frame_id', '') == ''   # derived from the camera: <camera>_left_camera_optical_frame
        depth = nodes[f'{camera}_depth']
        assert (depth['executable'], depth['namespace']) == ('zedx_nano_depth_node', f'/{camera}')
        assert depth['yamls'] == [config('zedx_nano_depth', 'zedx_nano_depth.yaml')]
        params = depth['params']
        assert (params['backend'], params['model'], params['models_dir']) == ('neural', 'raft-realtime', '')

    ui = nodes['camera_ui']
    assert (ui['executable'], ui['namespace']) == ('camera_ui_node', '')
    assert ui['yamls'] == [config('camera_ui', 'camera_ui.yaml')]
    assert (ui['params']['host'], ui['params']['port']) == ('0.0.0.0', 8001)
    assert ui['params']['cameras'] == ['zedx', 'zedx_nano'] and ui['params']['hub_node'] == '/camera_hub'


def test_launch_arguments_reach_the_nodes(launch):
    nodes = launch('cameras.launch.xml', 'host:=10.1.2.3', 'port:=9001', 'backend:=sgbm', 'model:=raft-fast',
                   'models_dir:=/models')
    assert sorted(nodes) == ['camera_hub', 'zedx_camera', 'zedx_depth', 'zedx_nano_camera', 'zedx_nano_depth']
    for name in ('camera_hub', 'zedx_camera', 'zedx_nano_camera'):
        assert (nodes[name]['params']['host'], nodes[name]['params']['port']) == ('10.1.2.3', 9001)
    for name in ('zedx_depth', 'zedx_nano_depth'):
        params = nodes[name]['params']
        assert (params['backend'], params['model'], params['models_dir']) == ('sgbm', 'raft-fast', '/models')

    nodes = launch('camera_ui.launch.xml', 'with_cameras:=false', 'ui_host:=127.0.0.1', 'ui_port:=8123')
    assert list(nodes) == ['camera_ui']
    assert (nodes['camera_ui']['params']['host'], nodes['camera_ui']['params']['port']) == ('127.0.0.1', 8123)
