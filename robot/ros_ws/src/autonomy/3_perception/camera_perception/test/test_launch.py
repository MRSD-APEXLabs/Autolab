"""perception.launch.xml under ros2 launch, with stand-in executables that record how each node was started (needs ROS 2).

`$(find-pkg-share ...)` resolves to the source packages through a temporary ament prefix (they install launch/ and
config/ unchanged); each stand-in writes its node name, namespace, remaps, arguments and parameters, then exits.
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
EXECUTABLES = {'camera_perception': ('perception_node',), 'tf2_ros': ('static_transform_publisher',),
               'zedx_nano_camera': ('camera_hub_node', 'zedx_nano_camera_node'),
               'zedx_nano_depth': ('zedx_nano_depth_node',), 'camera_ui': ('camera_ui_node',)}
SOURCES = {'camera_perception', 'zedx_nano_camera', 'zedx_nano_depth', 'camera_ui'}   # tf2_ros: executable only
STAND_IN = '''#!/usr/bin/env python3
import json, os, sys
args = sys.argv[1:]
ros = args[args.index('--ros-args') + 1:] if '--ros-args' in args else []
pairs = [(ros[i], ros[i + 1]) for i in range(len(ros) - 1)]
remaps = [v for f, v in pairs if f == '-r']
record = {'executable': os.path.basename(sys.argv[0]),
          'name': [r for r in remaps if r.startswith('__node:=')][0][8:],
          'namespace': ([r for r in remaps if r.startswith('__ns:=')] or ['__ns:='])[0][6:],
          'remaps': dict(r.split(':=', 1) for r in remaps if not r.startswith('__')),
          'params_files': [v for f, v in pairs if f == '--params-file'],
          'arguments': args[:args.index('--ros-args')] if '--ros-args' in args else args}
with open(os.path.join(os.environ['PERCEPTION_TEST_OUT'], record['name'] + '.json'), 'w') as f:
    json.dump(record, f)
'''


def flatten(params, prefix=''):
    flat = {}
    for key, value in params.items():
        if isinstance(value, dict):
            flat.update(flatten(value, f'{prefix}{key}.'))
        else:
            flat[prefix + key] = value
    return flat


@pytest.fixture
def launch(tmp_path):
    """launch(*args) -> {node name: record} of the nodes `ros2 launch camera_perception perception.launch.xml` starts."""
    ros2 = shutil.which('ros2')
    if ros2 is None:
        pytest.skip('ros2 not on PATH')
    prefix, out = tmp_path / 'prefix', tmp_path / 'out'
    index = prefix / 'share' / 'ament_index' / 'resource_index' / 'packages'
    index.mkdir(parents=True)
    for package, executables in EXECUTABLES.items():
        (index / package).touch()
        if package in SOURCES:
            (prefix / 'share' / package).symlink_to(PACKAGES / package)
        (prefix / 'lib' / package).mkdir(parents=True)
        for executable in executables:
            path = prefix / 'lib' / package / executable
            path.write_text(STAND_IN)
            path.chmod(0o755)
    env = dict(os.environ, AMENT_PREFIX_PATH=os.pathsep.join([str(prefix), os.environ.get('AMENT_PREFIX_PATH', '')]),
               PERCEPTION_TEST_OUT=str(out), TMPDIR=str(tmp_path), ROS_LOG_DIR=str(tmp_path / 'log'))

    def run(*args):
        shutil.rmtree(out, ignore_errors=True)
        out.mkdir()
        result = subprocess.run([ros2, 'launch', 'camera_perception', 'perception.launch.xml', *args], env=env,
                                capture_output=True, text=True, timeout=60)
        assert result.returncode == 0, result.stdout + result.stderr
        nodes = {}
        for path in out.glob('*.json'):
            record = json.loads(path.read_text())
            record['params'] = {}
            for params_file in record['params_files']:   # later files win, as in rcl
                for pattern, entry in yaml.safe_load(Path(params_file).read_text()).items():
                    assert pattern in ('/**', f'{record["namespace"].rstrip("/")}/{record["name"]}'), pattern
                    record['params'].update(flatten(entry['ros__parameters']))
            record['yamls'] = [os.path.realpath(f) for f in record['params_files'] if f.endswith('.yaml')]
            nodes[record['name']] = record
        return nodes
    return run


def config(name):
    return str(PACKAGES / 'camera_perception' / 'config' / name)


def test_default_launch_maps_both_cameras_to_the_pc3_topics(launch):
    nodes = launch()
    assert sorted(nodes) == ['top_camera_to_zedx', 'zedx_nano_perception', 'zedx_perception']

    zedx = nodes['zedx_perception']
    assert (zedx['executable'], zedx['namespace']) == ('perception_node', '/zedx')
    assert zedx['yamls'] == [config('zedx.yaml')]
    params = zedx['params']
    assert params['yolo.model'] == '/home/labx/apple_server_manip/data/models/yolo/best_wrist.pt'
    assert (params['yolo.clahe_clip'], params['yolo.clahe_tiles']) == (3.0, 4)
    assert (params['output_frame'], params['yolo.marker_ns'], params['yolo.conf']) == ('top_camera', 'wellplates', 0.8)
    assert params['apriltags.ids'] == [0, 1, 2, 21, 22] and params['yolo.classes'] == 'wellplate'
    assert (params['pointcloud.enable'], params['pointcloud.mode'], params['pointcloud.fresh']) == (True, 'live', False)
    assert zedx['remaps'] == {'perception/apriltags': '/inspect/apriltags', 'perception/detections': '/inspect/wellplates',
                              'perception/image': '/inspect/image',
                              'perception/annotated_image': '/inspect/annotated_image',
                              'perception/json': '/inspect/json', 'perception/points': '/zed_pointcloud',
                              'perception/capture_snapshot': '/inspect/capture_snapshot'}

    nano = nodes['zedx_nano_perception']
    assert (nano['executable'], nano['namespace']) == ('perception_node', '/zedx_nano')
    assert nano['yamls'] == [config('zedx_nano.yaml')]
    params = nano['params']
    assert params['yolo.model'] == '/home/labx/apple_server_manip/data/models/yolo/best_wrist.pt'
    assert (params['pointcloud.enable'], params['yolo.max_detections'], params['yolo.marker_ns']) == \
        (False, 1, 'servo_detections')
    assert set(nano['remaps'].values()) == {'/servo/apriltags', '/servo/detections', '/servo/image',
                                            '/servo/annotated_image', '/servo/json'}

    tf = nodes['top_camera_to_zedx']
    assert tf['executable'] == 'static_transform_publisher'
    assert tf['arguments'] == ['--frame-id', 'top_camera', '--child-frame-id', 'zedx_left_camera_optical_frame']


def test_launch_arguments(launch):
    nodes = launch('zedx_nano:=false', 'top_camera_tf:=false', 'cloud_mode:=snapshot', 'fresh:=true',
                   'yolo_models_dir:=/models', 'zedx_model:=best_seg.pt', 'device:=cpu')
    assert list(nodes) == ['zedx_perception']
    params = nodes['zedx_perception']['params']
    assert (params['yolo.model'], params['yolo.device']) == ('/models/best_seg.pt', 'cpu')
    assert (params['pointcloud.mode'], params['pointcloud.fresh']) == ('snapshot', True)

    nodes = launch('zedx:=false', 'with_cameras:=true', 'host:=10.1.2.3', 'depth_backend:=sgbm')
    assert sorted(nodes) == ['camera_hub', 'zedx_camera', 'zedx_depth', 'zedx_nano_camera', 'zedx_nano_depth',
                             'zedx_nano_perception']
    assert nodes['camera_hub']['params']['host'] == '10.1.2.3'
    assert nodes['zedx_nano_depth']['params']['backend'] == 'sgbm'
