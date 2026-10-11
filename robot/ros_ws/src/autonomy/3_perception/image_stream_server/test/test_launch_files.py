"""Static checks of the launch wiring (no ROS): the server's own launch file and perception_bringup's include."""
from pathlib import Path
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
BRINGUP = PACKAGE.parent / 'perception_bringup' / 'launch' / 'perception.launch.xml'


def args(root):
    return {arg.get('name'): arg.get('default') for arg in root.findall('arg')}


def test_server_launch_file_starts_the_node_with_params():
    root = ET.parse(PACKAGE / 'launch' / 'image_stream_server.launch.xml').getroot()
    assert args(root)['stream_port'] == '8002'
    node = root.find('node')
    assert (node.get('pkg'), node.get('exec'), node.get('name')) == (
        'image_stream_server', 'image_stream_server_node', 'image_stream_server')
    params = {p.get('name'): p.get('value') for p in node.findall('param') if p.get('name')}
    assert params == {'host': '$(var stream_host)', 'port': '$(var stream_port)'}


def test_perception_bringup_includes_the_server_by_default():
    root = ET.parse(BRINGUP).getroot()
    defaults = args(root)
    assert defaults['with_stream_server'] == 'true' and defaults['stream_port'] == '8002'
    includes = [i for i in root.findall('include') if 'image_stream_server.launch.xml' in i.get('file')]
    assert len(includes) == 1
    assert includes[0].get('if') == '$(var with_stream_server)'
    passed = {a.get('name'): a.get('value') for a in includes[0].findall('arg')}
    assert passed == {'stream_port': '$(var stream_port)'}


def test_config_streams_match_the_spec_defaults():
    text = (PACKAGE / 'config' / 'image_stream_server.yaml').read_text()
    for entry in ('annotated/zedx=/inspect/annotated_image', 'annotated/zedx_nano=/servo/annotated_image',
                  'raw/zedx=/zedx/left/image_rect_color', 'raw/zedx_nano=/zedx_nano/left/image_rect_color'):
        assert entry in text
