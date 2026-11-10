from pathlib import Path
from concurrent.futures import Future
from types import SimpleNamespace as NS
import xml.etree.ElementTree as ET
import pytest
import yaml
from action_msgs.msg import GoalStatus
from swerve_navigation.station_docking import StationDocking

BRINGUP = Path(__file__).resolve().parents[2] / 'navigation_bringup'


@pytest.mark.parametrize('leg', ['staging', 'align', 'stage_adjust', 'dock', 'undock'])
@pytest.mark.parametrize('success', [True, False])
def test_humble_empty_result_uses_action_status_without_error_code(leg, success):
    node = StationDocking.__new__(StationDocking)
    node.seq, node.handle = 1, None
    events = []
    node._settle = lambda callback: events.append('settle')
    node.cancel = lambda reason: events.append('stop')
    node.node = NS(_robot_place=lambda: None)
    node._recover = lambda *args: events.append('recover')
    result = Future()
    result.set_result(NS(status=GoalStatus.STATUS_SUCCEEDED if success else GoalStatus.STATUS_ABORTED,
                         result=NS()))
    node._result(1, leg, result)
    expected = 'settle' if success else 'recover' if leg in ('staging', 'stage_adjust', 'dock') else 'stop'
    assert events == [expected]


def test_humble_overrides_preserve_frames_types_and_dock_clearance():
    config = yaml.safe_load((BRINGUP/'config/nav2_humble_overlay.yaml').read_text())
    assert config['behavior_server']['ros__parameters']['global_frame'] == 'odom'
    for name in ('PolygonStop', 'PolygonSlow'):
        polygon = config['collision_monitor']['ros__parameters'][name]
        assert isinstance(polygon['points'], list) and polygon['max_points'] == 3
    fast = yaml.safe_load((BRINGUP/'config/nav2_fast_humble_overlay.yaml').read_text())
    polygons = fast['collision_monitor']['ros__parameters']
    assert polygons['PolygonSlow']['type'] == 'polygon'
    assert polygons['PolygonSlow']['points'] == config['collision_monitor']['ros__parameters']['PolygonSlow']['points']
    assert polygons['FastApproach']['time_before_collision'] * (.12**2+.06**2)**.5 < .08
    root = ET.parse(BRINGUP/'behavior_trees/navigate_to_station_staging_humble.xml').getroot()
    assert 'BTCPP_format' not in root.attrib
    assert all('error_code_id' not in element.attrib for element in root.iter())
