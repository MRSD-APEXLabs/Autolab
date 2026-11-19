import json
from unittest.mock import Mock

import pytest

from gcs_monitoring.ot2_parts import Ot2PartsRunner


def test_status_query_never_starts_hardware(tmp_path, monkeypatch):
    publish = Mock()
    runner = Ot2PartsRunner(publish, tmp_path / 'state.json')
    popen = Mock()
    monkeypatch.setattr('gcs_monitoring.ot2_parts.subprocess.Popen', popen)
    runner.relay(b'{"part":"status"}')
    assert publish.call_args.args[0]['status'] == 'idle'
    popen.assert_not_called()


@pytest.mark.parametrize('body', [
    {'part': 2}, {'part': 1, 'tip_index': -1},
    {'part': 1, 'tip_index': 96}, {'part': 1, 'tip_index': 2, 'volume': 201},
    {'part': 1, 'tip_index': 2, 'volume': float('nan')},
])
def test_bad_or_out_of_order_commands_do_not_run(tmp_path, monkeypatch, body):
    publish = Mock()
    thread = Mock()
    monkeypatch.setattr('gcs_monitoring.ot2_parts.threading.Thread', thread)
    runner = Ot2PartsRunner(publish, tmp_path / 'state.json')
    runner.relay(json.dumps(body).encode())
    thread.assert_not_called()
    assert publish.call_args.args[0]['error']


def test_duplicate_click_cannot_start_another_worker(tmp_path, monkeypatch):
    thread = Mock()
    monkeypatch.setattr('gcs_monitoring.ot2_parts.threading.Thread', thread)
    runner = Ot2PartsRunner(Mock(), tmp_path / 'state.json')
    runner.relay(b'{"part":1,"tip_index":2}')
    runner.relay(b'{"part":1,"tip_index":2}')
    assert thread.call_count == 1
    assert runner.busy


def test_part1_blocked_while_holding_liquid_and_part2_allowed(tmp_path, monkeypatch):
    state = tmp_path / 'state.json'
    state.write_text('{"stage":"ready_for_part2","volume_ul":200}')
    thread = Mock()
    monkeypatch.setattr('gcs_monitoring.ot2_parts.threading.Thread', thread)
    runner = Ot2PartsRunner(Mock(), state)
    runner.relay(b'{"part":1,"tip_index":2}')
    thread.assert_not_called()
    runner.relay(b'{"part":2}')
    assert thread.call_args.kwargs['args'][0] == 2


def test_restart_does_not_repeat_uncertain_aspiration(tmp_path, monkeypatch):
    state = tmp_path / 'state.json'
    state.write_text('{"stage":"aspiration_submitted_do_not_repeat"}')
    thread = Mock()
    monkeypatch.setattr('gcs_monitoring.ot2_parts.threading.Thread', thread)
    runner = Ot2PartsRunner(Mock(), state)
    assert runner.snapshot()['status'] == 'needs_inspection'
    runner.relay(b'{"part":1,"tip_index":2}')
    runner.relay(b'{"part":2}')
    thread.assert_not_called()


@pytest.mark.parametrize('source_tube,expected_x', [('A1', 13.88), ('A2', 38.88), ('A3', 63.88), ('A4', 88.88)])
def test_both_script_sequences_without_hardware(tmp_path, monkeypatch, source_tube, expected_x):
    from gcs_monitoring import ot2_part1 as part1, ot2_part2 as part2
    calls = []
    connected = True

    def request(host, base, path, body=None):
        nonlocal connected
        calls.append(('http', path, body))
        if path == '/robot/status':
            return {'connected': connected, 'host': '192.168.10.17'}
        if path == '/robot/connect':
            connected = True
        if path == '/debug/ot2-tip-state':
            return {'0': {'has_tip': False}}
        if path.startswith('/debug/ot2-commands'):
            return {'run_id': 'test-run', 'commands': [{
                'type': 'pickUpTip', 'status': 'succeeded', 'createdAt': '2026',
                'params': {'pipetteId': 'test-pipette'}}]}
        return {'status': 'ok'}

    def command(host, base, run, kind, params):
        calls.append(('command', kind, params))
        return {'id': str(len(calls)), 'completedAt': 'test', 'status': 'succeeded'}

    for module in (part1, part2):
        monkeypatch.setattr(module, 'request', request)
        monkeypatch.setattr(module, 'robot_command', command)
    state = tmp_path / 'state.json'
    monkeypatch.setattr('sys.argv', ['part1', '--state', str(state), '--tip-index', '2', '--source-tube', source_tube])
    part1.main()
    moves = [c for c in calls if c[1] == 'moveToCoordinates']
    assert moves[0][2]['coordinates'] == {'x': expected_x, 'y': 339.24, 'z': 7.85}
    assert json.loads(state.read_text())['source_tube'] == source_tube
    assert calls[-1][1] == 'home'
    assert calls[-1][2] == {'axes': ['leftZ']}
    assert json.loads(state.read_text())['position'] == f'fully_raised_above_tube_rack_{source_tube}'
    assert not any(c[1] == 'dispenseInPlace' for c in calls)
    assert json.loads(state.read_text())['stage'] == 'ready_for_part2'
    calls.clear()
    monkeypatch.setattr('sys.argv', ['part2', '--state', str(state)])
    part2.main()
    assert [c[1] for c in calls if c[0] == 'command'] == [
        'moveToCoordinates', 'dispenseInPlace', 'home']
    assert json.loads(state.read_text())['stage'] == 'completed'
    assert any(c[1] == '/commands/drop-tips' for c in calls)
    assert not any(c[1] in ('/robot/home', '/robot/disconnect') for c in calls)


@pytest.mark.parametrize('source_tube', ['A2', 'A3', 'A4'])
def test_selected_source_is_passed_to_worker(tmp_path, monkeypatch, source_tube):
    thread = Mock()
    monkeypatch.setattr('gcs_monitoring.ot2_parts.threading.Thread', thread)
    runner = Ot2PartsRunner(Mock(), tmp_path / 'state.json')
    runner.relay(json.dumps({'part': 1, 'tip_index': 2, 'source_tube': source_tube}).encode())
    assert thread.call_args.kwargs['args'] == (1, 2, 200, source_tube)


def test_invalid_source_does_not_start_worker(tmp_path, monkeypatch):
    thread = Mock()
    monkeypatch.setattr('gcs_monitoring.ot2_parts.threading.Thread', thread)
    runner = Ot2PartsRunner(Mock(), tmp_path / 'state.json')
    runner.relay(b'{"part":1,"tip_index":2,"source_tube":"A5"}')
    thread.assert_not_called()
