import json
from unittest.mock import Mock
import pytest
from gcs_monitoring.ot2_parts import Ot2PartsRunner
from gcs_monitoring import ot2_part1, ot2_cleanup


@pytest.mark.parametrize('connected', [True, False])
def test_cleanup_homes_once_and_discards_without_disconnect(tmp_path, monkeypatch, connected):
    calls=[]
    def request(host, base, path, body=None):
        calls.append(path)
        return {'connected': connected} if path == '/robot/status' else {'status':'ok'}
    monkeypatch.setattr(ot2_cleanup, 'request', request)
    state=tmp_path/'state.json'
    state.write_text('{"stage":"ready_for_part2"}')
    monkeypatch.setattr('sys.argv', ['cleanup', '--state', str(state)])
    ot2_cleanup.main()
    assert calls == ['/robot/status', '/robot/home' if connected else '/robot/connect', '/commands/drop-tips']
    assert json.loads(state.read_text())['stage']=='prepared'
    assert Ot2PartsRunner(Mock(), state).snapshot()['status']=='idle'


def test_cleanup_can_be_requested_while_a_tip_is_held(tmp_path, monkeypatch):
    state=tmp_path/'state.json';state.write_text('{"stage":"ready_for_part2"}')
    thread=Mock();monkeypatch.setattr('gcs_monitoring.ot2_parts.threading.Thread', thread)
    runner=Ot2PartsRunner(Mock(), state)
    runner.relay(b'{"part":"cleanup"}')
    assert thread.call_args.kwargs['args'][0]=='cleanup'


def test_part1_refuses_existing_tip_without_homing_or_discarding(tmp_path, monkeypatch):
    calls=[]
    def request(host,base,path,body=None):
        calls.append(path)
        if path=='/robot/status': return {'connected':True,'host':'192.168.10.17'}
        if path=='/debug/ot2-tip-state': return {'0':{'has_tip':True}}
        raise AssertionError('Unexpected hardware request: '+path)
    monkeypatch.setattr(ot2_part1,'request',request)
    monkeypatch.setattr('sys.argv',['part1','--state',str(tmp_path/'state.json'),'--tip-index','2'])
    with pytest.raises(SystemExit,match='tip is already mounted'):
        ot2_part1.main()
    assert calls==['/robot/status','/debug/ot2-tip-state']


def test_command_waits_on_robot_without_polling_or_resubmitting(monkeypatch):
    request=Mock(return_value={'data':{'id':'cmd','status':'succeeded'}})
    monkeypatch.setattr(ot2_part1,'request',request)
    assert ot2_part1.robot_command('', 'http://robot','run','home',{})['status']=='succeeded'
    assert request.call_count==1
    assert 'waitUntilComplete=true' in request.call_args.args[2]
    request.reset_mock()
    request.return_value={'data':{'id':'cmd','status':'running'}}
    with pytest.raises(TimeoutError):
        ot2_part1.robot_command('', 'http://robot','run','home',{})
    assert request.call_count==1


def test_worker_preserves_real_robot_error_instead_of_generic_last_line(tmp_path, monkeypatch):
    state=tmp_path/'state.json'
    state.write_text(json.dumps({'stage':'needs_inspection','error':'HTTP 500: real robot error'}))
    runner=Ot2PartsRunner(Mock(),state)
    process=Mock()
    process.stdout=['Do not repeat dispensing blindly.\n']
    process.wait.return_value=1
    from unittest.mock import MagicMock
    popen=MagicMock();popen.return_value.__enter__.return_value=process
    monkeypatch.setattr('gcs_monitoring.ot2_parts.subprocess.Popen',popen)
    runner.lock.acquire();runner.busy=True
    runner._run(2,None,200,'A1')
    assert json.loads(state.read_text())['error']=='HTTP 500: real robot error'
