"""The Camera-Edge control protocol that Autolab's manipulation_executive speaks (port 8765)."""
import asyncio
import json
import sys
import types

import pytest

from visual_servoing.edge_control import EdgeControlServer, EdgeModes


class FakeRuns:
    """start/abort/status of a ServoRunner, finished by hand."""

    def __init__(self):
        self.t = 0.0
        self.starts, self.aborts, self.callbacks = 0, 0, []
        self.refuse = False

    def start(self, on_done):
        if self.refuse:
            return False
        self.starts += 1
        self.callbacks.append(on_done)
        return True

    def abort(self):
        self.aborts += 1

    def status(self):
        return {'phase': 'servoing', 'result': None}

    def modes(self):
        return EdgeModes(self.start, self.abort, self.status, min_servo_visible=1.5, clock=lambda: self.t)


def test_servo_mode_returns_to_idle_when_the_run_ends_but_is_seen_first():
    runs = FakeRuns()
    modes = runs.modes()
    assert modes.handle(json.dumps({'cmd': 'mode', 'mode': 'servo'})) == {'ok': True, 'mode': 'servo'}
    assert runs.starts == 1
    runs.callbacks[0]({'result': 'grasped'})          # a run that ends at once...
    runs.t = 0.4
    assert modes.mode == 'servo'                        # ...still shows "servo" to the executive's first poll
    runs.t = 1.6
    status = modes.handle('{"cmd": "status"}')
    assert status['ok'] and status['mode'] == 'idle' and status['worker_alive'] is True
    assert status['servo'] == {'phase': 'servoing', 'result': None} and status['record_available'] is False


def test_idle_and_stop_abort_a_running_servo():
    runs = FakeRuns()
    modes = runs.modes()
    modes.switch('servo')
    assert modes.handle('{"cmd": "stop"}') == {'ok': True, 'mode': 'idle'} and runs.aborts == 1
    modes.switch('servo')
    assert modes.handle('{"cmd": "mode", "mode": "idle"}')['mode'] == 'idle' and runs.aborts == 2
    assert modes.switch('idle') == 'idle' and runs.aborts == 2            # nothing to abort


def test_a_late_finish_does_not_end_a_newer_run():
    runs = FakeRuns()
    modes = runs.modes()
    modes.switch('servo')
    modes.switch('idle')
    modes.switch('servo')
    runs.callbacks[0]({'result': 'aborted'})           # the first run's thread ends after the second started
    runs.t = 10.0
    assert modes.mode == 'servo'
    runs.callbacks[1]({'result': 'grasped'})
    assert modes.mode == 'idle'


def test_inspect_record_and_errors():
    runs = FakeRuns()
    modes = runs.modes()
    assert modes.handle('{"cmd": "mode", "mode": "inspect"}') == {'ok': True, 'mode': 'inspect'}
    answer = modes.handle('{"cmd": "mode", "mode": "record"}')
    assert not answer['ok'] and 'record mode unavailable' in answer['error'] and modes.mode == 'inspect'
    assert not modes.handle('{"cmd": "mode", "mode": "dance"}')['ok']
    assert modes.handle('not json') == {'ok': False, 'error': 'invalid JSON'}
    assert modes.handle('[1]') == {'ok': False, 'error': 'expected a JSON object'}
    assert modes.handle('{"cmd": "fly"}') == {'ok': False, 'error': 'unknown cmd: fly'}
    assert modes.handle('{"cmd": "time"}')['t_ns'] > 0
    runs.refuse = True                                  # the previous run's thread hasn't finished yet
    answer = modes.handle('{"cmd": "mode", "mode": "servo"}')
    assert not answer['ok'] and 'still finishing' in answer['error'] and modes.mode == 'inspect'


def test_websocket_server():
    websockets = pytest.importorskip('websockets')
    runs = FakeRuns()
    server = EdgeControlServer(runs.modes(), host='127.0.0.1', port=0)
    server.start()
    try:
        async def session():
            async with websockets.connect(f'ws://127.0.0.1:{server.port}/') as ws:
                greeting = json.loads(await ws.recv())
                answers = []
                for request in ({'cmd': 'mode', 'mode': 'servo'}, {'cmd': 'status'}, {'cmd': 'stop'}):
                    await ws.send(json.dumps(request))
                    answers.append(json.loads(await ws.recv()))
                return greeting, answers
        greeting, (mode, status, stop) = asyncio.run(session())
    finally:
        server.stop()
    assert greeting == {'ok': True, 'message': 'connected'}
    assert mode == {'ok': True, 'mode': 'servo'} and status['mode'] == 'servo' and stop['mode'] == 'idle'
    assert runs.starts == 1 and runs.aborts == 1


def test_a_taken_port_is_reported():
    runs = FakeRuns()
    first = EdgeControlServer(runs.modes(), host='127.0.0.1', port=0)
    first.start()
    try:
        with pytest.raises(RuntimeError, match='control port'):
            EdgeControlServer(runs.modes(), host='127.0.0.1', port=first.port).start()
    finally:
        first.stop()


def test_the_server_is_built_inside_the_running_loop(monkeypatch):
    """websockets >= 14 (the Humble container) takes asyncio.get_running_loop() in serve() itself, so serve()
    called from outside the loop -- all the older API needed -- dies with "no running event loop"."""

    class StrictServer:
        def __init__(self, handler, host, port):
            asyncio.get_running_loop()                  # what websockets >= 14 does while building the server
            self.sockets = [types.SimpleNamespace(getsockname=lambda: (host, port or 4242))]
            self.closed = False

        async def __aenter__(self):
            return self

        async def __aexit__(self, *exc_info):
            self.closed = True

    stub = types.ModuleType('websockets')
    stub.serve = StrictServer
    stub.exceptions = types.SimpleNamespace(ConnectionClosed=ConnectionError)
    monkeypatch.setitem(sys.modules, 'websockets', stub)

    runs = FakeRuns()
    server = EdgeControlServer(runs.modes(), host='127.0.0.1', port=0)
    server.start()
    assert server.port == 4242
    served = server._server
    server.stop()
    assert served.closed and not server._thread.is_alive()
