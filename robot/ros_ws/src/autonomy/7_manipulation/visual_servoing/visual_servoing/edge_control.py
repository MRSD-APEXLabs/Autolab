"""The Camera-Edge control WebSocket (port 8765), so Autolab's manipulation_executive drives this servo unchanged.

Protocol (Camera-Edge main.py control_handler): on connect the server sends {"ok": true, "message": "connected"};
each request gets one JSON answer.

  {"cmd": "mode", "mode": "servo"}    start a grasp run            -> {"ok": true, "mode": "servo"}
  {"cmd": "mode", "mode": "inspect"}  accepted, nothing to start: camera_perception runs all the time
  {"cmd": "mode", "mode": "idle"}     abort a run                  -> {"ok": true, "mode": "idle"}
  {"cmd": "stop"}                     same as idle
  {"cmd": "status"}                   {"ok": true, "mode": ..., "worker_alive": true, "servo": {...}}
  {"cmd": "time"}                     {"ok": true, "t_ns": ...}

As on the Xavier, "mode" returns to "idle" when a servo run ends, whether or not it grasped; the run's
outcome is in status "servo" -> "result". The executive polls status until the mode leaves "servo".
"""
from __future__ import annotations

import asyncio
import json
import logging
import threading
import time

log = logging.getLogger(__name__)

MODES = ('idle', 'servo', 'inspect', 'record')


class EdgeModes:
    """The mode bookkeeping, separate from the socket for the tests."""

    def __init__(self, start_servo, abort_servo, servo_status, min_servo_visible=1.5, clock=time.monotonic):
        self._start, self._abort, self._status = start_servo, abort_servo, servo_status
        # the executive polls every 0.5 s and needs to see "servo" once before it waits for "idle"
        self.min_servo_visible, self._clock = float(min_servo_visible), clock
        self._lock = threading.Lock()
        self._run = 0                 # counts servo starts, so a late finish can't end a newer run
        self._started = 0.0
        self._finished_run = None
        self._mode = 'idle'

    @property
    def mode(self):
        with self._lock:
            return self._current()

    def _current(self):
        if (self._mode == 'servo' and self._finished_run == self._run
                and self._clock() - self._started >= self.min_servo_visible):
            self._mode = 'idle'
        return self._mode

    def servo_finished(self, run):
        with self._lock:
            self._finished_run = run

    def switch(self, mode):
        mode = str(mode).lower().strip()
        if mode not in MODES:
            raise ValueError(f'Unsupported mode: {mode}')
        if mode == 'record':
            raise ValueError('record mode unavailable: recording runs in teleop/ on this machine')
        with self._lock:
            current = self._current()
            if mode == current:
                return current
            if current == 'servo':
                self._abort()
            if mode == 'servo':
                run = self._run + 1
                if not self._start(lambda _status: self.servo_finished(run)):
                    raise ValueError('a servo run is still finishing')
                self._run, self._started = run, self._clock()
            self._mode = mode
            return mode

    def stop(self):
        return self.switch('idle')

    def status(self):
        mode = self.mode
        return {'mode': mode, 'worker_alive': True, 'wrist_camera_connected': True,
                'base_camera_connected': True, 'record_available': False, 'servo': self._status()}

    def handle(self, text):
        """One request (JSON text) -> the answer dict."""
        try:
            data = json.loads(text)
        except (json.JSONDecodeError, TypeError):
            return {'ok': False, 'error': 'invalid JSON'}
        if not isinstance(data, dict):
            return {'ok': False, 'error': 'expected a JSON object'}
        cmd = str(data.get('cmd', '')).lower()
        try:
            if cmd == 'mode':
                return {'ok': True, 'mode': self.switch(data.get('mode', ''))}
            if cmd == 'stop':
                return {'ok': True, 'mode': self.stop()}
            if cmd == 'status':
                return {'ok': True, **self.status()}
            if cmd == 'time':
                return {'ok': True, 't_ns': time.time_ns()}
            return {'ok': False, 'error': f'unknown cmd: {cmd}'}
        except Exception as exc:
            return {'ok': False, 'error': str(exc)}


class EdgeControlServer:
    """Serves EdgeModes on ws://host:port/ from a background thread with its own asyncio loop."""

    def __init__(self, modes: EdgeModes, host='0.0.0.0', port=8765):
        self.modes, self.host, self.port = modes, host, int(port)
        self._loop = None
        self._server = None
        self._thread = None
        self._stop = None
        self._ready = threading.Event()
        self._error = None

    async def _handler(self, ws, path=None):
        import websockets
        await ws.send(json.dumps({'ok': True, 'message': 'connected'}))
        try:
            async for message in ws:
                await ws.send(json.dumps(self.modes.handle(message)))
        except websockets.exceptions.ConnectionClosed:
            pass

    def _serve(self):
        """The loop thread. websockets >= 14 (Humble's container) builds its server object with
        asyncio.get_running_loop(), so serve() has to be called inside the loop and not handed to
        run_until_complete() from outside it, which is all the older API needed."""
        import websockets
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._stop = asyncio.Event()          # 3.10+: an Event takes its loop when it is first awaited
        try:
            self._loop.run_until_complete(self._serve_until_stopped(websockets))
        except RuntimeError as exc:           # stop() arrived before the loop reached _stop.wait()
            log.debug('control loop ended early: %s', exc)
        finally:
            self._server = None
            asyncio.set_event_loop(None)
            self._loop.close()

    async def _serve_until_stopped(self, websockets):
        try:
            # `async with` is the one spelling both websockets generations serve: the old Serve object and
            # the new Server are both asynchronous context managers, and both close the socket on the way out.
            async with websockets.serve(self._handler, self.host, self.port) as server:
                self._server = server
                self.port = server.sockets[0].getsockname()[1]   # port 0: the one picked
                self._ready.set()
                await self._stop.wait()
        except Exception as exc:
            self._error = exc
        finally:
            self._ready.set()

    def start(self, timeout=5.0):
        self._thread = threading.Thread(target=self._serve, name='edge_control', daemon=True)
        self._thread.start()
        self._ready.wait(timeout)
        if self._error is not None:
            raise RuntimeError(f'control port {self.host}:{self.port}: {self._error}') from self._error
        log.info('Camera-Edge control protocol on ws://%s:%d/', self.host, self.port)

    def stop(self):
        loop, stop = self._loop, self._stop
        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(stop.set if stop is not None else loop.stop)
        if self._thread is not None:
            self._thread.join(5.0)
