#!/usr/bin/env python3
"""Mock routine executor: drive the APEX experiment UI with no robot and no ROS.

Stands in for everything behind the MQTT broker that the experiment flow needs:

* routine_executor   — listens on cmd/routine_executor/*, publishes ros2/routine_executor/status
                       at 1 Hz. Uses the REAL RoutineStateMachine and the real step list /
                       required params from step_config.py, so start/pause/cancel/retry behave
                       like prod.
* the robot          — each dispatched step "runs" for --step-time seconds (wait steps use
                       their time_s), then succeeds or fails per --fail-step / --fail-rate.
* manipulation exec. — pick/place steps publish ros2/<robot>/behavior/manipulation_phase and
                       ros2/planning_state like the real executive.
* lab_machine exec.  — an ot2 step walks through its commands (connecting → each command →
                       homing → disconnecting) and reports them as status.step_progress, like
                       lab_machine_executive's ot2_progress; after each aspirate the tip check runs
                       pass by pass (live progress) and then attaches its result (five averaged
                       checks); --clog / --tip-check-error / --tip-check-down inject outcomes.
                       ot2 steps take --ot2-time and are never auto-retried.
* tip-check service  — fake HTTP service on :8110 (/check, /jobs, /health, /runs/*.png) for the
                       debug console's standalone check and the Clog check card's images.

See README.md in this folder for how to run it alongside the UI dev server.
"""

import argparse
import ast
import importlib.util
import json
import random
import struct
import sys
import threading
import time
import uuid
import zlib
from collections import defaultdict, deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[3]
EXECUTOR_PKG = REPO_ROOT / 'gcs' / 'ros_ws' / 'src' / 'routine_executor' / 'routine_executor'

STATUS_TOPIC = 'ros2/routine_executor/status'
CMD_PREFIX = 'cmd/routine_executor/'
PLANNING_STATE_TOPIC = 'ros2/planning_state'
MANIP_STEPS = {'pick', 'place', 'pick_base', 'place_base'}
# Fraction of a manipulation step at which each phase starts (names from manipulation_executive.cpp).
MANIP_PHASES = [(0.0, 'SELECTING_CAMERA'), (0.15, 'PLANNING'), (0.35, 'RUNNING_ACT'),
                (0.6, 'SERVOING'), (0.85, 'PLANNING_SAFE')]
FAIL_AT_FRACTION = 0.5  # an injected failure fires halfway through the step


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RoutineStateMachine = _load_module(
    'routine_executor_state_machine', EXECUTOR_PKG / 'state_machine.py').RoutineStateMachine


def load_step_specs(path=EXECUTOR_PKG / 'step_config.py'):
    """Read {step_name: {'required_params', 'msg_type'}} out of step_config.py without importing it
    (it imports ROS message types). Parsed with ast, so it never drifts from the real executor."""
    tree = ast.parse(Path(path).read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == 'STEPS' for t in node.targets):
            specs = {}
            for key, value in zip(node.value.keys, node.value.values):
                fields = {k.value: v for k, v in zip(value.keys, value.values)}
                specs[key.value] = {
                    'required_params': ast.literal_eval(fields['required_params']),
                    'msg_type': ast.literal_eval(fields['msg_type']),
                    'retryable': (ast.literal_eval(fields['retryable'])
                                  if 'retryable' in fields else True),
                }
            return specs
    raise RuntimeError(f'No STEPS dict found in {path}')


def _ros_string(data):
    """How ros2_mqtt_bridge serialises a std_msgs/String."""
    return json.dumps({'data': data})


def _log(msg):
    print(f'[mock] {msg}', flush=True)


def _png(width=64, height=48, rgb=(120, 160, 200)):
    """A small solid-colour PNG, standing in for the tip crop / L515 snapshot."""
    row = b'\x00' + bytes(rgb) * width
    raw = row * height

    def chunk(kind, data):
        return (struct.pack('>I', len(data)) + kind + data
                + struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(raw)) + chunk(b'IEND', b''))


class FakeTipCheck:
    """The d3:4b tip check. result() is what lab_machine_executive publishes after an aspirate
    (five checks averaged); check() is one POST /check for the debug console.

    clog_checks / error_checks: 1-based numbers of the aspirate checks (across the session) that
    come out as a clog / as "service unreachable". down: every check fails.
    """

    PASSES = 5
    THRESHOLD = 0.5
    MODEL = 'mock-d3:4b'

    def __init__(self, clog_checks=(), error_checks=(), down=False):
        self._clog = set(clog_checks)
        self._error = set(error_checks)
        self.down = down
        self._n = 0
        self._jobs = []
        self._lock = threading.Lock()

    def _files(self, clip):
        return {'crop': f'/runs/{clip}/crop.png', 'preview': f'/runs/{clip}/preview.png'}

    def result(self, aspirate):
        with self._lock:
            self._n += 1
            n = self._n
        if self.down or n in self._error:
            return {'status': 'error', 'error': 'mock: tip-check service unreachable at http://mock:8110',
                    'sample_count': self.PASSES, 'samples_done': 0}
        clog = n in self._clog
        samples = [round((0.72 if clog else 0.05) + 0.01 * i, 3) for i in range(self.PASSES)]
        mean = sum(samples) / self.PASSES
        clip = f'mock-tip-{n}'
        return {
            'status': 'done', 'clip_id': clip, 'model': self.MODEL, 'threshold': self.THRESHOLD,
            'aggregation': 'mean', 'sample_count': self.PASSES, 'samples': samples,
            'p_tip_empty': round(mean, 4), 'p_tip_visible': 0.95, 'anomaly_detected': clog,
            'confidence': round(mean if clog else 1 - mean, 4),
            'summary': f'Average of {self.PASSES} checks: '
                       f'{"tip looks empty" if clog else "liquid in tip"} (P(empty)={mean:.2f})',
            'warning': None, 'error': None, 'files': self._files(clip),
            'timings_s': {'total': 3.0}, 'mock': True,
        }

    def check(self, body):
        """One POST /check (finished at once). Returns None when the service is down."""
        if self.down:
            return None
        with self._lock:
            clip = f'mock-debug-{len(self._jobs) + 1}'
            job = {'clip_id': clip, 'status': 'done', 'p_tip_empty': 0.06, 'p_tip_visible': 0.95,
                   'threshold': self.THRESHOLD, 'model': self.MODEL, 'step': (body or {}).get('step'),
                   'files': self._files(clip), 'timings_s': {'total': 0.6}, 'mock': True}
            self._jobs.append(job)
            return dict(job)

    def jobs(self):
        with self._lock:
            return [dict(j) for j in self._jobs]


def start_tip_check_server(tips, host='0.0.0.0', port=8110):
    """Serve the fake tip-check API in a daemon thread. Returns the server (call .shutdown())."""
    image = _png()

    class Handler(BaseHTTPRequestHandler):
        def _send(self, code, body, content_type='application/json'):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self._cors()
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _cors(self):
            self.send_header('Access-Control-Allow-Origin', '*')
            self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
            self.send_header('Access-Control-Allow-Headers', 'Content-Type')

        def do_OPTIONS(self):
            self.send_response(204)
            self._cors()
            self.end_headers()

        def do_GET(self):
            if self.path == '/jobs':
                self._send(200, tips.jobs())
            elif self.path == '/health':
                self._send(200, {'ok': not tips.down, 'camera_present': True,
                                 'model_reachable': not tips.down, 'busy': False, 'mock': True})
            elif self.path.startswith('/runs/') and self.path.endswith('.png'):
                self._send(200, image, 'image/png')
            else:
                self._send(404, {'error': 'not found'})

        def do_POST(self):
            if self.path != '/check':
                self._send(404, {'error': 'not found'})
                return
            length = int(self.headers.get('Content-Length') or 0)
            try:
                body = json.loads(self.rfile.read(length) or b'{}')
            except json.JSONDecodeError:
                body = {}
            job = tips.check(body)
            if job is None:
                self._send(503, {'error': 'mock: model not reachable'})
            else:
                self._send(200, job)

        def log_message(self, *_args):
            pass

    server = ThreadingHTTPServer((host, port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


class MockExecutor:
    """Mirrors routine_executor/node.py, with the robot replaced by timers.

    Thread-safe entry points: handle(), force_fail(), force_success() only queue work; step()
    (called from one loop) does it. Status goes out via status_message().
    """

    def __init__(self, specs, step_time=3.0, clock=time.monotonic, tip_check=None,
                 fail_steps=None, fail_rate=0.0, rng=None, ot2_time=None):
        self._specs = specs
        self._ot2_time = step_time if ot2_time is None else ot2_time
        self._no_retry = {name for name, spec in specs.items() if not spec.get('retryable', True)}
        self._step_time = step_time
        self._clock = clock
        self._tips = tip_check if tip_check is not None else FakeTipCheck()
        self._fail_steps = dict(fail_steps or {})  # step name -> fail the first N attempts
        self._fail_rate = fail_rate
        self._rng = rng or random.Random()
        self._queue = deque()
        self._lock = threading.Lock()
        self._sm = RoutineStateMachine([], no_retry=self._no_retry)
        self._robot = 'robot_1'
        self._attempts = defaultdict(int)  # step index -> attempts dispatched this run
        self._active = None
        self._last_phase = None
        self._last_phase_t = None

    # -- inputs ---------------------------------------------------------------------------------

    def handle(self, topic, payload):
        with self._lock:
            self._queue.append(('mqtt', topic, payload))

    def force_fail(self):
        with self._lock:
            self._queue.append(('force', 'fail', None))

    def force_success(self):
        with self._lock:
            self._queue.append(('force', 'success', None))

    # -- outputs --------------------------------------------------------------------------------

    def status_message(self):
        return STATUS_TOPIC, _ros_string(json.dumps(self._sm.to_dict()))

    def status(self):
        return self._sm.to_dict()

    # -- loop -----------------------------------------------------------------------------------

    def step(self):
        """Process queued commands, advance the fake robot, dispatch. Returns [(topic, payload)]."""
        out = []
        with self._lock:
            pending = list(self._queue)
            self._queue.clear()
        for kind, a, b in pending:
            if kind == 'mqtt':
                self._on_command(a, b)
            else:
                self._finish_active(a, out)

        now = self._clock()
        if self._active is not None:
            self._advance_active(now, out)

        if self._sm.state == 'running' and self._sm.needs_dispatch:
            self._dispatch(now)
            self._advance_active(now, out)

        self._publish_phase(now, out)
        return out

    # -- commands (same rules as node.py) -------------------------------------------------------

    def _on_command(self, topic, payload):
        cmd = topic[len(CMD_PREFIX):] if topic.startswith(CMD_PREFIX) else topic
        if cmd == 'start_routine_cmd':
            self._on_start(payload)
        elif cmd == 'cancel':
            if self._sm.state in ('running', 'paused'):
                self._sm.cancel()
                self._active = None
                _log('routine cancelled')
            else:
                _log('cancel received but no routine is running')
        elif cmd == 'pause':
            if self._sm.state == 'running':
                self._sm.pause()
                _log('paused (current step keeps running, next step held)')
            else:
                _log(f'pause received but state is {self._sm.state}')
        elif cmd == 'resume':
            if self._sm.state == 'paused':
                self._sm.resume()
                _log('resumed')
            else:
                _log(f'resume received but state is {self._sm.state}')
        elif cmd == 'retry':
            try:
                force = bool(payload) and json.loads(payload).get('force') is True
            except (json.JSONDecodeError, AttributeError):
                force = False
            if self._sm.retry(force=force):
                _log(f'retrying from step {self._sm.current_step_idx + 1} '
                     f'"{self._sm.current_step_name()}"')
            elif self._sm.to_dict()['retry_needs_force']:
                _log(f'retry refused: "{self._sm.current_step_name()}" handles liquid; '
                     'send {"force": true}')
            else:
                _log(f'retry received but state is {self._sm.state}')
        else:
            _log(f'ignoring unknown command topic {topic}')

    def _on_start(self, payload):
        try:
            routine = json.loads(payload)
        except json.JSONDecodeError as e:
            _log(f'ERROR invalid JSON in start_routine_cmd: {e}')
            return
        if self._sm.state in ('running', 'paused'):
            _log(f'routine already active (state={self._sm.state}) — ignoring start command')
            return
        steps = routine.get('steps', [])
        if not steps:
            _log('ERROR start_routine_cmd received empty steps list')
            return
        unknown = [s.get('name') for s in steps if s.get('name') not in self._specs]
        if unknown:
            _log(f'ERROR unknown steps: {unknown}. Known: {list(self._specs)}')
            return
        for s in steps:
            missing = [p for p in self._specs[s['name']]['required_params'] if p not in s]
            if missing:
                _log(f'ERROR step "{s["name"]}" missing required params: {missing}')
                return
        self._robot = routine.get('robot', 'robot_1')
        self._sm = RoutineStateMachine(steps, max_retries=int(routine.get('max_retries', 3)),
                                       no_retry=self._no_retry)
        self._attempts.clear()
        self._active = None
        self._sm.start()
        _log(f'starting routine: {len(steps)} steps, robot={self._robot}, '
             f'max_retries={self._sm.max_retries}')

    # -- fake robot -----------------------------------------------------------------------------

    def _dispatch(self, now):
        idx = self._sm.current_step_idx
        step = self._sm.current_step()
        name = step['name']
        self._sm.acknowledge_dispatch()
        self._attempts[idx] += 1
        attempt = self._attempts[idx]

        if self._specs[name]['msg_type'] == 'wait':
            duration = float(step.get('time_s', 0))
        else:
            duration = self._ot2_time if name == 'ot2' else self._step_time
        fail = name != 'wait' and (attempt <= self._fail_steps.get(name, 0)
                                   or self._rng.random() < self._fail_rate)
        # (time into the step, kind, data): 'progress' -> step_progress, 'tip' -> a tip-check result
        events = _ot2_timeline(_ot2_actions(step), duration, self._tips) if name == 'ot2' else []
        finish_at = duration * FAIL_AT_FRACTION if fail else duration
        tips_at = [t for t, kind, _ in events if kind == 'tip']
        if fail and tips_at:  # an injected OT-2 failure comes after the aspirates (tip may hold liquid)
            finish_at = (tips_at[-1] + duration) / 2
        self._active = {
            'idx': idx, 'name': name, 'started': now, 'duration': duration,
            'finish_at': finish_at,
            'outcome': 'fail' if fail else 'success', 'events': events,
        }
        _log(f'step {idx + 1}/{len(self._sm.steps)} "{name}" dispatched '
             f'(attempt {attempt}, {duration:.1f}s{", WILL FAIL" if fail else ""})')

    def _advance_active(self, now, out):
        act = self._active
        if act is None:
            return
        elapsed = now - act['started']
        while act['events'] and act['events'][0][0] <= elapsed:
            self._fire(act['events'].pop(0), out)
        if act['name'] in MANIP_STEPS:
            phase = _manip_phase(elapsed / act['duration'] if act['duration'] else 1.0)
            if phase != 'PLANNING' and act.get('planned') is None and elapsed > 0:
                act['planned'] = True
                out.append((PLANNING_STATE_TOPIC, _ros_string('SUCCESS')))
        if elapsed >= act['finish_at']:
            if act['outcome'] == 'fail' and act['name'] == 'ot2':
                progress = dict(self._sm.step_progress or {})
                progress.pop('step', None)
                self._sm.set_step_progress({**progress, 'machine': 'ot2', 'phase': 'failed',
                                            'tip_check': None})
            self._finish_active(act['outcome'], out)

    def _fire(self, event, out):
        _, kind, data = event
        if kind == 'tip':
            k, action, result = data
            self._add_tip_check(k, action, result)
        elif self._sm.set_step_progress({**data, 'machine': 'ot2'}):
            out.append(self.status_message())  # like node.py: publish at once, not on the 1 Hz tick

    def _finish_active(self, outcome, out):
        act = self._active
        if act is None or self._sm.state not in ('running', 'paused'):
            _log(f'nothing running to force {outcome}')
            return
        self._active = None
        if outcome == 'success':
            for event in act['events']:  # forced early: still check the remaining aspirates
                if event[1] == 'tip':
                    self._fire(event, out)
            self._sm.on_success()
            _log(f'step "{act["name"]}" succeeded')
        else:
            if act['name'] in MANIP_STEPS:
                out.append((PLANNING_STATE_TOPIC, _ros_string('ERROR')))
            self._sm.on_failure()
            _log(f'step "{act["name"]}" FAILED (retry {self._sm.retry_count}/'
                 f'{self._sm.max_retries}; state={self._sm.state})')

    def _add_tip_check(self, protocol_step, action, result=None):
        if result is None:
            result = self._tips.result(action)
        result = {**result, 'type': 'tip_check',
                  'id': f'mock-{uuid.uuid4().hex[:8]}-{protocol_step}',
                  'protocol_step': protocol_step, 'aspirate': action}
        if self._sm.add_recording(result):
            verdict = (result.get('error') if result['status'] == 'error'
                       else 'CLOG' if result['anomaly_detected'] else 'liquid in tip')
            _log(f'tip check after aspirate ({action.get("resource_name")} '
                 f'wells {action.get("well_indices")}): {verdict}')

    def _publish_phase(self, now, out):
        act = self._active
        if act is not None and act['name'] in MANIP_STEPS:
            elapsed = now - act['started']
            phase = _manip_phase(elapsed / act['duration'] if act['duration'] else 1.0)
        else:
            phase = 'IDLE'
        if phase != self._last_phase or self._last_phase_t is None or now - self._last_phase_t >= 1.0:
            out.append((f'ros2/{self._robot}/behavior/manipulation_phase', _ros_string(phase)))
            self._last_phase = phase
            self._last_phase_t = now


def _manip_phase(fraction):
    current = MANIP_PHASES[0][1]
    for start, name in MANIP_PHASES:
        if fraction >= start:
            current = name
    return current


# Relative length of each part of an OT-2 protocol in the mock (scaled to the step's duration).
OT2_UNITS = {'connecting': 1.0, 'command': 2.0, 'tip_pass': 1.0, 'homing': 1.5, 'disconnecting': 0.5}


def _ot2_timeline(actions, duration, tips):
    """Events of one OT-2 protocol, as lab_machine_executive reports them on ot2_progress.

    The tip check after an aspirate goes pass by pass (submitting → running → done, with the
    finished passes' P(empty)); its result is attached when the last pass is done. Each check's
    outcome (clog / error) is drawn from `tips` when the step is dispatched.
    """
    passes = FakeTipCheck.PASSES
    units = OT2_UNITS['connecting'] + OT2_UNITS['homing'] + OT2_UNITS['disconnecting']
    for a in actions:
        units += OT2_UNITS['command'] + (passes * OT2_UNITS['tip_pass'] if a.get('action') == 'aspirate' else 0)
    scale = duration / units if units else 0
    run_id = uuid.uuid4().hex[:8]
    n = len(actions)

    def progress(phase, command=None, tip_check=None):
        has = command is not None and 0 <= command < n
        return {'run_id': run_id, 'phase': phase, 'command': command if has else None, 'commands': n,
                'action': actions[command].get('action') if has else None, 'tip_check': tip_check}

    events, t = [(0.0, 'progress', progress('connecting'))], OT2_UNITS['connecting'] * scale
    for k, a in enumerate(actions):
        events.append((t, 'progress', progress('command', k)))
        t += OT2_UNITS['command'] * scale
        if a.get('action') != 'aspirate':
            continue
        result = tips.result(a)
        samples = result.get('samples') or []
        if result.get('status') == 'error':  # the service is down: the first pass fails at once
            events.append((t, 'progress', progress('tip_check', k, {
                'pass': 1, 'passes': passes, 'status': 'submitting', 'stage': None,
                'queue_position': None, 'samples': []})))
            t += OT2_UNITS['tip_pass'] * scale
            events.append((t, 'tip', (k, a, result)))
            continue
        for i in range(passes):
            base = {'pass': i + 1, 'passes': passes, 'queue_position': None}
            events.append((t, 'progress', progress('tip_check', k, {
                **base, 'status': 'submitting', 'stage': None, 'samples': samples[:i]})))
            events.append((t + 0.2 * OT2_UNITS['tip_pass'] * scale, 'progress', progress('tip_check', k, {
                **base, 'status': 'running', 'stage': 'analyzing', 'samples': samples[:i]})))
            events.append((t + 0.8 * OT2_UNITS['tip_pass'] * scale, 'progress', progress('tip_check', k, {
                **base, 'status': 'done', 'stage': None, 'samples': samples[:i + 1]})))
            t += OT2_UNITS['tip_pass'] * scale
        events.append((t, 'tip', (k, a, result)))
    events.append((t, 'progress', progress('homing')))
    t += OT2_UNITS['homing'] * scale
    events.append((t, 'progress', progress('disconnecting')))
    # 'done' is not sent: the step's success moves the routine on (and clears step_progress)
    return events


def _ot2_actions(step):
    params = step.get('parameters_json', {})
    if isinstance(params, str):
        try:
            params = json.loads(params)
        except json.JSONDecodeError:
            return []
    return params.get('steps', []) if isinstance(params, dict) else []


# ----------------------------------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------------------------------

def _parse_fail_steps(values):
    out = {}
    for v in values or []:
        name, _, n = v.partition(':')
        out[name] = int(n) if n else 10 ** 6  # no count: every attempt fails, UI retries included
    return out


def _make_mqtt_client(transport):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        sys.exit('Install paho-mqtt first:  pip3 install paho-mqtt')
    client_id = f'mock-executor-{uuid.uuid4().hex[:6]}'
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id, transport=transport)
    except AttributeError:  # paho-mqtt < 2.0
        return mqtt.Client(client_id=client_id, transport=transport)


def _stdin_loop(ex, stop):
    help_text = 'keys: f=fail current step  s=succeed current step  p=print status  q=quit'
    _log(help_text)
    for line in sys.stdin:
        key = line.strip().lower()
        if key == 'f':
            ex.force_fail()
        elif key == 's':
            ex.force_success()
        elif key == 'p':
            _log(json.dumps(ex.status(), indent=2))
        elif key == 'q':
            stop.set()
            return
        elif key:
            _log(help_text)
    # stdin closed (backgrounded / piped): keep running, Ctrl-C to stop


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--broker-host', default='localhost')
    p.add_argument('--broker-port', type=int, default=9001,
                   help='9001 = WebSocket listener (same as the UI); use 1883 with --transport tcp')
    p.add_argument('--transport', choices=['websockets', 'tcp'], default='websockets')
    p.add_argument('--step-time', type=float, default=3.0,
                   help='seconds each non-wait, non-ot2 step takes (default 3)')
    p.add_argument('--ot2-time', type=float, default=20.0,
                   help='seconds an ot2 step takes, tip checks included (default 20)')
    p.add_argument('--fail-step', action='append', metavar='NAME[:N]',
                   help='fail the first N attempts of every NAME step (no N = always fail). '
                        'max_retries counts re-dispatches after the first attempt, so '
                        '--fail-step pick:4 exhausts max_retries=3, then a UI retry succeeds')
    p.add_argument('--fail-rate', type=float, default=0.0, help='random failure probability per attempt')
    p.add_argument('--seed', type=int, help='seed for --fail-rate')
    p.add_argument('--clog', type=int, action='append', default=[], metavar='N',
                   help='the Nth tip check of the session (one per aspirate) is a clog (repeatable)')
    p.add_argument('--tip-check-error', type=int, action='append', default=[], metavar='N',
                   help='the Nth tip check reports the service unreachable (repeatable)')
    p.add_argument('--tip-check-down', action='store_true',
                   help='every tip check fails, and the fake :8110 answers /check with 503')
    p.add_argument('--tip-check-port', type=int, default=8110)
    p.add_argument('--no-tip-check-server', action='store_true',
                   help="don't start the fake :8110 (debug check and card images unreachable)")
    args = p.parse_args()

    tips = FakeTipCheck(clog_checks=args.clog, error_checks=args.tip_check_error,
                        down=args.tip_check_down)
    if not args.no_tip_check_server:
        start_tip_check_server(tips, port=args.tip_check_port)
        _log(f'fake tip-check service on http://0.0.0.0:{args.tip_check_port}')

    ex = MockExecutor(load_step_specs(), step_time=args.step_time, ot2_time=args.ot2_time, tip_check=tips,
                      fail_steps=_parse_fail_steps(args.fail_step), fail_rate=args.fail_rate,
                      rng=random.Random(args.seed))

    client = _make_mqtt_client(args.transport)

    def on_connect(c, *_):
        c.subscribe(CMD_PREFIX + '#')
        _log(f'connected to {args.broker_host}:{args.broker_port} ({args.transport}); '
             f'listening on {CMD_PREFIX}#')

    client.on_connect = on_connect
    client.on_message = lambda _c, _u, msg: ex.handle(msg.topic, msg.payload.decode())
    try:
        client.connect(args.broker_host, args.broker_port, keepalive=30)
    except OSError as e:
        sys.exit(f'Cannot reach MQTT broker at {args.broker_host}:{args.broker_port}: {e}\n'
                 'Start one first (see README.md).')
    client.loop_start()

    stop = threading.Event()
    threading.Thread(target=_stdin_loop, args=(ex, stop), daemon=True).start()
    next_status = 0.0
    try:
        while not stop.is_set():
            for topic, payload in ex.step():
                client.publish(topic, payload)
            now = time.monotonic()
            if now >= next_status:
                client.publish(*ex.status_message())
                next_status = now + 1.0
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == '__main__':
    main()
