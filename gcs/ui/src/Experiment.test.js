import { render, screen, act, within, fireEvent } from '@testing-library/react';
import mqtt from 'mqtt';
import Experiment from './Experiment';
import { PRESETS } from './shared/presets';

jest.mock('mqtt');

let handlers;

beforeEach(() => {
  handlers = {};
  mqtt.connect.mockImplementation(() => ({
    on: jest.fn((event, cb) => { (handlers[event] ||= []).push(cb); }),
    publish: jest.fn(), subscribe: jest.fn(), unsubscribe: jest.fn(), end: jest.fn(),
  }));
  global.fetch = jest.fn(() => new Promise(() => {}));
});

afterEach(() => { delete global.fetch; });

const steps = PRESETS[1].routine.steps;
const OT2 = steps.findIndex((s) => s.name === 'ot2');

function sendStatus(fields) {
  const payload = Buffer.from(JSON.stringify({ data: JSON.stringify({
    state: 'running', steps, current_step: OT2, current_step_name: 'ot2', retry_count: 0, max_retries: 3,
    recordings: [], ...fields,
  }) }));
  act(() => { (handlers.message || []).forEach((cb) => cb('ros2/routine_executor/status', payload)); });
}

const progress = (fields) => ({ machine: 'ot2', step: OT2, run_id: 'run1', commands: 4, tip_check: null, ...fields });

test('the run page follows the OT-2 command by command', () => {
  render(<Experiment />);
  sendStatus({ step_progress: progress({ phase: 'command', command: 0, action: 'pick_up_tips' }) });
  const panel = screen.getByLabelText('OT-2 protocol progress');
  expect(within(panel).getByText('Command 1 of 4')).toBeInTheDocument();

  sendStatus({ step_progress: progress({ phase: 'command', command: 2, action: 'dispense' }) });
  expect(within(panel).getByText('Command 3 of 4')).toBeInTheDocument();
  // the timeline's current step names the running command
  expect(within(screen.getByLabelText('Run progress')).getByText('Dispense')).toBeInTheDocument();
});

test('the clog check shows its live status, then its result', () => {
  render(<Experiment />);
  sendStatus({ step_progress: progress({
    phase: 'tip_check', command: 1, action: 'aspirate',
    tip_check: { pass: 2, passes: 5, status: 'running', samples: [0.71] },
  }) });
  expect(screen.getByTestId('pending-check')).toHaveTextContent('Pass 2 of 5 · analyzing');

  sendStatus({
    step_progress: progress({ phase: 'command', command: 2, action: 'dispense' }),
    recordings: [{ type: 'tip_check', id: 'run1-1', step: OT2, status: 'done', anomaly_detected: true,
      p_tip_empty: 0.72, threshold: 0.5, aspirate: steps[OT2].parameters_json.steps[1] }],
  });
  expect(screen.queryByTestId('pending-check')).not.toBeInTheDocument();
  expect(screen.getByText('Clog suspected · P(empty) 0.72')).toBeInTheDocument();
});

test('progress of an earlier step is not shown', () => {
  render(<Experiment />);
  sendStatus({ step_progress: progress({ step: OT2 - 1, phase: 'homing' }) });
  expect(within(screen.getByLabelText('OT-2 protocol progress')).getByText('Waiting for the OT-2…')).toBeInTheDocument();
});

// Each client gets its own handlers, so a stale client's events can be told apart.
function perClientMqtt() {
  const clients = [];
  mqtt.connect.mockImplementation(() => {
    const h = {};
    const c = {
      h, on: jest.fn((event, cb) => { (h[event] ||= []).push(cb); }),
      publish: jest.fn(), subscribe: jest.fn(), unsubscribe: jest.fn(), end: jest.fn(),
    };
    clients.push(c);
    return c;
  });
  return clients;
}
const fire = (c, event, ...args) => act(() => { (c.h[event] || []).forEach((cb) => cb(...args)); });
// The page's own clients (CameraStage has one more): connect them all, keep those that subscribe to status.
function runClients(clients) {
  clients.forEach((c) => fire(c, 'connect'));
  return clients.filter((c) => c.subscribe.mock.calls.some(([t]) => t === 'ros2/routine_executor/status'));
}

test('a client closed by a remount does not flip the badge to disconnected', () => {
  const clients = perClientMqtt();
  const { unmount } = render(<Experiment />);
  unmount();
  render(<Experiment />);
  const [stale] = runClients(clients);
  fire(stale, 'close'); // the ended client's socket closes after the new one connected
  expect(screen.getByText('Connected')).toBeInTheDocument();
});

test('a status message marks the page connected', () => {
  render(<Experiment />);
  sendStatus({});
  expect(screen.getByText('Connected')).toBeInTheDocument();
});

test('a failed step can be skipped after confirming', () => {
  const clients = perClientMqtt();
  render(<Experiment />);
  const [c] = runClients(clients);
  const payload = Buffer.from(JSON.stringify({ data: JSON.stringify({
    state: 'failed', steps, current_step: 1, error: 'boom', retry_count: 0, max_retries: 3, recordings: [],
  }) }));
  fire(c, 'message', 'ros2/routine_executor/status', payload);
  fireEvent.click(screen.getByRole('button', { name: /skip step 2/i }));
  expect(c.publish).not.toHaveBeenCalledWith('cmd/routine_executor/skip', '');
  fireEvent.click(screen.getByRole('button', { name: /confirm skip/i }));
  expect(c.publish).toHaveBeenCalledWith('cmd/routine_executor/skip', '');
});

test('skipped steps are marked on the timeline and in the result', () => {
  render(<Experiment />);
  sendStatus({ state: 'success', current_step: steps.length, skipped_steps: [1] });
  const timeline = screen.getByLabelText('Run progress');
  expect(within(timeline).getAllByText('Skipped')).toHaveLength(1);
  expect(screen.getByText(/1 step was skipped/i)).toBeInTheDocument();
});
