import { render, screen, fireEvent, act } from '@testing-library/react';
import mqtt from 'mqtt';
import App from './App';

jest.mock('mqtt');

let handlers;
let clients;

beforeEach(() => {
  window.sessionStorage.clear();
  handlers = {};
  clients = [];
  mqtt.connect.mockImplementation(() => clients[clients.push({
    on: jest.fn((event, cb) => { (handlers[event] ||= []).push(cb); }),
    publish: jest.fn((_topic, _payload, _opts, cb) => cb && cb()),
    subscribe: jest.fn(), unsubscribe: jest.fn(), end: jest.fn(),
  }) - 1]);
});

const published = (topic) => clients.flatMap((c) => c.publish.mock.calls).filter(([t]) => t === topic);

const emit = (event, ...args) => act(() => { (handlers[event] || []).forEach((cb) => cb(...args)); });
const routineStatus = (state, extra = {}) => Buffer.from(JSON.stringify({
  data: JSON.stringify({
    state, steps: [{ name: 'pick_base' }], current_step: 0, current_step_name: 'pick_base', ...extra,
  }),
}));

test('renders the welcome page', () => {
  render(<App />);
  expect(screen.getByText(/SYSTEM READY/i)).toBeInTheDocument();
});

test('the welcome page has no floating camera panel', () => {
  render(<App />);
  expect(screen.queryByText('Camera Feeds')).not.toBeInTheDocument();
});

test('the routine builder has no floating camera panel over the blocks', () => {
  render(<App />);
  fireEvent.click(screen.getByRole('button', { name: /begin/i }));
  expect(screen.queryByText('Camera Feeds')).not.toBeInTheDocument();
});

test('the run page shows cameras inline instead of the floating panel', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  expect(screen.queryByText('Camera Feeds')).not.toBeInTheDocument();
  expect(screen.getByRole('region', { name: 'Cameras' })).toBeInTheDocument();
});

test('a reload during an experiment comes back to the experiment view', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  expect(screen.getByText('Experiment Sequence')).toBeInTheDocument();
});

test('continue goes straight to the experiment page, with no loader in between', () => {
  render(<App />);
  fireEvent.click(screen.getByRole('button', { name: /begin/i }));
  fireEvent.click(screen.getByRole('button', { name: /continue/i }));
  emit('connect');
  expect(screen.getByText('Experiment Sequence')).toBeInTheDocument();
  expect(screen.queryByText('Preparing your experiment')).not.toBeInTheDocument();
  expect(window.sessionStorage.getItem('apex.view')).toBe('experiment');
});

test('opening the UI while a routine runs jumps to the experiment view', () => {
  render(<App />);
  expect(screen.getByText(/SYSTEM READY/i)).toBeInTheDocument();
  emit('message', 'ros2/routine_executor/status', routineStatus('running'));
  expect(screen.getByText('Experiment Sequence')).toBeInTheDocument();
});

test('an idle executor leaves the welcome page alone', () => {
  render(<App />);
  emit('message', 'ros2/routine_executor/status', routineStatus('idle'));
  expect(screen.getByText(/SYSTEM READY/i)).toBeInTheDocument();
});

const STATUS = 'ros2/routine_executor/status';

test('a failed routine can be retried from the failed step', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  emit('connect');
  emit('message', STATUS, routineStatus('failed', {
    steps: [{ name: 'pick_base' }, { name: 'place' }], current_step: 1, current_step_name: null,
    error: "Step 'place' failed after 3 retries",
  }));
  fireEvent.click(screen.getByRole('button', { name: /retry from step 2/i }));
  expect(published('cmd/routine_executor/retry')).toHaveLength(1);
});

test('new experiment after a failure (and a reload) opens the editor with the last routine', () => {
  const first = render(<App />);
  fireEvent.click(screen.getByRole('button', { name: /begin/i }));
  fireEvent.click(screen.getByRole('tab', { name: 'JSON' }));
  fireEvent.change(screen.getByLabelText('Routine JSON'), { target: { value: '{"steps": [{"name": "pick_base"}]}' } });
  fireEvent.click(screen.getByRole('button', { name: /continue/i }));
  emit('connect');
  first.unmount();
  handlers = {};
  render(<App />);
  emit('message', STATUS, routineStatus('failed', { error: 'cancelled' }));
  fireEvent.click(screen.getByRole('button', { name: /new experiment/i }));
  expect(screen.getAllByTestId('routine-block')).toHaveLength(1);
  fireEvent.click(screen.getByRole('tab', { name: 'JSON' }));
  expect(screen.getByLabelText('Routine JSON')).toHaveValue('{"steps": [{"name": "pick_base"}]}');
  expect(window.sessionStorage.getItem('apex.view')).toBeNull();
});

test('new experiment is offered after a successful routine', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  emit('message', STATUS, routineStatus('success', { current_step: 1, current_step_name: null }));
  fireEvent.click(screen.getByRole('button', { name: /new experiment/i }));
  expect(screen.getByText('Build the routine')).toBeInTheDocument();
});

test('no retry while a routine is running', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  emit('message', STATUS, routineStatus('running'));
  expect(screen.queryByRole('button', { name: /retry from step/i })).not.toBeInTheDocument();
});

test('an idle executor with no routine offers a way back home', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  emit('message', STATUS, routineStatus('idle', { steps: [], current_step_name: null }));
  expect(screen.getByText(/no routine running/i)).toBeInTheDocument();
  fireEvent.click(screen.getByRole('button', { name: /back to home/i }));
  expect(screen.getByText(/SYSTEM READY/i)).toBeInTheDocument();
  expect(window.sessionStorage.getItem('apex.view')).toBeNull();
});

test('an idle executor with no routine offers a new experiment', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  emit('message', STATUS, routineStatus('idle', { steps: [], current_step_name: null }));
  fireEvent.click(screen.getByRole('button', { name: /new experiment/i }));
  expect(screen.getByText('Build the routine')).toBeInTheDocument();
});

test('an idle executor that still has steps shows the tracker, not the idle card', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  emit('message', STATUS, routineStatus('idle'));
  expect(screen.getByText('0/1 Steps Completed')).toBeInTheDocument();
  expect(screen.queryByText(/the routine executor is idle/i)).not.toBeInTheDocument();
});

test('an idle executor that still has steps offers new experiment and home', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  emit('message', STATUS, routineStatus('idle'));
  expect(screen.getByRole('button', { name: /new experiment/i })).toBeInTheDocument();
  fireEvent.click(screen.getByRole('button', { name: /back to home/i }));
  expect(screen.getByText(/SYSTEM READY/i)).toBeInTheDocument();
});

test('a failed liquid-handling step needs a confirmed, forced retry', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  emit('connect');
  emit('message', STATUS, routineStatus('failed', {
    steps: [{ name: 'go_to' }, { name: 'ot2' }], current_step: 1, current_step_name: null,
    error: "Step 'ot2' failed and was not retried", retry_needs_force: true,
  }));
  fireEvent.click(screen.getByRole('button', { name: /retry from step 2/i }));
  expect(published('cmd/routine_executor/retry')).toHaveLength(0);
  expect(screen.getByText(/may still hold liquid/i)).toBeInTheDocument();
  fireEvent.click(screen.getByRole('button', { name: /inspected.*retry/i }));
  const sent = published('cmd/routine_executor/retry');
  expect(sent).toHaveLength(1);
  expect(JSON.parse(sent[0][1])).toEqual({ force: true });
});

test('tip-check results of the run show in the clog check card', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  emit('message', STATUS, routineStatus('running', {
    recordings: [{ type: 'tip_check', id: 'r-1', step: 0, status: 'done', anomaly_detected: true, p_tip_empty: 0.8 }],
  }));
  expect(screen.getByText('Clog check')).toBeInTheDocument();
  expect(screen.getByText('Clog suspected · P(empty) 0.80')).toBeInTheDocument();
});

const openBuilder = () => {
  render(<App />);
  fireEvent.click(screen.getByRole('button', { name: /begin/i }));
};
const sentRoutine = () => JSON.parse(published('cmd/routine_executor/start_routine_cmd')[0][1]);

test('the builder shows the default routine as blocks', () => {
  openBuilder();
  expect(screen.getAllByTestId('routine-block')).toHaveLength(11);
  expect(screen.getByText('11 steps')).toBeInTheDocument();
});

test('blocks can be added, edited, reordered and removed, and the routine sent', () => {
  openBuilder();
  fireEvent.click(screen.getByRole('tab', { name: 'JSON' }));
  fireEvent.change(screen.getByLabelText('Routine JSON'), { target: { value: '{"steps": [], "max_retries": 3}' } });
  fireEvent.click(screen.getByRole('tab', { name: 'Blocks' }));
  expect(screen.getByRole('button', { name: /continue/i })).toBeDisabled();

  fireEvent.click(screen.getByRole('button', { name: 'Add Go to station' }));
  fireEvent.click(screen.getByRole('button', { name: 'Add Wait' }));
  fireEvent.click(screen.getByRole('radio', { name: 'Shaker' }));
  fireEvent.change(screen.getByLabelText('For'), { target: { value: '25' } });
  fireEvent.click(screen.getByRole('button', { name: 'Move step 2 up' }));
  fireEvent.click(screen.getByRole('button', { name: 'Add Return home' }));
  fireEvent.click(screen.getByRole('button', { name: 'Remove step 3' }));

  fireEvent.click(screen.getByRole('button', { name: /continue/i }));
  emit('connect');
  expect(sentRoutine()).toEqual({
    steps: [{ name: 'wait', time_s: 25 }, { name: 'go_to', target_machine: 'shaker' }],
    max_retries: 3,
  });
});

test('an OT-2 block edits its commands', () => {
  openBuilder();
  fireEvent.click(screen.getByRole('tab', { name: 'JSON' }));
  fireEvent.change(screen.getByLabelText('Routine JSON'), { target: { value: '{"steps": []}' } });
  fireEvent.click(screen.getByRole('tab', { name: 'Blocks' }));
  fireEvent.click(screen.getByRole('button', { name: 'Add OT-2 protocol' }));
  fireEvent.change(screen.getByLabelText('Command 2 volumes'), { target: { value: '150' } });
  fireEvent.change(screen.getByLabelText('Command 3 wells'), { target: { value: '0, 5,' } });
  fireEvent.change(screen.getByLabelText('Command 3 wells'), { target: { value: '0, 5, 10' } });
  fireEvent.click(screen.getByRole('button', { name: 'Remove command 1' }));
  fireEvent.click(screen.getByRole('button', { name: /continue/i }));
  emit('connect');
  const cmds = sentRoutine().steps[0].parameters_json.steps;
  expect(cmds.map((c) => c.action)).toEqual(['aspirate', 'dispense', 'drop_tips']);
  expect(cmds[0].volumes).toEqual([150]);
  expect(cmds[1].well_indices).toEqual([0, 5, 10]);
});

test('a block missing a parameter blocks sending', () => {
  openBuilder();
  fireEvent.click(screen.getByRole('tab', { name: 'JSON' }));
  fireEvent.change(screen.getByLabelText('Routine JSON'), { target: { value: '{"steps": [{"name": "place"}]}' } });
  fireEvent.click(screen.getByRole('tab', { name: 'Blocks' }));
  expect(screen.getByText('Needs target_machine')).toBeInTheDocument();
  expect(screen.getByRole('button', { name: /continue/i })).toBeDisabled();
});

test('the run timeline lists every step with readable names', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  emit('message', STATUS, routineStatus('running', {
    steps: [{ name: 'go_to', target_machine: 'ot2' }, { name: 'shaker', pwm: 150 }], current_step: 1, current_step_name: 'shaker',
  }));
  const timeline = screen.getByRole('complementary', { name: 'Run progress' });
  expect(timeline).toHaveTextContent('Go to OT-2');
  expect(timeline).toHaveTextContent('1/2 Steps Completed');
  expect(screen.getAllByText('Run shaker').length).toBeGreaterThan(0);
});
