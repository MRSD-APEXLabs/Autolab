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

test('camera panel header is visible on the welcome page', () => {
  render(<App />);
  expect(screen.getByText('Camera Feeds')).toBeInTheDocument();
});

test('camera panel header is visible on the experiment page', () => {
  render(<App />);
  fireEvent.click(screen.getByRole('button', { name: /begin/i }));
  expect(screen.getByText('Camera Feeds')).toBeInTheDocument();
});

test('camera panel header is visible on the loader page', () => {
  render(<App />);
  fireEvent.click(screen.getByRole('button', { name: /begin/i }));
  fireEvent.click(screen.getByRole('button', { name: /continue/i }));
  expect(screen.getByText('Camera Feeds')).toBeInTheDocument();
});

test('a reload during an experiment comes back to the experiment view', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  expect(screen.getByText('Experiment Sequence')).toBeInTheDocument();
});

test('the loader page is remembered as the experiment view', () => {
  render(<App />);
  fireEvent.click(screen.getByRole('button', { name: /begin/i }));
  fireEvent.click(screen.getByRole('button', { name: /continue/i }));
  emit('connect');
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
  fireEvent.change(screen.getByRole('textbox'), { target: { value: '{"steps": [{"name": "pick_base"}]}' } });
  fireEvent.click(screen.getByRole('button', { name: /continue/i }));
  emit('connect');
  first.unmount();
  handlers = {};
  render(<App />);
  emit('message', STATUS, routineStatus('failed', { error: 'cancelled' }));
  fireEvent.click(screen.getByRole('button', { name: /new experiment/i }));
  expect(screen.getByRole('textbox')).toHaveValue('{"steps": [{"name": "pick_base"}]}');
  expect(window.sessionStorage.getItem('apex.view')).toBeNull();
});

test('new experiment is offered after a successful routine', () => {
  window.sessionStorage.setItem('apex.view', 'experiment');
  render(<App />);
  emit('message', STATUS, routineStatus('success', { current_step: 1, current_step_name: null }));
  fireEvent.click(screen.getByRole('button', { name: /new experiment/i }));
  expect(screen.getByRole('textbox')).toBeInTheDocument();
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
  expect(screen.getByRole('textbox')).toBeInTheDocument();
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
