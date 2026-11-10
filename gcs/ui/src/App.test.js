import { render, screen, fireEvent, act } from '@testing-library/react';
import mqtt from 'mqtt';
import App from './App';

jest.mock('mqtt');

let handlers;

beforeEach(() => {
  window.sessionStorage.clear();
  handlers = {};
  mqtt.connect.mockImplementation(() => ({
    on: jest.fn((event, cb) => { (handlers[event] ||= []).push(cb); }),
    publish: jest.fn((_topic, _payload, _opts, cb) => cb && cb()),
    subscribe: jest.fn(), unsubscribe: jest.fn(), end: jest.fn(),
  }));
});

const emit = (event, ...args) => act(() => { (handlers[event] || []).forEach((cb) => cb(...args)); });
const routineStatus = (state) => Buffer.from(JSON.stringify({
  data: JSON.stringify({ state, steps: ['pick_base'], current_step: 0, current_step_name: 'pick_base' }),
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
