import { render, screen, fireEvent, act } from '@testing-library/react';
import Ot2Routines, { transferProtocol, cleanupProtocol } from './Ot2Routines';
import { useMqttContext } from '../../shared/useMqtt';

jest.mock('../../shared/useMqtt');

const START = 'cmd/routine_executor/start_routine_cmd';
let publish;
let listeners;

function sendStatus(status) {
  act(() => {
    listeners['ros2/routine_executor/status'](JSON.stringify({ data: JSON.stringify(status) }));
  });
}

const started = () => publish.mock.calls.filter(([t]) => t === START).map(([, p]) => JSON.parse(p));

function confirm(name) {
  fireEvent.click(screen.getByRole('button', { name }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));
}

beforeEach(() => {
  publish = jest.fn();
  listeners = {};
  useMqttContext.mockReturnValue({
    publish, connected: true, subscribe: (topic, cb) => { listeners[topic] = cb; return () => {}; },
  });
});

test('a transfer is pick up tip, aspirate from the tube, dispense into plate A1, drop the tip', () => {
  expect(transferProtocol({ tip: 4, tube: 'A3', volume: 150 })).toEqual({ steps: [
    { action: 'pick_up_tips', resource_name: 'tip_rack', well_indices: [4] },
    { action: 'aspirate', resource_name: 'tube_rack', well_indices: [6], volumes: [150] },
    { action: 'dispense', resource_name: 'empty_plate', well_indices: [0], volumes: [150] },
    { action: 'drop_tips' },
  ] });
  expect(cleanupProtocol()).toEqual({ steps: [{ action: 'drop_tips' }] });
});

test('aspirate -> tip check -> dispense runs as one ot2 routine step, never retried', () => {
  render(<Ot2Routines />);
  sendStatus({ state: 'idle' });
  fireEvent.change(screen.getByLabelText(/tip index/i), { target: { value: '7' } });
  fireEvent.change(screen.getByLabelText(/source test tube/i), { target: { value: 'A2' } });
  confirm(/aspirate → tip check → dispense/i);
  expect(started()).toEqual([{
    steps: [{ name: 'ot2', parameters_json: transferProtocol({ tip: 7, tube: 'A2', volume: 200 }) }],
    max_retries: 0, robot: 'robot_1',
  }]);
  expect(publish.mock.calls.some(([t]) => /ot2_parts|ot2_lights/.test(t))).toBe(false);
});

test('the full workflow is two transfers: tube A1 with the chosen tip, then tube A2 with the next', () => {
  render(<Ot2Routines />);
  sendStatus({ state: 'idle' });
  fireEvent.change(screen.getByLabelText(/tip index/i), { target: { value: '0' } });
  confirm(/full workflow/i);
  expect(started()[0].steps).toEqual([
    { name: 'ot2', parameters_json: transferProtocol({ tip: 0, tube: 'A1', volume: 200 }) },
    { name: 'ot2', parameters_json: transferProtocol({ tip: 1, tube: 'A2', volume: 200 }) },
  ]);
});

test('discard tip sends a drop-tips-only protocol', () => {
  render(<Ot2Routines />);
  sendStatus({ state: 'failed', error: 'x', steps: [{ name: 'ot2' }], current_step: 0 });
  confirm(/discard tip/i);
  expect(started()[0].steps).toEqual([{ name: 'ot2', parameters_json: cleanupProtocol() }]);
});

test('invalid tip or volume disables the transfer', () => {
  render(<Ot2Routines />);
  sendStatus({ state: 'idle' });
  fireEvent.change(screen.getByLabelText(/tip index/i), { target: { value: '96' } });
  expect(screen.getByRole('button', { name: /aspirate → tip check → dispense/i })).toBeDisabled();
  fireEvent.change(screen.getByLabelText(/tip index/i), { target: { value: '3' } });
  fireEvent.change(screen.getByLabelText(/volume/i), { target: { value: '250' } });
  expect(screen.getByRole('button', { name: /aspirate → tip check → dispense/i })).toBeDisabled();
});

test('nothing can start while a routine is active or before the executor reports', () => {
  const onActive = jest.fn();
  render(<Ot2Routines onActive={onActive} />);
  expect(screen.getByRole('button', { name: /full workflow/i })).toBeDisabled();
  sendStatus({ state: 'running', steps: [{ name: 'ot2' }], current_step: 0 });
  expect(screen.getByRole('button', { name: /aspirate → tip check → dispense/i })).toBeDisabled();
  expect(screen.getByRole('button', { name: /discard tip/i })).toBeDisabled();
  expect(onActive).toHaveBeenLastCalledWith(true);
});

test('shows the run state, its error and the tip-check results', () => {
  render(<Ot2Routines />);
  sendStatus({
    state: 'failed', steps: [{ name: 'ot2' }], current_step: 0,
    error: "Step 'ot2' failed and was not retried: it handles liquid",
    recordings: [{ type: 'tip_check', id: 'r-1', step: 0, status: 'done', anomaly_detected: true, p_tip_empty: 0.8 }],
  });
  expect(screen.getByText(/not retried: it handles liquid/)).toBeInTheDocument();
  expect(screen.getByText('Clog suspected · P(empty) 0.80')).toBeInTheDocument();
});
