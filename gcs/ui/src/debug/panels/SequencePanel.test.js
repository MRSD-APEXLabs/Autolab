import { render, screen, fireEvent, act } from '@testing-library/react';
import SequencePanel, { FULL_SEQUENCE } from './SequencePanel';
import { useMqttContext } from '../../shared/useMqtt';

jest.mock('../../shared/useMqtt');

let publish;
let listeners;

function sendStatus(status) {
  act(() => {
    listeners['ros2/routine_executor/status'](JSON.stringify({ data: JSON.stringify(status) }));
  });
}

beforeEach(() => {
  publish = jest.fn();
  listeners = {};
  useMqttContext.mockReturnValue({
    publish,
    connected: true,
    subscribe: (topic, cb) => { listeners[topic] = cb; return () => {}; },
  });
});

test('the sequence is go to OT-2, pick base, place/pick OT-2, go to shaker, place/pick shaker, place base, home', () => {
  expect(FULL_SEQUENCE).toEqual([
    { name: 'go_to', target_machine: 'ot2' },
    { name: 'pick_base' },
    { name: 'place', target_machine: 'ot2' },
    { name: 'pick', target_machine: 'ot2' },
    { name: 'go_to', target_machine: 'shaker' },
    { name: 'place', target_machine: 'shaker' },
    { name: 'pick', target_machine: 'shaker' },
    { name: 'place_base' },
    { name: 'home' },
  ]);
});

test('start publishes the full sequence after confirmation, without retries', () => {
  render(<SequencePanel />);
  fireEvent.click(screen.getByRole('button', { name: 'Start Full Sequence' }));
  expect(publish).not.toHaveBeenCalled();
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));

  expect(publish).toHaveBeenCalledTimes(1);
  const [topic, payload] = publish.mock.calls[0];
  expect(topic).toBe('cmd/routine_executor/start_routine_cmd');
  expect(JSON.parse(payload)).toEqual({ steps: FULL_SEQUENCE, max_retries: 1, robot: 'robot_1' });
});

test('stop is one click while running and cancels the routine', () => {
  render(<SequencePanel />);
  expect(screen.getByRole('button', { name: 'Stop' })).toBeDisabled();

  sendStatus({ state: 'running', steps: FULL_SEQUENCE, current_step: 2, error: null });
  expect(screen.getByTestId('sequence-state')).toHaveTextContent('Running: step 3/9, place (ot2)');
  expect(screen.getByRole('button', { name: 'Start Full Sequence' })).toBeDisabled();

  fireEvent.click(screen.getByRole('button', { name: 'Stop' }));
  expect(publish).toHaveBeenCalledWith('cmd/routine_executor/cancel', '');
});

test('shows where a cancelled run stopped and allows a new start', () => {
  render(<SequencePanel />);
  sendStatus({ state: 'failed', steps: FULL_SEQUENCE, current_step: 4, error: 'cancelled' });

  expect(screen.getByTestId('sequence-state')).toHaveTextContent('Stopped at step 5: cancelled');
  expect(screen.getByRole('button', { name: 'Stop' })).toBeDisabled();
  expect(screen.getByRole('button', { name: 'Start Full Sequence' })).toBeEnabled();
});
