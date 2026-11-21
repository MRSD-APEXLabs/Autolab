import { render, screen, fireEvent } from '@testing-library/react';
import LabMachinePanel from './LabMachinePanel';
import { useMqttContext } from '../../shared/useMqtt';

jest.mock('../../shared/useMqtt');

let routine = { state: 'idle' };
const routineStatus = (topic, callback) => {
  if (topic === 'ros2/routine_executor/status') callback(JSON.stringify({ data: JSON.stringify(routine) }));
  return () => {};
};

beforeEach(() => {
  routine = { state: 'idle' };
  useMqttContext.mockReturnValue({ publish: jest.fn(), connected: true, subscribe: routineStatus });
});

test('OT2 send button is disabled when the JSON textarea is invalid', () => {
  render(<LabMachinePanel />);
  fireEvent.change(screen.getByLabelText(/Parameters JSON/i), { target: { value: '{not json' } });
  expect(screen.getByRole('button', { name: 'Send OT2 Protocol' })).toBeDisabled();
  expect(screen.getByText(/Invalid JSON/)).toBeInTheDocument();
});

test('OT2 publishes parsed parameters_json on confirm', () => {
  const publish = jest.fn();
  useMqttContext.mockReturnValue({ publish, connected: true, subscribe: routineStatus });

  render(<LabMachinePanel />);
  fireEvent.change(screen.getByLabelText(/Parameters JSON/i), { target: { value: '{"steps":[]}' } });
  fireEvent.click(screen.getByRole('button', { name: 'Send OT2 Protocol' }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));

  expect(publish).toHaveBeenCalledWith(
    'cmd/lab_machine/ot2_command',
    JSON.stringify({ action: 'protocol', parameters_json: { steps: [] } }),
  );
});

test('Shaker publishes pwm and wait_time_s on confirm', () => {
  const publish = jest.fn();
  useMqttContext.mockReturnValue({ publish, connected: true, subscribe: routineStatus });

  render(<LabMachinePanel />);
  fireEvent.change(screen.getByLabelText(/PWM/i), { target: { value: '200' } });
  fireEvent.change(screen.getByLabelText(/Wait Time/i), { target: { value: '30' } });
  fireEvent.click(screen.getByRole('button', { name: 'Send Shaker Command' }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));

  expect(publish).toHaveBeenCalledWith(
    'cmd/lab_machine/shaker_command',
    JSON.stringify({ pwm: 200, wait_time_s: 30 }),
  );
});


test('Shaker rejects invalid PWM and timeout values', () => {
  render(<LabMachinePanel />);
  fireEvent.change(screen.getByLabelText(/Wait Time/i), { target: { value: '-1' } });
  expect(screen.getByRole('button', { name: 'Send Shaker Command' })).toBeDisabled();
  fireEvent.change(screen.getByLabelText(/Wait Time/i), { target: { value: '0' } });
  fireEvent.change(screen.getByLabelText(/PWM/i), { target: { value: '256' } });
  expect(screen.getByRole('button', { name: 'Send Shaker Command' })).toBeDisabled();
  fireEvent.change(screen.getByLabelText(/PWM/i), { target: { value: '0' } });
  expect(screen.getByRole('button', { name: 'Send Shaker Command' })).toBeEnabled();
});


test('OT-2 debug runs through routines; no parts runner or rail lights', () => {
  render(<LabMachinePanel />);
  expect(screen.getByRole('button', { name: /aspirate → tip check → dispense/i })).toBeInTheDocument();
  expect(screen.queryByText(/rail lights/i)).not.toBeInTheDocument();
  expect(screen.queryByText(/Part 1/)).not.toBeInTheDocument();
});

test('raw OT2 protocol cannot be sent while a routine is running', () => {
  routine = { state: 'running', steps: [{ name: 'ot2' }], current_step: 0 };
  render(<LabMachinePanel />);
  expect(screen.getByRole('button', { name: 'Send OT2 Protocol' })).toBeDisabled();
});
