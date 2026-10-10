import { render, screen, fireEvent } from '@testing-library/react';
import LabMachinePanel from './LabMachinePanel';
import { useMqttContext } from '../useMqtt';

jest.mock('../useMqtt');

beforeEach(() => {
  useMqttContext.mockReturnValue({ publish: jest.fn(), connected: true, subscribe: () => () => {} });
});

test('OT2 send button is disabled when the JSON textarea is invalid', () => {
  render(<LabMachinePanel />);
  fireEvent.change(screen.getByLabelText(/Parameters JSON/i), { target: { value: '{not json' } });
  expect(screen.getByRole('button', { name: 'Send OT2 Protocol' })).toBeDisabled();
  expect(screen.getByText(/Invalid JSON/)).toBeInTheDocument();
});

test('OT2 publishes parsed parameters_json on confirm', () => {
  const publish = jest.fn();
  useMqttContext.mockReturnValue({ publish, connected: true, subscribe: () => () => {} });

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
  useMqttContext.mockReturnValue({ publish, connected: true, subscribe: () => () => {} });

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
