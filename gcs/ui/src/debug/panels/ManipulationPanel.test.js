import { render, screen, fireEvent } from '@testing-library/react';
import ManipulationPanel from './ManipulationPanel';
import { useMqttContext } from '../useMqtt';

jest.mock('../useMqtt');

beforeEach(() => {
  useMqttContext.mockReturnValue({ publish: jest.fn(), connected: true, subscribe: () => () => {} });
});

test('target machine field only appears for type "place"', () => {
  render(<ManipulationPanel />);
  expect(screen.queryByLabelText(/Target Machine/i)).not.toBeInTheDocument();

  fireEvent.change(screen.getByLabelText(/^Type$/i), { target: { value: 'place' } });
  expect(screen.getByLabelText(/Target Machine/i)).toBeInTheDocument();
});

test('publishes type/object_type/target_machine for a place command', () => {
  const publish = jest.fn();
  useMqttContext.mockReturnValue({ publish, connected: true, subscribe: () => () => {} });

  render(<ManipulationPanel />);
  fireEvent.change(screen.getByLabelText(/^Type$/i), { target: { value: 'place' } });
  fireEvent.change(screen.getByLabelText(/Object Type/i), { target: { value: 'well_plate' } });
  fireEvent.change(screen.getByLabelText(/Target Machine/i), { target: { value: 'shaker' } });

  fireEvent.click(screen.getByRole('button', { name: 'Send Command' }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));

  expect(publish).toHaveBeenCalledWith(
    'cmd/manipulation/command',
    JSON.stringify({ type: 'place', object_type: 'well_plate', target_machine: 'shaker' }),
  );
});

test('target_machine is an empty string for non-place types', () => {
  const publish = jest.fn();
  useMqttContext.mockReturnValue({ publish, connected: true, subscribe: () => () => {} });

  render(<ManipulationPanel />);
  fireEvent.click(screen.getByRole('button', { name: 'Send Command' }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));

  expect(publish).toHaveBeenCalledWith(
    'cmd/manipulation/command',
    JSON.stringify({ type: 'pick_up', object_type: 'well_plate', target_machine: '' }),
  );
});
