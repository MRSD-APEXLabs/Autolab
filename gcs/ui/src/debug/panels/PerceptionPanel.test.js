import { render, screen, fireEvent } from '@testing-library/react';
import PerceptionPanel from './PerceptionPanel';
import { useMqttContext } from '../useMqtt';

jest.mock('../useMqtt');

test('publishes the raw mode string for the clicked button', () => {
  const publish = jest.fn();
  useMqttContext.mockReturnValue({ publish, connected: true, subscribe: () => () => {} });

  render(<PerceptionPanel />);
  fireEvent.click(screen.getByRole('button', { name: 'Servo' }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));

  expect(publish).toHaveBeenCalledWith('cmd/perception/camera_mode_cmd', 'servo');
});

test('renders read-only apriltag and wellplate echoes', () => {
  useMqttContext.mockReturnValue({ publish: jest.fn(), connected: true, subscribe: () => () => {} });
  render(<PerceptionPanel />);
  expect(screen.getByText('AprilTags (read-only)')).toBeInTheDocument();
  expect(screen.getByText('Wellplates (read-only)')).toBeInTheDocument();
});
