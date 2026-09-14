import { render, screen, fireEvent } from '@testing-library/react';
import NavigationPanel from './NavigationPanel';
import { useMqttContext } from '../useMqtt';

jest.mock('../useMqtt');

test('publishes the entered x/y/theta after confirming', () => {
  const publish = jest.fn();
  useMqttContext.mockReturnValue({ publish, connected: true, subscribe: () => () => {} });

  render(<NavigationPanel />);
  fireEvent.change(screen.getByLabelText(/X \(m\)/i), { target: { value: '1.5' } });
  fireEvent.change(screen.getByLabelText(/Y \(m\)/i), { target: { value: '-2' } });
  fireEvent.change(screen.getByLabelText(/Theta \(deg\)/i), { target: { value: '90' } });

  fireEvent.click(screen.getByRole('button', { name: 'Send Goal Pose' }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));

  expect(publish).toHaveBeenCalledWith(
    'cmd/navigation/goal_pose',
    JSON.stringify({ x: 1.5, y: -2, theta: 90 }),
  );
});

test('send button is disabled while disconnected', () => {
  useMqttContext.mockReturnValue({ publish: jest.fn(), connected: false, subscribe: () => () => {} });
  render(<NavigationPanel />);
  expect(screen.getByRole('button', { name: 'Send Goal Pose' })).toBeDisabled();
});
