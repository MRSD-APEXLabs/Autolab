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

test.each([
  ['Home', 'home'],
  ['Shaker', 'shaker'],
  ['OT2', 'ot2'],
])('"%s" publishes go_to_location "%s" after confirming', (buttonName, location) => {
  const publish = jest.fn();
  useMqttContext.mockReturnValue({ publish, connected: true, subscribe: () => () => {} });

  render(<NavigationPanel />);
  fireEvent.click(screen.getByRole('button', { name: buttonName }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));

  expect(publish).toHaveBeenCalledWith('cmd/navigation/go_to_location', location);
});

test('all send buttons are disabled while disconnected', () => {
  useMqttContext.mockReturnValue({ publish: jest.fn(), connected: false, subscribe: () => () => {} });
  render(<NavigationPanel />);
  ['Home', 'Shaker', 'OT2', 'Send Goal Pose'].forEach((name) => {
    expect(screen.getByRole('button', { name })).toBeDisabled();
  });
});

test('echoes go_to_location and navigate_to_pose status topics', () => {
  const subscribe = jest.fn(() => () => {});
  useMqttContext.mockReturnValue({ publish: jest.fn(), connected: true, subscribe });
  render(<NavigationPanel />);
  const topics = subscribe.mock.calls.map(([topic]) => topic);
  expect(topics).toContain('ros2/robot_1/behavior/go_to_location_status');
  expect(topics).toContain('ros2/robot_1/behavior/navigate_to_pose_status');
});
