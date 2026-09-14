import { render, screen, fireEvent } from '@testing-library/react';
import PlanningPanel from './PlanningPanel';
import { useMqttContext } from '../useMqtt';

jest.mock('../useMqtt');

test('publishes the raw command string for the clicked button', () => {
  const publish = jest.fn();
  useMqttContext.mockReturnValue({ publish, connected: true, subscribe: () => () => {} });

  render(<PlanningPanel />);
  fireEvent.click(screen.getByRole('button', { name: 'Home' }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));

  expect(publish).toHaveBeenCalledWith('cmd/planning/command', 'plan_home');
});

test('every planning command button is present', () => {
  useMqttContext.mockReturnValue({ publish: jest.fn(), connected: true, subscribe: () => () => {} });
  render(<PlanningPanel />);
  ['Home', 'Home Offset', 'Wellplate', 'AprilTag: OT2', 'AprilTag: Shaker'].forEach((label) => {
    expect(screen.getByRole('button', { name: label })).toBeInTheDocument();
  });
});
