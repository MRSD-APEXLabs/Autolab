import { render, screen, act } from '@testing-library/react';
import StatusEcho from './StatusEcho';
import { useMqttContext } from './useMqtt';

jest.mock('./useMqtt');

test('shows placeholder before any message arrives', () => {
  useMqttContext.mockReturnValue({ subscribe: () => () => {} });
  render(<StatusEcho topic="ros2/planning_state" label="Planning State" />);
  expect(screen.getByText('—')).toBeInTheDocument();
  expect(screen.getByText('no data')).toBeInTheDocument();
});

test('renders the latest message received on its topic', () => {
  let handler;
  useMqttContext.mockReturnValue({
    subscribe: (topic, cb) => { handler = cb; return () => {}; },
  });
  render(<StatusEcho topic="ros2/planning_state" label="Planning State" />);

  act(() => { handler('{"state":"EXECUTING"}'); });

  expect(screen.getByText('{"state":"EXECUTING"}')).toBeInTheDocument();
  expect(screen.queryByText('no data')).not.toBeInTheDocument();
});

test('unsubscribes on unmount', () => {
  const unsubscribe = jest.fn();
  useMqttContext.mockReturnValue({ subscribe: () => unsubscribe });
  const { unmount } = render(<StatusEcho topic="ros2/planning_state" label="Planning State" />);
  unmount();
  expect(unsubscribe).toHaveBeenCalled();
});
