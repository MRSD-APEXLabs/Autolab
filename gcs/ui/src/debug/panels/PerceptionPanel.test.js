import { render, screen, fireEvent, act } from '@testing-library/react';
import PerceptionPanel, { SELECT_TIMEOUT_MS } from './PerceptionPanel';
import { useMqttContext } from '../../shared/useMqtt';

jest.mock('../../shared/useMqtt');

let handlers;
let publish;
beforeEach(() => {
  handlers = {};
  publish = jest.fn();
  global.fetch = jest.fn(() => new Promise(() => {}));
  useMqttContext.mockReturnValue({
    publish,
    connected: true,
    subscribe: (topic, cb) => { handlers[topic] = cb; return () => { delete handlers[topic]; }; },
  });
});
const hubStatus = (status) => act(() => handlers['ros2/camera_hub/status'](
  JSON.stringify({ data: JSON.stringify({ reachable: true, error: null, ...status }) })));

function confirm(name) {
  fireEvent.click(screen.getByRole('button', { name }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));
}

test.each([['ZED X (base)', '"zedx"'], ['ZED X Nano (wrist)', '"zedx_nano"'], ['Release', 'null']])(
  '%s publishes %s after confirmation', (label, payload) => {
    render(<PerceptionPanel />);
    fireEvent.click(screen.getByRole('button', { name: label }));
    expect(publish).not.toHaveBeenCalled();
    fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));
    expect(publish).toHaveBeenCalledWith('cmd/perception/select_camera', payload);
  },
);

test('shows switching until the result, then a failure message', () => {
  render(<PerceptionPanel />);
  confirm('ZED X (base)');
  expect(screen.getByText('Switching…')).toBeInTheDocument();
  expect(screen.getByRole('button', { name: 'ZED X Nano (wrist)' })).toBeDisabled();
  act(() => handlers['perception/select_camera_result'](
    JSON.stringify({ camera: 'zedx', ok: false, message: 'camera_hub node is not running' })));
  expect(screen.queryByText('Switching…')).not.toBeInTheDocument();
  expect(screen.getByText('camera_hub node is not running')).toBeInTheDocument();
  expect(screen.getByRole('button', { name: 'ZED X Nano (wrist)' })).toBeEnabled();
});

test('a switch with no reply times out instead of locking the buttons', () => {
  jest.useFakeTimers();
  try {
    render(<PerceptionPanel />);
    confirm('ZED X (base)');
    act(() => { jest.advanceTimersByTime(SELECT_TIMEOUT_MS); });
    expect(screen.queryByText('Switching…')).not.toBeInTheDocument();
    expect(screen.getByText('No reply from the MQTT bridge')).toBeInTheDocument();
  } finally {
    jest.useRealTimers();
  }
});

test('hub status line and video follow the active camera', () => {
  render(<PerceptionPanel />);
  expect(screen.getByText('Hub: no status yet')).toBeInTheDocument();
  expect(screen.getAllByText('No camera selected')).toHaveLength(2);
  hubStatus({ active: 'zedx_nano', state: 'streaming' });
  expect(screen.getByText(/ZED X Nano \(wrist\) · streaming/)).toBeInTheDocument();
  expect(screen.getAllByRole('img')[0].getAttribute('src')).toContain('/stream/annotated/zedx_nano.mjpg');
  expect(Object.keys(handlers)).toContain('ros2/servo/json');
});

test('old Camera-Edge controls are gone', () => {
  render(<PerceptionPanel />);
  expect(screen.queryByRole('button', { name: 'Inspect' })).not.toBeInTheDocument();
  expect(screen.queryByText('AprilTags (read-only)')).not.toBeInTheDocument();
});
