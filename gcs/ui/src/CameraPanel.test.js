import { render, screen, fireEvent, act } from '@testing-library/react';
import mqtt from 'mqtt';
import CameraPanel from './CameraPanel';

jest.mock('mqtt');

let client;
beforeEach(() => {
  const handlers = {};
  client = {
    on: jest.fn((event, cb) => { handlers[event] = cb; }),
    publish: jest.fn(),
    subscribe: jest.fn(),
    unsubscribe: jest.fn(),
    end: jest.fn(),
    emit: (event, ...args) => handlers[event]?.(...args),
  };
  mqtt.connect.mockReturnValue(client);
  global.fetch = jest.fn(() => new Promise(() => {}));
});

const bridged = (obj) => Buffer.from(JSON.stringify({ data: JSON.stringify(obj) }));
const hubStatus = (status) => act(() => {
  client.emit('connect');
  client.emit('message', 'ros2/camera_hub/status', bridged({ reachable: true, error: null, ...status }));
});
const expand = () => fireEvent.click(screen.getByRole('button', { name: /expand/i }));

test('renders minimized by default showing only the header', () => {
  render(<CameraPanel />);
  expect(screen.getByText('Camera Feeds')).toBeInTheDocument();
  expect(screen.queryByTestId('camera-pair')).not.toBeInTheDocument();
  expect(screen.queryByRole('img')).not.toBeInTheDocument();
});

test('header names the active camera', () => {
  render(<CameraPanel />);
  expect(screen.getByText('none')).toBeInTheDocument();
  hubStatus({ active: 'zedx', state: 'streaming' });
  expect(screen.getByText('ZED X (base)')).toBeInTheDocument();
});

test('expanded shows annotated and raw streams of the active camera', () => {
  render(<CameraPanel />);
  hubStatus({ active: 'zedx', state: 'streaming' });
  expand();
  const srcs = screen.getAllByRole('img').map((img) => img.getAttribute('src'));
  expect(srcs).toEqual([
    'http://localhost:8002/stream/annotated/zedx.mjpg', 'http://localhost:8002/stream/raw/zedx.mjpg']);
});

test('minimizing unmounts the streams', () => {
  render(<CameraPanel />);
  hubStatus({ active: 'zedx', state: 'streaming' });
  expand();
  fireEvent.click(screen.getByRole('button', { name: /minimize/i }));
  expect(screen.queryByRole('img')).not.toBeInTheDocument();
});

test('is view-only: no camera switch controls', () => {
  render(<CameraPanel />);
  expand();
  expect(screen.queryByRole('button', { name: /ZED X/ })).not.toBeInTheDocument();
  expect(screen.queryByRole('button', { name: /Release/ })).not.toBeInTheDocument();
});

test('detection strip summarizes tags and the planner target', () => {
  render(<CameraPanel />);
  hubStatus({ active: 'zedx', state: 'streaming' });
  expand();
  expect(screen.getByTestId('detection-strip')).toHaveTextContent('Tags —');
  act(() => client.emit('message', 'ros2/inspect/json', bridged({
    apriltags: [{ id: 21, pose: null }, { id: 0, pose: null }],
    wellplates: [{ name: 'wellplate', conf: 0.91, pose: { position: [0, 0, 0.62], orientation: [0, 0, 0, 1] } }],
  })));
  expect(screen.getByTestId('detection-strip')).toHaveTextContent('Tags 0 · 21');
  expect(screen.getByTestId('detection-strip')).toHaveTextContent('Wellplate 0.91 @ 0.62 m');
});
