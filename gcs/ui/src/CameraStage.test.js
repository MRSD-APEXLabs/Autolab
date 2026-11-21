import { render, screen, fireEvent, act } from '@testing-library/react';
import mqtt from 'mqtt';
import CameraStage from './CameraStage';

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
const robotSrc = () => screen.getAllByRole('img').map((img) => img.getAttribute('src')).find((u) => u && u.includes(':8002/'));
const fetched = () => fetch.mock.calls.map(([u]) => u);
const DECK = 'http://192.168.10.19:8766/stream';

test('shows the active robot camera annotated, with the three station cameras', () => {
  render(<CameraStage />);
  hubStatus({ active: 'zedx', state: 'streaming' });
  expect(robotSrc()).toMatch(/^http:\/\/192\.168\.10\.3:8002\/stream\/annotated\/zedx\.mjpg\?s=/);
  expect(fetched()).toEqual(expect.arrayContaining([`${DECK}/ot2.mjpg?fetch=1`, `${DECK}/shaker.mjpg?fetch=1`, `${DECK}/home.mjpg?fetch=1`]));
});

test('switches the robot view between annotated and raw', () => {
  render(<CameraStage />);
  hubStatus({ active: 'zedx_nano', state: 'streaming' });
  fireEvent.click(screen.getByRole('radio', { name: 'Raw' }));
  expect(robotSrc()).toMatch(/\/stream\/raw\/zedx_nano\.mjpg\?s=/);
});

test('a robot stream shown again never reuses an old URL', () => {
  render(<CameraStage />);
  hubStatus({ active: 'zedx', state: 'streaming' });
  const first = robotSrc();
  fireEvent.click(screen.getByRole('radio', { name: 'Raw' }));
  fireEvent.click(screen.getByRole('radio', { name: 'Annotated' }));
  expect(robotSrc()).toMatch(/annotated\/zedx\.mjpg/);
  expect(robotSrc()).not.toBe(first);
});

test('a station camera can be shown large; annotated/raw only applies to the robot', () => {
  render(<CameraStage />);
  hubStatus({ active: 'zedx', state: 'streaming' });
  fireEvent.click(screen.getByRole('button', { name: /show shaker large/i }));
  expect(screen.getAllByAltText('Shaker camera')).toHaveLength(1);
  expect(screen.getByRole('radio', { name: 'Raw' })).toBeDisabled();
});
test('never selects the robot camera: nothing is published', () => {
  render(<CameraStage />);
  hubStatus({ active: 'zedx', state: 'streaming' });
  fireEvent.click(screen.getByRole('radio', { name: 'Raw' }));
  fireEvent.click(screen.getByRole('button', { name: /show opentrons/i }));
  fireEvent.click(screen.getByRole('button', { name: /show zed x/i }));
  expect(client.publish).not.toHaveBeenCalled();
});

test('marks the station of the current step', () => {
  render(<CameraStage station="ot2" />);
  expect(screen.getByText('current step')).toBeInTheDocument();
});

test('no robot camera streaming shows no signal', () => {
  render(<CameraStage />);
  hubStatus({ active: null, state: null });
  expect(screen.getAllByText('No robot camera streaming').length).toBeGreaterThan(0);
});
