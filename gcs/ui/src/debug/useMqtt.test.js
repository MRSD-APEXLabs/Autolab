import React from 'react';
import { render, screen, act, fireEvent } from '@testing-library/react';
import mqtt from 'mqtt';
import { MqttProvider, useMqttContext } from './useMqtt';

jest.mock('mqtt');

function makeFakeClient() {
  const handlers = {};
  return {
    on: jest.fn((event, cb) => { handlers[event] = cb; }),
    publish: jest.fn(),
    subscribe: jest.fn(),
    unsubscribe: jest.fn(),
    end: jest.fn(),
    _handlers: handlers,
  };
}

function Probe() {
  const { connected, publish } = useMqttContext();
  return (
    <div>
      <span data-testid="status">{connected ? 'up' : 'down'}</span>
      <button onClick={() => publish('cmd/test', 'hi')}>send</button>
    </div>
  );
}

test('reports connected after the client fires connect', () => {
  const client = makeFakeClient();
  mqtt.connect.mockReturnValue(client);

  render(<MqttProvider><Probe /></MqttProvider>);
  expect(screen.getByTestId('status').textContent).toBe('down');

  act(() => { client._handlers.connect(); });
  expect(screen.getByTestId('status').textContent).toBe('up');
});

test('reports disconnected after the client fires close', () => {
  const client = makeFakeClient();
  mqtt.connect.mockReturnValue(client);

  render(<MqttProvider><Probe /></MqttProvider>);
  act(() => { client._handlers.connect(); });
  act(() => { client._handlers.close(); });
  expect(screen.getByTestId('status').textContent).toBe('down');
});

test('publish forwards to the underlying client', () => {
  const client = makeFakeClient();
  mqtt.connect.mockReturnValue(client);

  render(<MqttProvider><Probe /></MqttProvider>);
  fireEvent.click(screen.getByRole('button', { name: 'send' }));
  expect(client.publish).toHaveBeenCalledWith('cmd/test', 'hi');
});

test('closes the client on unmount', () => {
  const client = makeFakeClient();
  mqtt.connect.mockReturnValue(client);

  const { unmount } = render(<MqttProvider><Probe /></MqttProvider>);
  unmount();
  expect(client.end).toHaveBeenCalledWith(true);
});

test('subscribe only calls client.subscribe once per topic across multiple subscribers', () => {
  const client = makeFakeClient();
  mqtt.connect.mockReturnValue(client);

  let ctx;
  function Capture() { ctx = useMqttContext(); return null; }
  render(<MqttProvider><Capture /></MqttProvider>);

  const unsubA = ctx.subscribe('ros2/odom', () => {});
  const unsubB = ctx.subscribe('ros2/odom', () => {});
  expect(client.subscribe).toHaveBeenCalledTimes(1);

  unsubA();
  expect(client.unsubscribe).not.toHaveBeenCalled();
  unsubB();
  expect(client.unsubscribe).toHaveBeenCalledWith('ros2/odom');
});

test('message events are dispatched only to callbacks subscribed on that topic', () => {
  const client = makeFakeClient();
  mqtt.connect.mockReturnValue(client);

  let ctx;
  function Capture() { ctx = useMqttContext(); return null; }
  render(<MqttProvider><Capture /></MqttProvider>);

  const onA = jest.fn();
  const onB = jest.fn();
  ctx.subscribe('ros2/a', onA);
  ctx.subscribe('ros2/b', onB);

  act(() => { client._handlers.message('ros2/a', Buffer.from('hello')); });
  expect(onA).toHaveBeenCalledWith('hello');
  expect(onB).not.toHaveBeenCalled();
});
