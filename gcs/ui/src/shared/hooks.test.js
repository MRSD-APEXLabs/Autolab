import React from 'react';
import { render, screen, act } from '@testing-library/react';
import { useMqttContext } from './useMqtt';
import useCameraHub from './useCameraHub';
import usePerception from './usePerception';

jest.mock('./useMqtt');

let handlers;
beforeEach(() => {
  handlers = {};
  useMqttContext.mockReturnValue({
    subscribe: (topic, cb) => { handlers[topic] = cb; return () => { delete handlers[topic]; }; },
  });
});
const bridged = (obj) => JSON.stringify({ data: JSON.stringify(obj) });

function HubProbe() {
  const hub = useCameraHub();
  return <div>{`${hub.active}|${hub.state}|${hub.reachable}`}</div>;
}

test('useCameraHub unwraps the bridged status and ignores garbage', () => {
  render(<HubProbe />);
  expect(screen.getByText('null|null|null')).toBeInTheDocument();
  act(() => handlers['ros2/camera_hub/status'](bridged({ active: 'zedx', state: 'streaming', reachable: true })));
  expect(screen.getByText('zedx|streaming|true')).toBeInTheDocument();
  act(() => handlers['ros2/camera_hub/status']('garbage'));
  expect(screen.getByText('zedx|streaming|true')).toBeInTheDocument();
});

function PerceptionProbe({ camera }) {
  const p = usePerception(camera);
  return <div>{`${p.received}|${p.tags.map((t) => t.id).join(',')}|${p.error}`}</div>;
}

test('usePerception follows the active camera topic', () => {
  const { rerender } = render(<PerceptionProbe camera="zedx" />);
  expect(Object.keys(handlers)).toEqual(['ros2/inspect/json']);
  act(() => handlers['ros2/inspect/json'](bridged({ apriltags: [{ id: 3, pose: null }] })));
  expect(screen.getByText('true|3|null')).toBeInTheDocument();
  rerender(<PerceptionProbe camera="zedx_nano" />);
  expect(Object.keys(handlers)).toEqual(['ros2/servo/json']);
  expect(screen.getByText('false||null')).toBeInTheDocument();   // old camera's data cleared
  rerender(<PerceptionProbe camera={null} />);
  expect(Object.keys(handlers)).toEqual([]);
});

test('usePerception reports unparseable payloads', () => {
  render(<PerceptionProbe camera="zedx" />);
  act(() => handlers['ros2/inspect/json']('not json'));
  expect(screen.getByText(/^true\|\|/)).toBeInTheDocument();
  expect(screen.queryByText(/\|null$/)).not.toBeInTheDocument();
});
