import { render, screen, fireEvent, act } from '@testing-library/react';
import Ot2LightsControls, { LIGHT_COMMAND_TOPIC, LIGHT_STATUS_TOPIC } from './Ot2LightsControls';
import { useMqttContext } from '../../shared/useMqtt';

jest.mock('../../shared/useMqtt');

function setup(connected = true) {
  let receive;
  const publish = jest.fn();
  useMqttContext.mockReturnValue({ connected, publish, subscribe: (topic, callback) => {
    expect(topic).toBe(LIGHT_STATUS_TOPIC); receive = callback; return () => {};
  } });
  render(<Ot2LightsControls />);
  return { publish, update: (body) => act(() => receive(JSON.stringify(body))) };
}

test('reads initial state and sends on/off commands with readback feedback', () => {
  const { publish, update } = setup();
  expect(publish).toHaveBeenCalledWith(LIGHT_COMMAND_TOPIC, JSON.stringify({ action: 'get' }));
  update({ on: false, busy: false });
  fireEvent.click(screen.getByRole('button', { name: 'Lights on' }));
  expect(publish).toHaveBeenCalledWith(LIGHT_COMMAND_TOPIC, JSON.stringify({ action: 'set', on: true }));
  expect(screen.getByRole('button', { name: 'Lights off' })).toBeDisabled();
  update({ on: true, busy: false });
  expect(screen.getByText('Rail lights: on')).toBeInTheDocument();
  fireEvent.click(screen.getByRole('button', { name: 'Lights off' }));
  expect(publish).toHaveBeenCalledWith(LIGHT_COMMAND_TOPIC, JSON.stringify({ action: 'set', on: false }));
});

test('disconnected controls are disabled', () => {
  setup(false);
  expect(screen.getByRole('button', { name: 'Lights on' })).toBeDisabled();
  expect(screen.getByRole('button', { name: 'Lights off' })).toBeDisabled();
});
