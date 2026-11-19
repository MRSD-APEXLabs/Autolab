import { act, render, screen, fireEvent } from '@testing-library/react';
import JogControls, { JOG_TOPIC, JOG_RESULT_TOPIC } from './JogControls';
import { useMqttContext } from '../../shared/useMqtt';

jest.mock('../../shared/useMqtt');

function mockMqtt() {
  const handlers = {};
  const publish = jest.fn();
  useMqttContext.mockReturnValue({
    publish,
    connected: true,
    subscribe: (topic, callback) => { handlers[topic] = callback; return () => {}; },
  });
  return { publish, handlers };
}

test('every jog button is present', () => {
  mockMqtt();
  render(<JogControls />);
  ['X+', 'X-', 'Y+', 'Y-', 'Z+', 'Z-', 'RZ+', 'RZ-', 'Home', 'Clear errors'].forEach((label) => {
    expect(screen.getByRole('button', { name: label })).toBeInTheDocument();
  });
});

test.each([['Z+', 'z_plus'], ['RZ-', 'rz_minus'], ['Home', 'home'], ['Clear errors', 'clear_errors']])(
  '%s publishes %s as JSON on one click, without a confirm step', (label, value) => {
    const { publish } = mockMqtt();
    render(<JogControls />);
    fireEvent.click(screen.getByRole('button', { name: label }));
    expect(publish).toHaveBeenCalledWith(JOG_TOPIC, JSON.stringify(value));
    expect(screen.queryByRole('button', { name: 'Confirm?' })).not.toBeInTheDocument();
  });

test('disables the buttons while a jog runs, then shows the result', () => {
  const { handlers } = mockMqtt();
  render(<JogControls />);
  fireEvent.click(screen.getByRole('button', { name: 'X-' }));
  expect(screen.getByText('Jogging…')).toBeInTheDocument();
  expect(screen.getByRole('button', { name: 'Y+' })).toBeDisabled();

  act(() => handlers[JOG_RESULT_TOPIC](JSON.stringify({ jog: 'x_minus', ok: false, message: 'not jogging: busy' })));
  expect(screen.getByText('not jogging: busy')).toBeInTheDocument();
  expect(screen.getByRole('button', { name: 'Y+' })).toBeEnabled();
});
