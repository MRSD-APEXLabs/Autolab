import { render, screen, fireEvent, act } from '@testing-library/react';
import Ot2PartsControls, { COMMAND_TOPIC, STATUS_TOPIC } from './Ot2PartsControls';
import { useMqttContext } from '../../shared/useMqtt';

jest.mock('../../shared/useMqtt');

function setup(initial = 'idle') {
  const publish = jest.fn();
  let receive;
  useMqttContext.mockReturnValue({ connected: true, publish, subscribe: (topic, callback) => {
    expect(topic).toBe(STATUS_TOPIC);
    receive = callback;
    callback(JSON.stringify({ status: initial }));
    return () => {};
  } });
  render(<Ot2PartsControls onActive={jest.fn()} />);
  return { publish, update: (status) => act(() => receive(JSON.stringify({ status }))) };
}

test('part 1 sends selected tip and requires confirmation', () => {
  const { publish } = setup();
  fireEvent.change(screen.getByLabelText(/Unused tip index/), { target: { value: '4' } });
  fireEvent.click(screen.getByRole('button', { name: /OT-2 Part 1/ }));
  expect(publish).not.toHaveBeenCalledWith(COMMAND_TOPIC, expect.stringContaining('tip_index'));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));
  expect(publish).toHaveBeenCalledWith(COMMAND_TOPIC, JSON.stringify({ part: 1, tip_index: 4, volume: 200, source_tube: 'A1' }));
});

test('part 2 only becomes available when liquid is held', () => {
  const { publish, update } = setup();
  expect(screen.getByRole('button', { name: /OT-2 Part 2/ })).toBeDisabled();
  update('ready_for_part2');
  expect(screen.getByRole('button', { name: /OT-2 Part 1/ })).toBeDisabled();
  fireEvent.click(screen.getByRole('button', { name: /OT-2 Part 2/ }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));
  expect(publish).toHaveBeenCalledWith(COMMAND_TOPIC, JSON.stringify({ part: 2 }));
});

test('busy and uncertain states disable both parts', () => {
  const { update } = setup('running');
  for (const stage of ['running', 'needs_inspection']) {
    update(stage);
    expect(screen.getByRole('button', { name: /OT-2 Part 1/ })).toBeDisabled();
    expect(screen.getByRole('button', { name: /OT-2 Part 2/ })).toBeDisabled();
  }
});


test.each(['A2', 'A3', 'A4'])('part 1 sends the selected %s source tube', (sourceTube) => {
  const { publish } = setup();
  fireEvent.change(screen.getByLabelText('Source test tube'), { target: { value: sourceTube } });
  fireEvent.click(screen.getByRole('button', { name: /OT-2 Part 1/ }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));
  expect(publish).toHaveBeenCalledWith(COMMAND_TOPIC,
    JSON.stringify({ part: 1, tip_index: 2, volume: 200, source_tube: sourceTube }));
});

test('explicit cleanup is available while a tip is held and sends its own command', () => {
  const { publish } = setup('ready_for_part2');
  fireEvent.click(screen.getByRole('button', { name: 'Home & discard tip' }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));
  expect(publish).toHaveBeenCalledWith(COMMAND_TOPIC, JSON.stringify({ part: 'cleanup' }));
});
