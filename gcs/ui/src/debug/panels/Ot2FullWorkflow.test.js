import { render, screen, fireEvent, act, waitFor } from '@testing-library/react';
import Ot2ClogSequence from './Ot2ClogSequence';
import { COMMAND_TOPIC } from './Ot2PartsControls';
import { useMqttContext } from '../../shared/useMqtt';

jest.mock('../../shared/useMqtt');
const FIRST = { part: 1, tip_index: 0, volume: 200, source_tube: 'A1' };
const SECOND = { part: 1, tip_index: 1, volume: 200, source_tube: 'A2' };
const DONE = { clip_id: 'check2', status: 'done', anomaly_detected: false, p_tip_empty: 0.2 };
const response = (data, status = 200) => ({ ok: status < 300, status, json: async () => data });

function setup({ initial = 'completed', locked = false } = {}) {
  let receive;
  const publish = jest.fn();
  useMqttContext.mockReturnValue({ connected: true, publish, subscribe: (topic, callback) => {
    receive = callback; callback(JSON.stringify({ status: initial })); return () => {};
  } });
  render(<Ot2ClogSequence fullWorkflow locked={locked} />);
  const commands = () => publish.mock.calls.filter(([topic]) => topic === COMMAND_TOPIC)
    .map(([, payload]) => JSON.parse(payload)).filter((body) => body.part !== 'status');
  const update = (status) => act(() => receive(JSON.stringify({ status })));
  const finish = async (part, count) => {
    await act(async () => {
      receive(JSON.stringify({ status: 'running' }));
      receive(JSON.stringify({ status: part === 1 ? 'ready_for_part2' : 'completed' }));
    });
    if (count) await waitFor(() => expect(commands()).toHaveLength(count));
  };
  const start = async () => {
    fireEvent.click(screen.getByRole('button', { name: 'Run full OT-2 workflow' }));
    expect(commands()).toEqual([]);
    fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));
    await waitFor(() => expect(commands()).toEqual([FIRST]));
  };
  return { start, finish, update, commands };
}

afterEach(() => { delete global.fetch; });

test('runs both transfers in order and waits for detection before the final dispense', async () => {
  const resolveChecks = [];
  global.fetch = jest.fn(() => new Promise((resolve) => { resolveChecks.push(resolve); }));
  const flow = setup();
  await flow.start();
  expect(global.fetch).not.toHaveBeenCalled();
  await flow.finish(1, 2);
  expect(flow.commands()).toEqual([FIRST, { part: 2 }]);
  expect(global.fetch).not.toHaveBeenCalled();
  await flow.finish(2, 3);
  expect(flow.commands()).toEqual([FIRST, { part: 2 }, SECOND]);
  await flow.finish(1);
  await waitFor(() => expect(global.fetch).toHaveBeenCalledTimes(1));
  expect(flow.commands()).toHaveLength(3);
  for (let i = 0; i < 5; i += 1) {
    await waitFor(() => expect(global.fetch).toHaveBeenCalledTimes(i + 1));
    expect(flow.commands()).toHaveLength(3);
    await act(async () => resolveChecks[i](response(DONE)));
  }
  await waitFor(() => expect(flow.commands()).toEqual([FIRST, { part: 2 }, SECOND, { part: 2 }]));
  await flow.finish(2);
  expect(await screen.findByText(/Sequence finished/)).toBeInTheDocument();
});

test('detection failure leaves the second tip held without dispensing', async () => {
  global.fetch = jest.fn(async () => response({ error: 'Camera unavailable' }, 500));
  const flow = setup();
  await flow.start(); await flow.finish(1, 2); await flow.finish(2, 3); await flow.finish(1);
  expect(await screen.findByRole('alert')).toHaveTextContent('Camera unavailable');
  expect(flow.commands()).toEqual([FIRST, { part: 2 }, SECOND]);
});

test('failed first transfer never starts the second transfer or detection', async () => {
  global.fetch = jest.fn();
  const flow = setup(); await flow.start();
  flow.update('running');
  flow.update('needs_inspection'); flow.update('needs_inspection'); flow.update('needs_inspection');
  expect(await screen.findByRole('alert')).toBeInTheDocument();
  expect(flow.commands()).toEqual([FIRST]);
  expect(global.fetch).not.toHaveBeenCalled();
});

test('full workflow is disabled during an existing cycle or another sequence', () => {
  setup({ initial: 'ready_for_part2' });
  expect(screen.getByRole('button', { name: 'Run full OT-2 workflow' })).toBeDisabled();
});
