import { render, screen, fireEvent, act, waitFor } from '@testing-library/react';
import Ot2ClogSequence from './Ot2ClogSequence';
import { COMMAND_TOPIC, STATUS_TOPIC } from './Ot2PartsControls';
import { useMqttContext } from '../../shared/useMqtt';

jest.mock('../../shared/useMqtt');

function respond(data, status = 200) {
  return Promise.resolve({ ok: status < 300, status, json: () => Promise.resolve(data) });
}

const OK_JOB = { clip_id: 't1', status: 'done', anomaly_detected: false, p_tip_empty: 0.07, summary: 'Liquid in tip' };
const PART1 = { part: 1, tip_index: 2, volume: 200 };

function setup({ check = () => respond(OK_JOB), initial = { status: 'completed', tip_index: 1 } } = {}) {
  const publish = jest.fn();
  let receive;
  useMqttContext.mockReturnValue({
    connected: true,
    publish,
    subscribe: (topic, callback) => {
      expect(topic).toBe(STATUS_TOPIC);
      receive = callback;
      callback(JSON.stringify(initial));
      return () => {};
    },
  });
  global.fetch = jest.fn((url, opts) => check(url, opts));
  render(<Ot2ClogSequence />);
  const update = (status) => act(() => receive(JSON.stringify(status)));
  const sent = () => publish.mock.calls
    .filter(([topic]) => topic === COMMAND_TOPIC)
    .map(([, payload]) => JSON.parse(payload));
  const commands = () => sent().filter((p) => p.part !== 'status');
  const run = async () => {
    fireEvent.click(screen.getByRole('button', { name: /Run Part 1/ }));
    fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));
    await waitFor(() => expect(commands()).toEqual([PART1]));
  };
  return { update, sent, commands, run };
}

afterEach(() => {
  delete global.fetch;
});

test('runs part 1, the clog check, then part 2', async () => {
  const { update, commands, run } = setup();
  expect(screen.getByLabelText(/Sequence tip index/)).toHaveValue(2);  // the tip after the last one used
  await run();
  expect(global.fetch).not.toHaveBeenCalled();                          // no check before part 1 holds
  update({ status: 'running' });
  update({ status: 'ready_for_part2' });
  await waitFor(() => expect(commands()).toEqual([PART1, { part: 2 }]));
  const [url, opts] = global.fetch.mock.calls[0];
  expect(url).toMatch(/\/check$/);
  expect(opts.method).toBe('POST');
  update({ status: 'running' });
  update({ status: 'completed' });
  expect(await screen.findByText(/Sequence finished. Tip held liquid/)).toBeInTheDocument();
  expect(screen.getByText('OK')).toBeInTheDocument();
});

test('a one-off needs_inspection from a half-written state file is not a failure', async () => {
  const { update, sent, commands, run } = setup();
  await run();
  update({ status: 'running' });
  update({ status: 'needs_inspection', error: 'Expecting value: line 1 column 1 (char 0)' });
  expect(sent().filter((p) => p.part === 'status').length).toBeGreaterThanOrEqual(2);  // asked again
  update({ status: 'running' });
  update({ status: 'ready_for_part2' });
  await waitFor(() => expect(commands()).toEqual([PART1, { part: 2 }]));
  expect(screen.queryByRole('alert')).not.toBeInTheDocument();
});

test('a repeated part 1 failure stops the sequence before the check', async () => {
  const { update, commands, run } = setup();
  await run();
  update({ status: 'running' });
  for (let i = 0; i < 3; i += 1) update({ status: 'needs_inspection', error: 'Stopped: HTTP 500' });
  expect(await screen.findByText('Stopped: HTTP 500')).toBeInTheDocument();
  expect(global.fetch).not.toHaveBeenCalled();
  expect(commands()).toEqual([PART1]);
});

test('can stop before part 2 when a clog is suspected', async () => {
  const { update, commands, run } = setup({
    check: () => respond({ ...OK_JOB, anomaly_detected: true, p_tip_empty: 0.7 }),
  });
  fireEvent.click(screen.getByLabelText(/Stop before Part 2/));
  await run();
  update({ status: 'running' });
  update({ status: 'ready_for_part2' });
  expect(await screen.findByText(/stopped before Part 2/)).toBeInTheDocument();
  expect(screen.getByText('Clog suspected')).toBeInTheDocument();
  expect(commands()).toEqual([PART1]);
});

test('a failed check leaves the tip held and does not run part 2', async () => {
  const { update, commands, run } = setup({ check: () => respond({ error: 'camera not readable' }, 500) });
  await run();
  update({ status: 'running' });
  update({ status: 'ready_for_part2' });
  expect(await screen.findByText(/Clog check failed: camera not readable/)).toBeInTheDocument();
  expect(commands()).toEqual([PART1]);
});

test('cannot start while the OT-2 is mid-cycle', () => {
  setup({ initial: { status: 'ready_for_part2' } });
  expect(screen.getByRole('button', { name: /Run Part 1/ })).toBeDisabled();
});
