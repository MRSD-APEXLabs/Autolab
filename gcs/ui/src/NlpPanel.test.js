import { StrictMode, useState } from 'react';
import { act, fireEvent, render, screen, waitFor } from '@testing-library/react';
import RoutineBuilder from './RoutineBuilder';
import { DEMO_PROMPTS } from './NlpPanel';

// The planner's 11-step answer (gcs/nlp_proxy/mix_colors.py expected()).
const plan = (pair) => ({
  steps: [
    { name: 'go_to', target_machine: 'ot2' },
    { name: 'pick_base' },
    { name: 'place', target_machine: 'ot2' },
    { name: 'ot2', parameters_json: { steps: pair.map((l) => ({ action: 'transfer', source: `${l}_tube`, destination: 'well_plate', volume_ul: 200 })) } },
    { name: 'pick', target_machine: 'ot2' },
    { name: 'go_to', target_machine: 'shaker' },
    { name: 'place', target_machine: 'shaker' },
    { name: 'shaker', pwm: 150, wait_time_s: 20 },
    { name: 'pick', target_machine: 'shaker' },
    { name: 'place_base' },
    { name: 'go_to', target_machine: 'home' },
  ],
});

const START = JSON.stringify({ steps: [{ name: 'home' }], max_retries: 2, robot: 'robot_1' }, null, 2);

function Harness({ initial = START }) {
  const [value, setValue] = useState(initial);
  return (
    <>
      <RoutineBuilder value={value} onChange={setValue} />
      <output data-testid="json">{value}</output>
    </>
  );
}

const reply = (body, ok = true, status = 200) => Promise.resolve({ ok, status, json: () => Promise.resolve(body) });

// fetch: /api/nlp/status answers `status`; POST /api/nlp answers `answer` (a function or a value).
function mockFetch({ answer, status = { state: 'ready', detail: '', model: 'qwen3.5:9b' } } = {}) {
  global.fetch = jest.fn((url) => {
    if (url === '/api/nlp/status') return status instanceof Error ? Promise.reject(status) : reply(status);
    return typeof answer === 'function' ? answer() : reply(answer);
  });
}
const planCalls = () => global.fetch.mock.calls.filter(([url]) => url === '/api/nlp');

function build(text) {
  fireEvent.change(screen.getByLabelText('Request'), { target: { value: text } });
  fireEvent.click(screen.getByRole('button', { name: 'Build' }));
}

afterEach(() => { delete global.fetch; });

test('a live routine replaces the blocks and says LIVE', async () => {
  mockFetch({ answer: { ok: true, source: 'live', seconds: 1.1, plan: plan(['red', 'yellow']) } });
  render(<Harness />);
  build('Mix red and yellow and shake.');
  expect(await screen.findByText(/Built 11 steps/)).toHaveTextContent('LIVE · 1.1 s');
  expect(screen.getAllByTestId('routine-block')).toHaveLength(11);
  const routine = JSON.parse(screen.getByTestId('json').textContent);
  expect(routine.max_retries).toBe(2);
  expect(JSON.parse(planCalls()[0][1].body)).toEqual({ task: 'Mix red and yellow and shake.' });
});

test('a recorded answer is labelled RECORDED, not LIVE', async () => {
  mockFetch({ answer: { ok: true, source: 'recorded', why: 'down', recordedAt: 'x', plan: plan(['red', 'yellow']) } });
  render(<Harness />);
  build('Mix red and yellow and shake.');
  const line = await screen.findByText(/Built 11 steps/);
  expect(line).toHaveTextContent('RECORDED (planner offline)');
  expect(line).not.toHaveTextContent('LIVE');
});

test('a refusal shows the reason and keeps the routine', async () => {
  mockFetch({ answer: { ok: true, source: 'live', seconds: 0.9, plan: { error: 'Specify two different liquids to mix: red, yellow or blue.' } } });
  render(<Harness />);
  build('Mix yellow and green.');
  expect(await screen.findByText(/The planner refused: Specify two different liquids/)).toBeInTheDocument();
  expect(screen.getByTestId('json').textContent).toBe(START);
});

test('a planner error keeps the routine', async () => {
  mockFetch({ answer: { ok: false, error: 'the model did not answer within 20 s' } });
  render(<Harness />);
  build('Mix red and yellow.');
  expect(await screen.findByText('Planner unavailable: the model did not answer within 20 s')).toBeInTheDocument();
  expect(screen.getByTestId('json').textContent).toBe(START);
});

test('an HTML 502 is reported, not thrown', async () => {
  mockFetch({ answer: () => Promise.resolve({ ok: false, status: 502, json: () => Promise.reject(new SyntaxError('Unexpected token <')) }) });
  render(<Harness />);
  build('Mix red and yellow.');
  expect(await screen.findByText(/Planner unavailable: .*502/)).toBeInTheDocument();
  expect(screen.getByTestId('json').textContent).toBe(START);
});

test('a chip fills the box and sends its prompt', async () => {
  mockFetch({ answer: { ok: true, source: 'recorded', why: 'down', plan: plan(['yellow', 'red']) } });
  render(<Harness />);
  fireEvent.click(screen.getByRole('button', { name: DEMO_PROMPTS[1].label }));
  expect(screen.getByLabelText('Request')).toHaveValue(DEMO_PROMPTS[1].task);
  await screen.findByText(/Built 11 steps/);
  expect(JSON.parse(planCalls()[0][1].body).task).toBe(DEMO_PROMPTS[1].task);
});

test('Build is disabled for an empty box and while planning', async () => {
  let finish;
  mockFetch({ answer: () => new Promise((resolve) => { finish = () => resolve({ ok: true, status: 200, json: () => Promise.resolve({ ok: true, source: 'live', seconds: 1, plan: plan(['red', 'yellow']) }) }); }) });
  render(<Harness />);
  expect(screen.getByRole('button', { name: 'Build' })).toBeDisabled();
  build('Mix red and yellow.');
  expect(screen.getByRole('button', { name: 'Build' })).toBeDisabled();
  fireEvent.click(screen.getByRole('button', { name: 'Build' }));
  fireEvent.click(screen.getByRole('button', { name: DEMO_PROMPTS[0].label }));
  fireEvent.submit(screen.getByLabelText('Request'));
  expect(screen.getByText('Planning…')).toBeInTheDocument();
  expect(planCalls()).toHaveLength(1);
  await act(async () => finish());
  expect(await screen.findByText(/Built 11 steps/)).toBeInTheDocument();
});

test('a routine with blue liquid notes that its tube is missing, so the clog check should flag it', async () => {
  mockFetch({ answer: { ok: true, source: 'live', seconds: 1, plan: plan(['blue', 'yellow']) } });
  render(<Harness />);
  build('Mix blue and yellow.');
  expect(await screen.findByText(/There is no blue tube on the deck/)).toBeInTheDocument();
});

test('a routine without blue liquid has no missing-tube note', async () => {
  mockFetch({ answer: { ok: true, source: 'live', seconds: 1, plan: plan(['red', 'yellow']) } });
  render(<Harness />);
  build('Mix red and yellow.');
  await screen.findByText(/Built 11 steps/);
  expect(screen.queryByText(/no blue tube/)).not.toBeInTheDocument();
});

test('building from invalid JSON still gives the default retries and robot', async () => {
  mockFetch({ answer: { ok: true, source: 'live', seconds: 1, plan: plan(['red', 'yellow']) } });
  render(<Harness initial="{" />);
  build('Mix red and yellow.');
  await screen.findByText(/Built 11 steps/);
  const routine = JSON.parse(screen.getByTestId('json').textContent);
  expect(routine.max_retries).toBe(3);
  expect(routine.robot).toBe('robot_1');
  expect(screen.getAllByTestId('routine-block')).toHaveLength(11);
});

test('status shows ready, or offline with the reason', async () => {
  mockFetch();
  const { unmount } = render(<Harness />);
  expect(await screen.findByText('Planner ready')).toBeInTheDocument();
  unmount();
  mockFetch({ status: { state: 'down', detail: 'Connection refused', model: 'qwen3.5:9b' } });
  render(<Harness />);
  expect(await screen.findByText('Planner offline')).toHaveAttribute('title', 'Connection refused');
});

test('status is offline when the proxy cannot be reached', async () => {
  mockFetch({ status: new TypeError('Failed to fetch') });
  render(<Harness />);
  await waitFor(() => expect(screen.getByText('Planner offline')).toBeInTheDocument());
});

test('works under StrictMode (dev server): the answer is applied', async () => {
  mockFetch({ answer: { ok: true, source: 'live', seconds: 1, plan: plan(['red', 'yellow']) } });
  render(<StrictMode><Harness /></StrictMode>);
  build('Mix red and yellow.');
  expect(await screen.findByText(/Built 11 steps/)).toBeInTheDocument();
  expect(screen.queryByText('Planning…')).not.toBeInTheDocument();
});
