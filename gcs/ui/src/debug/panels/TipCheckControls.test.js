import { render, screen, fireEvent } from '@testing-library/react';
import TipCheckControls from './TipCheckControls';

function respond(data, status = 200) {
  return Promise.resolve({ ok: status < 300, status, json: () => Promise.resolve(data) });
}

function mockService(check) {
  global.fetch = jest.fn((url, opts = {}) => {
    const path = url.replace(/^https?:\/\/[^/]+/, '');
    if ((opts.method || 'GET') === 'GET' && path === '/health') {
      return respond({ ok: true, camera_present: true, model_reachable: true, busy: false });
    }
    if (opts.method === 'POST' && path === '/check') return check(opts);
    return respond({ error: 'not found' }, 404);
  });
}

afterEach(() => {
  delete global.fetch;
});

test('runs a check and shows the verdict, probability and images', async () => {
  mockService(() => respond({
    clip_id: 't1', status: 'done', anomaly_detected: true, p_tip_empty: 0.66,
    summary: 'Tip looks empty', timings_s: { total: 2.9 },
    files: { crop: '/runs/t1/crop.jpg', preview: '/runs/t1/preview.jpg' },
  }));
  render(<TipCheckControls />);
  expect(await screen.findByText(/L515: ok/)).toBeInTheDocument();
  fireEvent.click(screen.getByRole('button', { name: 'Run clog check' }));
  expect(await screen.findByText('Clog suspected')).toBeInTheDocument();
  expect(screen.getByText('P(tip empty) = 0.66')).toBeInTheDocument();
  expect(screen.getByAltText('Tip crop sent to the model')).toHaveAttribute('src', expect.stringMatching(/\/runs\/t1\/crop\.jpg$/));
  const post = global.fetch.mock.calls.find(([, o]) => o?.method === 'POST');
  expect(JSON.parse(post[1].body)).toEqual({ step: 'debug tip check' });
});

test('shows a warning when the tip is not clearly visible', async () => {
  mockService(() => respond({
    clip_id: 't2', status: 'done', anomaly_detected: false, p_tip_empty: 0.4,
    warning: 'Tip not clearly visible in the crop (P=0.51); check the tip region',
  }));
  render(<TipCheckControls />);
  fireEvent.click(screen.getByRole('button', { name: 'Run clog check' }));
  expect(await screen.findByText(/Tip not clearly visible/)).toBeInTheDocument();
  expect(screen.getByText('OK')).toBeInTheDocument();
});

test('shows the service error', async () => {
  mockService(() => respond({ error: 'a check is already running' }, 409));
  render(<TipCheckControls />);
  fireEvent.click(screen.getByRole('button', { name: 'Run clog check' }));
  expect(await screen.findByText('Error: a check is already running')).toBeInTheDocument();
});
