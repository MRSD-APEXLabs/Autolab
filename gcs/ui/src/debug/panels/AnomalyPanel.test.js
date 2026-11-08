import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import AnomalyPanel from './AnomalyPanel';

function respond(data, status = 200) {
  return Promise.resolve({ ok: status < 300, status, json: () => Promise.resolve(data) });
}

function mockService(overrides = {}) {
  global.fetch = jest.fn((url, opts = {}) => {
    const path = url.replace(/^https?:\/\/[^/]+/, '');
    const key = `${opts.method || 'GET'} ${path}`;
    if (overrides[key]) return overrides[key](opts);
    if (key === 'GET /health') return respond({ ok: true, camera_present: true, recording: null, running: null, queued: 0 });
    if (key === 'GET /jobs') return respond([
      { clip_id: 'c1', status: 'done', anomaly_detected: true, severity: 'low', summary: 'tip empty', step: 'debug replay' },
    ]);
    return respond({ clip_id: 'c2', status: 'recording' });
  });
}

afterEach(() => {
  delete global.fetch;
});

test('renders and polls /jobs', async () => {
  mockService();
  render(<AnomalyPanel />);
  expect(screen.getByText('Clog Detection')).toBeInTheDocument();
  expect(await screen.findByText('c1')).toBeInTheDocument();
  expect(screen.getByText(/Clog suspected · low/)).toBeInTheDocument();
  fireEvent.click(screen.getByText('c1'));
  expect(screen.getByText('tip empty')).toBeInTheDocument();
});

test('Run replay posts start with replay_file then stop', async () => {
  mockService();
  render(<AnomalyPanel />);
  await screen.findByText(/Camera: present/);
  fireEvent.change(screen.getByLabelText('Replay video path'), { target: { value: '/data/clip.mp4' } });
  fireEvent.click(screen.getByText('Run replay'));
  await waitFor(() => {
    const posts = global.fetch.mock.calls.filter(([, o]) => o?.method === 'POST');
    expect(posts.map(([u]) => u.replace(/^https?:\/\/[^/]+/, ''))).toEqual(['/recording/start', '/recording/stop']);
    expect(JSON.parse(posts[0][1].body)).toEqual({ replay_file: '/data/clip.mp4', step: 'debug replay' });
  });
});

test('shows service error on start', async () => {
  mockService({ 'POST /recording/start': () => respond({ error: 'already recording c9' }, 409) });
  render(<AnomalyPanel />);
  await screen.findByText(/Camera: present/);
  fireEvent.click(screen.getByText('Start recording'));
  expect(await screen.findByText('Error: already recording c9')).toBeInTheDocument();
});
