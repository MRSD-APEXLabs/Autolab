import { render, screen } from '@testing-library/react';
import ClogCheckCard from './ClogCheckCard';

function mockFetch(byId) {
  global.fetch = jest.fn((url) => {
    const id = url.split('/result/')[1];
    const r = byId[id];
    if (r instanceof Error) return Promise.reject(r);
    return Promise.resolve({ ok: true, status: 200, json: () => Promise.resolve(r) });
  });
}

afterEach(() => {
  delete global.fetch;
});

test('renders one row per clip with its verdict', async () => {
  mockFetch({
    c1: { clip_id: 'c1', status: 'done', anomaly_detected: false },
    c2: { clip_id: 'c2', status: 'done', anomaly_detected: true, severity: 'high' },
    c3: { clip_id: 'c3', status: 'running', stage: 'stage2' },
  });
  render(<ClogCheckCard clipIds={['c1', 'c2', 'c3']} />);
  expect(await screen.findByText('OK')).toBeInTheDocument();
  expect(await screen.findByText(/Clog suspected · high/)).toBeInTheDocument();
  expect(await screen.findByText(/Analyzing… · stage2/)).toBeInTheDocument();
  expect(global.fetch).toHaveBeenCalledWith(expect.stringContaining('/result/c1'), expect.anything());
});

test('unreachable service shows no result', async () => {
  mockFetch({ c1: new TypeError('Failed to fetch') });
  render(<ClogCheckCard clipIds={['c1']} />);
  expect(await screen.findByText(/Can't reach the clog-detection service/)).toBeInTheDocument();
  expect(screen.getByText('No result')).toBeInTheDocument();
  expect(screen.queryByText('OK')).not.toBeInTheDocument();
});
