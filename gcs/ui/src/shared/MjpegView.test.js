import { TextEncoder } from 'util';
import { render, act, screen } from '@testing-library/react';
import MjpegView from './MjpegView';

// A fetch whose body hands out chunks we push by hand, and that rejects when aborted.
function fakeStream() {
  const waiting = [];
  const queued = [];
  let aborted = false;
  const reader = {
    read: () => new Promise((resolve, reject) => {
      if (aborted) { reject(new DOMException('aborted', 'AbortError')); return; }
      if (queued.length) resolve(queued.shift());
      else waiting.push({ resolve, reject });
    }),
  };
  return {
    response: { ok: true, body: { getReader: () => reader } },
    push(bytes) {
      const chunk = { value: bytes, done: false };
      if (waiting.length) waiting.shift().resolve(chunk);
      else queued.push(chunk);
    },
    abort() { aborted = true; waiting.splice(0).forEach((w) => w.reject(new DOMException('aborted', 'AbortError'))); },
    get aborted() { return aborted; },
  };
}

const enc = (s) => new TextEncoder().encode(s);
const frame = () => {
  const jpg = new Uint8Array([0xff, 0xd8, 1, 0xff, 0xd9]);
  const head = enc(`--frame\r\nContent-Length: ${jpg.length}\r\n\r\n`);
  const out = new Uint8Array(head.length + jpg.length + 2);
  out.set(head); out.set(jpg, head.length); out.set(enc('\r\n'), head.length + jpg.length);
  return out;
};

let streams;
beforeEach(() => {
  jest.useFakeTimers();
  streams = [];
  global.fetch = jest.fn((url, { signal }) => {
    const s = fakeStream();
    signal.addEventListener('abort', () => s.abort());
    streams.push(s);
    return Promise.resolve(s.response);
  });
  global.URL.createObjectURL = jest.fn(() => `blob:${Math.random()}`);
  global.URL.revokeObjectURL = jest.fn();
});
afterEach(() => jest.useRealTimers());

const flush = async () => { await act(async () => { for (let i = 0; i < 5; i++) await Promise.resolve(); }); };
const advance = async (ms) => { await act(async () => { jest.advanceTimersByTime(ms); }); await flush(); };

test('reads the stream with fetch and shows frames', async () => {
  render(<MjpegView url="http://cam/stream/ot2.mjpg" alt="OT-2" />);
  await flush();
  expect(fetch).toHaveBeenCalledWith('http://cam/stream/ot2.mjpg', expect.objectContaining({ cache: 'no-store' }));
  streams[0].push(frame());
  await flush();
  expect(screen.getByAltText('OT-2').getAttribute('src')).toMatch(/^blob:/);
  expect(screen.queryByText(/reconnecting/i)).not.toBeInTheDocument();
});

test('a stream that stops sending frames is dropped and reopened', async () => {
  render(<MjpegView url="http://cam/stream/shaker.mjpg" alt="Shaker" />);
  await flush();
  streams[0].push(frame());
  await flush();
  await advance(3500);                 // no frames for longer than the stall limit
  expect(streams[0].aborted).toBe(true);
  expect(screen.getByText(/reconnecting/i)).toBeInTheDocument();
  await advance(1000);                 // backoff
  expect(fetch).toHaveBeenCalledTimes(2);
  streams[1].push(frame());
  await flush();
  expect(screen.queryByText(/reconnecting/i)).not.toBeInTheDocument();
});

test('a stream the server ends is reopened', async () => {
  global.fetch.mockImplementationOnce(() => Promise.resolve({
    ok: true, body: { getReader: () => ({ read: () => Promise.resolve({ done: true }) }) },
  }));
  render(<MjpegView url="http://cam/stream/home.mjpg" alt="Home" />);
  await flush();
  await advance(1000);
  expect(fetch).toHaveBeenCalledTimes(2);
});

test('unmounting closes the stream', async () => {
  const { unmount } = render(<MjpegView url="http://cam/stream/ot2.mjpg" alt="OT-2" />);
  await flush();
  unmount();
  expect(streams[0].aborted).toBe(true);
});

test('falls back to a fresh <img> when fetch is refused (no CORS)', async () => {
  global.fetch = jest.fn(() => Promise.reject(new TypeError('Failed to fetch')));
  render(<MjpegView url="http://cam/stream/ot2.mjpg" alt="OT-2" />);
  for (let i = 0; i < 4; i++) await advance(5000);
  const src = screen.getByAltText('OT-2').getAttribute('src');
  expect(src).toMatch(/^http:\/\/cam\/stream\/ot2\.mjpg\?s=/);
});
test('reads from fetchUrl when given, and the fallback <img> uses the plain url', async () => {
  render(<MjpegView url="http://cam/stream/ot2.mjpg" fetchUrl="http://cam/stream/ot2.mjpg?fetch=1" alt="OT-2" />);
  await flush();
  expect(fetch.mock.calls[0][0]).toBe('http://cam/stream/ot2.mjpg?fetch=1');
});

test('a response whose body cannot be read (Safari + multipart) also falls back to <img>', async () => {
  global.fetch = jest.fn(() => Promise.resolve({
    ok: true, body: { getReader: () => ({ read: () => Promise.reject(new TypeError('Load failed')) }) },
  }));
  render(<MjpegView url="http://cam/stream/ot2.mjpg" alt="OT-2" />);
  for (let i = 0; i < 4; i++) await advance(5000);
  expect(screen.getByAltText('OT-2').getAttribute('src')).toMatch(/^http:\/\/cam\/stream\/ot2\.mjpg\?s=/);
});

test('after the fallback <img> errors, fetch is tried again', async () => {
  global.fetch = jest.fn(() => Promise.reject(new TypeError('Failed to fetch')));
  render(<MjpegView url="http://cam/stream/ot2.mjpg" alt="OT-2" />);
  for (let i = 0; i < 4; i++) await advance(5000);
  const calls = fetch.mock.calls.length;
  act(() => { screen.getByAltText('OT-2').dispatchEvent(new Event('error')); });
  await advance(3000);
  expect(fetch.mock.calls.length).toBeGreaterThan(calls);
});
