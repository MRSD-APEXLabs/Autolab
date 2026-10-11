import { render, screen, fireEvent, act } from '@testing-library/react';
import CameraFeed from './CameraFeed';
import CameraPair from './CameraPair';

beforeEach(() => {
  jest.useFakeTimers();
  global.fetch = jest.fn();
});
afterEach(() => jest.useRealTimers());

const health = (key, age) => Promise.resolve({ json: () => Promise.resolve({ [key]: { last_frame_age_s: age } }) });

test('streams the given kind and camera', () => {
  global.fetch.mockReturnValue(health('annotated/zedx', 0.1));
  render(<CameraFeed camera="zedx" kind="annotated" title="Annotated" />);
  expect(screen.getByRole('img').getAttribute('src')).toBe('http://localhost:8002/stream/annotated/zedx.mjpg');
});

test('placeholders make no stream request', () => {
  const { rerender } = render(<CameraFeed camera={null} kind="raw" title="Raw" />);
  expect(screen.getByText('No camera selected')).toBeInTheDocument();
  rerender(<CameraFeed camera="zedx" kind="raw" title="Raw" hubReachable={false} />);
  expect(screen.getByText('Camera hub unreachable')).toBeInTheDocument();
  expect(screen.queryByRole('img')).not.toBeInTheDocument();
  act(() => { jest.advanceTimersByTime(3000); });
  expect(global.fetch).not.toHaveBeenCalled();
});

test('stream error shows retry, which reloads with a cache-busting query', () => {
  global.fetch.mockReturnValue(health('raw/zedx', 0.1));
  render(<CameraFeed camera="zedx" kind="raw" title="Raw" />);
  fireEvent.error(screen.getByRole('img'));
  expect(screen.getByText('Stream unavailable')).toBeInTheDocument();
  fireEvent.click(screen.getByRole('button', { name: 'Retry' }));
  expect(screen.getByRole('img').getAttribute('src')).toBe('http://localhost:8002/stream/raw/zedx.mjpg?retry=1');
});

test('stale overlay from /health frame age', async () => {
  global.fetch.mockReturnValue(health('annotated/zedx', 5));
  render(<CameraFeed camera="zedx" kind="annotated" title="Annotated" />);
  await act(async () => { jest.advanceTimersByTime(1000); });
  expect(screen.getByText('No frames: is perception / depth running?')).toBeInTheDocument();
});

test('fresh frames show no stale overlay', async () => {
  global.fetch.mockReturnValue(health('raw/zedx', 0.2));
  render(<CameraFeed camera="zedx" kind="raw" title="Raw" />);
  await act(async () => { jest.advanceTimersByTime(1000); });
  expect(screen.queryByText(/No frames/)).not.toBeInTheDocument();
});

test('switching camera swaps the stream URL', () => {
  global.fetch.mockReturnValue(health('raw/zedx', 0.2));
  const { rerender } = render(<CameraPair camera="zedx" />);
  expect(screen.getAllByRole('img').map((i) => i.getAttribute('src'))).toEqual([
    'http://localhost:8002/stream/annotated/zedx.mjpg', 'http://localhost:8002/stream/raw/zedx.mjpg']);
  rerender(<CameraPair camera="zedx_nano" />);
  expect(screen.getAllByRole('img').map((i) => i.getAttribute('src'))).toEqual([
    'http://localhost:8002/stream/annotated/zedx_nano.mjpg', 'http://localhost:8002/stream/raw/zedx_nano.mjpg']);
});
