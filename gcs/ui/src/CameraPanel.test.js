import { render, screen, fireEvent, act } from '@testing-library/react';
import CameraPanel from './CameraPanel';

// ── Mock WebSocket ─────────────────────────────────────────────────────────────
let mockWsInstances = [];

class MockWebSocket {
  constructor(url) {
    this.url = url;
    this.readyState = MockWebSocket.OPEN;
    this.onopen = null;
    this.onmessage = null;
    this.onclose = null;
    this.onerror = null;
    this.send = jest.fn();
    mockWsInstances.push(this);
  }
  close() {
    this.readyState = MockWebSocket.CLOSED;
    this.onclose?.();
  }
}
MockWebSocket.OPEN   = 1;
MockWebSocket.CLOSED = 3;
global.WebSocket = MockWebSocket;

beforeEach(() => { mockWsInstances = []; });

// ── Tests ──────────────────────────────────────────────────────────────────────

test('renders minimized by default showing only header', () => {
  render(<CameraPanel />);
  expect(screen.getByText('Camera Feeds')).toBeInTheDocument();
  expect(screen.queryByTestId('camera-grid')).not.toBeInTheDocument();
});

test('opens three WebSocket connections on mount', () => {
  render(<CameraPanel />);
  // stream(8766), wrist(8767), base(8767)
  expect(mockWsInstances).toHaveLength(3);
  expect(mockWsInstances[0].url).toContain(':8766');
  expect(mockWsInstances[1].url).toContain(':8767');
  expect(mockWsInstances[2].url).toContain(':8767');
});

test('wrist and base sockets subscribe with correct camera name on open', () => {
  render(<CameraPanel />);
  mockWsInstances[1].onopen?.();
  mockWsInstances[2].onopen?.();
  expect(mockWsInstances[1].send).toHaveBeenCalledWith(JSON.stringify({ camera: 'wrist' }));
  expect(mockWsInstances[2].send).toHaveBeenCalledWith(JSON.stringify({ camera: 'base' }));
});

test('expands when header button is clicked', () => {
  render(<CameraPanel />);
  fireEvent.click(screen.getByRole('button', { name: /expand/i }));
  expect(screen.getByTestId('camera-grid')).toBeInTheDocument();
});

test('minimizes when header button is clicked while expanded', () => {
  render(<CameraPanel />);
  fireEvent.click(screen.getByRole('button', { name: /expand/i }));
  fireEvent.click(screen.getByRole('button', { name: /minimize/i }));
  expect(screen.queryByTestId('camera-grid')).not.toBeInTheDocument();
});

test('shows no-signal placeholder before any frames arrive', () => {
  render(<CameraPanel />);
  fireEvent.click(screen.getByRole('button', { name: /expand/i }));
  const placeholders = screen.getAllByText('No signal');
  expect(placeholders.length).toBe(3);
});

test('active stream image updates when stream WS receives a frame', async () => {
  render(<CameraPanel />);
  fireEvent.click(screen.getByRole('button', { name: /expand/i }));

  await act(async () => {
    mockWsInstances[0].onmessage?.({
      data: JSON.stringify({ image_jpeg_b64: 'abc123' }),
    });
  });

  const img = screen.getAllByRole('img')[0];
  expect(img.src).toContain('data:image/jpeg;base64,abc123');
});

test('wrist image updates when wrist WS receives a frame', async () => {
  render(<CameraPanel />);
  fireEvent.click(screen.getByRole('button', { name: /expand/i }));

  await act(async () => {
    mockWsInstances[1].onopen?.();
    mockWsInstances[1].onmessage?.({
      data: JSON.stringify({ image_jpeg_b64: 'wristdata' }),
    });
  });

  const imgs = screen.getAllByRole('img');
  expect(imgs.some(img => img.src.includes('wristdata'))).toBe(true);
});

test('WebSockets are closed on unmount', () => {
  const { unmount } = render(<CameraPanel />);
  unmount();
  expect(mockWsInstances.every(ws => ws.readyState === MockWebSocket.CLOSED)).toBe(true);
});
