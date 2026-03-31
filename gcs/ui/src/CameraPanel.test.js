import { render, screen, fireEvent, act } from '@testing-library/react';
import CameraPanel from './CameraPanel';

// Mock WebSocket
let mockWsInstances = [];
class MockWebSocket {
  constructor(url) {
    this.url = url;
    this.onmessage = null;
    this.onerror = null;
    this.onclose = null;
    this.readyState = MockWebSocket.OPEN;
    mockWsInstances.push(this);
  }
  close() { this.readyState = MockWebSocket.CLOSED; }
}
MockWebSocket.OPEN = 1;
MockWebSocket.CLOSED = 3;
global.WebSocket = MockWebSocket;

beforeEach(() => {
  mockWsInstances = [];
});

// Mock URL.createObjectURL / revokeObjectURL (not in jsdom)
global.URL.createObjectURL = jest.fn(() => 'blob:mock-url');
global.URL.revokeObjectURL = jest.fn();

test('renders minimized by default showing only header', () => {
  render(<CameraPanel />);
  expect(screen.getByText('Camera Feeds')).toBeInTheDocument();
  expect(screen.queryByTestId('camera-grid')).not.toBeInTheDocument();
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

test('StreamCell shows Disconnected placeholder before connection sends data', () => {
  render(<CameraPanel />);
  fireEvent.click(screen.getByRole('button', { name: /expand/i }));
  const cells = screen.getAllByText('Disconnected');
  expect(cells.length).toBeGreaterThan(0);
});

test('StreamCell updates image src when WebSocket message arrives', async () => {
  render(<CameraPanel />);
  fireEvent.click(screen.getByRole('button', { name: /expand/i }));

  await act(async () => {
    mockWsInstances[0].onmessage({ data: 'abc123' });
  });

  const img = screen.getAllByRole('img')[0];
  expect(img.src).toContain('data:image/jpeg;base64,abc123');
});

test('WebSocket is closed when panel minimizes', async () => {
  render(<CameraPanel />);
  fireEvent.click(screen.getByRole('button', { name: /expand/i }));
  fireEvent.click(screen.getByRole('button', { name: /minimize/i }));
  expect(mockWsInstances.every(ws => ws.readyState === MockWebSocket.CLOSED)).toBe(true);
});
