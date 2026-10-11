import { render, screen, fireEvent } from '@testing-library/react';
import mqtt from 'mqtt';
import App from './App';

jest.mock('mqtt');

beforeEach(() => {
  mqtt.connect.mockReturnValue({
    on: jest.fn(), publish: jest.fn(), subscribe: jest.fn(), unsubscribe: jest.fn(), end: jest.fn(),
  });
});

test('renders the welcome page', () => {
  render(<App />);
  expect(screen.getByText(/SYSTEM READY/i)).toBeInTheDocument();
});

test('camera panel header is visible on the welcome page', () => {
  render(<App />);
  expect(screen.getByText('Camera Feeds')).toBeInTheDocument();
});

test('camera panel header is visible on the experiment page', () => {
  render(<App />);
  fireEvent.click(screen.getByRole('button', { name: /begin/i }));
  expect(screen.getByText('Camera Feeds')).toBeInTheDocument();
});

test('camera panel header is visible on the loader page', () => {
  render(<App />);
  fireEvent.click(screen.getByRole('button', { name: /begin/i }));
  fireEvent.click(screen.getByRole('button', { name: /continue/i }));
  expect(screen.getByText('Camera Feeds')).toBeInTheDocument();
});
