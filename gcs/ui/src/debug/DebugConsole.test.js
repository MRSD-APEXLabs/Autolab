import { render, screen } from '@testing-library/react';
import mqtt from 'mqtt';
import DebugConsole from './DebugConsole';

jest.mock('mqtt');

beforeEach(() => {
  mqtt.connect.mockReturnValue({
    on: jest.fn(),
    publish: jest.fn(),
    subscribe: jest.fn(),
    unsubscribe: jest.fn(),
    end: jest.fn(),
  });
});

test('renders all five subsystem panels', () => {
  render(<DebugConsole />);
  expect(screen.getByText('Navigation')).toBeInTheDocument();
  expect(screen.getByText('Planning')).toBeInTheDocument();
  expect(screen.getByText('Perception')).toBeInTheDocument();
  expect(screen.getByText('Manipulation')).toBeInTheDocument();
  expect(screen.getByText('Lab Machine Integration')).toBeInTheDocument();
});

test('shows a disconnected banner before the mqtt client connects', () => {
  render(<DebugConsole />);
  expect(screen.getByText(/MQTT: disconnected/)).toBeInTheDocument();
});
