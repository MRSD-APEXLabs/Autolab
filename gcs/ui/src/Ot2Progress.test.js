import { render, screen } from '@testing-library/react';
import Ot2Progress from './Ot2Progress';

const step = {
  name: 'ot2',
  parameters_json: {
    steps: [
      { action: 'pick_up_tips', resource_name: 'tip_rack', well_indices: [0] },
      { action: 'aspirate', resource_name: 'tube_rack', well_indices: [3], volumes: [200] },
      { action: 'dispense', resource_name: 'empty_plate', well_indices: [0], volumes: [200] },
      { action: 'return_tips' },
    ],
  },
};

test('lists the protocol commands and marks the running one', () => {
  render(<Ot2Progress step={step} progress={{ phase: 'command', command: 2, commands: 4 }} />);
  expect(screen.getByText('Command 3 of 4')).toBeInTheDocument();
  const items = screen.getAllByRole('listitem');
  expect(items.map((li) => li.className.split(' ')[1])).toEqual(['done', 'done', 'now', 'todo']);
  expect(items[2]).toHaveAttribute('aria-current', 'step');
});

test('shows the live tip check under the aspirate', () => {
  render(<Ot2Progress step={step} progress={{
    phase: 'tip_check', command: 1, commands: 4,
    tip_check: { pass: 3, passes: 5, status: 'running', samples: [0.1, 0.2] },
  }} />);
  expect(screen.getByRole('status')).toHaveTextContent('Pass 3 of 5 · analyzing');
  expect(screen.getByText('P(empty) so far 0.15')).toBeInTheDocument();
});

test('waits for the OT-2 before any progress arrives, and names the other phases', () => {
  const { rerender } = render(<Ot2Progress step={step} progress={null} />);
  expect(screen.getByText('Waiting for the OT-2…')).toBeInTheDocument();
  rerender(<Ot2Progress step={step} progress={{ phase: 'homing', command: null, commands: 4 }} />);
  expect(screen.getByText('Homing the OT-2')).toBeInTheDocument();
});

test('marks an aspirate that skips the clog check', () => {
  const skip = { name: 'ot2', parameters_json: { steps: [
    { action: 'aspirate', resource_name: 'tube_rack', well_indices: [0], volumes: [200], tip_check: false },
  ] } };
  render(<Ot2Progress step={skip} progress={{ phase: 'command', command: 0, commands: 1 }} />);
  expect(screen.getByText('no clog check')).toBeInTheDocument();
});
