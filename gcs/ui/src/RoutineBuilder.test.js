import { useState } from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import RoutineBuilder from './RoutineBuilder';
import { PRESETS } from './shared/presets';

function Harness({ initial }) {
  const [value, setValue] = useState(initial);
  return (
    <>
      <RoutineBuilder value={value} onChange={setValue} />
      <output data-testid="json">{value}</output>
    </>
  );
}

const aspirateWell = (json) => JSON.parse(json).steps.find((s) => s.name === 'ot2')
  .parameters_json.steps.find((c) => c.action === 'aspirate').well_indices[0];

test('offers a no-anomaly and an anomaly experiment that differ in the aspirated tube', () => {
  render(<Harness initial={JSON.stringify({ steps: [{ name: 'home' }] })} />);
  expect(screen.getByText('Custom routine', { exact: false })).toBeInTheDocument();

  fireEvent.click(screen.getByRole('radio', { name: /^No anomaly/ }));
  expect(aspirateWell(screen.getByTestId('json').textContent)).toBe(3);
  expect(screen.getByRole('radio', { name: /^No anomaly/ })).toHaveAttribute('aria-checked', 'true');

  fireEvent.click(screen.getByRole('radio', { name: /^Anomaly/ }));
  expect(aspirateWell(screen.getByTestId('json').textContent)).toBe(9);
  expect(screen.getByRole('radio', { name: /^Anomaly/ })).toHaveAttribute('aria-checked', 'true');
  expect(screen.getAllByTestId('routine-block')).toHaveLength(PRESETS[1].routine.steps.length);
});

test('editing a preset makes it a custom routine again', () => {
  render(<Harness initial={JSON.stringify(PRESETS[0].routine)} />);
  expect(screen.getByRole('radio', { name: /^No anomaly/ })).toHaveAttribute('aria-checked', 'true');
  fireEvent.click(screen.getByRole('button', { name: 'Remove step 1' }));
  expect(screen.getByRole('radio', { name: /^No anomaly/ })).toHaveAttribute('aria-checked', 'false');
});

const shakerStep = (json) => JSON.parse(json).steps.find((s) => s.name === 'shaker');

test('the presets run the shaker as one timed step, not run / wait / stop', () => {
  PRESETS.forEach((p) => {
    const shakers = p.routine.steps.filter((s) => s.name === 'shaker');
    expect(shakers).toEqual([{ name: 'shaker', pwm: 150, wait_time_s: 20 }]);
  });
});

test('a shaker block sets its speed and how long it runs in one step', () => {
  render(<Harness initial={JSON.stringify({ steps: [{ name: 'shaker', pwm: 150, wait_time_s: 20 }] })} />);
  expect(screen.getByText('then stops')).toBeInTheDocument();
  fireEvent.change(screen.getByLabelText('Run for'), { target: { value: '45' } });
  expect(shakerStep(screen.getByTestId('json').textContent)).toEqual({ name: 'shaker', pwm: 150, wait_time_s: 45 });
  fireEvent.change(screen.getByLabelText('Run for'), { target: { value: '0' } });
  expect(screen.getByText('0 s keeps it running')).toBeInTheDocument();
});

test('a PWM 0 shaker block is a stop and has no duration', () => {
  render(<Harness initial={JSON.stringify({ steps: [{ name: 'shaker', pwm: 0 }] })} />);
  expect(screen.getByText('stops the shaker')).toBeInTheDocument();
  expect(screen.queryByLabelText('Run for')).not.toBeInTheDocument();
});
