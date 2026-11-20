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
