import { useState } from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import RoutineBuilder from './RoutineBuilder';
import { MIN_DISPENSE_UL, PRESETS } from './shared/presets';
import { EMPTY_TUBE, TIP_FOR_LIQUID, TUBE_FOR_LIQUID } from './shared/tips';

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
  expect(aspirateWell(screen.getByTestId('json').textContent)).toBe(TUBE_FOR_LIQUID.red);
  expect(screen.getByRole('radio', { name: /^No anomaly/ })).toHaveAttribute('aria-checked', 'true');

  fireEvent.click(screen.getByRole('radio', { name: /^Anomaly/ }));
  expect(aspirateWell(screen.getByTestId('json').textContent)).toBe(EMPTY_TUBE);
  expect(screen.getByRole('radio', { name: /^Anomaly/ })).toHaveAttribute('aria-checked', 'true');
  expect(screen.getAllByTestId('routine-block')).toHaveLength(PRESETS[1].routine.steps.length);
});

const pickedTip = (routine) => routine.steps.find((s) => s.name === 'ot2')
  .parameters_json.steps.find((c) => c.action === 'pick_up_tips').well_indices;

test('presets use one tip per liquid: red tip 0, empty (blue) tube tip 3', () => {
  expect(pickedTip(PRESETS[0].routine)).toEqual([0]);
  expect(pickedTip(PRESETS[1].routine)).toEqual([3]);
});

const presetById = (id) => PRESETS.find((p) => p.id === id);
const ot2Of = (routine) => routine.steps.find((s) => s.name === 'ot2').parameters_json.steps;

test('Gradient fills column 1 from all red to all yellow, 100 µL per well over A1-D1, each 200 µL aspirate fully dispensed, no dispense under 30 µL', () => {
  const cmds = ot2Of(presetById('gradient').routine);
  expect(cmds.map((c) => c.action)).toEqual([
    'pick_up_tips', 'aspirate', 'dispense', 'return_tips', 'pick_up_tips', 'aspirate', 'dispense', 'return_tips',
  ]);
  const [redTip, redAsp, redDisp, , yelTip, yelAsp, yelDisp] = cmds;
  expect(redTip.well_indices).toEqual([TIP_FOR_LIQUID.red]);
  expect(redAsp).toEqual({ action: 'aspirate', resource_name: 'tube_rack', well_indices: [TUBE_FOR_LIQUID.red], volumes: [200] });
  expect(redDisp.well_indices).toEqual([0, 1, 2]);
  expect(redDisp.volumes).toEqual([100, 67, 33]);
  expect(yelTip.well_indices).toEqual([TIP_FOR_LIQUID.yellow]);
  expect(yelAsp).toEqual({ action: 'aspirate', resource_name: 'tube_rack', well_indices: [TUBE_FOR_LIQUID.yellow], volumes: [200] });
  expect(yelDisp.well_indices).toEqual([1, 2, 3]);
  expect(yelDisp.volumes).toEqual([33, 67, 100]);
  expect(redDisp.volumes.reduce((a, b) => a + b)).toBe(200);
  expect(yelDisp.volumes.reduce((a, b) => a + b)).toBe(200);
  redDisp.volumes.slice(1).forEach((v, i) => expect(v + yelDisp.volumes[i]).toBe(100));
  [...redDisp.volumes, ...yelDisp.volumes].forEach((v) => expect(v).toBeGreaterThanOrEqual(MIN_DISPENSE_UL));
});

test('the Gradient preset can be picked', () => {
  render(<Harness initial={JSON.stringify({ steps: [{ name: 'home' }] })} />);
  fireEvent.click(screen.getByRole('radio', { name: /^Gradient/ }));
  expect(screen.getByRole('radio', { name: /^Gradient/ })).toHaveAttribute('aria-checked', 'true');
  expect(JSON.parse(screen.getByTestId('json').textContent)).toEqual(presetById('gradient').routine);
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

const ot2Cmds = (json) => JSON.parse(json).steps.find((s) => s.name === 'ot2').parameters_json.steps;

test('an aspirate can skip the clog check and turn it back on', () => {
  const initial = { steps: [{ name: 'ot2', parameters_json: { steps: [
    { action: 'aspirate', resource_name: 'tube_rack', well_indices: [0], volumes: [200] },
    { action: 'dispense', resource_name: 'empty_plate', well_indices: [0], volumes: [200] },
  ] } }] };
  render(<Harness initial={JSON.stringify(initial)} />);
  const box = screen.getByRole('checkbox', { name: 'Command 1 clog check' });
  expect(box).toBeChecked();
  expect(screen.queryByRole('checkbox', { name: 'Command 2 clog check' })).not.toBeInTheDocument();

  fireEvent.click(box);
  expect(ot2Cmds(screen.getByTestId('json').textContent)[0].tip_check).toBe(false);

  fireEvent.click(screen.getByRole('checkbox', { name: 'Command 1 clog check' }));
  expect(ot2Cmds(screen.getByTestId('json').textContent)[0]).not.toHaveProperty('tip_check');
});
