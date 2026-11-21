import { planToRoutine, usesLiquid } from './nlpRoutine';

// The planner's answer (FVD-Presentation/NLP/mix_colors.py expected()): 11 steps, two transfers.
const plan = (pair, vols = [200, 200], seconds = 20, pwm = 150) => ({
  steps: [
    { name: 'go_to', target_machine: 'ot2' },
    { name: 'pick_base' },
    { name: 'place', target_machine: 'ot2' },
    {
      name: 'ot2',
      parameters_json: {
        steps: pair.map((liquid, i) => ({
          action: 'transfer', source: `${liquid}_tube`, destination: 'well_plate', volume_ul: vols[i],
        })),
      },
    },
    { name: 'pick', target_machine: 'ot2' },
    { name: 'go_to', target_machine: 'shaker' },
    { name: 'place', target_machine: 'shaker' },
    { name: 'shaker', pwm, wait_time_s: seconds },
    { name: 'pick', target_machine: 'shaker' },
    { name: 'place_base' },
    { name: 'go_to', target_machine: 'home' },
  ],
});

const ot2 = (routine) => routine.steps.find((s) => s.name === 'ot2').parameters_json.steps;

test('red then clear becomes pick/aspirate/dispense/return per liquid, in order; clear skips the clog check', () => {
  const r = planToRoutine(plan(['red', 'clear'], [100, 50]), null);
  expect(ot2(r)).toEqual([
    { action: 'pick_up_tips', resource_name: 'tip_rack', well_indices: [2] },
    { action: 'aspirate', resource_name: 'tube_rack', well_indices: [6], volumes: [100] },
    { action: 'dispense', resource_name: 'empty_plate', well_indices: [0], volumes: [100] },
    { action: 'return_tips' },
    { action: 'pick_up_tips', resource_name: 'tip_rack', well_indices: [0] },
    { action: 'aspirate', resource_name: 'tube_rack', well_indices: [0], volumes: [50], tip_check: false },
    { action: 'dispense', resource_name: 'empty_plate', well_indices: [0], volumes: [50] },
    { action: 'return_tips' },
  ]);
});

test('brown uses tube 3 (A2) and tip 1', () => {
  const cmds = ot2(planToRoutine(plan(['brown', 'clear']), null));
  expect(cmds[0].well_indices).toEqual([1]);
  expect(cmds[1].well_indices).toEqual([3]);
});

test('non-ot2 steps are copied unchanged', () => {
  const p = plan(['clear', 'brown'], [200, 200], 45, 90);
  const r = planToRoutine(p, null);
  expect(r.steps).toHaveLength(11);
  r.steps.forEach((s, i) => { if (i !== 3) expect(s).toEqual(p.steps[i]); });
  expect(r.steps[7]).toEqual({ name: 'shaker', pwm: 90, wait_time_s: 45 });
});

test('keeps max_retries and robot from the base routine', () => {
  const r = planToRoutine(plan(['brown', 'red']), { steps: [], max_retries: 5, robot: 'robot_2' });
  expect(r.max_retries).toBe(5);
  expect(r.robot).toBe('robot_2');
});

test('defaults to 3 retries and robot_1 without a base', () => {
  const r = planToRoutine(plan(['brown', 'red']), null);
  expect(r.max_retries).toBe(3);
  expect(r.robot).toBe('robot_1');
});

test('an unknown liquid throws', () => {
  expect(() => planToRoutine(plan(['brown', 'blue']), null)).toThrow(/blue/);
});

test('an ot2 step without transfers throws', () => {
  const p = plan(['brown', 'clear']);
  p.steps[3] = { name: 'ot2', parameters_json: { steps: [] } };
  expect(() => planToRoutine(p, null)).toThrow(/transfer/);
});

test('usesLiquid finds the clear liquid by its tube', () => {
  expect(usesLiquid(planToRoutine(plan(['red', 'clear']), null), 'clear')).toBe(true);
  expect(usesLiquid(planToRoutine(plan(['red', 'brown']), null), 'clear')).toBe(false);
  expect(usesLiquid({ steps: [{ name: 'home' }] }, 'clear')).toBe(false);
});
