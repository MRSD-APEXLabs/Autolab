import { commandState, ot2ProgressFor, runningMean, tipCheckStageText } from './ot2Progress';

const prog = (fields) => ({ machine: 'ot2', step: 3, run_id: 'r', commands: 4, ...fields });

test('progress is only shown for the current step while it runs or after it failed', () => {
  const p = prog({ phase: 'command', command: 1 });
  expect(ot2ProgressFor({ state: 'running', current_step: 3, step_progress: p })).toBe(p);
  expect(ot2ProgressFor({ state: 'failed', current_step: 3, step_progress: p })).toBe(p);
  expect(ot2ProgressFor({ state: 'running', current_step: 4, step_progress: p })).toBeNull();
  expect(ot2ProgressFor({ state: 'success', current_step: 3, step_progress: p })).toBeNull();
  expect(ot2ProgressFor({ state: 'running', current_step: 3, step_progress: { ...p, machine: 'shaker' } })).toBeNull();
  expect(ot2ProgressFor({ state: 'running', current_step: 3 })).toBeNull();
});

test('command states follow the phase and the current command', () => {
  const at = (phase, command) => [0, 1, 2, 3].map((i) => commandState(prog({ phase, command }), i));
  expect(at('connecting', null)).toEqual(['todo', 'todo', 'todo', 'todo']);
  expect(at('command', 1)).toEqual(['done', 'now', 'todo', 'todo']);
  expect(at('tip_check', 1)).toEqual(['done', 'check', 'todo', 'todo']);
  expect(at('failed', 2)).toEqual(['done', 'done', 'fail', 'todo']);
  expect(at('homing', null)).toEqual(['done', 'done', 'done', 'done']);
  expect(commandState(null, 0)).toBe('todo');
});

test('tip-check stage text names the pass and what the service is doing', () => {
  expect(tipCheckStageText({ pass: 2, passes: 5, status: 'running', stage: 'vlm' })).toBe('Pass 2 of 5 · vlm');
  expect(tipCheckStageText({ pass: 2, passes: 5, status: 'running' })).toBe('Pass 2 of 5 · analyzing');
  expect(tipCheckStageText({ pass: 1, passes: 5, status: 'queued', queue_position: 2 })).toBe('Pass 1 of 5 · queued #2');
  expect(tipCheckStageText({ pass: 1, passes: 5, status: 'submitting' })).toBe('Pass 1 of 5 · capturing');
  expect(tipCheckStageText(null)).toBe('Starting…');
});

test('running mean averages the finished passes', () => {
  expect(runningMean({ samples: [0.1, 0.3] })).toBeCloseTo(0.2);
  expect(runningMean({ samples: [] })).toBeNull();
  expect(runningMean(null)).toBeNull();
});
