import { STEP_TYPES, missingParams, stepDetail, stepTitle } from './steps';

test('a new shaker block runs for a set time', () => {
  expect(STEP_TYPES.shaker.make()).toEqual({ name: 'shaker', pwm: 150, wait_time_s: 20 });
});

test('shaker title and detail show speed and duration', () => {
  expect(stepTitle({ name: 'shaker', pwm: 150, wait_time_s: 20 })).toBe('Run shaker');
  expect(stepDetail({ name: 'shaker', pwm: 150, wait_time_s: 20 })).toBe('150 PWM · 20 s');
  expect(stepDetail({ name: 'shaker', pwm: 150 })).toBe('150 PWM · until stopped');
  expect(stepTitle({ name: 'shaker', pwm: 0 })).toBe('Stop shaker');
  expect(stepDetail({ name: 'shaker', pwm: 0 })).toBe('');
});

test('wait_time_s is optional, like the executor', () => {
  expect(missingParams({ name: 'shaker', pwm: 150 })).toEqual([]);
  expect(missingParams({ name: 'shaker' })).toEqual(['pwm']);
});
