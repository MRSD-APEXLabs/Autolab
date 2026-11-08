import { verdict, stepLabel } from './verdict';

test.each([
  ['recording', 'Analyzing…', 'pending', false],
  ['finishing', 'Analyzing…', 'pending', false],
  ['queued', 'Analyzing…', 'pending', false],
  ['running', 'Analyzing…', 'pending', false],
  ['aborted', 'Not analyzed', 'warn', true],
])('status %s -> %s', (status, label, tone, terminal) => {
  expect(verdict({ status })).toMatchObject({ label, tone, terminal });
});

test('done without anomaly is OK', () => {
  expect(verdict({ status: 'done', anomaly_detected: false })).toMatchObject({ label: 'OK', tone: 'ok' });
});

test('done with anomaly is clog suspected with severity', () => {
  expect(verdict({ status: 'done', anomaly_detected: true, severity: 'medium' }))
    .toMatchObject({ label: 'Clog suspected', tone: 'bad', detail: 'medium' });
});

test('error is never healthy', () => {
  const v = verdict({ status: 'error', anomaly_detected: null });
  expect(v.label).toBe('Analysis failed');
  expect(v.tone).not.toBe('ok');
});

test('done with null anomaly_detected is not healthy', () => {
  expect(verdict({ status: 'done', anomaly_detected: null }).tone).not.toBe('ok');
});

test('missing job is no result', () => {
  expect(verdict(undefined)).toMatchObject({ label: 'No result', tone: 'none', terminal: false });
});

test('stepLabel parses executive step JSON', () => {
  const step = JSON.stringify({ action: 'aspirate', resource_name: 'tube_rack', well_indices: [0, 1] });
  expect(stepLabel({ step })).toBe('tube_rack wells 0, 1');
  expect(stepLabel({ step: 'debug replay' })).toBe('debug replay');
  expect(stepLabel(undefined, 'c1')).toBe('c1');
});
