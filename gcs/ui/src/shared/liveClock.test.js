import { formatDuration } from './liveClock';

test('formats seconds as m:ss or h:mm:ss', () => {
  expect(formatDuration(0)).toBe('0:00');
  expect(formatDuration(7.9)).toBe('0:07');
  expect(formatDuration(754)).toBe('12:34');
  expect(formatDuration(3723)).toBe('1:02:03');
  expect(formatDuration(NaN)).toBe('0:00');
  expect(formatDuration(-3)).toBe('0:00');
});
