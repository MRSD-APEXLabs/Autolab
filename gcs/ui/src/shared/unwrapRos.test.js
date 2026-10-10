import { unwrapString } from './unwrapRos';

test('returns the inner string of a bridged std_msgs/String', () => {
  expect(unwrapString(JSON.stringify({ data: '{"a":1}' }))).toBe('{"a":1}');
});

test.each(['not json', '{"data": 3}', '{}', 'null', '[1]'])('throws on %s', (payload) => {
  expect(() => unwrapString(payload)).toThrow();
});
