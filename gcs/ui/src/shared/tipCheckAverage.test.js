import { runTipCheck, CHECK_PASSES } from './tipCheckApi';

const response = (job) => ({ ok: true, json: async () => job });
afterEach(() => { delete global.fetch; });

test('runs exactly five checks and averages probabilities instead of majority votes', async () => {
  const probabilities = [0.49, 0.49, 0.49, 0.49, 1.0];
  global.fetch = jest.fn();
  probabilities.forEach((p, i) => global.fetch.mockResolvedValueOnce(response({
    clip_id: `check${i}`, status: 'done', model: 'd3:4b', threshold: 0.5,
    p_tip_empty: p, p_tip_visible: 0.9, timings_s: { total: 2 },
  })));
  const progress = jest.fn();
  const result = await runTipCheck({ step: 'test' }, { onProgress: progress });
  expect(global.fetch).toHaveBeenCalledTimes(CHECK_PASSES);
  expect(result.p_tip_empty).toBeCloseTo(0.592);
  expect(result.anomaly_detected).toBe(true);
  expect(result.confidence).toBeCloseTo(0.592);
  expect(result.sample_count).toBe(5);
  expect(result.samples).toHaveLength(5);
  expect(result.timings_s.total).toBe(10);
  expect(progress.mock.calls.map(([n]) => n)).toEqual([1, 2, 3, 4, 5]);
});

test('a failed check rejects without using a partial average', async () => {
  global.fetch = jest.fn()
    .mockResolvedValueOnce(response({ status: 'done', p_tip_empty: 0.1 }))
    .mockResolvedValueOnce(response({ status: 'done', p_tip_empty: 0.2 }))
    .mockResolvedValueOnce({ ok: false, status: 500, json: async () => ({ error: 'Camera failed' }) });
  await expect(runTipCheck()).rejects.toThrow('Camera failed');
  expect(global.fetch).toHaveBeenCalledTimes(3);
});

test('invalid probabilities are rejected', async () => {
  global.fetch = jest.fn().mockResolvedValue(response({ status: 'done', p_tip_empty: 1.2 }));
  await expect(runTipCheck()).rejects.toThrow('valid tip-empty probability');
});

test('a changed model cannot be mixed into the average', async () => {
  global.fetch = jest.fn();
  for (let i = 0; i < 5; i += 1) global.fetch.mockResolvedValueOnce(response({
    status: 'done', p_tip_empty: 0.1, model: i ? 'different' : 'd3:4b',
  }));
  await expect(runTipCheck()).rejects.toThrow('model changed');
});
