import { runSingleTipCheckToCompletion as runTipCheckToCompletion } from './tipCheckApi';

const response = (data) => ({ ok: true, json: async () => data });
afterEach(() => { delete global.fetch; });

test('waits for the submitted job, ignoring other completed jobs', async () => {
  global.fetch = jest.fn()
    .mockResolvedValueOnce(response({ clip_id: 'ours', status: 'queued' }))
    .mockResolvedValueOnce(response([
      { clip_id: 'other', status: 'done' }, { clip_id: 'ours', status: 'running' },
    ]))
    .mockResolvedValueOnce(response([{ clip_id: 'ours', status: 'done', anomaly_detected: true }]));
  const result = await runTipCheckToCompletion({}, { pollMs: 0 });
  expect(result.clip_id).toBe('ours');
  expect(result.status).toBe('done');
  expect(global.fetch).toHaveBeenCalledTimes(3);
});

test('a failed queued job rejects instead of allowing dispense', async () => {
  global.fetch = jest.fn()
    .mockResolvedValueOnce(response({ clip_id: 'ours', status: 'queued' }))
    .mockResolvedValueOnce(response([{ clip_id: 'ours', status: 'failed', error: 'Inference failed' }]));
  await expect(runTipCheckToCompletion({}, { pollMs: 0 })).rejects.toThrow('Inference failed');
});
