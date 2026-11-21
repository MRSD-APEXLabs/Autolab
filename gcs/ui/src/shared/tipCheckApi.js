// HTTP client for the d3:4b tip-check service (vlm_clog/clog_service.py on the
// PC with the L515 camera, mrsd-lab-cmptr3). One POST /check takes an L515
// snapshot, crops the pipette tip and asks d3:4b on p5090 whether it is empty.
// lab_machine_executive runs the same five-check average after every aspirate
// (src/tip_check.cpp); keep the two in step. This copy serves the debug console.
// REACT_APP_TIP_CHECK_URL overrides the address.
import { requestAt } from './anomalyApi';

export const TIP_CHECK_URL = process.env.REACT_APP_TIP_CHECK_URL || 'http://192.168.10.19:8110';

const request = (method, path, body) => requestAt(TIP_CHECK_URL, method, path, body);

export const getTipCheckHealth = () => request('GET', '/health');
export const getTipCheckJobs = () => request('GET', '/jobs');
export const CHECK_PASSES = 5;
export const runTipCheck = (body = {}, options = {}) => runTipCheckToCompletion(body, options);
export const tipCheckFileUrl = (path) => `${TIP_CHECK_URL}${path}`;


// Some service versions answer /check only after completion; others queue a job.
// Match that exact job, never another result from the job history.
export async function runSingleTipCheckToCompletion(body = {}, { timeoutMs = 15 * 60 * 1000, pollMs = 1000 } = {}) {
  let job = await request('POST', '/check', body);
  const deadline = Date.now() + timeoutMs;
  const clipId = job?.clip_id;
  while (job?.status !== 'done') {
    if (!['queued', 'running', 'pending', 'processing'].includes(job?.status)) {
      throw new Error(job?.error || `Clog detection ended with status ${job?.status}`);
    }
    if (!clipId) throw new Error('Clog detection returned no job ID');
    if (Date.now() >= deadline) throw new Error('Clog detection timed out; the tip remains held');
    await new Promise((resolve) => setTimeout(resolve, pollMs));
    const response = await getTipCheckJobs();
    const jobs = Array.isArray(response) ? response : response?.jobs;
    if (!Array.isArray(jobs)) throw new Error('Unreadable clog-detection job list');
    const current = jobs.find((item) => item.clip_id === clipId);
    if (current) job = current;
  }
  return job;
}


// Five sequential checks keep the tip stationary and the camera service single-job.
// Each request uses the same configured model and captures its own current image.
export async function runTipCheckToCompletion(body = {}, { onProgress, ...options } = {}) {
  const samples = [];
  for (let i = 0; i < CHECK_PASSES; i += 1) {
    const job = await runSingleTipCheckToCompletion(body, options);
    if (typeof job.p_tip_empty !== 'number' || !Number.isFinite(job.p_tip_empty)
        || job.p_tip_empty < 0 || job.p_tip_empty > 1) {
      throw new Error(`Check ${i + 1} returned no valid tip-empty probability`);
    }
    samples.push(job);
    onProgress?.(samples.length, CHECK_PASSES);
  }
  const models = new Set(samples.map((job) => job.model).filter(Boolean));
  if (models.size > 1) throw new Error('The model changed during the five checks');
  const thresholds = samples.map((job) => job.threshold ?? 0.5);
  if (thresholds.some((value) => !Number.isFinite(value) || value < 0 || value > 1
      || value !== thresholds[0])) throw new Error('The detection threshold changed during the checks');
  const mean = samples.reduce((sum, job) => sum + job.p_tip_empty, 0) / CHECK_PASSES;
  const visible = samples.map((job) => job.p_tip_visible);
  const warnings = samples.map((job) => job.warning).filter(Boolean);
  const anomaly = mean >= thresholds[0];
  const total = samples.reduce((sum, job) => sum + (job.timings_s?.total || 0), 0);
  return {
    ...samples[0],
    status: 'done', threshold: thresholds[0],
    aggregation: 'mean', sample_count: CHECK_PASSES, samples,
    p_tip_empty: mean,
    p_tip_visible: visible.every((p) => typeof p === 'number' && Number.isFinite(p))
      ? visible.reduce((sum, p) => sum + p, 0) / CHECK_PASSES : undefined,
    anomaly_detected: anomaly,
    confidence: anomaly ? mean : 1 - mean,
    summary: `Average of ${CHECK_PASSES} checks: ${anomaly ? 'tip looks empty' : 'liquid in tip'} (P(empty)=${mean.toFixed(2)})`,
    warning: warnings.length ? `${warnings.length} of ${CHECK_PASSES} checks reported a warning. ${warnings[0]}` : null,
    error: null,
    timings_s: { total: Number(total.toFixed(2)) },
  };
}
