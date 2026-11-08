// Maps a clog-detection job to what the operator sees.
// Only a finished job with anomaly_detected === false is healthy.

const PENDING = new Set(['recording', 'finishing', 'queued', 'running']);
const TERMINAL = new Set(['done', 'error', 'aborted']);

export const TONE_CLASSES = {
  ok: 'bg-green-100 text-green-700',
  bad: 'bg-red-100 text-red-700',
  warn: 'bg-amber-100 text-amber-700',
  pending: 'bg-orange-100 text-orange-600',
  none: 'bg-gray-200 text-gray-600',
};

export function verdict(job) {
  const status = job?.status;
  if (PENDING.has(status)) {
    let detail = null;
    if (status === 'running' && job.stage) detail = job.stage;
    if (status === 'queued' && job.queue_position) detail = `queue #${job.queue_position}`;
    return { label: 'Analyzing…', tone: 'pending', terminal: false, detail };
  }
  if (status === 'done' && job.anomaly_detected === false) {
    return { label: 'OK', tone: 'ok', terminal: true, detail: null };
  }
  if (status === 'done' && job.anomaly_detected === true) {
    return { label: 'Clog suspected', tone: 'bad', terminal: true, detail: job.severity || null };
  }
  if (status === 'error') {
    return { label: 'Analysis failed', tone: 'warn', terminal: true, detail: job.failed_stage || null };
  }
  if (status === 'aborted') {
    return { label: 'Not analyzed', tone: 'warn', terminal: true, detail: null };
  }
  return { label: 'No result', tone: 'none', terminal: TERMINAL.has(status), detail: null };
}

// "tube_rack A1" style label from the job's step (JSON string from the executive).
export function stepLabel(job, fallback) {
  const raw = job?.step;
  if (raw) {
    try {
      const step = typeof raw === 'string' ? JSON.parse(raw) : raw;
      if (step && typeof step === 'object' && step.resource_name) {
        const wells = Array.isArray(step.well_indices) ? ` wells ${step.well_indices.join(', ')}` : '';
        return `${step.resource_name}${wells}`;
      }
    } catch {
      // plain string step, e.g. "debug replay"
    }
    if (typeof raw === 'string') return raw;
  }
  return fallback ?? job?.clip_id ?? '—';
}
