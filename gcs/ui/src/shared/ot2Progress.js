// Live OT-2 progress as the routine executor reports it (status.step_progress, from
// lab_machine_executive's ot2_progress): which protocol command runs, and how far the tip check is.

export const OT2_PHASE_LABEL = {
  connecting: 'Connecting to the OT-2',
  command: 'Running',
  tip_check: 'Clog check',
  homing: 'Homing the OT-2',
  disconnecting: 'Disconnecting',
  done: 'Protocol done',
  failed: 'Stopped',
};

// The OT-2 progress of the routine step being shown, or null (stale, other step, other machine).
export function ot2ProgressFor(status) {
  const p = status?.step_progress;
  if (!p || typeof p !== 'object' || p.machine !== 'ot2') return null;
  if (p.step !== (status.current_step ?? 0)) return null;
  if (!['running', 'paused', 'failed'].includes(status.state)) return null;
  return p;
}

// State of protocol command `i`: done | now | check (tip check after it) | fail | todo.
export function commandState(progress, i) {
  if (!progress) return 'todo';
  const { phase, command } = progress;
  if (phase === 'homing' || phase === 'disconnecting' || phase === 'done') return 'done';
  if (phase === 'connecting' || !Number.isInteger(command)) return 'todo';
  if (i < command) return 'done';
  if (i > command) return 'todo';
  if (phase === 'tip_check') return 'check';
  if (phase === 'failed') return 'fail';
  return 'now';
}

// "Pass 2 of 5 · analyzing" for a tip check in progress.
export function tipCheckStageText(tc) {
  if (!tc || typeof tc !== 'object') return 'Starting…';
  const parts = [];
  if (Number.isInteger(tc.pass) && Number.isInteger(tc.passes)) parts.push(`Pass ${tc.pass} of ${tc.passes}`);
  const status = tc.status;
  if (status === 'queued') parts.push(Number.isInteger(tc.queue_position) ? `queued #${tc.queue_position}` : 'queued');
  else if (status === 'submitting') parts.push('capturing');
  else if (status === 'done') parts.push('done');
  else if (typeof tc.stage === 'string' && tc.stage) parts.push(tc.stage);
  else if (status) parts.push(status === 'running' ? 'analyzing' : status);
  return parts.join(' · ');
}

// Mean P(empty) of the finished passes, or null.
export function runningMean(tc) {
  const s = Array.isArray(tc?.samples) ? tc.samples.filter((v) => typeof v === 'number') : [];
  if (!s.length) return null;
  return s.reduce((a, b) => a + b, 0) / s.length;
}
