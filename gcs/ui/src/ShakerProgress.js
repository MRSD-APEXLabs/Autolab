import React from 'react';
import { formatDuration, useLiveSeconds } from './shared/liveClock';

// The shaker phase the executor reports for the current step, or null.
export function shakerProgressFor(status) {
  const p = status?.step_progress;
  if (!p || typeof p !== 'object' || p.machine !== 'shaker') return null;
  if (p.step !== (status.current_step ?? 0)) return null;
  if (!['running', 'paused', 'failed'].includes(status.state)) return null;
  return p;
}

const PHASE_TEXT = {
  starting: 'Starting the shaker…',
  hold: 'Shaking',
  stopping: 'Stopping the shaker…',
};

// Run-shaker step: how long it has been shaking out of wait_time_s, as a bar.
export default function ShakerProgress({ step, progress, ticking }) {
  const pwm = Number(step?.pwm ?? 0);
  const total = Number(progress?.duration_s ?? step?.wait_time_s ?? 0);
  const timed = pwm > 0 && total > 0;
  const phase = progress?.phase;
  const live = useLiveSeconds(phase === 'hold' ? progress.elapsed_s : null, ticking && phase === 'hold', { max: total });

  if (!timed) {
    return (
      <div className="sk" aria-label="Shaker progress">
        <div className="sk-head">
          <span className="ax-h3">Shaker</span>
          <span className="ax-hint">{pwm > 0 ? `Set to PWM ${pwm} and left running` : 'Setting PWM 0'}</span>
        </div>
      </div>
    );
  }

  const elapsed = phase === 'hold' ? live ?? 0 : phase === 'stopping' ? total : 0;
  const frac = Math.max(0, Math.min(1, elapsed / total));
  const text = !progress ? 'Waiting for the shaker…' : PHASE_TEXT[phase] ?? phase;
  return (
    <div className="sk" aria-label="Shaker progress">
      <div className="sk-head">
        <span className="ax-h3">Shaker · PWM {pwm}</span>
        <span className="ax-hint">{text}</span>
      </div>
      <div className={`sk-bar ${phase === 'hold' ? 'on' : ''}`} role="progressbar"
        aria-valuemin={0} aria-valuemax={Math.round(total)} aria-valuenow={Math.floor(elapsed)}
        aria-valuetext={`${formatDuration(elapsed)} of ${formatDuration(total)}`}>
        <i style={{ width: `${(frac * 100).toFixed(1)}%` }} />
      </div>
      <div className="sk-times">
        <span><b>{formatDuration(elapsed)}</b> / {formatDuration(total)}</span>
        <span>{phase === 'stopping' ? 'Done shaking' : `${formatDuration(Math.ceil(total - elapsed))} left`}</span>
      </div>
    </div>
  );
}
