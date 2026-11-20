import React from 'react';
import { OT2_ACTIONS, ot2Commands } from './shared/steps';
import { aspirateLabel } from './shared/verdict';
import { OT2_PHASE_LABEL, commandState, runningMean, tipCheckStageText } from './shared/ot2Progress';

// Pass dots of a running tip check: filled = finished, pulsing = this pass.
export function TipPasses({ tc }) {
  const passes = Number.isInteger(tc?.passes) && tc.passes > 0 ? tc.passes : 0;
  if (!passes) return null;
  const done = Array.isArray(tc.samples) ? tc.samples.length : 0;
  return (
    <span className="o2-passes" aria-hidden="true">
      {Array.from({ length: passes }, (_, i) => (
        <i key={i} className={i < done ? 'on' : i === done && tc.status !== 'done' ? 'now' : ''} />
      ))}
    </span>
  );
}

function TipCheckLine({ tc }) {
  const mean = runningMean(tc);
  return (
    <div className="o2-check" role="status">
      <span className="o2-check-name">Clog check</span>
      <TipPasses tc={tc} />
      <span className="o2-check-stage">{tipCheckStageText(tc)}</span>
      {mean !== null && <span className="o2-check-mean">P(empty) so far {mean.toFixed(2)}</span>}
    </div>
  );
}

// The OT-2 protocol of the current routine step, command by command, with the live tip check.
export default function Ot2Progress({ step, progress }) {
  const cmds = ot2Commands(step) ?? [];
  if (!cmds.length) return null;
  const phase = progress?.phase;
  const phaseNote = phase && phase !== 'command' && phase !== 'tip_check' ? OT2_PHASE_LABEL[phase] : null;
  return (
    <div className="o2" aria-label="OT-2 protocol progress">
      <div className="o2-head">
        <span className="ax-h3">OT-2 protocol</span>
        <span className="ax-hint">
          {!progress ? 'Waiting for the OT-2…' : phaseNote ?? `Command ${progress.command + 1} of ${cmds.length}`}
        </span>
      </div>
      <ol className="o2-list">
        {cmds.map((c, i) => {
          const st = commandState(progress, i);
          const label = OT2_ACTIONS[c?.action]?.label ?? c?.action ?? '?';
          return (
            <li key={i} className={`o2-cmd ${st}`} aria-current={st === 'now' || st === 'check' ? 'step' : undefined}>
              <i className="o2-node">{st === 'done' || st === 'check' ? '✓' : st === 'fail' ? '✕' : i + 1}</i>
              <div className="min-w-0">
                <span className="o2-name">{label}</span>
                {c?.resource_name && <small>{c.action === 'aspirate' || c.action === 'dispense' ? aspirateLabel(c) : c.resource_name.replace('_', ' ')}</small>}
                {st === 'check' && <TipCheckLine tc={progress.tip_check} />}
              </div>
            </li>
          );
        })}
      </ol>
    </div>
  );
}
