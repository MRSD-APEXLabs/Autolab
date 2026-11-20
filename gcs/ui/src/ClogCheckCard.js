import React, { useState } from 'react';
import { tipCheckFileUrl } from './shared/tipCheckApi';
import { aspirateLabel, tipCheckVerdict } from './shared/verdict';
import { runningMean, tipCheckStageText } from './shared/ot2Progress';
import { TipPasses } from './Ot2Progress';

// verdict tone → deck badge / accent class
const TONE = { ok: 'ok', bad: 'bad', warn: 'warn', pending: 'busy', none: '' };

// One tip-check image, shown whole (never cropped); opens full size in a new tab.
function CheckImage({ path, alt, caption }) {
  const src = tipCheckFileUrl(path);
  return (
    <figure className="cc-img">
      <a href={src} target="_blank" rel="noreferrer" title="Open full size">
        <img src={src} alt={alt} />
      </a>
      <figcaption>{caption}</figcaption>
    </figure>
  );
}

// The check running right now (from status.step_progress), before its result arrives.
function PendingRow({ pending, n }) {
  const tc = pending.progress;
  const mean = runningMean(tc);
  return (
    <li className="py-3" data-testid="pending-check">
      <div className="w-full flex items-center justify-between gap-4">
        <span className="min-w-0">
          <span className="block" style={{ font: '700 17px/1.2 var(--display)' }}>
            Aspirate {n}
            {Number.isInteger(pending.step) ? ` (step ${pending.step + 1})` : ''}
          </span>
          <span className="block ax-mono text-xs mt-1" style={{ color: 'var(--dim)' }}>
            {aspirateLabel(pending.aspirate)}
          </span>
        </span>
        <span className="ax-badge busy shrink-0 !normal-case !tracking-normal" role="status">
          <i className="cc-pulse" />
          Checking… {tipCheckStageText(tc)}
        </span>
      </div>
      <div className="mt-2 flex items-center gap-3 ax-mono text-xs" style={{ color: 'var(--dim)' }}>
        <TipPasses tc={tc} />
        {Array.isArray(tc?.samples) && tc.samples.length > 0 && (
          <span>
            Checks so far: {tc.samples.map((p) => Number(p).toFixed(2)).join(', ')}
            {mean !== null ? ` (mean ${mean.toFixed(2)})` : ''}
          </span>
        )}
      </div>
    </li>
  );
}

// Tip-check results the robot attached to the current/last run (status.recordings), plus the
// check in progress (`pending`: {progress, aspirate, step}) while lab_machine_executive runs it.
// Nothing is polled; only the images are loaded from the tip-check service.
export default function ClogCheckCard({ recordings = [], pending = null }) {
  const [expanded, setExpanded] = useState(null);
  const checks = recordings.filter((r) => r && typeof r === 'object' && r.type === 'tip_check' && r.id);
  if (checks.length === 0 && !pending) return null;

  const verdicts = checks.map(tipCheckVerdict);
  const counts = verdicts.reduce((acc, v) => ({ ...acc, [v.label]: (acc[v.label] || 0) + 1 }), {});
  const summary = Object.entries(counts).map(([label, n]) => `${n} ${label}`).join(', ');

  const worst = verdicts.some((v) => v.tone === 'bad') ? 'bad'
    : verdicts.some((v) => v.tone === 'warn') ? 'warn'
    : pending ? 'busy' : 'ok';
  const head = [
    checks.length ? `${checks.length} tip check${checks.length === 1 ? '' : 's'} — ${summary}` : null,
    pending ? 'checking the tip now' : null,
  ].filter(Boolean).join(' · ');

  return (
    <div className={`ax-panel ax-accent k-${worst} ax-rise`} style={{ '--i': 3 }}>
      <div className="ax-panel-head">
        <h3 className="ax-h3">Clog check</h3>
        <span className="ax-hint">{head}</span>
      </div>
      <ul className="ax-rows">
        {checks.map((r, i) => {
          const v = verdicts[i];
          const open = expanded === r.id;
          return (
            <li key={r.id} className="py-3">
              <button
                type="button"
                onClick={() => setExpanded(open ? null : r.id)}
                aria-expanded={open}
                className="w-full flex items-center justify-between gap-4 text-left"
              >
                <span className="min-w-0">
                  <span className="block" style={{ font: '700 17px/1.2 var(--display)' }}>
                    Aspirate {i + 1}
                    {Number.isInteger(r.step) ? ` (step ${r.step + 1})` : ''}
                    <span className="sr-only">: </span>
                  </span>
                  <span className="block ax-mono text-xs mt-1" style={{ color: 'var(--dim)' }}>
                    {aspirateLabel(r.aspirate)}
                  </span>
                </span>
                <span className={`ax-badge ${TONE[v.tone] ?? ''} shrink-0 !normal-case !tracking-normal`}>
                  {v.label}
                  {v.detail ? ` · ${v.detail}` : ''}
                </span>
              </button>
              {open && (
                <div className="mt-3 ax-mono text-xs space-y-1" style={{ color: 'var(--dim)' }}>
                  {r.summary && <p>{r.summary}</p>}
                  {Array.isArray(r.samples) && r.samples.length > 0 && (
                    <p>
                      Checks: {r.samples.map((p) => Number(p).toFixed(2)).join(', ')}
                      {typeof r.threshold === 'number' ? ` (clog at ≥ ${r.threshold.toFixed(2)})` : ''}
                    </p>
                  )}
                  {r.model && <p style={{ color: 'var(--faint)' }}>Model: {r.model}</p>}
                  {r.warning && <p style={{ color: 'var(--amber)' }}>{r.warning}</p>}
                  {r.error && <p style={{ color: 'var(--amber)' }}>{r.error}</p>}
                  {(r.files?.crop || r.files?.preview) && (
                    <div className="cc-imgs">
                      {r.files.crop && <CheckImage path={r.files.crop} alt="Tip crop sent to the model" caption="Tip crop (model input)" />}
                      {r.files.preview && <CheckImage path={r.files.preview} alt="L515 snapshot with the tip region" caption="Camera snapshot" />}
                    </div>
                  )}
                </div>
              )}
            </li>
          );
        })}
        {pending && <PendingRow pending={pending} n={checks.length + 1} />}
      </ul>
    </div>
  );
}
