import React, { useState } from 'react';
import { tipCheckFileUrl } from './shared/tipCheckApi';
import { aspirateLabel, tipCheckVerdict } from './shared/verdict';

// verdict tone → deck badge / accent class
const TONE = { ok: 'ok', bad: 'bad', warn: 'warn', pending: 'busy', none: '' };

// Tip-check results the robot attached to the current/last run (status.recordings).
// lab_machine_executive runs the check after each aspirate and the result arrives complete,
// so nothing is polled; only the images are loaded from the tip-check service.
export default function ClogCheckCard({ recordings = [] }) {
  const [expanded, setExpanded] = useState(null);
  const checks = recordings.filter((r) => r && typeof r === 'object' && r.type === 'tip_check' && r.id);
  if (checks.length === 0) return null;

  const verdicts = checks.map(tipCheckVerdict);
  const counts = verdicts.reduce((acc, v) => ({ ...acc, [v.label]: (acc[v.label] || 0) + 1 }), {});
  const summary = Object.entries(counts).map(([label, n]) => `${n} ${label}`).join(', ');

  const worst = verdicts.some((v) => v.tone === 'bad') ? 'bad' : verdicts.some((v) => v.tone === 'warn') ? 'warn' : 'ok';

  return (
    <div className={`ax-panel ax-accent k-${worst} ax-rise`} style={{ '--i': 3 }}>
      <div className="ax-panel-head">
        <h3 className="ax-h3">Clog check</h3>
        <span className="ax-hint">
          {checks.length} tip check{checks.length === 1 ? '' : 's'} — {summary}
        </span>
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
                  {r.files?.crop && (
                    <div className="flex gap-2 pt-2 h-36">
                      <img src={tipCheckFileUrl(r.files.crop)} alt="Tip crop sent to the model"
                        className="h-full rounded-lg object-cover" style={{ border: '1px solid var(--line2)' }} />
                      {r.files.preview && (
                        <img src={tipCheckFileUrl(r.files.preview)} alt="L515 snapshot with the tip region"
                          className="h-full min-w-0 flex-1 rounded-lg object-cover" style={{ border: '1px solid var(--line2)' }} />
                      )}
                    </div>
                  )}
                </div>
              )}
            </li>
          );
        })}
      </ul>
    </div>
  );
}
