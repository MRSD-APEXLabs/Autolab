import React, { useState } from 'react';
import { tipCheckFileUrl } from './shared/tipCheckApi';
import { aspirateLabel, tipCheckVerdict, TONE_CLASSES } from './shared/verdict';

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

  return (
    <div className="bg-white p-4 rounded-2xl shadow mb-8">
      <div className="flex items-center justify-between mb-2">
        <h3 className="font-semibold text-gray-800">Clog check</h3>
        <span className="text-sm text-gray-500">
          {checks.length} tip check{checks.length === 1 ? '' : 's'} — {summary}
        </span>
      </div>
      <ul className="divide-y">
        {checks.map((r, i) => {
          const v = verdicts[i];
          const open = expanded === r.id;
          return (
            <li key={r.id} className="py-2">
              <button
                type="button"
                onClick={() => setExpanded(open ? null : r.id)}
                className="w-full flex items-center justify-between gap-4 text-left"
              >
                <span className="text-sm text-gray-700">
                  Aspirate {i + 1}
                  {Number.isInteger(r.step) ? ` (step ${r.step + 1})` : ''}: {aspirateLabel(r.aspirate)}
                </span>
                <span className={`text-xs px-2 py-1 rounded-full shrink-0 ${TONE_CLASSES[v.tone]}`}>
                  {v.label}
                  {v.detail ? ` · ${v.detail}` : ''}
                </span>
              </button>
              {open && (
                <div className="mt-2 text-xs text-gray-600 space-y-1">
                  {r.summary && <p>{r.summary}</p>}
                  {Array.isArray(r.samples) && r.samples.length > 0 && (
                    <p>
                      Checks: {r.samples.map((p) => Number(p).toFixed(2)).join(', ')}
                      {typeof r.threshold === 'number' ? ` (clog at ≥ ${r.threshold.toFixed(2)})` : ''}
                    </p>
                  )}
                  {r.model && <p>Model: {r.model}</p>}
                  {r.warning && <p className="text-amber-700">{r.warning}</p>}
                  {r.error && <p className="text-amber-700">{r.error}</p>}
                  {r.files?.crop && (
                    <div className="flex flex-wrap gap-2 pt-1">
                      <img src={tipCheckFileUrl(r.files.crop)} alt="Tip crop sent to the model"
                        className="h-32 rounded border" />
                      {r.files.preview && (
                        <img src={tipCheckFileUrl(r.files.preview)} alt="L515 snapshot with the tip region"
                          className="h-32 rounded border" />
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
