import React, { useCallback, useState } from 'react';
import { getResult } from './shared/anomalyApi';
import { verdict, stepLabel, TONE_CLASSES } from './shared/verdict';
import usePolling from './shared/usePolling';

const POLL_MS = 5000;

// Clog-detection verdicts for the clips recorded during the current/last run.
export default function ClogCheckCard({ clipIds }) {
  // clip_id -> { job } | { fetchError, unreachable }
  const [results, setResults] = useState({});
  const [expanded, setExpanded] = useState(null);

  const pendingIds = clipIds.filter((id) => !(results[id]?.job && verdict(results[id].job).terminal));

  const poll = useCallback(() => {
    pendingIds.forEach((id) => {
      getResult(id)
        .then((job) => setResults((prev) => ({ ...prev, [id]: { job } })))
        .catch((e) =>
          setResults((prev) => ({
            ...prev,
            [id]: { job: prev[id]?.job, fetchError: e.message, unreachable: !e.status },
          }))
        );
    });
  }, [pendingIds.join(',')]); // eslint-disable-line react-hooks/exhaustive-deps

  usePolling(poll, POLL_MS, pendingIds.length > 0);

  const verdicts = clipIds.map((id) => verdict(results[id]?.job));
  const counts = verdicts.reduce((acc, v) => ({ ...acc, [v.label]: (acc[v.label] || 0) + 1 }), {});
  const summary = Object.entries(counts).map(([label, n]) => `${n} ${label}`).join(', ');
  const unreachable = clipIds.some((id) => results[id]?.unreachable);

  return (
    <div className="bg-white p-4 rounded-2xl shadow mb-8">
      <div className="flex items-center justify-between mb-2">
        <h3 className="font-semibold text-gray-800">Clog check</h3>
        <span className="text-sm text-gray-500">
          {clipIds.length} recording{clipIds.length === 1 ? '' : 's'} — {summary}
        </span>
      </div>
      {unreachable && (
        <p className="text-xs text-amber-700 mb-2">Can't reach the clog-detection service.</p>
      )}
      <ul className="divide-y">
        {clipIds.map((id, i) => {
          const entry = results[id];
          const job = entry?.job;
          const v = verdicts[i];
          const open = expanded === id;
          return (
            <li key={id} className="py-2">
              <button
                type="button"
                onClick={() => setExpanded(open ? null : id)}
                className="w-full flex items-center justify-between gap-4 text-left"
              >
                <span className="text-sm text-gray-700">
                  Aspirate {i + 1}: {stepLabel(job, id)}
                </span>
                <span className={`text-xs px-2 py-1 rounded-full shrink-0 ${TONE_CLASSES[v.tone]}`}>
                  {v.label}
                  {v.detail ? ` · ${v.detail}` : ''}
                </span>
              </button>
              {open && (
                <div className="mt-2 text-xs text-gray-600 space-y-1">
                  <p className="font-mono">{id}</p>
                  {job?.summary && <p>{job.summary}</p>}
                  {job?.error && <p className="text-amber-700">{job.error}</p>}
                  {entry?.fetchError && <p className="text-amber-700">Fetch: {entry.fetchError}</p>}
                </div>
              )}
            </li>
          );
        })}
      </ul>
    </div>
  );
}
