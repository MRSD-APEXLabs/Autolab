import React, { useCallback, useState } from 'react';
import {
  TIP_CHECK_URL, CHECK_PASSES, getTipCheckHealth, runTipCheck, tipCheckFileUrl,
} from '../../shared/tipCheckApi';
import { verdict, TONE_CLASSES } from '../../shared/verdict';
import usePolling from '../../shared/usePolling';

const HEALTH_POLL_MS = 5000;

// Verdict, probability, timing and the images the model saw, for one tip-check job.
export function TipCheckResult({ job }) {
  if (!job) return null;
  const v = verdict(job);
  return (
    <div className="mt-2 text-xs">
      <span className={`px-2 py-0.5 rounded-full ${TONE_CLASSES[v.tone]}`}>{v.label}</span>
      {typeof job.p_tip_empty === 'number' && (
        <span className="ml-2">P(tip empty) = {job.p_tip_empty.toFixed(2)}</span>
      )}
      {job.sample_count && <p className="mt-1">
        Average of {job.sample_count} predictions · Individual P(empty):{' '}
        {job.samples.map((sample) => sample.p_tip_empty.toFixed(2)).join(', ')}
      </p>}
      {job.sample_count && job.files?.crop && <p className="text-gray-500">Images below are from check 1.</p>}
      {job.timings_s?.total != null && <span className="ml-2 text-gray-500">{job.timings_s.total} s</span>}
      {job.summary && <p className="mt-1 text-gray-700">{job.summary}</p>}
      {job.warning && <p role="alert" className="mt-1 text-amber-700">{job.warning}</p>}
      {job.error && <p role="alert" className="mt-1 text-red-600">{job.error}</p>}
      {job.files?.crop && (
        <div className="flex gap-2 mt-2">
          <img src={tipCheckFileUrl(job.files.crop)} alt="Tip crop sent to the model"
               className="h-40 border rounded" />
          <img src={tipCheckFileUrl(job.files.preview)} alt="L515 snapshot with the tip region"
               className="h-40 border rounded" />
        </div>
      )}
    </div>
  );
}

function ServiceLine({ health, error }) {
  if (error) return <p className="text-xs text-red-600 mb-2">Tip-check service unreachable at {TIP_CHECK_URL}: {error}</p>;
  if (!health) return <p className="text-xs text-gray-500 mb-2">Checking {TIP_CHECK_URL}…</p>;
  return (
    <p className="text-xs text-gray-600 mb-2">
      L515: {health.camera_present ? 'ok' : 'NOT found'} · Model on p5090: {health.model_reachable ? 'reachable' : 'not connected yet'}
      {health.busy ? ' · check running' : ''}
    </p>
  );
}

// One click: L515 snapshot -> tip crop -> d3:4b on p5090 -> verdict.
export default function TipCheckControls() {
  const [health, setHealth] = useState(null);
  const [healthError, setHealthError] = useState(null);
  const [job, setJob] = useState(null);
  const [busy, setBusy] = useState(false);
  const [progress, setProgress] = useState(0);
  const [error, setError] = useState(null);

  const refresh = useCallback(() => {
    getTipCheckHealth()
      .then((h) => { setHealth(h); setHealthError(null); })
      .catch((e) => setHealthError(e.message));
  }, []);
  usePolling(refresh, HEALTH_POLL_MS);

  const onRun = async () => {
    setBusy(true);
    setProgress(0);
    setError(null);
    try {
      setJob(await runTipCheck({ step: 'debug tip check' }, { onProgress: setProgress }));
    } catch (e) {
      setError(e.message);
    } finally {
      setBusy(false);
      refresh();
    }
  };

  return (
    <div className="mb-4 border rounded p-3">
      <h3 className="text-sm font-semibold text-gray-700 mb-1">Tip check (L515 + d3:4b)</h3>
      <p className="text-xs text-gray-600 mb-2">
        Runs d3:4b five times on snapshots of the held tip and averages the prediction probabilities.
        Use it while the OT-2 holds a tip in view of the L515 (the robot checks every aspirate by itself).
      </p>
      <ServiceLine health={health} error={healthError} />
      <button type="button" onClick={onRun} disabled={busy || health?.busy}
              className="px-3 py-1.5 rounded text-sm text-white bg-blue-600 hover:bg-blue-700 disabled:bg-gray-300">
        {busy ? `Checking… ${progress}/${CHECK_PASSES}` : 'Run clog check'}
      </button>
      {error && <p className="text-xs text-red-600 mt-2">Error: {error}</p>}
      <TipCheckResult job={job} />
    </div>
  );
}
