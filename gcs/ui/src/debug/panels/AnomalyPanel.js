import React, { useCallback, useState } from 'react';
import {
  ANOMALY_URL, getHealth, getJobs, startRecording, stopRecording, abortRecording,
} from '../../shared/anomalyApi';
import { verdict, stepLabel, TONE_CLASSES } from '../../shared/verdict';
import usePolling from '../../shared/usePolling';

const JOBS_POLL_MS = 3000;
const HEALTH_POLL_MS = 5000;

const BTN = 'px-3 py-1.5 rounded text-sm text-white disabled:bg-gray-300';

function HealthLine({ health, error }) {
  if (error) return <p className="text-xs text-red-600 mb-3">Service unreachable at {ANOMALY_URL}: {error}</p>;
  if (!health) return <p className="text-xs text-gray-500 mb-3">Checking {ANOMALY_URL}…</p>;
  return (
    <p className="text-xs text-gray-600 mb-3">
      Camera: {health.camera_present ? 'present' : 'NOT found'} · Recording: {health.recording || 'none'} ·
      Running: {health.running || 'none'} · Queued: {health.queued ?? 0}
    </p>
  );
}

function JobDetails({ job }) {
  const rows = [
    ['Summary', job.summary],
    ['Anomaly types', job.anomaly_types?.join(', ')],
    ['Tip readings', job.tip_readings && JSON.stringify(job.tip_readings)],
    ['Timings (s)', job.timings_s && JSON.stringify(job.timings_s)],
    ['Error', job.error && `${job.error}${job.failed_stage ? ` (${job.failed_stage})` : ''}`],
    ['Video', job.video],
    ['Report', job.report_path],
  ].filter(([, v]) => v);
  return (
    <dl className="grid grid-cols-[auto_1fr] gap-x-3 gap-y-1 text-xs p-2 bg-gray-50">
      {rows.map(([k, v]) => (
        <React.Fragment key={k}>
          <dt className="text-gray-500">{k}</dt>
          <dd className="font-mono break-all">{v}</dd>
        </React.Fragment>
      ))}
    </dl>
  );
}

export default function AnomalyPanel() {
  const [health, setHealth] = useState(null);
  const [healthError, setHealthError] = useState(null);
  const [jobs, setJobs] = useState([]);
  const [replayPath, setReplayPath] = useState('');
  const [actionError, setActionError] = useState(null);
  const [busy, setBusy] = useState(false);
  const [expanded, setExpanded] = useState(null);

  const refreshHealth = useCallback(() => {
    getHealth()
      .then((h) => { setHealth(h); setHealthError(null); })
      .catch((e) => setHealthError(e.message));
  }, []);
  const refreshJobs = useCallback(() => {
    getJobs().then((j) => setJobs(Array.isArray(j) ? j : [])).catch(() => {});
  }, []);

  usePolling(refreshHealth, HEALTH_POLL_MS);
  usePolling(refreshJobs, JOBS_POLL_MS);

  const run = async (fn) => {
    setBusy(true);
    setActionError(null);
    try {
      await fn();
    } catch (e) {
      setActionError(e.message);
    } finally {
      setBusy(false);
      refreshHealth();
      refreshJobs();
    }
  };

  const recording = !!health?.recording;

  const onStart = () => run(() => startRecording({ step: 'debug live clip' }));
  const onStop = () => run(() => stopRecording());
  const onAbort = () => run(() => abortRecording());
  const onReplay = () => run(async () => {
    await startRecording({ replay_file: replayPath.trim(), step: 'debug replay' });
    await stopRecording();
  });

  return (
    <section className="border border-gray-200 rounded-xl p-4 bg-white md:col-span-2">
      <h2 className="font-semibold text-gray-800 mb-1">Clog Detection</h2>
      <HealthLine health={health} error={healthError} />

      <div className="mb-3">
        <h3 className="text-sm font-semibold text-gray-700 mb-2">Live clip</h3>
        <div className="flex gap-2">
          <button type="button" onClick={onStart} disabled={busy || recording}
                  className={`${BTN} bg-blue-600 hover:bg-blue-700`}>Start recording</button>
          <button type="button" onClick={onStop} disabled={busy || !recording}
                  className={`${BTN} bg-green-600 hover:bg-green-700`}>Stop &amp; analyze</button>
          <button type="button" onClick={onAbort} disabled={busy || !recording}
                  className={`${BTN} bg-red-600 hover:bg-red-700`}>Abort</button>
        </div>
      </div>

      <div className="mb-3">
        <h3 className="text-sm font-semibold text-gray-700 mb-2">Replay a video</h3>
        <div className="flex gap-2">
          <input
            value={replayPath}
            onChange={(e) => setReplayPath(e.target.value)}
            placeholder="/path/on/gcs/desktop/clip.mp4"
            aria-label="Replay video path"
            className="flex-1 border rounded px-2 py-1 text-sm font-mono"
          />
          <button type="button" onClick={onReplay} disabled={busy || recording || !replayPath.trim()}
                  className={`${BTN} bg-blue-600 hover:bg-blue-700`}>Run replay</button>
        </div>
      </div>

      {actionError && <p className="text-xs text-red-600 mb-3">Error: {actionError}</p>}

      <h3 className="text-sm font-semibold text-gray-700 mb-2">Recent jobs</h3>
      {jobs.length === 0 ? (
        <p className="text-xs text-gray-500">No jobs yet.</p>
      ) : (
        <div className="overflow-x-auto">
          <table className="w-full text-xs">
            <thead>
              <tr className="text-left text-gray-500">
                <th className="py-1 pr-2">Clip</th>
                <th className="py-1 pr-2">Created</th>
                <th className="py-1 pr-2">Step</th>
                <th className="py-1 pr-2">Status</th>
                <th className="py-1 pr-2">Verdict</th>
                <th className="py-1 pr-2">Confidence</th>
              </tr>
            </thead>
            <tbody>
              {jobs.map((job) => {
                const v = verdict(job);
                const open = expanded === job.clip_id;
                return (
                  <React.Fragment key={job.clip_id}>
                    <tr className="border-t cursor-pointer hover:bg-gray-50"
                        onClick={() => setExpanded(open ? null : job.clip_id)}>
                      <td className="py-1 pr-2 font-mono">{job.clip_id}</td>
                      <td className="py-1 pr-2">{job.created ?? '—'}</td>
                      <td className="py-1 pr-2">{stepLabel(job, '—')}</td>
                      <td className="py-1 pr-2">{job.status}{job.stage ? ` (${job.stage})` : ''}</td>
                      <td className="py-1 pr-2">
                        <span className={`px-2 py-0.5 rounded-full ${TONE_CLASSES[v.tone]}`}>
                          {v.label}{v.detail ? ` · ${v.detail}` : ''}
                        </span>
                      </td>
                      <td className="py-1 pr-2">{job.confidence ?? '—'}</td>
                    </tr>
                    {open && (
                      <tr><td colSpan={6}><JobDetails job={job} /></td></tr>
                    )}
                  </React.Fragment>
                );
              })}
            </tbody>
          </table>
        </div>
      )}
    </section>
  );
}
