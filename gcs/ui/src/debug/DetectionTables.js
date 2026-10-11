import React from 'react';
import { cameraLabel } from '../shared/cameras';

const STALE_AFTER_S = 2;
const fmt = (v, dp) => (v == null || !Number.isFinite(v) ? '—' : v.toFixed(dp));
const fmtVec = (v, dp) => (v ? v.map((x) => fmt(x, dp)).join(', ') : '—');

function pretty(raw) {
  try {
    return JSON.stringify(JSON.parse(raw), null, 2);
  } catch {
    return raw;
  }
}

const TH = 'px-2 py-1 text-left font-semibold text-gray-600 whitespace-nowrap';
const TD = 'px-2 py-1 font-mono text-right whitespace-nowrap';

function Badge({ children, tone }) {
  const tones = { warn: 'bg-amber-100 text-amber-800', ok: 'bg-green-100 text-green-800' };
  return <span className={`ml-1 px-1.5 rounded text-[10px] font-sans ${tones[tone]}`}>{children}</span>;
}

function TagTable({ tags }) {
  return (
    <table aria-label="AprilTags" className="w-full text-xs">
      <thead>
        <tr className="border-b border-gray-200">
          <th className={TH}>ID</th><th className={TH}>x, y, z (m)</th><th className={TH}>dist (m)</th>
          <th className={TH}>roll, pitch, yaw (°)</th><th className={TH}>pose src</th>
          <th className={TH}>size meas / nom (mm)</th><th className={TH}>reproj (px)</th>
        </tr>
      </thead>
      <tbody>
        {tags.length === 0 && <tr><td colSpan={7} className="px-2 py-2 text-gray-400">No tags</td></tr>}
        {tags.map((t) => (
          <tr key={t.id} className={`border-b border-gray-100 ${t.position ? '' : 'text-gray-400'}`}>
            <td className={TD}>{t.id}</td>
            {t.position ? (
              <>
                <td className={TD}>{fmtVec(t.position, 3)}</td>
                <td className={TD}>{fmt(t.distance, 3)}</td>
                <td className={TD}>{fmtVec(t.rpyDeg, 1)}</td>
              </>
            ) : (
              <td colSpan={3} className="px-2 py-1 italic">no pose</td>
            )}
            <td className={TD}>{t.poseSource ?? '—'}</td>
            <td className={TD}>
              {fmt(t.measuredSizeM != null ? t.measuredSizeM * 1000 : null, 1)}
              {' / '}
              {fmt(t.sizeM != null ? t.sizeM * 1000 : null, 1)}
              {t.sizeWarning && <Badge tone="warn">{`${fmt(t.sizeOffPct, 0)}% off`}</Badge>}
            </td>
            <td className={TD}>{fmt(t.reprojectionPx, 2)}</td>
          </tr>
        ))}
      </tbody>
    </table>
  );
}

function PlateTable({ plates, targetLabel }) {
  return (
    <table aria-label="Wellplates" className="w-full text-xs">
      <thead>
        <tr className="border-b border-gray-200">
          <th className={TH}>#</th><th className={TH}>class</th><th className={TH}>conf</th>
          <th className={TH}>x, y, z (m)</th><th className={TH}>size (mm)</th><th className={TH}>pose src</th>
        </tr>
      </thead>
      <tbody>
        {plates.length === 0 && <tr><td colSpan={6} className="px-2 py-2 text-gray-400">No detections</td></tr>}
        {plates.map((p, i) => (
          // detections carry no id; their rank is their identity within a frame
          // eslint-disable-next-line react/no-array-index-key
          <tr key={i} className={`border-b border-gray-100 ${p.position ? '' : 'text-gray-400'}`}>
            <td className={TD}>{i + 1}</td>
            <td className="px-2 py-1 whitespace-nowrap">
              {p.name}
              {p.plannerTarget && <Badge tone="ok">{targetLabel}</Badge>}
            </td>
            <td className={TD}>{fmt(p.conf, 2)}</td>
            {p.position ? (
              <>
                <td className={TD}>{fmtVec(p.position, 3)}</td>
                <td className={TD}>{p.sizeMm ? p.sizeMm.map((v) => fmt(v, 0)).join(' × ') : '—'}</td>
              </>
            ) : (
              <td colSpan={2} className="px-2 py-1 italic">no depth → no marker</td>
            )}
            <td className={TD}>{p.poseSource ?? '—'}</td>
          </tr>
        ))}
      </tbody>
    </table>
  );
}

// Readable view of camera_perception's per-frame JSON (usePerception's result).
export default function DetectionTables({ camera, perception }) {
  const { tags, plates, frameId, error, raw, received, ageS } = perception;
  const stale = received && ageS > STALE_AFTER_S;
  return (
    <div className="grid gap-3">
      <div className="flex flex-wrap gap-x-3 text-xs text-gray-600">
        <span className="font-semibold">{cameraLabel(camera)}</span>
        <span className="font-mono">{frameId ?? '—'}</span>
        <span className={stale ? 'text-red-600 font-semibold' : 'text-gray-500'}>
          {received ? `updated ${fmt(ageS, 1)} s ago` : 'no data'}
        </span>
      </div>
      {error && <p className="text-xs text-red-600">Unparseable — see Raw JSON</p>}
      <div className={`grid gap-3 overflow-x-auto ${stale ? 'opacity-50' : ''}`}>
        <div>
          <h3 className="text-xs font-semibold text-gray-700 mb-1">AprilTags</h3>
          <TagTable tags={tags} />
        </div>
        <div>
          <h3 className="text-xs font-semibold text-gray-700 mb-1">Wellplates</h3>
          <PlateTable plates={plates} targetLabel={camera === 'zedx' ? 'planner target' : 'best'} />
        </div>
      </div>
      <details className="text-xs">
        <summary className="cursor-pointer text-gray-500">Raw JSON</summary>
        <pre className="mt-1 p-2 bg-gray-50 rounded overflow-auto max-h-64 whitespace-pre-wrap break-all">
          {raw ? pretty(raw) : '—'}
        </pre>
      </details>
    </div>
  );
}
