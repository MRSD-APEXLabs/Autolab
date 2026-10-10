import React, { useEffect, useRef, useState } from 'react';
import { useMqttContext } from '../../shared/useMqtt';
import useCameraHub from '../../shared/useCameraHub';
import usePerception from '../../shared/usePerception';
import CameraPair from '../../shared/CameraPair';
import { SELECT_CAMERA_TOPIC, SELECT_RESULT_TOPIC, cameraLabel } from '../../shared/cameras';
import ConfirmButton from '../ConfirmButton';
import DetectionTables from '../DetectionTables';

// The bridge answers within 45 s (camera_select.SELECT_TIMEOUT_S); give up a little later.
export const SELECT_TIMEOUT_MS = 50000;

const CAMERAS = [
  { label: 'ZED X (base)', value: 'zedx' },
  { label: 'ZED X Nano (wrist)', value: 'zedx_nano' },
  { label: 'Release', value: null },
];

function hubLine(hub) {
  if (hub.receivedAt == null) return 'Hub: no status yet';
  const parts = [`Hub: ${cameraLabel(hub.active)} · ${hub.state ?? '—'}`];
  if (hub.error) parts.push(hub.error);
  parts.push(hub.reachable === false ? 'unreachable' : 'reachable');
  return parts.join(' · ');
}

export default function PerceptionPanel() {
  const { publish, subscribe, connected } = useMqttContext();
  const hub = useCameraHub();
  const perception = usePerception(hub.active);
  const [pending, setPending] = useState(false);
  const [result, setResult] = useState(null);
  const timer = useRef(null);

  const finish = (next) => {
    clearTimeout(timer.current);
    setPending(false);
    setResult(next);
  };

  useEffect(() => subscribe(SELECT_RESULT_TOPIC, (message) => {
    try {
      finish(JSON.parse(message));
    } catch {
      finish({ ok: false, message: 'Unreadable select result' });
    }
  }), [subscribe]);

  useEffect(() => () => clearTimeout(timer.current), []);

  const select = (camera) => {
    setResult(null);
    setPending(true);
    clearTimeout(timer.current);
    timer.current = setTimeout(
      () => finish({ ok: false, message: 'No reply from the MQTT bridge' }), SELECT_TIMEOUT_MS);
    publish(SELECT_CAMERA_TOPIC, JSON.stringify(camera));
  };

  const hubBad = hub.state === 'error' || hub.reachable === false;
  return (
    <section className="border border-gray-200 rounded-xl p-4 bg-white md:col-span-2">
      <h2 className="font-semibold text-gray-800 mb-3">Perception</h2>

      <div className="flex flex-wrap items-center gap-2 mb-2">
        {CAMERAS.map(({ label, value }) => (
          <ConfirmButton
            key={value ?? 'release'}
            disabled={!connected || pending}
            active={value !== null && hub.active === value}
            onConfirm={() => select(value)}
          >
            {label}
          </ConfirmButton>
        ))}
        {pending && <span className="text-sm text-gray-500">Switching…</span>}
        {!pending && result && !result.ok && <span className="text-sm text-red-600">{result.message}</span>}
      </div>
      <p className={`text-xs mb-3 ${hubBad ? 'text-red-600' : 'text-gray-600'}`}>{hubLine(hub)}</p>

      <div className="mb-4">
        <CameraPair camera={hub.active} hubReachable={hub.reachable !== false} />
      </div>

      <DetectionTables camera={hub.active} perception={perception} />
    </section>
  );
}
