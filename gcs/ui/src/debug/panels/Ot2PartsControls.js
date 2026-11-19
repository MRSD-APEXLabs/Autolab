import React, { useEffect, useRef, useState } from 'react';
import { useMqttContext } from '../../shared/useMqtt';
import ConfirmButton from '../ConfirmButton';

export const COMMAND_TOPIC = 'cmd/lab_machine/ot2_parts';
export const STATUS_TOPIC = 'ot2/parts/status';

// locked: another control (the Part 1 -> clog check -> Part 2 sequence) is driving the parts.
export default function Ot2PartsControls({ onActive, locked = false }) {
  const { publish, subscribe, connected } = useMqttContext();
  const [status, setStatus] = useState(null);
  const [pending, setPending] = useState(false);
  const [tip, setTip] = useState('2');
  const [sourceTube, setSourceTube] = useState('A1');
  const timer = useRef(null);
  useEffect(() => subscribe(STATUS_TOPIC, (message) => {
    try {
      setStatus(JSON.parse(message));
      setPending(false);
      clearTimeout(timer.current);
    } catch { setStatus({ status: 'needs_inspection', error: 'Unreadable OT-2 status' }); }
  }), [subscribe]);
  useEffect(() => {
    if (connected) publish(COMMAND_TOPIC, JSON.stringify({ part: 'status' }));
  }, [connected, publish]);
  useEffect(() => () => clearTimeout(timer.current), []);
  const stage = status?.status;
  const active = pending || !status || !['idle', 'completed'].includes(stage);
  useEffect(() => { onActive(active); }, [active, onActive]);
  const validTip = /^\d+$/.test(tip) && Number(tip) >= 0 && Number(tip) < 96;
  const send = (part) => {
    setPending(true);
    timer.current = setTimeout(() => {
      setPending(false);
      setStatus({ status: 'needs_inspection', error: 'No acknowledgement. Check robot before retrying.' });
    }, 10000);
    publish(COMMAND_TOPIC, JSON.stringify(part === 1
      ? { part: 1, tip_index: Number(tip), volume: 200, source_tube: sourceTube } : { part }));
  };
  return (
    <div className="border rounded p-3 mb-3">
      <h4 className="text-sm font-semibold mb-2">OT-2 two-part trial</h4>
      <p className="text-xs text-gray-600 mb-2">
        Part 1: pick up a fresh tip and 200 µL from tube {sourceTube}, and wait fully raised above that tube.
        Part 2: dispense into plate A1 and discard the tip. The connection stays open for the next transfer.
      </p>
      <p className="text-xs text-gray-600 mb-2">
        Use Home &amp; discard tip to initialize or reset. This discards any mounted tip, including held liquid.
      </p>
      <label className="text-xs block mb-2">
        Source test tube
        <select value={sourceTube} disabled={active || locked}
          onChange={(e) => setSourceTube(e.target.value)}
          className="border rounded px-2 py-1 ml-2">
          <option value="A1">Test tube 1 (A1)</option>
          <option value="A2">Test tube 2 (A2)</option>
          <option value="A3">Test tube 3 (A3)</option>
          <option value="A4">Test tube 4 (A4)</option>
        </select>
      </label>
      <label className="text-xs block mb-2">
        Unused tip index (0=A1, 1=B1, 2=C1)
        <input type="number" min="0" max="95" step="1" value={tip}
          disabled={active} onChange={(e) => setTip(e.target.value)}
          className="border rounded px-2 py-1 ml-2 w-20" />
      </label>
      <div className="flex gap-2 flex-wrap">
        <ConfirmButton disabled={!connected || locked || pending || !status || stage === 'running'}
          onConfirm={() => send('cleanup')}>
          Home &amp; discard tip
        </ConfirmButton>
        <ConfirmButton disabled={!connected || locked || active || !validTip} onConfirm={() => send(1)}>
          OT-2 Part 1 — Aspirate & Hold
        </ConfirmButton>
        <ConfirmButton disabled={!connected || locked || pending || stage !== 'ready_for_part2'} onConfirm={() => send(2)}>
          OT-2 Part 2 — Dispense & Finish
        </ConfirmButton>
      </div>
      <p role="status" className="text-xs mt-2">
        {pending ? 'Sending command…' : stage === 'ready_for_part2'
          ? `Holding above tube ${status?.source_tube || 'A1'}. Ready for Part 2.` : stage || 'Waiting for OT-2 status…'}
        {status?.message && stage !== 'completed' && ` · ${status.message}`}
      </p>
      {status?.error && <p role="alert" className="text-xs text-red-600 mt-1">{status.error}</p>}
    </div>
  );
}
