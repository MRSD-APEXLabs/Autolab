import React, { useEffect, useState } from 'react';
import { useMqttContext } from '../../shared/useMqtt';
import { unwrapString } from '../../shared/unwrapRos';
import ConfirmButton from '../ConfirmButton';
import ClogCheckCard from '../../ClogCheckCard';

const START_TOPIC = 'cmd/routine_executor/start_routine_cmd';
const STATUS_TOPIC = 'ros2/routine_executor/status';
const ACTIVE = ['running', 'paused'];
const TUBES = { A1: 0, A2: 3, A3: 6, A4: 9 };  // tube_rack children of the four source tubes

// One OT-2 transfer. lab_machine_executive runs the tip check (five averaged checks) right after
// the aspirate and continues to the dispense whatever it says.
export function transferProtocol({ tip, tube, volume }) {
  return { steps: [
    { action: 'pick_up_tips', resource_name: 'tip_rack', well_indices: [tip] },
    { action: 'aspirate', resource_name: 'tube_rack', well_indices: [TUBES[tube]], volumes: [volume] },
    { action: 'dispense', resource_name: 'empty_plate', well_indices: [0], volumes: [volume] },
    { action: 'drop_tips' },
  ] };
}

// Discards the mounted tip (and anything in it); the executive then homes and disconnects.
export function cleanupProtocol() {
  return { steps: [{ action: 'drop_tips' }] };
}

const ot2Step = (parameters_json) => ({ name: 'ot2', parameters_json });

// OT-2 debug flows as routines on the routine executor (ot2 steps are never retried).
export default function Ot2Routines({ onActive = () => {} }) {
  const { publish, subscribe, connected } = useMqttContext();
  const [status, setStatus] = useState(null);
  const [tip, setTip] = useState('0');
  const [tube, setTube] = useState('A1');
  const [volume, setVolume] = useState('200');

  useEffect(() => subscribe(STATUS_TOPIC, (message) => {
    try {
      setStatus(JSON.parse(unwrapString(message)));
    } catch {
      // keep the last good status
    }
  }), [subscribe]);

  const state = status?.state ?? 'idle';
  const active = status === null || ACTIVE.includes(state);
  useEffect(() => { onActive(active); }, [active, onActive]);

  const tipN = Number(tip);
  const volumeN = Number(volume);
  const validTip = /^\d+$/.test(tip) && tipN >= 0 && tipN < 96;
  const validNextTip = validTip && tipN < 95;
  const validVolume = volume.trim() !== '' && Number.isFinite(volumeN) && volumeN > 0 && volumeN <= 200;
  const blocked = !connected || active;

  const start = (steps) => publish(START_TOPIC, JSON.stringify({ steps, max_retries: 0, robot: 'robot_1' }));
  const runTransfer = () => start([ot2Step(transferProtocol({ tip: tipN, tube, volume: volumeN }))]);
  const runFull = () => start([
    ot2Step(transferProtocol({ tip: tipN, tube: 'A1', volume: volumeN })),
    ot2Step(transferProtocol({ tip: tipN + 1, tube: 'A2', volume: volumeN })),
  ]);
  const runCleanup = () => start([ot2Step(cleanupProtocol())]);

  const current = status?.current_step ?? 0;
  const total = status?.steps?.length ?? 0;

  return (
    <div className="border rounded p-3 mb-3">
      <h4 className="text-sm font-semibold mb-2">OT-2 routines</h4>
      <p className="text-xs text-gray-600 mb-2">
        Sent to the routine executor. After each aspirate the robot runs the tip check, then
        dispenses into plate A1 and drops the tip. A failed OT-2 step is never retried automatically.
      </p>
      <div className="flex flex-wrap gap-3 mb-2">
        <label className="text-xs">
          Source test tube
          <select value={tube} disabled={active} onChange={(e) => setTube(e.target.value)}
            className="border rounded px-2 py-1 ml-2">
            {Object.keys(TUBES).map((t) => <option key={t} value={t}>{t}</option>)}
          </select>
        </label>
        <label className="text-xs">
          Unused tip index (0=A1, 1=B1)
          <input type="number" min="0" max="95" step="1" value={tip} disabled={active}
            onChange={(e) => setTip(e.target.value)} className="border rounded px-2 py-1 ml-2 w-20" />
        </label>
        <label className="text-xs">
          Volume (µL)
          <input type="number" min="1" max="200" value={volume} disabled={active}
            onChange={(e) => setVolume(e.target.value)} className="border rounded px-2 py-1 ml-2 w-20" />
        </label>
      </div>
      <div className="flex gap-2 flex-wrap">
        <ConfirmButton disabled={blocked || !validTip || !validVolume} onConfirm={runTransfer}>
          Aspirate → tip check → dispense
        </ConfirmButton>
        <ConfirmButton disabled={blocked || !validNextTip || !validVolume} onConfirm={runFull}>
          Full workflow (tube A1, then A2)
        </ConfirmButton>
        <ConfirmButton disabled={blocked} onConfirm={runCleanup}>
          Discard tip (cleanup)
        </ConfirmButton>
      </div>
      <p role="status" className="text-xs mt-2">
        {status === null && 'Waiting for routine executor…'}
        {status !== null && ACTIVE.includes(state) && `${state === 'paused' ? 'Paused' : 'Running'}: step ${current + 1}/${total}`}
        {status !== null && state === 'success' && 'Routine complete'}
        {status !== null && state === 'idle' && 'Idle'}
      </p>
      {state === 'failed' && (
        <p role="alert" className="text-xs text-red-600 mt-1">
          Stopped at step {current + 1}: {status.error ?? 'failed'}
        </p>
      )}
      {status?.recordings?.length > 0 && <ClogCheckCard recordings={status.recordings} />}
    </div>
  );
}
