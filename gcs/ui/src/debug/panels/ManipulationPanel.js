import React, { useState } from 'react';
import { useMqttContext } from '../useMqtt';
import ConfirmButton from '../ConfirmButton';
import StatusEcho from '../StatusEcho';

const TYPES = ['pick_up', 'place', 'pick_base', 'pick_wellplate'];
const TARGET_MACHINES = ['ot2', 'shaker'];

export default function ManipulationPanel() {
  const { publish, connected } = useMqttContext();
  const [type, setType] = useState(TYPES[0]);
  const [objectType, setObjectType] = useState('well_plate');
  const [targetMachine, setTargetMachine] = useState(TARGET_MACHINES[0]);

  const send = () => {
    publish('cmd/manipulation/command', JSON.stringify({
      type,
      object_type: objectType,
      target_machine: type === 'place' ? targetMachine : '',
    }));
  };

  return (
    <section className="border border-gray-200 rounded-xl p-4 bg-white">
      <h2 className="font-semibold text-gray-800 mb-3">Manipulation</h2>
      <div className="grid grid-cols-2 gap-2 mb-3">
        <label className="text-xs text-gray-600">
          Type
          <select value={type} onChange={(e) => setType(e.target.value)} className="w-full border rounded px-2 py-1 mt-1">
            {TYPES.map((t) => <option key={t} value={t}>{t}</option>)}
          </select>
        </label>
        <label className="text-xs text-gray-600">
          Object Type
          <input value={objectType} onChange={(e) => setObjectType(e.target.value)} className="w-full border rounded px-2 py-1 mt-1" />
        </label>
        {type === 'place' && (
          <label className="text-xs text-gray-600">
            Target Machine
            <select value={targetMachine} onChange={(e) => setTargetMachine(e.target.value)} className="w-full border rounded px-2 py-1 mt-1">
              {TARGET_MACHINES.map((m) => <option key={m} value={m}>{m}</option>)}
            </select>
          </label>
        )}
      </div>
      <ConfirmButton onConfirm={send} disabled={!connected}>Send Command</ConfirmButton>
      <div className="mt-3">
        <StatusEcho topic="ros2/robot_1/behavior/manipulation_phase" label="Manipulation Phase" />
      </div>
    </section>
  );
}
