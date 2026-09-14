import React, { useState } from 'react';
import { useMqttContext } from '../useMqtt';
import ConfirmButton from '../ConfirmButton';

const DEFAULT_OT2_PARAMS = JSON.stringify({
  steps: [
    { action: 'pick_up_tips', resource_name: 'tip_rack', well_indices: [0] },
    { action: 'aspirate', resource_name: 'tube_rack', well_indices: [0], volumes: [200] },
    { action: 'dispense', resource_name: 'empty_plate', well_indices: [0], volumes: [40] },
    { action: 'return_tips' },
  ],
}, null, 2);

function Ot2Section({ publish, connected }) {
  const [paramsJson, setParamsJson] = useState(DEFAULT_OT2_PARAMS);
  const [error, setError] = useState(null);

  const onChange = (value) => {
    setParamsJson(value);
    try {
      JSON.parse(value);
      setError(null);
    } catch (e) {
      setError(e.message);
    }
  };

  const send = () => {
    publish('cmd/lab_machine/ot2_command', JSON.stringify({
      action: 'protocol',
      parameters_json: JSON.parse(paramsJson),
    }));
  };

  return (
    <div className="mb-4">
      <h3 className="text-sm font-semibold text-gray-700 mb-2">OT2</h3>
      <label className="text-xs text-gray-600 block mb-2">
        Parameters JSON
        <textarea
          value={paramsJson}
          onChange={(e) => onChange(e.target.value)}
          rows={8}
          className="w-full border rounded px-2 py-1 mt-1 font-mono text-xs"
        />
      </label>
      {error && <p className="text-xs text-red-600 mb-2">Invalid JSON: {error}</p>}
      <ConfirmButton onConfirm={send} disabled={!connected || !!error}>Send OT2 Protocol</ConfirmButton>
    </div>
  );
}

function ShakerSection({ publish, connected }) {
  const [pwm, setPwm] = useState('150');
  const [waitTimeS, setWaitTimeS] = useState('20');

  const send = () => {
    publish('cmd/lab_machine/shaker_command', JSON.stringify({
      pwm: parseFloat(pwm),
      wait_time_s: parseFloat(waitTimeS),
    }));
  };

  return (
    <div>
      <h3 className="text-sm font-semibold text-gray-700 mb-2">Shaker</h3>
      <div className="grid grid-cols-2 gap-2 mb-2">
        <label className="text-xs text-gray-600">
          PWM
          <input value={pwm} onChange={(e) => setPwm(e.target.value)} className="w-full border rounded px-2 py-1 mt-1" />
        </label>
        <label className="text-xs text-gray-600">
          Wait Time (s)
          <input value={waitTimeS} onChange={(e) => setWaitTimeS(e.target.value)} className="w-full border rounded px-2 py-1 mt-1" />
        </label>
      </div>
      <ConfirmButton onConfirm={send} disabled={!connected}>Send Shaker Command</ConfirmButton>
    </div>
  );
}

export default function LabMachinePanel() {
  const { publish, connected } = useMqttContext();

  return (
    <section className="border border-gray-200 rounded-xl p-4 bg-white">
      <h2 className="font-semibold text-gray-800 mb-3">Lab Machine Integration</h2>
      <Ot2Section publish={publish} connected={connected} />
      <ShakerSection publish={publish} connected={connected} />
    </section>
  );
}
