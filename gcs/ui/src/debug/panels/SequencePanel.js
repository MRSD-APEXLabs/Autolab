import React, { useEffect, useState } from 'react';
import { useMqttContext } from '../../shared/useMqtt';
import { unwrapString } from '../../shared/unwrapRos';
import ConfirmButton from '../ConfirmButton';

const START_TOPIC = 'cmd/routine_executor/start_routine_cmd';
const CANCEL_TOPIC = 'cmd/routine_executor/cancel';
const STATUS_TOPIC = 'ros2/routine_executor/status';

// The whole well-plate run, as routine_executor steps (step_config.py).
export const FULL_SEQUENCE = [
  { name: 'go_to', target_machine: 'ot2' },
  { name: 'pick_base' },
  { name: 'place', target_machine: 'ot2' },
  { name: 'pick', target_machine: 'ot2' },
  { name: 'go_to', target_machine: 'shaker' },
  { name: 'place', target_machine: 'shaker' },
  { name: 'pick', target_machine: 'shaker' },
  { name: 'place_base' },
  { name: 'home' },
];

// A failed step stops the run (no automatic retry).
const MAX_RETRIES = 1;

const ACTIVE = ['running', 'paused'];

function stepLabel(step) {
  return step.target_machine ? `${step.name} (${step.target_machine})` : step.name;
}

export default function SequencePanel() {
  const { publish, subscribe, connected } = useMqttContext();
  const [status, setStatus] = useState(null);

  useEffect(() => subscribe(STATUS_TOPIC, (message) => {
    try {
      setStatus(JSON.parse(unwrapString(message)));
    } catch {
      // keep the last good status
    }
  }), [subscribe]);

  const state = status?.state ?? 'idle';
  const active = ACTIVE.includes(state);
  const steps = status?.steps?.length ? status.steps : FULL_SEQUENCE;
  const current = status?.current_step ?? 0;

  const start = () => {
    publish(START_TOPIC, JSON.stringify({ steps: FULL_SEQUENCE, max_retries: MAX_RETRIES, robot: 'robot_1' }));
  };
  // One click: an interrupt should not wait for a confirmation.
  const stop = () => publish(CANCEL_TOPIC, '');

  const stepClass = (i) => {
    if (state === 'success' || i < current) return 'text-green-700';
    if (i === current && active) return 'text-orange-600 font-semibold';
    if (i === current && state === 'failed') return 'text-red-600 font-semibold';
    return 'text-gray-500';
  };

  return (
    <section className="border border-gray-200 rounded-xl p-4 bg-white">
      <h2 className="font-semibold text-gray-800 mb-3">Full Sequence</h2>
      <ol className="text-sm list-decimal list-inside mb-3 space-y-0.5">
        {steps.map((step, i) => (
          <li key={i} className={stepClass(i)}>{stepLabel(step)}</li>
        ))}
      </ol>
      <div className="flex flex-wrap gap-2 items-center">
        <ConfirmButton onConfirm={start} disabled={!connected || active}>Start Full Sequence</ConfirmButton>
        <button
          type="button"
          onClick={stop}
          disabled={!connected || !active}
          className="bg-red-600 hover:bg-red-700 disabled:bg-gray-300 text-white px-3 py-1.5 rounded text-sm"
        >
          Stop
        </button>
      </div>
      <p className="text-xs text-gray-600 mt-3" data-testid="sequence-state">
        {status === null && 'Waiting for routine executor…'}
        {status !== null && active && `${state === 'paused' ? 'Paused' : 'Running'}: step ${current + 1}/${steps.length}, ${stepLabel(steps[current] ?? {})}`}
        {status !== null && state === 'success' && 'Sequence complete'}
        {status !== null && state === 'failed' && `Stopped at step ${current + 1}: ${status.error ?? 'failed'}`}
        {status !== null && state === 'idle' && 'Idle'}
      </p>
    </section>
  );
}
