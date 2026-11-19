import React, { useEffect, useState } from 'react';
import { useMqttContext } from '../../shared/useMqtt';

export const LIGHT_COMMAND_TOPIC = 'cmd/lab_machine/ot2_lights';
export const LIGHT_STATUS_TOPIC = 'ot2/lights/status';

export default function Ot2LightsControls() {
  const { connected, publish, subscribe } = useMqttContext();
  const [state, setState] = useState(null);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState(null);
  useEffect(() => subscribe(LIGHT_STATUS_TOPIC, (message) => {
    try {
      const next = JSON.parse(message);
      setState(next);
      setPending(!!next.busy);
      setError(next.error || null);
    } catch { setError('Unreadable light status'); setPending(false); }
  }), [subscribe]);
  useEffect(() => {
    if (connected) publish(LIGHT_COMMAND_TOPIC, JSON.stringify({ action: 'get' }));
  }, [connected, publish]);
  useEffect(() => {
    if (!pending) return undefined;
    const timer = setTimeout(() => {
      setPending(false);
      setError('No light-control reply. Refresh the light state before trying again.');
    }, 15000);
    return () => clearTimeout(timer);
  }, [pending]);
  const send = (body) => {
    setPending(true);
    setError(null);
    publish(LIGHT_COMMAND_TOPIC, JSON.stringify(body));
  };
  const disabled = !connected || pending;
  return (
    <div className="border rounded p-3 mb-3">
      <h4 className="text-sm font-semibold mb-2">OT-2 rail lights</h4>
      <div className="flex gap-2 flex-wrap">
        {[true, false].map((on) => (
          <button key={String(on)} type="button" disabled={disabled}
            aria-pressed={state?.on === on}
            onClick={() => send({ action: 'set', on })}
            className="bg-blue-600 hover:bg-blue-700 disabled:bg-gray-300 text-white px-3 py-1.5 rounded text-sm">
            {on ? 'Lights on' : 'Lights off'}
          </button>
        ))}
        <button type="button" disabled={disabled} onClick={() => send({ action: 'get' })}
          className="border rounded px-3 py-1.5 text-sm disabled:text-gray-400">Refresh lights</button>
      </div>
      <p className="text-xs mt-2">{pending ? 'Updating lights…'
        : `Rail lights: ${state?.on === true ? 'on' : state?.on === false ? 'off' : 'unknown'}`}</p>
      {error && <p role="alert" className="text-xs text-red-600 mt-1">{error}</p>}
    </div>
  );
}
