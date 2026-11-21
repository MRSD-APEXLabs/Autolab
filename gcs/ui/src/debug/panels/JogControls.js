import React, { useEffect, useRef, useState } from 'react';
import { useMqttContext } from '../../shared/useMqtt';
import StatusEcho from '../StatusEcho';

export const JOG_TOPIC = 'cmd/manipulation/jog';
export const JOG_RESULT_TOPIC = 'manipulation/jog_result';
// The bridge answers within 60 s (arm_jog.JOG_TIMEOUT_S); give up a little later.
export const JOG_TIMEOUT_MS = 65000;

// act_inference/jog/<value>: 10 mm along that xArm base-frame axis, orientation held;
// RZ turns the TCP (and the wrist camera) 10 degrees about the base z axis, XYZ held;
// Home moves straight to the home_offset pose with the arm's own IK: no planning, no obstacle avoidance;
// Clear errors (act_inference/clear_errors) clears the arm's errors and enables it again, without moving it
const JOGS = [
  { label: 'X+', value: 'x_plus' },
  { label: 'X-', value: 'x_minus' },
  { label: 'Y+', value: 'y_plus' },
  { label: 'Y-', value: 'y_minus' },
  { label: 'Z+', value: 'z_plus' },
  { label: 'Z-', value: 'z_minus' },
  { label: 'RZ+', value: 'rz_plus' },
  { label: 'RZ-', value: 'rz_minus' },
  { label: 'Home', value: 'home', title: 'Straight to home_offset, no planning: nothing on the way is avoided' },
  { label: 'Clear errors', value: 'clear_errors',
    title: 'Clear the arm errors and enable the motors again (no motion), then give the arm back to the planner' },
];

export default function JogControls() {
  const { publish, subscribe, connected } = useMqttContext();
  const [pending, setPending] = useState(false);
  const [result, setResult] = useState(null);
  const timer = useRef(null);

  const finish = (next) => {
    clearTimeout(timer.current);
    setPending(false);
    setResult(next);
  };

  useEffect(() => subscribe(JOG_RESULT_TOPIC, (message) => {
    try {
      finish(JSON.parse(message));
    } catch {
      finish({ ok: false, message: 'Unreadable jog result' });
    }
  }), [subscribe]);

  useEffect(() => () => clearTimeout(timer.current), []);

  const jog = (value) => {
    setResult(null);
    setPending(true);
    clearTimeout(timer.current);
    timer.current = setTimeout(
      () => finish({ ok: false, message: 'No reply from the MQTT bridge' }), JOG_TIMEOUT_MS);
    publish(JOG_TOPIC, JSON.stringify(value));
  };

  return (
    <div className="border rounded p-3 mt-3">
      <h4 className="text-sm font-semibold mb-2">Arm jog (10 mm / RZ 10°, xArm base frame)</h4>
      <div className="flex flex-wrap items-center gap-2 mb-2">
        {JOGS.map(({ label, value, title }) => (
          <button
            key={value}
            type="button"
            title={title}
            disabled={!connected || pending}
            onClick={() => jog(value)}
            className="bg-orange-600 hover:bg-orange-700 disabled:bg-gray-300 text-white px-3 py-1.5 rounded text-sm"
          >
            {label}
          </button>
        ))}
        {pending && <span className="text-sm text-gray-500">Jogging…</span>}
        {!pending && result && (
          <span className={`text-sm ${result.ok ? 'text-gray-600' : 'text-red-600'}`}>{result.message}</span>
        )}
      </div>
      <StatusEcho topic="ros2/act_inference/status" label="ACT / Jog Status" />
    </div>
  );
}
