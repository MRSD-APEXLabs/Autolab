import React, { useState } from 'react';
import { useMqttContext } from '../useMqtt';
import ConfirmButton from '../ConfirmButton';
import StatusEcho from '../StatusEcho';

export default function NavigationPanel() {
  const { publish, connected } = useMqttContext();
  const [x, setX] = useState('0');
  const [y, setY] = useState('0');
  const [theta, setTheta] = useState('0');

  const sendGoal = () => {
    publish('cmd/navigation/goal_pose', JSON.stringify({
      x: parseFloat(x),
      y: parseFloat(y),
      theta: parseFloat(theta),
    }));
  };

  return (
    <section className="border border-gray-200 rounded-xl p-4 bg-white">
      <h2 className="font-semibold text-gray-800 mb-3">Navigation</h2>
      <div className="grid grid-cols-3 gap-2 mb-3">
        <label className="text-xs text-gray-600">
          X (m)
          <input value={x} onChange={(e) => setX(e.target.value)} className="w-full border rounded px-2 py-1 mt-1" />
        </label>
        <label className="text-xs text-gray-600">
          Y (m)
          <input value={y} onChange={(e) => setY(e.target.value)} className="w-full border rounded px-2 py-1 mt-1" />
        </label>
        <label className="text-xs text-gray-600">
          Theta (deg)
          <input value={theta} onChange={(e) => setTheta(e.target.value)} className="w-full border rounded px-2 py-1 mt-1" />
        </label>
      </div>
      <ConfirmButton onConfirm={sendGoal} disabled={!connected}>Send Goal Pose</ConfirmButton>
      <div className="mt-3">
        <StatusEcho topic="ros2/robot_1/odom" label="Odometry" />
      </div>
    </section>
  );
}
