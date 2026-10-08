import React, { useState } from 'react';
import { useMqttContext } from '../useMqtt';
import ConfirmButton from '../ConfirmButton';
import StatusEcho from '../StatusEcho';

// Saved place names from navigation_bringup/maps/map.locations.yaml.
const LOCATIONS = [
  { label: 'Home', name: 'home' },
  { label: 'Shaker', name: 'shaker' },
  { label: 'OT2', name: 'ot2' },
];

function GoToLocationSection({ publish, connected }) {
  return (
    <div className="mb-4">
      <h3 className="text-sm font-semibold text-gray-700 mb-2">Go To Location</h3>
      <div className="flex flex-wrap gap-2">
        {LOCATIONS.map(({ label, name }) => (
          <ConfirmButton
            key={name}
            onConfirm={() => publish('cmd/navigation/go_to_location', name)}
            disabled={!connected}
          >
            {label}
          </ConfirmButton>
        ))}
      </div>
    </div>
  );
}

function GoalPoseSection({ publish, connected }) {
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
    <div className="mb-4">
      <h3 className="text-sm font-semibold text-gray-700 mb-2">Navigate To Pose (map frame)</h3>
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
    </div>
  );
}

export default function NavigationPanel() {
  const { publish, connected } = useMqttContext();

  return (
    <section className="border border-gray-200 rounded-xl p-4 bg-white">
      <h2 className="font-semibold text-gray-800 mb-3">Navigation</h2>
      <GoToLocationSection publish={publish} connected={connected} />
      <GoalPoseSection publish={publish} connected={connected} />
      <div className="space-y-2">
        <StatusEcho topic="ros2/robot_1/behavior/go_to_location_status" label="Go To Location Status" />
        <StatusEcho topic="ros2/robot_1/behavior/navigate_to_pose_status" label="Navigate To Pose Status" />
        <StatusEcho topic="ros2/robot_1/odom" label="Odometry" />
      </div>
    </section>
  );
}
