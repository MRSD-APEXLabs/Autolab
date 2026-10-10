import React from 'react';
import { useMqttContext } from '../useMqtt';
import ConfirmButton from '../ConfirmButton';
import StatusEcho from '../StatusEcho';

const MODES = [
  { label: 'Inspect', value: 'inspect' },
  { label: 'Servo', value: 'servo' },
  { label: 'Idle', value: 'idle' },
];

export default function PerceptionPanel() {
  const { publish, connected } = useMqttContext();

  return (
    <section className="border border-gray-200 rounded-xl p-4 bg-white">
      <h2 className="font-semibold text-gray-800 mb-3">Perception</h2>
      <div className="flex flex-wrap gap-2 mb-3">
        {MODES.map(({ label, value }) => (
          <ConfirmButton
            key={value}
            disabled={!connected}
            onConfirm={() => publish('cmd/perception/camera_mode_cmd', value)}
          >
            {label}
          </ConfirmButton>
        ))}
      </div>
      <div className="grid gap-2">
        <StatusEcho topic="ros2/robot_1/behavior/perception/camera_mode_status" label="Camera Mode Status" />
        <StatusEcho topic="ros2/inspect/apriltags" label="AprilTags (read-only)" />
        <StatusEcho topic="ros2/inspect/wellplates" label="Wellplates (read-only)" />
      </div>
    </section>
  );
}
