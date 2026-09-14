import React from 'react';
import { useMqttContext } from '../useMqtt';
import ConfirmButton from '../ConfirmButton';
import StatusEcho from '../StatusEcho';

const COMMANDS = [
  { label: 'Home', value: 'plan_home' },
  { label: 'Home Offset', value: 'plan_home_offset' },
  { label: 'Wellplate', value: 'plan_wellplate' },
  { label: 'AprilTag: OT2', value: 'plan_april_1' },
  { label: 'AprilTag: Shaker', value: 'plan_april_2' },
];

export default function PlanningPanel() {
  const { publish, connected } = useMqttContext();

  return (
    <section className="border border-gray-200 rounded-xl p-4 bg-white">
      <h2 className="font-semibold text-gray-800 mb-3">Planning</h2>
      <div className="flex flex-wrap gap-2 mb-3">
        {COMMANDS.map(({ label, value }) => (
          <ConfirmButton
            key={value}
            disabled={!connected}
            onConfirm={() => publish('cmd/planning/command', value)}
          >
            {label}
          </ConfirmButton>
        ))}
      </div>
      <StatusEcho topic="ros2/planning_state" label="Planning State" />
    </section>
  );
}
