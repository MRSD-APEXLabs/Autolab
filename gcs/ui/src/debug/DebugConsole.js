import React from 'react';
import { MqttProvider, useMqttContext } from './useMqtt';
import NavigationPanel from './panels/NavigationPanel';
import PlanningPanel from './panels/PlanningPanel';
import PerceptionPanel from './panels/PerceptionPanel';
import ManipulationPanel from './panels/ManipulationPanel';
import LabMachinePanel from './panels/LabMachinePanel';

function ConnectionBanner() {
  const { connected } = useMqttContext();
  return (
    <div className={`text-sm px-4 py-2 ${connected ? 'bg-green-100 text-green-700' : 'bg-red-100 text-red-700'}`}>
      MQTT: {connected ? 'connected' : 'disconnected'}
    </div>
  );
}

export default function DebugConsole() {
  return (
    <MqttProvider>
      <div className="min-h-screen bg-gray-100">
        <ConnectionBanner />
        <header className="px-6 py-4">
          <h1 className="text-xl font-semibold text-gray-800">Debug / Testing Console</h1>
        </header>
        <main className="grid grid-cols-1 md:grid-cols-2 gap-4 px-6 pb-6">
          <NavigationPanel />
          <PlanningPanel />
          <PerceptionPanel />
          <ManipulationPanel />
          <LabMachinePanel />
        </main>
      </div>
    </MqttProvider>
  );
}
