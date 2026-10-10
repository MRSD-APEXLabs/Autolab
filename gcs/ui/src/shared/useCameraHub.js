import { useEffect, useState } from 'react';
import { useMqttContext } from './useMqtt';
import { unwrapString } from './unwrapRos';
import { HUB_STATUS_TOPIC } from './cameras';

const NO_STATUS = { active: null, state: null, error: null, reachable: null, receivedAt: null };

// Latest /camera_hub/status (zedx_nano_camera hub_node): which camera streams, and how.
export default function useCameraHub() {
  const { subscribe } = useMqttContext();
  const [hub, setHub] = useState(NO_STATUS);

  useEffect(() => subscribe(HUB_STATUS_TOPIC, (message) => {
    try {
      const s = JSON.parse(unwrapString(message));
      setHub({
        active: s.active ?? null,
        state: s.state ?? null,
        error: s.error ?? null,
        reachable: s.reachable ?? null,
        receivedAt: Date.now(),
      });
    } catch {
      // keep the last good status
    }
  }), [subscribe]);

  return hub;
}
