import { useEffect, useState } from 'react';
import { useMqttContext } from './useMqtt';
import { unwrapString } from './unwrapRos';
import { PERCEPTION_TOPICS } from './cameras';
import { parsePerception, EMPTY_PERCEPTION } from './parsePerception';

const NOTHING = { parsed: EMPTY_PERCEPTION, raw: null, receivedAt: null };

// The active camera's camera_perception JSON, parsed, with its age in seconds (refreshed every 500 ms).
export default function usePerception(camera) {
  const { subscribe } = useMqttContext();
  const [state, setState] = useState(NOTHING);
  const [now, setNow] = useState(Date.now());
  const topic = camera ? PERCEPTION_TOPICS[camera] : undefined;

  useEffect(() => {
    setState(NOTHING);
    if (!topic) return undefined;
    return subscribe(topic, (message) => {
      let raw;
      let parsed;
      try {
        raw = unwrapString(message);
        parsed = parsePerception(raw);
      } catch (e) {
        raw = message;
        parsed = { ...EMPTY_PERCEPTION, error: e.message };
      }
      setState({ parsed, raw, receivedAt: Date.now() });
    });
  }, [topic, subscribe]);

  useEffect(() => {
    const id = setInterval(() => setNow(Date.now()), 500);
    return () => clearInterval(id);
  }, []);

  const received = state.receivedAt != null;
  return {
    ...state.parsed,
    raw: state.raw,
    received,
    ageS: received ? Math.max(0, (now - state.receivedAt) / 1000) : null,
  };
}
