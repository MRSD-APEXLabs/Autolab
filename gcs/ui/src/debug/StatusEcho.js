import React, { useEffect, useState } from 'react';
import { useMqttContext } from './useMqtt';

export default function StatusEcho({ topic, label }) {
  const { subscribe } = useMqttContext();
  const [payload, setPayload] = useState(null);
  const [receivedAt, setReceivedAt] = useState(null);

  useEffect(() => {
    return subscribe(topic, (message) => {
      setPayload(message);
      setReceivedAt(new Date());
    });
  }, [topic, subscribe]);

  return (
    <div className="border border-gray-200 rounded-lg p-3 bg-gray-50">
      <div className="flex justify-between items-baseline mb-1">
        <span className="text-xs font-semibold text-gray-600">{label}</span>
        <span className="text-xs text-gray-400">
          {receivedAt ? receivedAt.toLocaleTimeString() : 'no data'}
        </span>
      </div>
      <pre className="text-xs text-gray-800 whitespace-pre-wrap break-all m-0">
        {payload ?? '—'}
      </pre>
    </div>
  );
}
