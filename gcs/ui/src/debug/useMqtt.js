import React, { createContext, useContext, useCallback, useEffect, useRef, useState } from 'react';
import mqtt from 'mqtt';

const DEFAULT_BROKER_URL = `ws://${window.location.hostname}:9001`;

const MqttContext = createContext(null);

export function MqttProvider({ children, brokerUrl = DEFAULT_BROKER_URL }) {
  const clientRef = useRef(null);
  const subsRef = useRef(new Map()); // topic -> Set<callback>
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const client = mqtt.connect(brokerUrl);
    clientRef.current = client;

    client.on('connect', () => setConnected(true));
    client.on('close', () => setConnected(false));
    client.on('error', () => setConnected(false));
    client.on('message', (topic, message) => {
      const callbacks = subsRef.current.get(topic);
      if (!callbacks) return;
      callbacks.forEach((cb) => cb(message.toString()));
    });

    return () => client.end(true);
  }, [brokerUrl]);

  const publish = useCallback((topic, payload) => {
    clientRef.current?.publish(topic, payload);
  }, []);

  const subscribe = useCallback((topic, callback) => {
    if (!subsRef.current.has(topic)) {
      subsRef.current.set(topic, new Set());
      clientRef.current?.subscribe(topic);
    }
    subsRef.current.get(topic).add(callback);

    return () => {
      const callbacks = subsRef.current.get(topic);
      if (!callbacks) return;
      callbacks.delete(callback);
      if (callbacks.size === 0) {
        subsRef.current.delete(topic);
        clientRef.current?.unsubscribe(topic);
      }
    };
  }, []);

  return (
    <MqttContext.Provider value={{ connected, publish, subscribe }}>
      {children}
    </MqttContext.Provider>
  );
}

export function useMqttContext() {
  const ctx = useContext(MqttContext);
  if (!ctx) throw new Error('useMqttContext must be used within a MqttProvider');
  return ctx;
}
