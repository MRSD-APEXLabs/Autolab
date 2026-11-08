import { useEffect, useRef } from 'react';

// Calls fn immediately and then every intervalMs while enabled.
export default function usePolling(fn, intervalMs, enabled = true) {
  const fnRef = useRef(fn);
  fnRef.current = fn;

  useEffect(() => {
    if (!enabled) return undefined;
    fnRef.current();
    const id = setInterval(() => fnRef.current(), intervalMs);
    return () => clearInterval(id);
  }, [intervalMs, enabled]);
}
