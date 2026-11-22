import { useEffect, useRef, useState } from 'react';

// "0:07", "12:34", "1:02:03".
export function formatDuration(seconds) {
  if (!Number.isFinite(seconds) || seconds < 0) return '0:00';
  const s = Math.floor(seconds);
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  const ss = String(s % 60).padStart(2, '0');
  return h > 0 ? `${h}:${String(m).padStart(2, '0')}:${ss}` : `${m}:${ss}`;
}

// A seconds count the executor reports (about once a second), counted on locally between reports
// so the display ticks smoothly. The executor's number is the truth: no browser/GCS clock skew.
// Never steps back by less than a second and a half (report jitter); a bigger drop is a new run.
export function useLiveSeconds(seconds, ticking, { max = Infinity, tickMs = 250 } = {}) {
  const anchor = useRef({ s: null, at: 0 });
  const shown = useRef(null);
  const [, setTick] = useState(0);
  const known = Number.isFinite(seconds);

  if (known && seconds !== anchor.current.s) anchor.current = { s: seconds, at: Date.now() };

  useEffect(() => {
    if (!ticking || !known) return undefined;
    const id = setInterval(() => setTick((t) => t + 1), tickMs);
    return () => clearInterval(id);
  }, [ticking, known, tickMs]);

  if (!known) {
    shown.current = null;
    return null;
  }
  let value = ticking ? anchor.current.s + (Date.now() - anchor.current.at) / 1000 : seconds;
  if (ticking && shown.current !== null && value < shown.current && shown.current - value < 1.5) {
    value = shown.current;
  }
  value = Math.min(value, max);
  shown.current = value;
  return value;
}
