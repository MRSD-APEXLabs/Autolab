import { useEffect, useState } from 'react';
import { healthUrl } from './cameras';

// True when image_stream_server's /health says the stream's newest frame is older than staleAfterS
// (or no frame arrived within staleAfterS of mounting). Browsers don't fire onLoad per MJPEG frame.
export default function useStreamStale(streamKey, { intervalMs = 1000, staleAfterS = 2 } = {}) {
  const [stale, setStale] = useState(false);

  useEffect(() => {
    setStale(false);
    if (!streamKey) return undefined;
    let cancelled = false;
    const mountedAt = Date.now();
    const poll = async () => {
      try {
        const response = await fetch(healthUrl(), { cache: 'no-store' });
        const body = await response.json();
        const age = body?.[streamKey]?.last_frame_age_s;
        const isStale = age == null ? (Date.now() - mountedAt) / 1000 > staleAfterS : age > staleAfterS;
        if (!cancelled) setStale(isStale);
      } catch {
        // an unreachable server shows up as the <img> error state instead
      }
    };
    const id = setInterval(poll, intervalMs);
    return () => { cancelled = true; clearInterval(id); };
  }, [streamKey, intervalMs, staleAfterS]);

  return stale;
}
