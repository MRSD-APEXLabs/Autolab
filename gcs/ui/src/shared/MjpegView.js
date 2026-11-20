import React, { useEffect, useRef, useState } from 'react';
import { createMjpegParser } from './mjpeg';

const STALL_MS = 3000;           // no frame for this long: drop the connection and reopen it
const MAX_BACKOFF_MS = 5000;
const FETCH_REFUSALS_FOR_IMG = 3; // fetch refused this many times with no response: assume no CORS
const BLANK_SRC = 'data:image/gif;base64,R0lGODlhAQABAAAAACH5BAEKAAEALAAAAAABAAEAAAICTAEAOw==';

let session = 0;
export const uniqueUrl = (url) => `${url}${url.includes('?') ? '&' : '?'}s=${Date.now().toString(36)}-${++session}`;

// An <img> on a plain MJPEG URL can't tell when frames stop (a dropped stream raises no error, the
// picture just freezes), and WebKit hands a new <img> with the same URL the old, dead stream.
// So the stream is read with fetch: every frame is seen, a stall is noticed, the connection is
// aborted for certain on unmount, and it reconnects by itself. If the server doesn't allow
// cross-origin reads (no CORS), or the browser can't read the body (Safari's fetch refuses
// multipart/x-mixed-replace: give it a `fetchUrl` the server sends as octet-stream), it falls back
// to an <img> on a never-reused URL, and goes back to fetch when that <img> fails.
export default function MjpegView({ url, fetchUrl = url, alt }) {
  const imgRef = useRef(null);
  const [state, setState] = useState('connecting');   // connecting | live | reconnecting | img
  const [imgSrc, setImgSrc] = useState(null);
  const [round, setRound] = useState(0);              // bumped to leave the <img> fallback

  useEffect(() => {
    const img = imgRef.current;
    let stopped = false;
    let ctrl = null;
    let pending = null;
    let decoding = false;
    let shown = null;
    let wake = null;

    const sleep = (ms) => new Promise((resolve) => {
      const t = setTimeout(resolve, ms);
      wake = () => { clearTimeout(t); resolve(); };
    });

    // Show only the newest frame; while one decodes, newer ones replace the waiting one.
    const showNext = () => {
      if (stopped || !pending) { decoding = false; return; }
      const objectUrl = URL.createObjectURL(new Blob([pending], { type: 'image/jpeg' }));
      pending = null;
      decoding = true;
      img.onload = img.onerror = () => {
        if (shown) URL.revokeObjectURL(shown);
        shown = objectUrl;
        showNext();
      };
      img.src = objectUrl;
    };
    const show = (bytes) => {
      pending = bytes;
      if (!decoding) showNext();
    };

    async function run() {
      let backoff = 500;
      let refusals = 0;
      let everLive = false;
      while (!stopped) {
        const attempt = new AbortController();
        ctrl = attempt;
        let lastFrame = Date.now();
        const watchdog = setInterval(() => {
          if (Date.now() - lastFrame > STALL_MS) attempt.abort();
        }, 500);
        try {
          const res = await fetch(fetchUrl, { signal: ctrl.signal, cache: 'no-store' });
          if (!res.ok || !res.body) throw new Error(`HTTP ${res.status}`);
          const reader = res.body.getReader();
          const parse = createMjpegParser();
          for (;;) {
            const { value, done } = await reader.read();
            if (done || stopped) break;
            const frame = parse(value);
            if (frame) {
              lastFrame = Date.now();
              if (!everLive || backoff !== 500) setState('live');
              everLive = true;
              backoff = 500;
              refusals = 0;
              show(frame);
            }
          }
        } catch (e) {
          // refused before any frame (no CORS, or a body this browser can't read); aborts don't count
          if (!everLive && e && e.name === 'TypeError') refusals += 1;
        } finally {
          clearInterval(watchdog);
          ctrl.abort();
        }
        if (stopped) return;
        if (!everLive && refusals >= FETCH_REFUSALS_FOR_IMG) {
          setImgSrc(uniqueUrl(url));
          setState('img');
          return;
        }
        setState(everLive ? 'reconnecting' : 'connecting');
        await sleep(backoff);
        backoff = Math.min(backoff * 2, MAX_BACKOFF_MS);
      }
    }

    setState('connecting');
    setImgSrc(null);
    run();
    return () => {
      stopped = true;
      if (ctrl) ctrl.abort();
      if (wake) wake();
      img.onload = img.onerror = null;
      img.src = BLANK_SRC;
      if (shown) URL.revokeObjectURL(shown);
    };
  }, [url, fetchUrl, round]);

  // Fallback: an <img> whose URL is never reused. If it fails, try fetch again (the server may have
  // been down, not CORS-less).
  useEffect(() => {
    if (state !== 'img' || !imgSrc) return undefined;
    const img = imgRef.current;
    let t = null;
    img.onerror = () => { t = setTimeout(() => setRound((r) => r + 1), 2000); };
    img.src = imgSrc;
    return () => { clearTimeout(t); img.onerror = null; img.src = BLANK_SRC; };
  }, [state, imgSrc]);

  return (
    <>
      {/* eslint-disable-next-line jsx-a11y/alt-text */}
      <img ref={imgRef} alt={alt} />
      {state === 'connecting' && <div className="cs-nosig"><b>Connecting…</b></div>}
      {state === 'reconnecting' && <div className="cs-stale">Reconnecting…</div>}
    </>
  );
}
