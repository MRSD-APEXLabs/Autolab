import React, { useEffect, useRef, useState } from 'react';
import { streamUrl } from './cameras';
import useStreamStale from './useStreamStale';

const STALE_TEXT = {
  annotated: 'No frames: is perception / depth running?',
  raw: 'No frames: is the depth node running?',
};

// 1x1 transparent GIF. Pointing a stream <img> here aborts its MJPEG request.
const BLANK_SRC = 'data:image/gif;base64,R0lGODlhAQABAAAAACH5BAEKAAEALAAAAAABAAEAAAICTAEAOw==';

// One <img> per stream URL (keyed by src). Safari doesn't reliably abort a multipart/x-mixed-replace
// request when an <img> changes src or leaves the DOM: the old stream keeps its connection (and the
// last frame on screen), and the leaked connections fill the 6-per-host limit so the new camera's
// request never starts. Blanking src on unmount closes the old stream explicitly.
// The effect owns src (not a JSX prop) so StrictMode's mount → cleanup → mount restores the stream
// instead of leaving the blank GIF.
function MjpegImg({ src, ...props }) {
  const ref = useRef(null);
  useEffect(() => {
    const img = ref.current;
    img.src = src;
    return () => { img.src = BLANK_SRC; };
  }, [src]);
  // eslint-disable-next-line jsx-a11y/alt-text
  return <img ref={ref} {...props} />;
}

// One MJPEG tile from image_stream_server. Unmounting the <img> closes the stream on the server.
export default function CameraFeed({ camera, kind, title, hubReachable = true, compact = false }) {
  const [attempt, setAttempt] = useState(0);
  const [failed, setFailed] = useState(false);

  let placeholder = null;
  if (!hubReachable) placeholder = 'Camera hub unreachable';
  else if (!camera) placeholder = 'No camera selected';
  const streaming = !placeholder && !failed;
  const stale = useStreamStale(streaming ? `${kind}/${camera}` : null);

  useEffect(() => { setFailed(false); setAttempt(0); }, [camera, kind]);

  const src = `${streamUrl(kind, camera)}${attempt ? `?retry=${attempt}` : ''}`;
  const message = placeholder || (failed ? 'Stream unavailable' : null);

  return (
    <div className="flex flex-col min-h-0 rounded-md overflow-hidden border border-gray-700 bg-gray-950">
      <div className={`px-2 ${compact ? 'py-0.5 text-[11px]' : 'py-1 text-xs'} font-semibold text-gray-300 bg-gray-800`}>
        {title}
      </div>
      <div className="relative flex-1 min-h-0 flex items-center justify-center bg-black aspect-video">
        {message ? (
          <div className="flex flex-col items-center gap-2 text-gray-400 text-xs">
            <span>{message}</span>
            {failed && !placeholder && (
              <button
                type="button"
                onClick={() => { setFailed(false); setAttempt((a) => a + 1); }}
                className="px-2 py-0.5 rounded bg-gray-700 hover:bg-gray-600 text-gray-100"
              >
                Retry
              </button>
            )}
          </div>
        ) : (
          <MjpegImg
            key={src}
            src={src}
            alt={`${title} ${camera}`}
            onError={() => setFailed(true)}
            className="max-w-full max-h-full object-contain block"
          />
        )}
        {streaming && stale && (
          <div className="absolute inset-x-0 bottom-0 bg-black/70 text-amber-300 text-xs text-center py-1">
            {STALE_TEXT[kind] ?? 'No frames'}
          </div>
        )}
      </div>
    </div>
  );
}
