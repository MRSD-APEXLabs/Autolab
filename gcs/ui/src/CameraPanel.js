import React, { useState, useEffect } from 'react';

// Configure streams here. Each entry becomes one cell in the grid.
export const STREAMS = [
  { id: 'cam1', label: 'Camera 1', url: 'ws://localhost:8765/cam1' },
  { id: 'cam2', label: 'Camera 2', url: 'ws://localhost:8765/cam2' },
];

// Converts a raw WebSocket MessageEvent into an img src string.
// Swap this function out when the wire format is finalized.
function handleMessage(event) {
  if (typeof event.data === 'string') {
    return `data:image/jpeg;base64,${event.data}`;
  }
  return URL.createObjectURL(new Blob([event.data], { type: 'image/jpeg' }));
}

function StreamCell({ stream }) {
  const [imgSrc, setImgSrc] = useState(null);

  useEffect(() => {
    let currentSrc = null;
    const ws = new WebSocket(stream.url);
    ws.onmessage = (event) => {
      const src = handleMessage(event);
      setImgSrc((prev) => {
        if (prev && prev.startsWith('blob:')) {
          URL.revokeObjectURL(prev);
        }
        currentSrc = src;
        return src;
      });
    };
    ws.onerror = () => { currentSrc = null; setImgSrc(null); };
    ws.onclose = () => { currentSrc = null; setImgSrc(null); };
    return () => {
      ws.close();
      if (currentSrc && currentSrc.startsWith('blob:')) {
        URL.revokeObjectURL(currentSrc);
      }
    };
  }, [stream.url]);

  return (
    <div className="flex flex-col items-center bg-gray-900 rounded overflow-hidden">
      {imgSrc ? (
        <img
          src={imgSrc}
          alt={stream.label}
          className="w-full h-32 object-cover"
        />
      ) : (
        <div className="w-full h-32 bg-gray-800 flex items-center justify-center text-gray-500 text-xs">
          Disconnected
        </div>
      )}
      <span className="text-gray-400 text-xs py-1">{stream.label}</span>
    </div>
  );
}

export default function CameraPanel() {
  const [minimized, setMinimized] = useState(true);

  return (
    <div className="fixed bottom-4 right-4 z-50 w-[480px] shadow-2xl rounded-xl overflow-hidden border border-gray-700">
      {/* Header */}
      <div className="bg-gray-900 text-white flex justify-between items-center px-4 py-2 h-10">
        <span className="text-sm font-semibold tracking-wide">Camera Feeds</span>
        {minimized ? (
          <button
            aria-label="expand"
            onClick={() => setMinimized(false)}
            className="text-gray-400 hover:text-white text-lg leading-none"
          >
            ▲
          </button>
        ) : (
          <button
            aria-label="minimize"
            onClick={() => setMinimized(true)}
            className="text-gray-400 hover:text-white text-lg leading-none"
          >
            ▼
          </button>
        )}
      </div>

      {/* Grid — only rendered when expanded */}
      {!minimized && (
        <div
          data-testid="camera-grid"
          className="bg-gray-950 p-3 grid grid-cols-2 gap-3"
        >
          {STREAMS.map((stream) => (
            <StreamCell key={stream.id} stream={stream} />
          ))}
        </div>
      )}
    </div>
  );
}
