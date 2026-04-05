import React, { useState, useEffect, useRef, useCallback } from 'react';

// ── Config ────────────────────────────────────────────────────────────────────
const HOST = 'localhost';

// ── Constants ─────────────────────────────────────────────────────────────────
const DEFAULT_W = 520;
const DEFAULT_H = 400;
const MIN_W     = 320;
const MIN_H     = 200;
const HEADER_H  = 36;

// ── CamTile ───────────────────────────────────────────────────────────────────
function Dot({ live }) {
  return (
    <div style={{
      width: 7, height: 7, borderRadius: '50%', flexShrink: 0,
      background: live ? '#44cc66' : '#333',
      boxShadow: live ? '0 0 5px #44cc66' : 'none',
    }} />
  );
}

function CamTile({ title, img, dotLive, gridStyle }) {
  return (
    <div style={{
      background: '#111116',
      border: '1px solid #2d2d3a',
      borderRadius: 5,
      display: 'flex',
      flexDirection: 'column',
      overflow: 'hidden',
      minHeight: 0,
      ...gridStyle,
    }}>
      <div style={{
        display: 'flex', alignItems: 'center', gap: 6,
        padding: '4px 8px',
        background: '#1a1a22',
        borderBottom: '1px solid #2d2d3a',
        flexShrink: 0,
      }}>
        <Dot live={dotLive} />
        <span style={{ fontWeight: 600, fontSize: 11, color: '#888' }}>{title}</span>
      </div>
      <div style={{
        flex: 1, display: 'flex',
        alignItems: 'center', justifyContent: 'center',
        background: '#0a0a0d',
        overflow: 'hidden',
        minHeight: 0,
      }}>
        {img
          ? <img src={img} alt={title} style={{ maxWidth: '100%', maxHeight: '100%', objectFit: 'contain', display: 'block' }} />
          : <div style={{ color: '#333', fontSize: 11 }}>No signal</div>
        }
      </div>
    </div>
  );
}

// ── CameraPanel ───────────────────────────────────────────────────────────────
export default function CameraPanel() {
  const [minimized, setMinimized] = useState(true);
  const [size, setSize] = useState({ w: DEFAULT_W, h: DEFAULT_H });

  const [activeImg, setActiveImg] = useState(null);
  const [activeDot, setActiveDot] = useState(false);
  const [wristImg,  setWristImg]  = useState(null);
  const [wristDot,  setWristDot]  = useState(false);
  const [baseImg,   setBaseImg]   = useState(null);
  const [baseDot,   setBaseDot]   = useState(false);

  const dragging   = useRef(false);
  const dragOrigin = useRef({ x: 0, y: 0, w: 0, h: 0 });

  // ── WebSocket connections (open on mount, close on unmount) ───────────────
  useEffect(() => {
    // Active stream — port 8766, JSON frames
    const stream = new WebSocket(`ws://${HOST}:8766`);
    stream.onmessage = (evt) => {
      try {
        const d = JSON.parse(evt.data);
        if (!d.image_jpeg_b64) { setActiveImg(null); setActiveDot(false); return; }
        setActiveImg(`data:image/jpeg;base64,${d.image_jpeg_b64}`);
        setActiveDot(true);
      } catch { /* ignore */ }
    };
    stream.onclose = () => { setActiveImg(null); setActiveDot(false); };
    stream.onerror = () => { setActiveImg(null); setActiveDot(false); };

    // Wrist cam — port 8767, subscribe { camera: 'wrist' }
    const wrist = new WebSocket(`ws://${HOST}:8767`);
    wrist.onopen = () => wrist.send(JSON.stringify({ camera: 'wrist' }));
    wrist.onmessage = (evt) => {
      try {
        const d = JSON.parse(evt.data);
        if (!d.image_jpeg_b64) return;
        setWristImg(`data:image/jpeg;base64,${d.image_jpeg_b64}`);
        setWristDot(true);
      } catch { /* ignore */ }
    };
    wrist.onclose = () => { setWristImg(null); setWristDot(false); };
    wrist.onerror = () => { setWristImg(null); setWristDot(false); };

    // Base cam — port 8767, subscribe { camera: 'base' }
    const base = new WebSocket(`ws://${HOST}:8767`);
    base.onopen = () => base.send(JSON.stringify({ camera: 'base' }));
    base.onmessage = (evt) => {
      try {
        const d = JSON.parse(evt.data);
        if (!d.image_jpeg_b64) return;
        setBaseImg(`data:image/jpeg;base64,${d.image_jpeg_b64}`);
        setBaseDot(true);
      } catch { /* ignore */ }
    };
    base.onclose = () => { setBaseImg(null); setBaseDot(false); };
    base.onerror = () => { setBaseImg(null); setBaseDot(false); };

    return () => { stream.close(); wrist.close(); base.close(); };
  }, []);

  // ── Resize drag (NW handle — panel anchored bottom-right) ────────────────
  const onResizeDown = useCallback((e) => {
    e.preventDefault();
    dragging.current = true;
    dragOrigin.current = { x: e.clientX, y: e.clientY, w: size.w, h: size.h };

    const onMove = (ev) => {
      if (!dragging.current) return;
      setSize({
        w: Math.max(MIN_W, dragOrigin.current.w + (dragOrigin.current.x - ev.clientX)),
        h: Math.max(MIN_H, dragOrigin.current.h + (dragOrigin.current.y - ev.clientY)),
      });
    };
    const onUp = () => {
      dragging.current = false;
      window.removeEventListener('mousemove', onMove);
      window.removeEventListener('mouseup', onUp);
    };
    window.addEventListener('mousemove', onMove);
    window.addEventListener('mouseup', onUp);
  }, [size]);

  // ── Render ────────────────────────────────────────────────────────────────
  return (
    <div style={{
      position: 'fixed', bottom: 16, right: 16, zIndex: 50,
      width: size.w,
      height: minimized ? HEADER_H : size.h,
      background: '#0f0f13',
      border: '1px solid #2d2d3a',
      borderRadius: 10,
      boxShadow: '0 8px 32px rgba(0,0,0,0.6)',
      color: '#e0e0e0',
      fontFamily: "'Segoe UI', system-ui, monospace",
      fontSize: 12,
      display: 'flex',
      flexDirection: 'column',
      overflow: 'hidden',
      userSelect: 'none',
    }}>

      {/* NW resize handle */}
      <div
        onMouseDown={onResizeDown}
        title="Drag to resize"
        style={{
          position: 'absolute', top: 0, left: 0,
          width: 20, height: 20,
          cursor: 'nw-resize',
          zIndex: 10,
          display: 'flex', alignItems: 'flex-start', justifyContent: 'flex-start',
          padding: 4,
        }}
      >
        <svg width="10" height="10" viewBox="0 0 10 10" style={{ opacity: 0.3 }}>
          <line x1="1" y1="9" x2="9" y2="1" stroke="#aaa" strokeWidth="1.5" strokeLinecap="round" />
          <line x1="1" y1="5" x2="5" y2="1" stroke="#aaa" strokeWidth="1.5" strokeLinecap="round" />
        </svg>
      </div>

      {/* Header */}
      <div style={{
        display: 'flex', alignItems: 'center',
        padding: '0 10px',
        height: HEADER_H,
        background: '#1a1a22',
        borderBottom: '1px solid #2d2d3a',
        flexShrink: 0,
      }}>
        <span style={{ fontWeight: 600, fontSize: 13, color: '#aaa', paddingLeft: 16 }}>
          Camera Feeds
        </span>
        <button
          aria-label={minimized ? 'expand' : 'minimize'}
          onClick={() => setMinimized(v => !v)}
          style={{
            background: 'none', border: 'none', cursor: 'pointer',
            color: '#555', fontSize: 13, padding: '2px 5px', marginLeft: 'auto',
          }}
        >
          {minimized ? '▲' : '▼'}
        </button>
      </div>

      {/* Camera grid */}
      {!minimized && (
        <div
          data-testid="camera-grid"
          style={{
            display: 'grid',
            gridTemplateColumns: '1fr 185px',
            gridTemplateRows: '1fr 1fr',
            gap: 6,
            padding: 6,
            flex: 1,
            minHeight: 0,
          }}
        >
          <CamTile title="Active"       img={activeImg} dotLive={activeDot} gridStyle={{ gridRow: '1 / 3' }} />
          <CamTile title="Wrist (cam1)" img={wristImg}  dotLive={wristDot} />
          <CamTile title="Base (cam2)"  img={baseImg}   dotLive={baseDot}  />
        </div>
      )}
    </div>
  );
}
