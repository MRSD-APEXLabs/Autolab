import React, { useState, useRef, useCallback } from 'react';
import { MqttProvider, useMqttContext } from './shared/useMqtt';
import useCameraHub from './shared/useCameraHub';
import usePerception from './shared/usePerception';
import CameraPair from './shared/CameraPair';
import { cameraLabel } from './shared/cameras';

// ── Constants ─────────────────────────────────────────────────────────────────
const DEFAULT_W = 720;
const DEFAULT_H = 400;
const MIN_W     = 480;
const MIN_H     = 200;
const HEADER_H  = 36;

// ── Status dot ────────────────────────────────────────────────────────────────
const DOT_COLORS = { ok: '#44cc66', warn: '#e0a030', bad: '#e05050', idle: '#333' };

function dotTone(hub, connected) {
  if (!connected || !hub.active) return 'idle';
  if (hub.reachable === false || hub.state === 'error') return 'bad';
  return hub.state === 'streaming' ? 'ok' : 'warn';
}

function Dot({ tone }) {
  const color = DOT_COLORS[tone];
  return (
    <div style={{
      width: 7, height: 7, borderRadius: '50%', flexShrink: 0,
      background: color,
      boxShadow: tone === 'idle' ? 'none' : `0 0 5px ${color}`,
    }} />
  );
}

// ── Detection strip ───────────────────────────────────────────────────────────
function DetectionStrip({ camera }) {
  const { tags, plates } = usePerception(camera);
  const target = plates.find((p) => p.plannerTarget);
  return (
    <div data-testid="detection-strip" style={{ padding: '2px 8px 4px', color: '#aaa', fontSize: 11, flexShrink: 0 }}>
      <span>Tags {tags.length ? tags.map((t) => t.id).join(' · ') : '—'}</span>
      <span style={{ marginLeft: 16 }}>
        Wellplate {target ? `${target.conf.toFixed(2)} @ ${target.distance.toFixed(2)} m` : '—'}
      </span>
    </div>
  );
}

// ── CameraPanel ───────────────────────────────────────────────────────────────
// View-only: shows the camera the hub streams; switching happens in the debug console (/debug).
function CameraPanelBody() {
  const { connected } = useMqttContext();
  const hub = useCameraHub();
  const [minimized, setMinimized] = useState(true);
  const [size, setSize] = useState({ w: DEFAULT_W, h: DEFAULT_H });

  const dragging   = useRef(false);
  const dragOrigin = useRef({ x: 0, y: 0, w: 0, h: 0 });

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
        <span style={{ display: 'flex', alignItems: 'center', gap: 6, marginLeft: 12, color: '#888', fontSize: 12 }}>
          <Dot tone={dotTone(hub, connected)} />
          <span>{cameraLabel(hub.active)}</span>
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

      {/* Active camera: annotated + raw */}
      {!minimized && (
        <div style={{ display: 'flex', flexDirection: 'column', flex: 1, minHeight: 0, padding: 6, gap: 4 }}>
          <div style={{ flex: 1, minHeight: 0 }}>
            <CameraPair camera={hub.active} hubReachable={hub.reachable !== false} compact />
          </div>
          <DetectionStrip camera={hub.active} />
        </div>
      )}
    </div>
  );
}

export default function CameraPanel() {
  return (
    <MqttProvider>
      <CameraPanelBody />
    </MqttProvider>
  );
}
