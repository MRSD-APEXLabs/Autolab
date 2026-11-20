import React from 'react';
import './theme.css';

// Dot grid + soft glows behind every main page (the FVD deck's slide background).
// The second glow follows --accent, so a page can tint it with the run state.
export default function Backdrop() {
  return (
    <div className="ax-bg" aria-hidden="true">
      <div className="grid" />
      <div className="glow g1" />
      <div className="glow g2" />
    </div>
  );
}
