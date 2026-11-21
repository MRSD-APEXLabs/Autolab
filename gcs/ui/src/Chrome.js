import React from 'react';

// Logo + wordmark, as in the corner of every deck slide.
export function Brand() {
  return (
    <div className="ax-brand">
      <span className="ax-mark" />
      <span className="ax-word">APEX<small>LABS</small></span>
    </div>
  );
}

// The deck's slide title: navy Bebas on a pale-blue bar with a rounded right end.
export function TitleBar({ title, right }) {
  return (
    <div className="ax-titlebar">
      <h1>{title}</h1>
      {right && <div className="ax-titlebar-right">{right}</div>}
    </div>
  );
}
