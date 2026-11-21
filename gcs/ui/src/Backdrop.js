import React from 'react';
import './theme.css';

// The deck's section-slide shapes (orange half-discs, blue quarter, red block), drawn behind a page.
// variant "title" = the three growing half-discs of the section slides; "corner" = the small stack.
export default function Backdrop({ variant = 'corner' }) {
  return (
    <div className={`ax-shapes ${variant}`} aria-hidden="true">
      {variant === 'title' ? (
        <>
          <i className="d1" />
          <i className="d2" />
          <i className="d3" />
        </>
      ) : (
        <>
          <i className="q" />
          <i className="r" />
          <i className="c" />
        </>
      )}
    </div>
  );
}
