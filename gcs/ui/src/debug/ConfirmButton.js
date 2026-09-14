import React, { useState } from 'react';

export default function ConfirmButton({ onConfirm, disabled, children }) {
  const [awaitingConfirm, setAwaitingConfirm] = useState(false);

  if (awaitingConfirm) {
    return (
      <span className="inline-flex gap-2">
        <button
          type="button"
          disabled={disabled}
          onClick={() => { setAwaitingConfirm(false); onConfirm(); }}
          className="bg-red-600 hover:bg-red-700 disabled:bg-gray-300 text-white px-3 py-1.5 rounded text-sm"
        >
          Confirm?
        </button>
        <button
          type="button"
          onClick={() => setAwaitingConfirm(false)}
          className="text-gray-500 text-sm px-2"
        >
          Cancel
        </button>
      </span>
    );
  }

  return (
    <button
      type="button"
      disabled={disabled}
      onClick={() => setAwaitingConfirm(true)}
      className="bg-orange-600 hover:bg-orange-700 disabled:bg-gray-300 text-white px-3 py-1.5 rounded text-sm"
    >
      {children}
    </button>
  );
}
