import React from 'react';
import CameraFeed from './CameraFeed';

// Annotated (tags, detections, axes drawn by camera_perception) and raw (rectified) views of one camera.
export default function CameraPair({ camera, hubReachable = true, compact = false }) {
  return (
    <div data-testid="camera-pair" className="grid grid-cols-2 gap-2 min-h-0">
      <CameraFeed camera={camera} kind="annotated" title="Annotated" hubReachable={hubReachable} compact={compact} />
      <CameraFeed camera={camera} kind="raw" title="Raw" hubReachable={hubReachable} compact={compact} />
    </div>
  );
}
