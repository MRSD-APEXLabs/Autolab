// Camera names, MQTT topics and stream URLs of the perception stack (3_perception on the robot).
export const CAMERA_LABELS = { zedx: 'ZED X (base)', zedx_nano: 'ZED X Nano (wrist)' };
export const PERCEPTION_TOPICS = { zedx: 'ros2/inspect/json', zedx_nano: 'ros2/servo/json' };
export const HUB_STATUS_TOPIC = 'ros2/camera_hub/status';
export const SELECT_CAMERA_TOPIC = 'cmd/perception/select_camera';
export const SELECT_RESULT_TOPIC = 'perception/select_camera_result';

// Stream directly from the camera server by IP; no client DNS or GCS relay.
// An explicit base URL can still be supplied for other deployments.
const streamBase = () => (process.env.REACT_APP_CAMERA_STREAM_BASE || 'http://192.168.10.3:8002').replace(/\/$/, '');
export const streamUrl = (kind, camera) => `${streamBase()}/stream/${kind}/${camera}.mjpg`;
export const healthUrl = () => `${streamBase()}/health`;
export const cameraLabel = (camera) => (camera ? CAMERA_LABELS[camera] ?? camera : 'none');
