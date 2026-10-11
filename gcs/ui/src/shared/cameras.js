// Camera names, MQTT topics and stream URLs of the perception stack (3_perception on the robot).
export const CAMERA_LABELS = { zedx: 'ZED X (base)', zedx_nano: 'ZED X Nano (wrist)' };
export const PERCEPTION_TOPICS = { zedx: 'ros2/inspect/json', zedx_nano: 'ros2/servo/json' };
export const HUB_STATUS_TOPIC = 'ros2/camera_hub/status';
export const SELECT_CAMERA_TOPIC = 'cmd/perception/select_camera';
export const SELECT_RESULT_TOPIC = 'perception/select_camera_result';

export const ROBOT_HOST = process.env.REACT_APP_ROBOT_HOST || window.location.hostname;
export const STREAM_PORT = process.env.REACT_APP_STREAM_PORT || '8002';

const streamBase = () => `http://${ROBOT_HOST}:${STREAM_PORT}`;
export const streamUrl = (kind, camera) => `${streamBase()}/stream/${kind}/${camera}.mjpg`;
export const healthUrl = () => `${streamBase()}/health`;
export const cameraLabel = (camera) => (camera ? CAMERA_LABELS[camera] ?? camera : 'none');
