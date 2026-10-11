// Turns camera_perception's /inspect/json or /servo/json payload into table rows (see markers.summary_json).
const SIZE_WARNING_PCT = 15;   // perception_node warns when a tag's measured size is this far off

export const EMPTY_PERCEPTION = { tags: [], plates: [], frameId: null, stampS: null, error: null };

function quatToRpyDeg([x, y, z, w]) {
  const deg = 180 / Math.PI;
  const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
  const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))));
  const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  return [roll * deg, pitch * deg, yaw * deg];
}

function vector(value, length) {
  if (!Array.isArray(value) || value.length !== length || !value.every(Number.isFinite)) {
    throw new Error(`expected ${length} numbers, got ${JSON.stringify(value)}`);
  }
  return value;
}

function parsePose(pose) {
  if (pose == null) return { position: null, distance: null, rpyDeg: null };
  const position = vector(pose.position, 3);
  return { position, distance: Math.hypot(...position), rpyDeg: quatToRpyDeg(vector(pose.orientation, 4)) };
}

function parseTag(t) {
  const sizeOffPct = t.measured_size_m != null && t.size_m
    ? Math.abs(t.measured_size_m / t.size_m - 1) * 100 : null;
  return {
    id: t.id,
    ...parsePose(t.pose),
    poseSource: t.pose_source ?? null,
    sizeM: t.size_m ?? null,
    measuredSizeM: t.measured_size_m ?? null,
    sizeOffPct,
    sizeWarning: sizeOffPct != null && sizeOffPct > SIZE_WARNING_PCT,
    reprojectionPx: t.reprojection_px ?? null,
  };
}

function parsePlate(d) {
  const { position, distance } = parsePose(d.pose);
  return {
    name: d.name ?? 'object',
    conf: Number(d.conf),
    position,
    distance,
    sizeMm: d.metric_size ? vector(d.metric_size, 2).map((v) => v * 1000) : null,
    poseSource: d.pose_source ?? null,
    plannerTarget: false,
  };
}

export function parsePerception(jsonString) {
  try {
    const p = JSON.parse(jsonString);
    if (!p || typeof p !== 'object' || Array.isArray(p)) throw new Error('not a JSON object');
    const tags = (Array.isArray(p.apriltags) ? p.apriltags : []).map(parseTag).sort((a, b) => a.id - b.id);
    // the ZED X node names its list "wellplates", the Nano node "detections" (yolo.json_key)
    const rawPlates = Array.isArray(p.wellplates) ? p.wellplates : Array.isArray(p.detections) ? p.detections : [];
    const plates = rawPlates.map(parsePlate).sort((a, b) => b.conf - a.conf);
    // The planner takes the most confident detection that got a marker, i.e. that has a pose.
    const target = plates.find((plate) => plate.position);
    if (target) target.plannerTarget = true;
    const stampS = p.stamp ? p.stamp.sec + p.stamp.nanosec / 1e9 : null;
    return { tags, plates, frameId: p.frame_id ?? null, stampS, error: null };
  } catch (e) {
    return { ...EMPTY_PERCEPTION, error: e.message };
  }
}
