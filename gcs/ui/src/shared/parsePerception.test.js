import { parsePerception } from './parsePerception';

const q90z = [0, 0, Math.SQRT1_2, Math.SQRT1_2];   // yaw +90°
const tag = (id, extra = {}) => ({
  id, center: [1, 2], corners: [], pose: { position: [0.1, 0.2, 0.2], orientation: q90z },
  pose_source: 'pnp', size_m: 0.0625, measured_size_m: 0.0625, reprojection_px: 0.4, ...extra,
});
const plate = (conf, extra = {}) => ({
  center: [1, 2], size: [10, 20], theta: 0, conf, cls_id: 0, name: 'wellplate',
  pose: { position: [0, 0, 0.5], orientation: [0, 0, 0, 1] }, pose_source: 'plane',
  metric_size: [0.085, 0.127], polygon: [], ...extra,
});
const frame = (body) => JSON.stringify({ stamp: { sec: 10, nanosec: 500000000 }, frame_id: 'top_camera', ...body });

test('parses tags sorted by id with distance, rpy and size check', () => {
  const r = parsePerception(frame({ apriltags: [tag(21), tag(1, { measured_size_m: 0.075 })], wellplates: [] }));
  expect(r.error).toBeNull();
  expect(r.frameId).toBe('top_camera');
  expect(r.stampS).toBeCloseTo(10.5);
  expect(r.tags.map((t) => t.id)).toEqual([1, 21]);
  const [t1, t21] = r.tags;
  expect(t21.distance).toBeCloseTo(0.3);
  expect(t21.rpyDeg[0]).toBeCloseTo(0);
  expect(t21.rpyDeg[1]).toBeCloseTo(0);
  expect(t21.rpyDeg[2]).toBeCloseTo(90);
  expect(t21.sizeWarning).toBe(false);
  expect(t1.sizeOffPct).toBeCloseTo(20);
  expect(t1.sizeWarning).toBe(true);
  expect(t21.poseSource).toBe('pnp');
  expect(t21.reprojectionPx).toBe(0.4);
});

test('tags without pose or measured size parse to nulls', () => {
  const r = parsePerception(frame({ apriltags: [tag(0, { pose: null, measured_size_m: null, pose_source: null })] }));
  const [t] = r.tags;
  expect(t.position).toBeNull();
  expect(t.distance).toBeNull();
  expect(t.rpyDeg).toBeNull();
  expect(t.sizeOffPct).toBeNull();
  expect(t.sizeWarning).toBe(false);
});

test('plates sort by confidence and the best posed plate is the planner target', () => {
  const r = parsePerception(frame({
    wellplates: [plate(0.5), plate(0.95, { pose: null, metric_size: null }), plate(0.8)],
  }));
  expect(r.plates.map((p) => p.conf)).toEqual([0.95, 0.8, 0.5]);
  expect(r.plates.map((p) => p.plannerTarget)).toEqual([false, true, false]);
  expect(r.plates[1].sizeMm[0]).toBeCloseTo(85);
  expect(r.plates[1].sizeMm[1]).toBeCloseTo(127);
  expect(r.plates[1].distance).toBeCloseTo(0.5);
  expect(r.plates[0].sizeMm).toBeNull();
});

test('accepts the Nano detections key', () => {
  const r = parsePerception(frame({ apriltags: [], detections: [plate(0.7)] }));
  expect(r.plates).toHaveLength(1);
});

test('missing lists give empty results', () => {
  const r = parsePerception(frame({}));
  expect(r.tags).toEqual([]);
  expect(r.plates).toEqual([]);
  expect(r.error).toBeNull();
});

test.each(['not json', '[]', 'null', JSON.stringify({ apriltags: [{ id: 1, pose: { position: [0, 0, 1] } }] })])(
  'malformed input %s returns an error, never throws',
  (input) => {
    const r = parsePerception(input);
    expect(r.error).toEqual(expect.any(String));
    expect(r.tags).toEqual([]);
    expect(r.plates).toEqual([]);
  },
);
