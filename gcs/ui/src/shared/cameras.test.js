import { streamUrl, healthUrl, cameraLabel } from './cameras';

test('stream and health URLs use the camera server IP directly by default', () => {
  expect(streamUrl('annotated', 'zedx')).toBe('http://192.168.10.3:8002/stream/annotated/zedx.mjpg');
  expect(healthUrl()).toBe('http://192.168.10.3:8002/health');
});

test('camera labels', () => {
  expect(cameraLabel('zedx')).toBe('ZED X (base)');
  expect(cameraLabel('zedx_nano')).toBe('ZED X Nano (wrist)');
  expect(cameraLabel(null)).toBe('none');
  expect(cameraLabel('other')).toBe('other');
});
