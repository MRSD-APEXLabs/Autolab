import { render, screen, within, fireEvent } from '@testing-library/react';
import DetectionTables from './DetectionTables';
import { parsePerception } from '../shared/parsePerception';

const json = JSON.stringify({
  stamp: { sec: 1, nanosec: 0 }, frame_id: 'top_camera',
  apriltags: [
    { id: 21, pose: { position: [0.1, 0.2, 0.2], orientation: [0, 0, 0, 1] }, pose_source: 'pnp',
      size_m: 0.0625, measured_size_m: 0.075, reprojection_px: 0.42 },
    { id: 5, pose: null, pose_source: null, size_m: 0.0625, measured_size_m: null, reprojection_px: null },
  ],
  wellplates: [
    { name: 'wellplate', conf: 0.91, pose: { position: [0, 0, 0.62], orientation: [0, 0, 0, 1] },
      pose_source: 'plane', metric_size: [0.085, 0.127] },
    { name: 'wellplate', conf: 0.95, pose: null, pose_source: null, metric_size: null },
  ],
});
const perception = (overrides = {}) => ({
  ...parsePerception(json), raw: json, received: true, ageS: 0.3, ...overrides,
});

test('renders a readable tag table', () => {
  render(<DetectionTables camera="zedx" perception={perception()} />);
  const table = screen.getByRole('table', { name: 'AprilTags' });
  const rows = within(table).getAllByRole('row');
  expect(rows).toHaveLength(3);   // header + 2 tags
  expect(within(rows[1]).getByText('5')).toBeInTheDocument();
  expect(within(rows[1]).getByText('no pose')).toBeInTheDocument();
  expect(within(rows[2]).getByText('0.100, 0.200, 0.200')).toBeInTheDocument();
  expect(within(rows[2]).getByText('0.300')).toBeInTheDocument();
  expect(within(rows[2]).getByText(/20% off/)).toBeInTheDocument();
  expect(within(rows[2]).getByText('0.42')).toBeInTheDocument();
});

test('renders wellplates with the planner target on the ZED X', () => {
  render(<DetectionTables camera="zedx" perception={perception()} />);
  const rows = within(screen.getByRole('table', { name: 'Wellplates' })).getAllByRole('row');
  expect(within(rows[1]).getByText('0.95')).toBeInTheDocument();
  expect(within(rows[1]).getByText('no depth → no marker')).toBeInTheDocument();
  expect(within(rows[2]).getByText('planner target')).toBeInTheDocument();
  expect(within(rows[2]).getByText('85 × 127')).toBeInTheDocument();
});

test('labels the best detection "best" on the Nano', () => {
  render(<DetectionTables camera="zedx_nano" perception={perception()} />);
  expect(screen.getByText('best')).toBeInTheDocument();
  expect(screen.queryByText('planner target')).not.toBeInTheDocument();
});

test('header shows frame and age; stale data is flagged', () => {
  const { rerender } = render(<DetectionTables camera="zedx" perception={perception()} />);
  expect(screen.getByText('top_camera')).toBeInTheDocument();
  expect(screen.getByText('updated 0.3 s ago')).toHaveClass('text-gray-500');
  rerender(<DetectionTables camera="zedx" perception={perception({ ageS: 3.2 })} />);
  expect(screen.getByText('updated 3.2 s ago')).toHaveClass('text-red-600');
});

test('no data and unparseable states', () => {
  const { rerender } = render(<DetectionTables camera="zedx" perception={{
    ...parsePerception('{}'), raw: null, received: false, ageS: null }} />);
  expect(screen.getByText('no data')).toBeInTheDocument();
  expect(screen.getByText('No tags')).toBeInTheDocument();
  expect(screen.getByText('No detections')).toBeInTheDocument();
  rerender(<DetectionTables camera="zedx" perception={{
    ...parsePerception('garbage'), raw: 'garbage', received: true, ageS: 0.1 }} />);
  expect(screen.getByText('Unparseable — see Raw JSON')).toBeInTheDocument();
});

test('raw JSON disclosure pretty-prints', () => {
  render(<DetectionTables camera="zedx" perception={perception()} />);
  fireEvent.click(screen.getByText('Raw JSON'));
  expect(screen.getByText(/"frame_id": "top_camera"/)).toBeInTheDocument();
});
