import { render, screen, fireEvent } from '@testing-library/react';
import ClogCheckCard from './ClogCheckCard';
import { TIP_CHECK_URL } from './shared/tipCheckApi';

const aspirate = { action: 'aspirate', resource_name: 'tube_rack', well_indices: [0], volumes: [200] };
const tip = (id, fields) => ({ type: 'tip_check', id, step: 4, protocol_step: 1, aspirate, ...fields });

beforeEach(() => {
  global.fetch = jest.fn();
});

afterEach(() => {
  delete global.fetch;
});

test('shows one row per tip check with its verdict, never calling a service', () => {
  render(<ClogCheckCard recordings={[
    tip('r-1', { status: 'done', anomaly_detected: false, p_tip_empty: 0.08, threshold: 0.5 }),
    tip('r-3', { status: 'done', anomaly_detected: true, p_tip_empty: 0.71, threshold: 0.5 }),
    tip('r-5', { status: 'error', error: 'tip-check service unreachable at http://x:8110' }),
  ]} />);
  expect(screen.getByText('Liquid in tip · P(empty) 0.08')).toBeInTheDocument();
  expect(screen.getByText('Clog suspected · P(empty) 0.71')).toBeInTheDocument();
  expect(screen.getByText('Check failed')).toBeInTheDocument();
  expect(screen.getAllByText(/tube_rack wells 0 · 200 µL/)).toHaveLength(3);
  expect(screen.getByText(/3 tip checks/)).toBeInTheDocument();
  expect(global.fetch).not.toHaveBeenCalled();
});

test('expanding a row shows the samples, warning, error and the images from the tip-check service', () => {
  render(<ClogCheckCard recordings={[
    tip('r-1', {
      status: 'done', anomaly_detected: true, p_tip_empty: 0.7, threshold: 0.5, samples: [0.6, 0.8],
      summary: 'Average of 2 checks: tip looks empty (P(empty)=0.70)', warning: '1 of 2 checks reported a warning. blur',
      files: { crop: '/runs/c1/crop.jpg', preview: '/runs/c1/preview.jpg' },
    }),
  ]} />);
  fireEvent.click(screen.getByRole('button', { name: /aspirate/i }));
  expect(screen.getByText(/Average of 2 checks/)).toBeInTheDocument();
  expect(screen.getByText(/0\.60, 0\.80/)).toBeInTheDocument();
  expect(screen.getByText(/reported a warning\. blur/)).toBeInTheDocument();
  expect(screen.getByAltText(/tip crop/i)).toHaveAttribute('src', `${TIP_CHECK_URL}/runs/c1/crop.jpg`);
  expect(screen.getByAltText(/snapshot/i)).toHaveAttribute('src', `${TIP_CHECK_URL}/runs/c1/preview.jpg`);
});

test('ignores entries that are not tip checks', () => {
  const { container } = render(<ClogCheckCard recordings={['old-clip', { id: 'x' }]} />);
  expect(container).toBeEmptyDOMElement();
});
