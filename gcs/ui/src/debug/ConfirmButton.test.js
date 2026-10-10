import { render, screen, fireEvent } from '@testing-library/react';
import ConfirmButton from './ConfirmButton';

test('does not call onConfirm on first click', () => {
  const onConfirm = jest.fn();
  render(<ConfirmButton onConfirm={onConfirm}>Send Goal</ConfirmButton>);
  fireEvent.click(screen.getByRole('button', { name: 'Send Goal' }));
  expect(onConfirm).not.toHaveBeenCalled();
});

test('calls onConfirm only after the confirm step is clicked', () => {
  const onConfirm = jest.fn();
  render(<ConfirmButton onConfirm={onConfirm}>Send Goal</ConfirmButton>);
  fireEvent.click(screen.getByRole('button', { name: 'Send Goal' }));
  fireEvent.click(screen.getByRole('button', { name: 'Confirm?' }));
  expect(onConfirm).toHaveBeenCalledTimes(1);
});

test('cancel returns to the initial state without calling onConfirm', () => {
  const onConfirm = jest.fn();
  render(<ConfirmButton onConfirm={onConfirm}>Send Goal</ConfirmButton>);
  fireEvent.click(screen.getByRole('button', { name: 'Send Goal' }));
  fireEvent.click(screen.getByRole('button', { name: 'Cancel' }));
  expect(screen.getByRole('button', { name: 'Send Goal' })).toBeInTheDocument();
  expect(onConfirm).not.toHaveBeenCalled();
});

test('disabled prop disables the initial button', () => {
  render(<ConfirmButton onConfirm={() => {}} disabled>Send Goal</ConfirmButton>);
  expect(screen.getByRole('button', { name: 'Send Goal' })).toBeDisabled();
});
