#!/usr/bin/env python3
"""Regression coverage for controller transitions with ambiguous service replies."""

import importlib.util
from pathlib import Path
import unittest


path = Path(__file__).resolve().parents[1] / 'scripts' / 'start_trajectory_controller.py'
spec = importlib.util.spec_from_file_location('controller_startup', path)
startup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(startup)


class Backend:
    def __init__(self, state=None, lost=(), reject=None, action=True):
        self.current = state
        self.lost = set(lost)
        self.reject = reject
        self.action = action
        self.operations = []
        self.warnings = []
        self.now = 0.0
        self.query_failures = 0
        self.inspections = 0

    def running(self):
        return True

    def state(self, deadline):
        self.inspections += 1
        if self.query_failures:
            self.query_failures -= 1
            raise startup.ReplyTimeout('list reply lost')
        return self.current

    def transition(self, operation, deadline):
        self.operations.append(operation)
        if self.reject == operation:
            return False
        self.current = {'load': 'unconfigured', 'configure': 'inactive', 'activate': 'active'}[operation]
        if operation in self.lost:
            raise startup.ReplyTimeout(f'{operation} reply lost after server executed it')
        return True

    def action_ready(self, deadline):
        return self.action

    def warn(self, message):
        self.warnings.append(message)

    def sleep(self, seconds):
        self.now += seconds

    def start(self, timeout=3.0):
        startup.ensure_ready(self, timeout, clock=lambda: self.now, sleep=self.sleep)


class ControllerStartupTest(unittest.TestCase):
    def test_lost_mutation_replies_do_not_repeat_completed_transitions(self):
        for lost in [('load',), ('configure',), ('activate',), ('load', 'configure', 'activate')]:
            with self.subTest(lost=lost):
                backend = Backend(lost=lost)
                backend.start()
                self.assertEqual(backend.operations, ['load', 'configure', 'activate'])
                self.assertEqual(backend.current, 'active')
                self.assertEqual(len(backend.warnings), len(lost))

    def test_failed_query_does_not_trigger_a_load(self):
        backend = Backend(state='active')
        backend.query_failures = 2
        backend.start()
        self.assertEqual(backend.operations, [])

    def test_timed_out_request_that_did_not_execute_can_be_retried(self):
        backend = Backend()
        transition = backend.transition
        attempts = []

        def fail_first_load(operation, deadline):
            attempts.append(operation)
            if len(attempts) == 1:
                raise startup.ReplyTimeout('load request never reached the server')
            return transition(operation, deadline)

        backend.transition = fail_first_load
        backend.start()
        self.assertEqual(attempts, ['load', 'load', 'configure', 'activate'])
        self.assertEqual(backend.current, 'active')

    def test_existing_controller_resumes_from_its_lifecycle_state(self):
        for state, expected in [('unconfigured', ['configure', 'activate']),
                                ('inactive', ['activate']), ('active', [])]:
            with self.subTest(state=state):
                backend = Backend(state=state)
                backend.start()
                self.assertEqual(backend.operations, expected)

    def test_rejected_activation_fails_without_other_mutations(self):
        backend = Backend(state='inactive', reject='activate')
        with self.assertRaisesRegex(RuntimeError, 'activate was rejected'):
            backend.start()
        self.assertEqual(backend.operations, ['activate'])

    def test_missing_action_server_never_reports_ready_or_reactivates(self):
        backend = Backend(state='active', action=False)
        with self.assertRaisesRegex(RuntimeError, 'trajectory action server is unavailable'):
            backend.start()
        self.assertEqual(backend.operations, [])

    def test_discovery_failure_has_a_bounded_deadline(self):
        backend = Backend()
        backend.query_failures = 100
        with self.assertRaisesRegex(RuntimeError, 'within 1s'):
            backend.start(timeout=1.0)
        self.assertEqual(backend.operations, [])
        self.assertLessEqual(backend.now, 1.0)

    def test_terminal_state_is_not_reloaded(self):
        backend = Backend(state='finalized')
        with self.assertRaisesRegex(RuntimeError, 'lifecycle state'):
            backend.start()
        self.assertEqual(backend.operations, [])


if __name__ == '__main__':
    unittest.main()
