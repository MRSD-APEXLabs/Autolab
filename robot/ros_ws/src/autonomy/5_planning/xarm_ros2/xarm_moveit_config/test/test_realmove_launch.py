"""Construct the real launch on the installed ROS version without starting hardware."""

from pathlib import Path
import runpy
import unittest

from launch import LaunchContext
from launch.actions import EmitEvent, RegisterEventHandler, TimerAction
from launch.events import Shutdown
from launch.events.process import ProcessExited


class RealmoveLaunchTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1] / 'launch' / '_robot_moveit_realmove.launch.py'
        module = runpy.run_path(str(path))
        cls.context = LaunchContext()
        cls.context.launch_configurations.update({
            'robot_ip': '192.168.1.236', 'dof': '6', 'robot_type': 'xarm',
            'show_rviz': 'false', 'hw_ns': 'xarm',
        })
        cls.actions = module['launch_setup'](cls.context)
        index = next(i for i, action in enumerate(cls.actions)
                     if isinstance(action, RegisterEventHandler))
        cls.handler = cls.actions[index].event_handler
        cls.starter = cls.actions[index + 1]

    def result(self, returncode):
        event = ProcessExited(
            action=self.starter, returncode=returncode,
            name='diagnostic-starter', cmd=['unused'], cwd=None, env=None, pid=0)
        self.assertTrue(self.handler.matches(event))
        return self.handler.handle(event, self.context)

    def test_success_schedules_planner_only_after_controller_exit(self):
        self.assertFalse(any(isinstance(action, TimerAction) for action in self.actions))
        result = self.result(0)
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], TimerAction)

    def test_failed_startup_shuts_down_without_scheduling_planner(self):
        result = self.result(1)
        self.assertEqual(len(result), 1)
        self.assertIsInstance(result[0], EmitEvent)
        self.assertIsInstance(result[0].event, Shutdown)


if __name__ == '__main__':
    unittest.main()
