"""Fake event/timer tests only; no ROS imports, child process, or hardware."""
import unittest

from run_first_live_cycle import INITIAL_POSE, RESUME_MARKER, LiveCycleSupervisor


class SupervisorTests(unittest.TestCase):
    def first_trial(self):
        state = LiveCycleSupervisor(10)
        command = list(INITIAL_POSE)
        command[0] = 0.1
        state.command('/official_collector', command, 11, '/master/joint_left', 1001)
        return state

    def test_initial_pose_is_ignored_and_policy_trial_is_two_seconds(self):
        state = LiveCycleSupervisor(10)
        state.command('/official_collector', INITIAL_POSE, 10.5)
        self.assertFalse(state.snapshot()['first_policy_seen'])
        command = list(INITIAL_POSE)
        command[0] = 0.1
        state.command('/official_collector', command, 11)
        self.assertIsNone(state.tick(12.999))
        self.assertEqual(state.tick(13), 'd')
        self.assertIsNone(state.tick(13.01))
        self.assertEqual(state.snapshot()['first_policy']['callerid'], '/official_collector')

    def test_no_policy_startup_fallback_fires_once_at_fifteen_seconds(self):
        state = LiveCycleSupervisor(10)
        self.assertIsNone(state.tick(24.999))
        self.assertEqual(state.tick(25), 'd')
        self.assertIsNone(state.tick(25.1))
        self.assertEqual(state.snapshot()['takeover_reason'], 'no_policy_startup_fallback')

    def test_late_first_policy_gets_two_seconds_instead_of_no_policy_fallback(self):
        state = LiveCycleSupervisor(10)
        command = list(INITIAL_POSE)
        command[0] = 0.1
        state.command('/official_collector', command, 24)
        self.assertIsNone(state.tick(25))
        self.assertEqual(state.tick(26), 'd')

    def test_resume_requires_marker_and_same_caller_then_stops_after_one_second(self):
        state = self.first_trial()
        self.assertEqual(state.tick(13), 'd')
        state.output('DAGGER MODE ACTIVE', 20)
        state.forwarded_key('r', 30)
        state.command('/official_collector', INITIAL_POSE, 30.1)
        self.assertFalse(state.snapshot()['resume_policy_seen'])
        state.output(RESUME_MARKER, 31)
        state.command('/master_arm', INITIAL_POSE, 31.1)
        self.assertFalse(state.snapshot()['resume_policy_seen'])
        state.command('/official_collector', INITIAL_POSE, 31.2)
        self.assertTrue(state.snapshot()['resume_policy_seen'])
        self.assertIsNone(state.tick(32.199))
        self.assertEqual(state.tick(32.2), 'stop')
        self.assertEqual(state.snapshot()['stop_reason'], 'completed_one_second_resume')

    def test_resume_timeout_without_correct_policy_source(self):
        state = self.first_trial()
        state.tick(13)
        state.output('DAGGER MODE ACTIVE', 20)
        state.forwarded_key('r', 30)
        state.output(RESUME_MARKER, 31)
        state.command('/master_arm', INITIAL_POSE, 31.1)
        self.assertIsNone(state.tick(69.999))
        self.assertEqual(state.tick(70), 'stop')
        self.assertEqual(state.snapshot()['stop_reason'], 'resume_policy_timeout')

    def test_missing_dagger_ack_is_bounded_and_invalid_commands_are_ignored(self):
        state = LiveCycleSupervisor(10)
        state.command('/official_collector', [float('nan')] * 7, 11)
        state.command('', [0.0] * 7, 11)
        state.command('/official_collector', [0.0] * 6, 11)
        self.assertFalse(state.snapshot()['first_policy_seen'])
        state.tick(25)
        self.assertEqual(state.tick(70), 'stop')
        self.assertEqual(state.snapshot()['stop_reason'], 'takeover_ack_timeout')


if __name__ == '__main__':
    unittest.main()
