"""Wait for real handoff conditions without a fixed FA deadline, using fake clocks."""
from types import SimpleNamespace
import unittest

import test_auto_dagger_recording as auto_tests
import test_teaching_handoff as handoff_tests


class UnboundedFAHandoffTests(unittest.TestCase):
    setUp = handoff_tests.TeachingReadinessTests.setUp
    sleep = handoff_tests.TeachingReadinessTests.sleep
    status = handoff_tests.TeachingReadinessTests.status
    commands = handoff_tests.TeachingReadinessTests.commands

    def test_fa_can_complete_after_fifteen_seconds_without_early_acknowledgement(self):
        since = self.clock
        second_ready_at = since + 20.0
        self.commands(sides=('left',))

        def update():
            self.status(sides=('left',))
            if self.clock >= second_ready_at:
                self.status(sides=('right',))
                self.commands(sides=('right',))

        self.on_sleep = update
        self.operator.wait_for_master_control(0xFA, True, since=since, timeout=None)
        self.assertGreaterEqual(self.clock, second_ready_at)
        self.assertLess(self.clock, second_ready_at + .03)
        self.assertTrue(self.operator.dagger_recording_status()[0])

    def test_shutdown_ends_a_deadline_free_wait_that_has_already_exceeded_fifteen_seconds(self):
        since = self.clock

        def shutdown():
            if self.clock >= since + 20.0:
                self.shutdown = True

        self.on_sleep = shutdown
        with self.assertRaisesRegex(RuntimeError, 'Master handoff incomplete'):
            self.operator.wait_for_master_control(0xFA, True, since=since, timeout=None)
        self.assertTrue(self.shutdown)
        self.assertGreaterEqual(self.clock - since, 20.0)
        self.assertLess(self.clock - since, 20.03)

    def test_fc_default_timeout_remains_fifteen_seconds(self):
        since = self.clock
        with self.assertRaisesRegex(RuntimeError, 'Master handoff incomplete'):
            self.operator.wait_for_master_control(0xFC, False, since=since)
        self.assertGreaterEqual(self.clock - since, 15.0)
        self.assertLess(self.clock - since, 15.03)

    def test_exit_dagger_keeps_the_default_fc_timeout(self):
        calls = []
        self.namespace['String'] = SimpleNamespace
        self.namespace['rospy'].loginfo = lambda *args: None
        for side in ('left', 'right'):
            setattr(self.operator, 'master_' + side + '_config_pub',
                    SimpleNamespace(publish=lambda message: calls.append(('request', message.data))))
        self.operator.wait_for_master_control = lambda *args, **options: calls.append(('wait', args, options))
        self.operator.exit_dagger_mode()
        self.assertEqual(calls[:2], [('request', 'slave'), ('request', 'slave')])
        self.assertEqual(calls[2][1], (0xFC, False, self.clock))
        self.assertEqual(calls[2][2], {'quiet_period': .25})

    def test_enter_dagger_explicitly_requests_no_fa_timeout_after_the_existing_hold(self):
        case = auto_tests.AutoDAggerRecordingTests()
        case.setUp()
        self.addCleanup(case.doCleanups)
        observed = []
        acknowledge = case.operator.wait_for_master_control

        def wait(config, teaching, since, **options):
            observed.append((config, teaching, options))
            acknowledge(config, teaching, since, **options)

        case.operator.wait_for_master_control = wait
        self.assertTrue(case.operator.enter_dagger_mode())
        self.assertEqual(observed, [(0xFA, True, {'timeout': None})])
        self.assertTrue(case.acknowledged)
        hold = next(phase for phase in case.phases if phase['name'] == 'hold')
        self.assertGreaterEqual(hold['report']['stable_elapsed_s'], 3.0)


if __name__ == '__main__':
    unittest.main()
