"""A failed explicit r request is retryable and never releases policy control."""
from contextlib import redirect_stdout
import io
import threading
from types import SimpleNamespace
import unittest

import test_teaching_runtime_state as runtime
import test_teaching_handoff as handoff


class RetryHandoffTests(unittest.TestCase):
    def fixture(self):
        case = runtime.RuntimeModelTests()
        case.setUp()
        self.addCleanup(case.doCleanups)
        return case

    def test_failed_r_keeps_data_accepts_save_and_folds_keys_during_attempt(self):
        case = self.fixture()
        attempts = []
        prefix = []
        saved = []
        def exit_failed():
            attempts.append('attempt')
            self.assertTrue(case.ns['resume_inference_in_progress'])
            self.assertTrue(case.ns['dagger_mode_lock'].acquire(False), 'handoff waited while holding keyboard lock')
            case.ns['dagger_mode_lock'].release()
            for _ in range(6):
                case.key('r')
            self.assertFalse(case.ns['resume_inference_requested'])
            raise RuntimeError('right mode=0x00; publisher=quiet; expected FC')
        case.node.exit_dagger_mode = exit_failed
        case.collector.save_current_episode = lambda **kwargs: saved.append(list(case.collector.actions))
        def tick(count):
            if count == 2:
                prefix.append(case.collector.timesteps[0])
                case.key('r')
            elif count == 3:
                self.assertTrue(case.ns['dagger_mode_active'])
                self.assertFalse(case.ns['resume_inference_requested'])
                self.assertFalse(case.ns['resume_inference_in_progress'])
                self.assertFalse(case.collector.is_collecting)
                self.assertEqual(len(case.collector.actions), 1)
                self.assertIs(case.collector.timesteps[0], prefix[0])
                case.key(' ')
            elif count == 4:
                self.assertEqual(case.start_calls, 1, 'Space discarded the unsaved correction prefix')
                case.key('s')
            elif count == 5:
                self.assertEqual(len(saved[0]), 1)
                raise runtime.StopModel
        case.on_tick = tick
        output = io.StringIO()
        with redirect_stdout(output), self.assertRaises(runtime.StopModel):
            case.run_model()
        self.assertEqual(attempts, ['attempt'])
        self.assertEqual(case.actions, ['startup-only'])
        self.assertEqual(case.ns['inference_generation'], 0)
        self.assertEqual(output.getvalue().count('Policy resume requested'), 1)
        self.assertIn("Press 'r' to retry", output.getvalue())

    def test_later_manual_r_retries_once_then_uses_original_success_path(self):
        case = self.fixture()
        publications = []
        attempts = []
        for side in ('left', 'right'):
            setattr(case.node, 'master_' + side + '_config_pub', SimpleNamespace(
                publish=lambda message, side=side: publications.append((side, message.data))))
        case.node.exit_dagger_mode = case.ns['RosOperator'].exit_dagger_mode.__get__(case.node)
        case.ns['String'] = SimpleNamespace
        def wait(*args, **kwargs):
            attempts.append('attempt')
            for _ in range(3):
                case.key('r')
            if len(attempts) == 1:
                raise RuntimeError('right mode=0x00; expected FC')
        case.node.wait_for_master_control = wait
        def tick(count):
            if count == 2:
                case.key('r')
            elif count == 4:
                self.assertEqual(len(attempts), 1)
                self.assertTrue(case.ns['dagger_mode_active'])
                case.key('r')
        case.on_tick = tick
        case.ns['rospy'].loginfo = lambda *args: None
        with self.assertRaises(runtime.StopModel):
            case.run_model()
        self.assertEqual(len(attempts), 2)
        self.assertEqual(publications, [('left', 'slave'), ('right', 'slave')] * 2)
        self.assertEqual(case.actions, ['startup-only', 'explicit-restart'])
        self.assertEqual(case.ns['inference_generation'], 1)
        self.assertFalse(case.ns['resume_inference_in_progress'])

    def test_key_read_during_lock_hold_is_dropped_without_blocking_later_retry(self):
        case = self.fixture()
        finished = threading.Event()
        case.ns['dagger_mode_lock'].acquire()
        worker = threading.Thread(target=lambda: (case.key('r'), finished.set()))
        worker.start()
        try:
            self.assertTrue(finished.wait(0.5), 'r key queued behind the mode lock')
            self.assertFalse(case.ns['resume_inference_requested'])
        finally:
            case.ns['dagger_mode_lock'].release()
            worker.join(timeout=1)
        case.key('r')
        self.assertTrue(case.ns['resume_inference_requested'])

    def test_fc_timeout_describes_side_mode_and_live_publisher_not_teaching_stream(self):
        case = handoff.TeachingReadinessTests()
        case.setUp()
        self.addCleanup(case.doCleanups)
        def refresh():
            case.operator.master_control_feedback = {
                'left': (case.clock, 0xFC, 0), 'right': (case.clock, 0, 0)}
            case.operator.master_prepared_feedback = {
                side: {'enabled': 1} for side in ('left', 'right')}
            case.operator.master_command_seen_at['right'] = case.clock
        refresh()
        case.on_sleep = refresh
        with self.assertRaises(RuntimeError) as raised:
            case.operator.wait_for_master_control(0xFC, False, since=10, timeout=0.4, quiet_period=0.25)
        message = str(raised.exception)
        self.assertIn('left[mode=0xFC', message)
        self.assertIn('right[mode=0x00', message)
        self.assertIn('publisher=still-publishing', message)
        self.assertNotIn('teaching command streams', message)

    def test_readiness_preflight_uses_raw_followers_without_rosgraph_or_cameras(self):
        case = handoff.FollowerHoldingProbeTests()
        case.setUp()
        self.addCleanup(case.doCleanups)
        handoff.bind_method(case.namespace, case.operator, 'assert_followers_ready')
        case.operator.args = SimpleNamespace(puppet_arm_left_cmd_topic='/left', puppet_arm_right_cmd_topic='/right')
        for side in ('left', 'right'):
            setattr(case.operator, 'puppet_arm_' + side + '_publisher', SimpleNamespace(get_num_connections=lambda: 2))
        case.operator.require_fresh_policy_inputs = lambda: self.fail('camera must not gate mechanical preparation')
        calls = []
        case.namespace['check_followers_ready'] = lambda topics, connections: calls.append((topics, connections)) or {'passed': True}
        case.operator.assert_followers_ready()
        self.assertEqual(len(calls), 1)
        self.assertTrue(case.operator.follower_readiness_report['passed'])


if __name__ == '__main__':
    unittest.main()
