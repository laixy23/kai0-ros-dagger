"""Check teaching readiness without importing ROS or touching CAN hardware."""
import ast
from collections import deque
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest

import test_auto_dagger_recording as auto_tests
from test_compatibility import load_official_methods


SOURCE = Path(__file__).with_name('agilex_openpi_dagger_collect.py')


def bind_method(namespace, operator, name):
    tree = ast.parse(SOURCE.read_text())
    cls = next(node for node in tree.body
               if isinstance(node, ast.ClassDef) and node.name == 'RosOperator')
    method = next(node for node in cls.body
                  if isinstance(node, ast.FunctionDef) and node.name == name)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), namespace)
    setattr(operator, name, namespace[method.name].__get__(operator))


def bind_real_switch(namespace, operator):
    bind_method(namespace, operator, 'switch_masters_to_teach_mode')


class TeachingReadinessTests(unittest.TestCase):
    def setUp(self):
        self.namespace, operator_class = load_official_methods()
        self.operator = operator_class.__new__(operator_class)
        self.operator.master_handoff_lock = threading.Lock()
        self.operator.master_control_feedback = {}
        self.operator.master_prepared_feedback = {}
        self.operator.master_command_seen_at = {}
        self.clock = 10.0
        self.shutdown = False
        self.on_sleep = lambda: None
        self.namespace.update(
            time=SimpleNamespace(monotonic=lambda: self.clock, time=lambda: 1000 + self.clock, sleep=self.sleep),
            rospy=SimpleNamespace(is_shutdown=lambda: self.shutdown))

    def sleep(self, seconds):
        self.clock += seconds
        self.on_sleep()

    def status(self, stamp=None, sides=('left', 'right')):
        for side in sides:
            value = self.clock if stamp is None else stamp
            self.operator.master_control_feedback[side] = (
                value, 0xFA, 1)
            self.operator.master_prepared_feedback[side] = dict(
                received_monotonic=value, source_time=1000 + value, config=0xFA,
                teaching=1, enabled=1)

    def commands(self, stamp=None, sides=('left', 'right')):
        for side in sides:
            self.operator.master_command_seen_at[side] = self.clock if stamp is None else stamp

    def assert_blocked(self):
        with self.assertRaisesRegex(RuntimeError, 'Master handoff incomplete'):
            self.operator.wait_for_master_control(0xFA, True, since=10.0, timeout=0.3)

    def test_fa_state_without_command_streams_is_not_ready(self):
        self.status()
        self.on_sleep = self.status
        self.assert_blocked()

    def test_single_sided_command_stream_is_not_ready(self):
        def update():
            self.status()
            self.commands(sides=('left',))
        update()
        self.on_sleep = update
        self.assert_blocked()

    def test_pre_request_commands_cannot_acknowledge_new_handoff(self):
        self.status()
        self.commands(stamp=9.99)
        self.on_sleep = self.status
        self.assert_blocked()

    def test_fresh_commands_cannot_replace_pre_request_mode_ack(self):
        self.status(stamp=9.99)
        self.commands()
        self.on_sleep = self.commands
        self.assert_blocked()

    def test_seen_post_request_commands_can_be_quiet_when_fa_mode_is_fresh(self):
        self.clock = 11.0
        self.status()
        self.commands(stamp=10.7)
        self.on_sleep = self.status
        self.operator.wait_for_master_control(0xFA, True, since=10.0, timeout=0.3)

    def test_future_command_time_is_rejected(self):
        self.status()
        self.commands(stamp=11.0)
        self.on_sleep = self.status
        self.assert_blocked()

    def test_both_new_command_streams_and_mode_acks_are_required(self):
        def update():
            self.status()
            if self.clock >= 10.1:
                self.commands(sides=('left',))
            if self.clock >= 10.2:
                self.commands(sides=('right',))
        self.on_sleep = update
        self.operator.wait_for_master_control(0xFA, True, since=10.0, timeout=0.5)
        self.assertGreaterEqual(self.clock, 10.2)
        self.assertLess(self.clock, 10.3)

    def test_switch_only_requests_configuration_and_never_unlocks_early(self):
        events = []
        self.namespace.update(
            String=SimpleNamespace,
            rospy=SimpleNamespace(loginfo=lambda text, *args: events.append(('log', text))),
        )
        for side in ('left', 'right'):
            setattr(self.operator, 'master_' + side + '_config_pub',
                    SimpleNamespace(publish=lambda msg, side=side: events.append(('config', side, msg.data))))
            setattr(self.operator, 'master_' + side + '_teach_mode_pub',
                    SimpleNamespace(publish=lambda msg: self.fail('collector unlocked teaching before driver transition')))
        bind_real_switch(self.namespace, self.operator)
        self.operator.assert_followers_ready = lambda: events.append(('followers-ready',))
        self.operator.probe_followers_for_teaching = lambda: events.append(('probe-acknowledged',))
        self.operator.switch_masters_to_teach_mode()
        self.assertEqual(events[0], ('followers-ready',))
        self.assertEqual(events[1], ('probe-acknowledged',))
        self.assertEqual([event for event in events if event[0] == 'config'],
                         [('config', 'left', 'master'), ('config', 'right', 'master')])
        self.assertFalse(any('can now' in event[1] or 'in teaching mode' in event[1]
                             for event in events if event[0] == 'log'))


class TeachingRecordingOrderTests(unittest.TestCase):
    def simulation(self, mode='ready'):
        case = auto_tests.AutoDAggerRecordingTests()
        case.setUp()
        self.addCleanup(case.doCleanups)
        operator = case.operator
        operator.master_handoff_lock = threading.Lock()
        operator.master_control_feedback = {}
        operator.master_prepared_feedback = {}
        operator.master_command_seen_at = {}
        # Keep both real production operations: request the driver-owned
        # transition, then wait for mode acknowledgement AND command streams.
        bind_method(case.namespace, operator, 'release_prepared_masters')
        operator.assert_followers_ready = lambda: case.events.append(('followers-ready', case.clock))
        operator.probe_followers_for_teaching = lambda: case.events.append(('probe-acknowledged', case.clock))
        real_wait = case.namespace['RosOperator'].wait_for_master_control.__get__(operator)
        requested_at = {}
        first_stream = {}

        def request(message, side):
            self.assertEqual(message.data, 'release_master_gripper')
            requested_at[side] = case.clock
            case.events.append(('config-request', case.clock, side))

        for side in ('left', 'right'):
            setattr(operator, 'master_' + side + '_config_pub',
                    SimpleNamespace(publish=lambda message, side=side: request(message, side)))
            setattr(operator, 'master_' + side + '_teach_mode_pub',
                    SimpleNamespace(publish=lambda _message: self.fail('early independent teach request')))

        def sleep(seconds):
            case.clock += seconds
            # Prepared drivers commit the already-primed FA handoff promptly.
            for side, started in requested_at.items():
                if case.clock >= started + 0.05:
                    operator.master_control_feedback[side] = (case.clock, 0xFA, 1)
                    operator.master_prepared_feedback[side] = dict(
                        received_monotonic=case.clock, source_time=1000 + case.clock,
                        config=0xFA, teaching=1, enabled=1)
                    if mode == 'ready' or mode == 'one-sided' and side == 'left':
                        operator.master_command_seen_at[side] = case.clock
                        if side not in first_stream:
                            first_stream[side] = case.clock
                            case.events.append(('stream-start', case.clock, side))
                    elif mode == 'old':
                        operator.master_command_seen_at[side] = started - 0.01

        case.namespace['time'].sleep = sleep

        def wait(config, teaching, since, **options):
            self.assertEqual((config, teaching), (0xFA, True))
            case.events.append(('wait', case.clock))
            real_wait(config, teaching, since, timeout=0.5)
            case.acknowledged = True
            case.events.append(('ack', case.clock))
        operator.wait_for_master_control = wait
        case.key('d')
        case.on_tick = lambda _tick: (_ for _ in ()).throw(auto_tests.StopLoop())
        return case, requested_at, first_stream

    def test_recording_starts_after_driver_transition_and_both_command_streams(self):
        case, requested, streams = self.simulation()
        with self.assertRaises(auto_tests.StopLoop):
            case.run_model()
        events = [event[0] for event in case.events]
        self.assertLess(events.index('hold-finished'), events.index('config-request'))
        self.assertLess(events.index('prepare-start'), events.index('hold-finished'))
        self.assertLess(events.index('config-request'), events.index('stream-start'))
        self.assertLess(events.index('stream-start'), events.index('ack'))
        self.assertLess(events.index('ack'), events.index('record'))
        self.assertEqual(set(streams), {'left', 'right'})
        record = next(event[1] for event in case.events if event[0] == 'record')
        self.assertGreaterEqual(record, max(streams.values()))
        self.assertEqual(case.record_count, 1)
        self.assertGreaterEqual(case.phases[-1]['report']['stable_elapsed_s'], 3.0)
        for side in ('left', 'right'):
            self.assertGreaterEqual(streams[side] - requested[side], 0.05)

    def test_missing_single_sided_or_old_commands_block_automatic_recording(self):
        for mode in ('missing', 'one-sided', 'old'):
            with self.subTest(mode=mode):
                case, _requested, _streams = self.simulation(mode)
                with self.assertRaisesRegex(RuntimeError, 'Master handoff incomplete'):
                    case.run_model()
                self.assertEqual(case.record_count, 0)
                self.assertFalse(case.namespace['collection_active'])
                self.assertFalse(case.acknowledged)

    def test_follower_preflight_failure_keeps_master_configuration_unchanged(self):
        case, requested, _streams = self.simulation()
        def reject():
            raise RuntimeError('right follower is not ready')
        case.operator.assert_followers_ready = reject
        state = dict(deadline=case.clock + 15, cancel=threading.Event(), done=threading.Event())
        case.operator._prepare_following_worker(state)
        self.assertIn('right follower is not ready', str(state['error']))
        self.assertEqual(requested, {})
        self.assertEqual(case.record_count, 0)
        self.assertFalse(case.namespace['collection_active'])

    def test_failed_holding_probe_does_not_send_fa_or_start_recording(self):
        case, requested, _streams = self.simulation()
        def reject(**kwargs):
            raise RuntimeError('follower holding probe was not acknowledged')
        case.operator.probe_followers_for_teaching = reject
        state = dict(deadline=case.clock + 15, cancel=threading.Event(), done=threading.Event())
        case.operator._prepare_following_worker(state)
        self.assertIn('holding probe was not acknowledged', str(state['error']))
        self.assertEqual(requested, {})
        self.assertEqual(case.record_count, 0)
        self.assertFalse(case.namespace['collection_active'])


class FollowerHoldingProbeTests(unittest.TestCase):
    def setUp(self):
        from teaching_readiness import require_pair_ready
        self.namespace, operator_class = load_official_methods()
        self.operator = operator_class.__new__(operator_class)
        self.clock = 10.0
        self.shutdown = False
        self.published = []
        self.ack_mode = 'ready'
        self.ready_after = 10.2
        self.on_publish = lambda: None
        self.operator.master_handoff_lock = threading.Lock()
        self.operator.follower_ready_feedback = {}
        self.operator.require_fresh_policy_inputs = lambda: None
        self.namespace.update(
            time=SimpleNamespace(monotonic=lambda: self.clock, time=lambda: 1000 + self.clock,
                                 sleep=lambda seconds: setattr(self, 'clock', self.clock + seconds)),
            rospy=SimpleNamespace(is_shutdown=lambda: self.shutdown,
                                  Time=SimpleNamespace(now=lambda: self.stamp(1000 + self.clock))),
            math=__import__('math'), alignment_targets=auto_tests.alignment_targets,
            require_pair_ready=require_pair_ready)
        for name in ('probe_followers_for_teaching', 'follower_ready_callback'):
            bind_method(self.namespace, self.operator, name)
        for side, width in (('left', 0.04), ('right', 0.03)):
            setattr(self.operator, 'puppet_arm_' + side + '_deque', deque([
                SimpleNamespace(header=SimpleNamespace(stamp=self.stamp(1010.0)),
                                position=[0.1] * 6 + [width])]))
        self.operator.puppet_arm_publish = self.publish

    @staticmethod
    def stamp(value):
        return SimpleNamespace(to_sec=lambda: value)

    def publish(self, left, right, **kwargs):
        self.published.append((self.clock, list(left), list(right)))
        self.on_publish()
        if self.clock < self.ready_after or self.ack_mode == 'missing':
            return
        for side in ('left', 'right'):
            if self.ack_mode == 'one-sided' and side == 'right':
                continue
            source = 1000 + self.clock
            command = source if self.ack_mode != 'old-command' else 1009.99
            if self.ack_mode == 'stale-source':
                source -= 0.5
            message = SimpleNamespace(angular=SimpleNamespace(x=source),
                                      linear=SimpleNamespace(x=1.0, y=command))
            self.operator.follower_ready_callback(message, side)

    def test_real_callback_acknowledges_probe_before_fa_request_is_allowed(self):
        self.operator.probe_followers_for_teaching()
        self.assertTrue(self.operator.follower_probe_report['passed'])
        self.assertGreaterEqual(self.clock, self.ready_after)
        self.assertGreaterEqual(len(self.published), 3)
        self.assertEqual(set(self.operator.follower_ready_feedback), {'left', 'right'})

    def test_probe_repeats_fixed_actual_snapshot_even_if_feedback_changes(self):
        def drift_feedback():
            for side in ('left', 'right'):
                getattr(self.operator, 'puppet_arm_' + side + '_deque')[-1].position = [0.2] * 7
        self.on_publish = drift_feedback
        self.operator.probe_followers_for_teaching()
        for _stamp, left, right in self.published:
            self.assertEqual(left, [0.1] * 6 + [0.04])
            self.assertEqual(right, [0.1] * 6 + [0.03])

    def test_missing_one_sided_old_command_or_stale_source_ack_is_rejected(self):
        for mode in ('missing', 'one-sided', 'old-command', 'stale-source'):
            with self.subTest(mode=mode):
                self.clock = 10.0
                self.ack_mode = mode
                self.operator.follower_ready_feedback = {}
                with self.assertRaisesRegex(RuntimeError, 'before master release'):
                    self.operator.probe_followers_for_teaching(timeout=0.5)
                self.assertFalse(self.operator.follower_probe_report['passed'])

    def test_pre_probe_heartbeats_do_not_pass_without_new_callback_ack(self):
        self.ack_mode = 'missing'
        self.operator.follower_ready_feedback = {
            side: dict(received_monotonic=9.99, source_time=1009.99,
                       ready=True, command_time=1009.99)
            for side in ('left', 'right')}
        with self.assertRaisesRegex(RuntimeError, 'before master release'):
            self.operator.probe_followers_for_teaching(timeout=0.3)

    def test_stale_actual_snapshot_is_rejected_before_any_probe_command(self):
        self.operator.puppet_arm_right_deque[-1].header.stamp = self.stamp(1009.7)
        with self.assertRaisesRegex(RuntimeError, 'Stale actual follower feedback'):
            self.operator.probe_followers_for_teaching()
        self.assertEqual(self.published, [])

    def test_malformed_snapshot_is_rejected_before_any_probe_command(self):
        self.operator.puppet_arm_left_deque[-1].position[2] = float('nan')
        with self.assertRaises(ValueError):
            self.operator.probe_followers_for_teaching()
        self.assertEqual(self.published, [])

    def test_probe_needs_fresh_followers_but_not_camera_inputs_required_by_policy(self):
        checked = []
        def missing_camera():
            checked.append('policy-check')
            raise RuntimeError('Missing/nonfinite policy input: camera_front')
        self.operator.require_fresh_policy_inputs = missing_camera
        self.namespace.update(JointState=SimpleNamespace, Header=SimpleNamespace)
        for side in ('left', 'right'):
            setattr(self.operator, 'puppet_arm_' + side + '_publisher', SimpleNamespace(publish=lambda message: None))
        real_publish = self.namespace['RosOperator'].puppet_arm_publish.__get__(self.operator)
        with self.assertRaisesRegex(RuntimeError, 'camera_front'):
            real_publish([0.1] * 7, [0.2] * 7)
        def probe_publish(left, right, **kwargs):
            self.assertIs(kwargs['require_policy_inputs'], False)
            real_publish(left, right, **kwargs)
            self.publish(left, right, **kwargs)
        self.operator.puppet_arm_publish = probe_publish
        self.operator.probe_followers_for_teaching()
        self.assertTrue(self.operator.follower_probe_report['passed'])
        self.assertEqual(checked, ['policy-check'])


if __name__ == '__main__':
    unittest.main()
