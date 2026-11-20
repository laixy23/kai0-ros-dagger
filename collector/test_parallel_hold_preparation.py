"""Real-thread preparation/hold overlap with fake ROS/CAN endpoints only."""
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest

from test_compatibility import load_official_methods

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'setup'))
from master_alignment_feedback import alignment_targets, COMPONENTS, wait_for_master_alignment


TARGETS = {'left': [0.1] * 6 + [0.04], 'right': [0.2] * 6 + [0.03]}


def status(phase_since, *, source=None, config=0xFC, teaching=0, enabled=1):
    return SimpleNamespace(angular=SimpleNamespace(x=time.time() if source is None else source,
                                                   y=config, z=phase_since),
                           linear=SimpleNamespace(x=1.0, y=enabled, z=teaching))


class PreparedStatusTests(unittest.TestCase):
    def setUp(self):
        self.ns, cls = load_official_methods()
        self.node = cls.__new__(cls)
        self.node.master_handoff_lock = threading.Lock()
        self.node.master_control_feedback = {}
        self.node.master_prepared_feedback = {}
        self.ns.update(time=SimpleNamespace(monotonic=lambda: 10.2, time=lambda: 1000.2), math=math)
        for side in ('left', 'right'):
            self.node.master_control_status_callback(status(1000.1, source=1000.2), side)

    def test_prepared_marker_is_separate_from_legacy_control_tuple(self):
        self.assertEqual(self.node.master_control_feedback['left'], (10.2, 0xFC, 0))
        self.assertTrue(self.node._masters_prepared_current(10.0, 1000.0))

    def test_both_sides_need_current_phase_fresh_receipt_fc_and_enable(self):
        baseline = dict(self.node.master_prepared_feedback['right'])
        cases = ({'phase_since': 0}, {'phase_since': 999.99}, {'received_monotonic': 9.99},
                 {'source_time': 999.6}, {'config': 0xFA}, {'teaching': 1}, {'enabled': 0},
                 {'phase_since': 1001}, {'phase_since': float('nan')})
        for mutation in cases:
            with self.subTest(mutation=mutation):
                self.node.master_prepared_feedback['right'] = dict(baseline, **mutation)
                self.assertFalse(self.node._masters_prepared_current(10.0, 1000.0))
        self.node.master_prepared_feedback.pop('right')
        self.assertFalse(self.node._masters_prepared_current(10.0, 1000.0))

    def test_first_release_immediately_consuming_left_prepared_does_not_block_right(self):
        done = threading.Event()
        done.set()
        self.ns['rospy'] = SimpleNamespace(is_shutdown=lambda: False)
        self.ns['String'] = SimpleNamespace
        self.node.parallel_following_preparation = dict(
            done=done, cancel=threading.Event(), success=True, error=None, hold_passed=True,
            requested_monotonic=10.0, requested_wall=1000.0, deadline=20.0)
        calls = []
        def release(side, message):
            self.assertEqual(message.data, 'release_master_gripper')
            calls.append(side)
            self.node.master_control_status_callback(status(0, source=1000.2, config=0xFA, teaching=1), side)
        for side in ('left', 'right'):
            setattr(self.node, 'master_' + side + '_config_pub',
                    SimpleNamespace(publish=lambda message, side=side: release(side, message)))
        self.node.release_prepared_masters()
        self.assertEqual(calls, ['left', 'right'])

    def release_with_temporary_prepared_loss(self, restore_at):
        clock = [10.2]
        calls = []
        self.ns['rospy'] = SimpleNamespace(is_shutdown=lambda: False)
        self.ns['String'] = SimpleNamespace
        def refresh():
            marker = 1000.1 if restore_at is not None and clock[0] >= restore_at else 0.0
            for side in ('left', 'right'):
                self.node.master_control_status_callback(status(marker, source=1000 + clock[0] - 10), side)
        def sleep(seconds):
            clock[0] += seconds
            refresh()
        self.ns['time'] = SimpleNamespace(monotonic=lambda: clock[0],
                                          time=lambda: 1000 + clock[0] - 10, sleep=sleep)
        done = threading.Event()
        done.set()
        self.node.parallel_following_preparation = dict(
            done=done, cancel=threading.Event(), success=True, error=None, hold_passed=True,
            requested_monotonic=10.0, requested_wall=1000.0, deadline=10.4)
        refresh()
        for side in ('left', 'right'):
            setattr(self.node, 'master_' + side + '_config_pub', SimpleNamespace(
                publish=lambda message, side=side: calls.append((side, message.data, clock[0]))))
        return clock, calls

    def test_completed_hold_waits_for_prepared_status_to_recover_before_releasing_pair(self):
        clock, calls = self.release_with_temporary_prepared_loss(restore_at=10.26)
        self.node.release_prepared_masters()
        self.assertGreaterEqual(clock[0], 10.26)
        self.assertLess(clock[0], 10.4)
        self.assertEqual([(side, command) for side, command, _at in calls],
                         [('left', 'release_master_gripper'), ('right', 'release_master_gripper')])

    def test_completed_hold_with_no_prepared_recovery_times_out_without_release(self):
        clock, calls = self.release_with_temporary_prepared_loss(restore_at=None)
        with self.assertRaisesRegex(RuntimeError, 'deadline'):
            self.node.release_prepared_masters()
        self.assertGreaterEqual(clock[0], 10.4)
        self.assertEqual(calls, [])


class ParallelHoldTests(unittest.TestCase):
    def setUp(self):
        self.ns, cls = load_official_methods()
        self.node = cls.__new__(cls)
        self.node.master_handoff_lock = threading.Lock()
        self.node.master_control_feedback = {}
        self.node.master_prepared_feedback = {}
        self.node.master_alignment_reports = {}
        self.shutdown = False
        self.stopped = threading.Event()
        self.probe_entered = threading.Event()
        self.probe_blocked = False
        self.preflight_error = None
        self.probe_error = None
        self.raw_fault = False
        self.raw_drift = False
        self.cancel_after = None
        self.temporary_not_ready = None
        self.native_failure_at = None
        self.prep_delay = {'left': 0.2, 'right': 0.2}
        self.requested = {}
        self.events = []
        self.published = []
        self.probe_done = None
        self.ns.update(
            time=time, math=math, threading=threading, String=SimpleNamespace,
            rospy=SimpleNamespace(is_shutdown=lambda: self.shutdown,
                                  loginfo=lambda *args: None, logerr=lambda *args: None))
        self.converted = alignment_targets(TARGETS)
        self.ns['wait_for_master_alignment'] = lambda targets, **options: wait_for_master_alignment(
            targets, reader=self.read, clock=time.monotonic, wall=time.time, pause=time.sleep, **options)
        self.node.assert_followers_ready = self.preflight
        self.node.probe_followers_for_teaching = self.probe
        for side in ('left', 'right'):
            setattr(self.node, 'master_' + side + '_config_pub',
                    SimpleNamespace(publish=lambda message, side=side: self.config(side, message)))
        self.heartbeat = threading.Thread(target=self.heartbeats, daemon=True)
        self.heartbeat.start()
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.stopped.set()
        self.heartbeat.join(timeout=1)
        state = getattr(self.node, 'parallel_following_preparation', None)
        if state is not None:
            state['cancel'].set()
            state['thread'].join(timeout=0.5)
            self.assertFalse(state['thread'].is_alive(), 'preparation worker leaked after cancellation')

    def preflight(self):
        self.events.append(('preflight', time.monotonic()))
        if self.preflight_error:
            raise self.preflight_error

    def probe(self, *, timeout, cancel_event):
        self.probe_entered.set()
        self.events.append(('probe-start', time.monotonic()))
        if self.probe_error:
            raise self.probe_error
        while self.probe_blocked:
            if cancel_event.is_set():
                raise RuntimeError('probe cancelled')
            time.sleep(0.005)
        self.probe_done = time.monotonic()
        self.events.append(('probe-done', self.probe_done))

    def config(self, side, message):
        now = time.monotonic()
        self.events.append((message.data, now, side))
        if message.data == 'prepare_master':
            self.assertIsNotNone(self.probe_done)
            self.assertGreaterEqual(now, self.probe_done)
            self.requested[side] = (now, time.time())
        elif message.data == 'release_master_gripper':
            # A fast left driver can publish FA before the right release send.
            # The collector must commit the pair from one validated snapshot.
            self.node.master_control_status_callback(status(0, config=0xFA, teaching=1), side)
        else:
            self.fail('parallel path used legacy serial master request')

    def heartbeats(self):
        while not self.stopped.is_set():
            now = time.monotonic()
            for side in ('left', 'right'):
                requested = self.requested.get(side)
                delay = self.prep_delay.get(side)
                marker = 0.0
                if requested and delay is not None and now - requested[0] >= delay:
                    marker = requested[1] + 0.00001
                elapsed = now - requested[0] if requested else 0
                if self.temporary_not_ready and self.temporary_not_ready[0] <= elapsed < self.temporary_not_ready[1]:
                    marker = 0.0
                config = 0 if self.native_failure_at is not None and elapsed >= self.native_failure_at else 0xFC
                self.node.master_control_status_callback(status(marker, config=config), side)
            time.sleep(0.005)

    def read(self, channel, since_timestamp=0):
        side = 'left' if 'left' in channel else 'right'
        joints = list(self.converted[side]['joint_mdeg'])
        if self.raw_drift:
            joints[1] += 5000
        now = time.time()
        return dict(ctrl_mode=1, arm_status=0, err_code=0, gripper_mode=0,
                    driver_enabled=[True] * 5 + [not self.raw_fault], driver_foc_bytes=[0x40] * 6,
                    joint_mdeg=joints, gripper_um=self.converted[side]['gripper_um'],
                    timestamp_sec=now, component_age_s=dict.fromkeys(COMPONENTS, 0.0))

    def publish(self):
        self.published.append((time.monotonic(), {side: list(goal) for side, goal in TARGETS.items()}))
        if self.cancel_after is not None:
            state = self.node.parallel_following_preparation
            if time.monotonic() - state['started_monotonic'] >= self.cancel_after:
                state['cancel'].set()

    def hold(self, *, timeout=None, hold_seconds=3.0):
        if timeout is not None:
            original = self.node._start_following_preparation
            self.node._start_following_preparation = lambda started_at, **kwargs: original(started_at, timeout=timeout)
        started = time.monotonic()
        passed = self.node._hold_final_pose_and_prepare(TARGETS, self.publish, hold_seconds, started, 'slave pose')
        return passed, started, time.monotonic() - started

    def assert_no_release(self):
        self.assertFalse(any(event[0] == 'release_master_gripper' for event in self.events))
        with self.assertRaises(RuntimeError):
            self.node.release_prepared_masters()
        self.assertFalse(any(event[0] == 'release_master_gripper' for event in self.events))

    def test_point_two_second_preparation_runs_during_three_second_hold(self):
        passed, started, elapsed = self.hold()
        self.assertTrue(passed)
        self.assertGreaterEqual(elapsed, 3.0)
        self.assertLess(elapsed, 3.35)
        self.assertLess(min(value[0] for value in self.requested.values()) - started, 0.1)
        state = self.node.parallel_following_preparation
        self.assertLess(state['prepared_at'] - started, 0.5)
        self.node.release_prepared_masters()
        self.assertEqual([event[2] for event in self.events if event[0] == 'release_master_gripper'], ['left', 'right'])
        self.assertGreater(len(self.published), 100)
        self.assertTrue(all(goal == TARGETS for _stamp, goal in self.published))

    def test_one_slow_side_four_point_two_seconds_does_not_add_three_more(self):
        self.prep_delay['right'] = 4.2
        passed, _started, elapsed = self.hold()
        self.assertTrue(passed)
        self.assertGreaterEqual(elapsed, 4.2)
        self.assertLess(elapsed, 4.6)
        self.node.release_prepared_masters()
        self.assertEqual(sum(event[0] == 'release_master_gripper' for event in self.events), 2)

    def test_worker_errors_abort_hold_without_prepare_or_release(self):
        for target in ('preflight_error', 'probe_error'):
            with self.subTest(target=target):
                setattr(self, target, RuntimeError(target))
                passed, _started, elapsed = self.hold(timeout=1)
                self.assertFalse(passed)
                self.assertLess(elapsed, 0.3)
                self.assert_no_release()
                self.assertFalse(self.requested)
                setattr(self, target, None)

    def test_disabled_feedback_or_continuous_drift_cannot_satisfy_hold(self):
        self.raw_fault = True
        self.assertFalse(self.hold(timeout=0.6)[0])
        self.assert_no_release()
        self.raw_fault = False
        self.raw_drift = True
        self.assertFalse(self.hold(timeout=0.6)[0])
        self.assert_no_release()

    def test_unprepared_single_side_times_out_without_release(self):
        self.prep_delay['right'] = None
        passed, _started, elapsed = self.hold(timeout=0.6)
        self.assertFalse(passed)
        self.assertLess(elapsed, 0.7)
        self.assert_no_release()

    def test_cancellation_stops_probe_before_it_can_publish_prepare(self):
        self.probe_blocked = True
        self.cancel_after = 0.05
        self.assertFalse(self.hold(timeout=1)[0])
        self.assertTrue(self.probe_entered.is_set())
        self.assertFalse(self.requested)
        self.assert_no_release()

    def test_temporary_unprepared_status_waits_while_continuing_the_same_hold(self):
        self.prep_delay = {'left': 0.01, 'right': 0.01}
        self.temporary_not_ready = (0.05, 0.3)
        passed, _started, elapsed = self.hold(timeout=1.0, hold_seconds=0.1)
        self.assertTrue(passed)
        self.assertGreaterEqual(elapsed, 0.3)
        self.assertLess(elapsed, 0.55)
        self.node.release_prepared_masters()

    def test_explicit_native_failure_aborts_instead_of_waiting_for_recovery(self):
        self.native_failure_at = 0.05
        passed, _started, elapsed = self.hold(timeout=1.0, hold_seconds=0.3)
        self.assertFalse(passed)
        self.assertLess(elapsed, 0.2)
        self.assert_no_release()

    def test_keyboard_interrupt_during_hold_cancels_worker_and_never_releases(self):
        self.probe_blocked = True
        self.publish = lambda: (_ for _ in ()).throw(KeyboardInterrupt())
        with self.assertRaises(KeyboardInterrupt):
            self.hold(timeout=1)
        self.assert_no_release()


if __name__ == '__main__':
    unittest.main()
