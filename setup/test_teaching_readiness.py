"""Pure heartbeat and extracted-driver tests; no ROS, SDK or CAN is started."""
import ast
from copy import deepcopy
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).parent))
from prepare_follower_mode import REQUIRED_IDS
from prepare_follower_enabled import checked_enabled_feedback
import teaching_readiness as module

ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / 'ros_ws/src/piper/scripts/piper_start_ms_node_new.py'


def raw(now=1000.0, pose=None):
    return {'ctrl_mode': 1, 'arm_status': 0, 'err_code': 0, 'gripper_mode': 0,
            'driver_enabled': [True] * 6, 'driver_foc_bytes': [0x40] * 6,
            'joint_mdeg': list(pose or [1000, 20000, -30000, 4000, 5000, 6000]),
            'timestamp_sec': now,
            'component_age_s': {f'0x{key:03X}': 0.01 for key in REQUIRED_IDS}}


def hearts(wall=1000.0, mono=10.0, command=999.9):
    return {side: {'ready': True, 'source_time': wall, 'received_monotonic': mono,
                   'command_time': command} for side in ('left', 'right')}


def twist():
    return SimpleNamespace(linear=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                           angular=SimpleNamespace(x=0.0, y=0.0, z=0.0))


class PredicateTests(unittest.TestCase):
    def follower(self, row=None, **patch):
        values = dict(now_wall=1000.0, now_monotonic=10.0, loop_monotonic=9.99,
                      enabled_flag=True, subscriber_ready=True, command_time=999.9)
        values.update(patch)
        return module.follower_readiness(raw() if row is None else row, **values)

    def test_enabled_can_raw_live_loop_and_callback_make_ready(self):
        self.assertTrue(self.follower()['ready'])

    def test_stopped_loop_disabled_callback_and_missing_subscription_block(self):
        for patch in ({'loop_monotonic': 9.89}, {'enabled_flag': False},
                      {'subscriber_ready': False}, {'loop_monotonic': float('nan')}):
            with self.subTest(patch=patch):
                self.assertFalse(self.follower(**patch)['ready'])

    def test_single_stale_raw_component_is_not_hidden_by_fresh_aggregate(self):
        row = raw()
        row['component_age_s']['0x264'] = 0.21
        self.assertFalse(self.follower(row)['ready'])

    def test_raw_age_is_recomputed_at_heartbeat_publication(self):
        # This sample was healthy at 999.8; publication at 1000.0 makes the
        # individual component 0.21s old even though its recorded age is 0.01.
        self.assertFalse(self.follower(raw(now=999.8))['ready'])

    def test_nonadvancing_raw_components_cannot_refresh_readiness(self):
        result = self.follower()
        self.assertFalse(self.follower(previous_raw_stamps=result['raw_stamps'])['ready'])

    def test_fault_wrong_mode_and_partial_enable_block(self):
        for patch in ({'ctrl_mode': 2}, {'err_code': 1}, {'gripper_mode': 1},
                      {'driver_enabled': [True] * 5 + [False]}):
            with self.subTest(patch=patch):
                row = raw()
                row.update(patch)
                self.assertFalse(self.follower(row)['ready'])

    def test_invalid_future_and_expired_executed_source_timestamp_block(self):
        for stamp in (0, float('nan'), 1000.06, 989.9):
            with self.subTest(stamp=stamp):
                self.assertFalse(self.follower(command_time=stamp)['ready'])

    def test_both_new_heartbeats_and_probe_source_acks_pass(self):
        self.assertTrue(module.require_pair_ready(
            hearts(command=1000.0), now_wall=1000.0, now_monotonic=10.0,
            minimum_command_time=1000.0))

    def test_missing_false_or_single_side_blocks_pair(self):
        for rows in ({}, {'left': hearts()['left']},
                     {**hearts(), 'right': {**hearts()['right'], 'ready': False}}):
            with self.subTest(rows=rows), self.assertRaises(RuntimeError):
                module.require_pair_ready(rows, now_wall=1000.0, now_monotonic=10.0)

    def test_source_and_receipt_age_are_independently_required(self):
        for patch in ({'source_time': 999.64}, {'received_monotonic': 9.64},
                      {'source_time': 1000.06}, {'received_monotonic': 10.06}):
            rows = hearts()
            rows['right'].update(patch)
            with self.subTest(patch=patch), self.assertRaises(RuntimeError):
                module.require_pair_ready(rows, now_wall=1000.0, now_monotonic=10.0)

    def test_old_queued_source_cannot_ack_probe_even_if_completed_after_probe(self):
        with self.assertRaisesRegex(RuntimeError, 'pre-probe'):
            module.require_pair_ready(hearts(command=999.99), now_wall=1000.0,
                                      now_monotonic=10.0, minimum_command_time=1000.0)

    def test_old_missing_and_future_command_acks_block(self):
        for command in (0, 989.9, 1000.06, float('nan')):
            with self.subTest(command=command), self.assertRaises(RuntimeError):
                module.require_pair_ready(hearts(command=command), now_wall=1000.0, now_monotonic=10.0)


class DriverTests(unittest.TestCase):
    def setUp(self):
        self.mono = 10.0
        self.events = []
        self.on_sleep = lambda _seconds: self.refresh_hearts()
        self.shutdown = lambda: False
        self.fail_sdk = None
        self.raw_failure = False
        self.raw_fault = False
        self.raw_calls = 0
        methods = {'GetEnableFlag', 'follower_ready_callback', 'FollowerReadinessThread',
                   '_hold_last_alignment_pose', '_return_to_alignment_hold',
                   '_wait_for_follower_readiness', 'linkage_config_callback',
                   '_set_linkage_config', '_init_master_teach_mode', '_init_slave_follow_mode',
                   'teach_mode_callback', '_set_teach_mode', 'joint_callback', '_apply_joint_command',
                   'SubJointThread', 'master_enable_callback', '_set_master_enable',
                   '_raise_if_manual_disabled', '_mark_manually_disabled_state', '_checked_sdk_write'}
        methods.update({'_stop_native_forwarding', '_mark_native_mode_unknown',
                        '_return_to_current_native_hold', '_native_stage',
                        '_alignment_command_key', '_invalidate_native_preparation'})
        tree = ast.parse(DRIVER.read_text())
        cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'C_PiperRosNode')
        cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name in methods]
        namespace = dict(
            math=math, threading=threading, Twist=twist,
            time=SimpleNamespace(time=self.wall, monotonic=lambda: self.mono, sleep=self.sleep),
            rospy=SimpleNamespace(is_shutdown=lambda: self.shutdown(),
                                  loginfo=lambda *_: None, logwarn=lambda *_: None,
                                  logerr=lambda *args: self.events.append(('error', args))),
            follower_readiness=module.follower_readiness,
            require_pair_ready=module.require_pair_ready,
            checked_enabled_feedback=checked_enabled_feedback,
            sample_feedback=self.sample,
        )
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(DRIVER), 'exec'), namespace)
        self.namespace = namespace
        self.node = namespace['C_PiperRosNode'].__new__(namespace['C_PiperRosNode'])
        node = self.node
        node.mode = 0
        node.can_port = 'can_left_mas'
        node.require_follower_ready = True
        node.follower_ready_timeout_s = 0.2
        node._readiness_lock = threading.Lock()
        node._mode_transition_lock = threading.RLock()
        node._torque_write_lock = threading.Lock()
        node._manual_disable_requested = threading.Event()
        node._follower_ready_rows = {}
        node._last_alignment_target = (1000, 20000, -30000, 4000, 5000, 6000)
        node._last_alignment_command_gripper_um = 40000
        node._native_forward_lock = threading.Lock()
        node._native_forward_phase = None
        node._last_joint_command_wall = self.wall() - 0.1
        node._last_joint_command_valid = True
        node._last_publish_loop_monotonic = self.mono
        node._joint_subscription_ready = True
        node._transition_in_progress = False
        node._master_publish_enabled = False
        node.native_handoff_report = None
        node.is_enabled = True
        node._C_PiperRosNode__enable_flag = True
        node.in_teach_mode = False
        node.current_linkage_config = 0xFC
        node.new_config_ = 0xFC
        node._publish_mode_status = lambda: self.events.append(('ack', node.current_linkage_config, node.in_teach_mode))

        class FakeSDK:
            def __getattr__(_self, method):
                def invoke(*args, **kwargs):
                    self.events.append((method, args, kwargs, self.wall(), node._master_publish_enabled))
                    if self.fail_sdk == method:
                        raise RuntimeError('simulated SDK failure')
                return invoke
        node.piper = FakeSDK()

    def wall(self):
        return 1000.0 + self.mono - 10.0

    def sleep(self, seconds):
        self.mono += seconds
        self.on_sleep(seconds)

    def refresh_hearts(self):
        self.node._follower_ready_rows = hearts(self.wall(), self.mono, command=1000.0)

    def sample(self, _channel, since_timestamp=0.0):
        self.raw_calls += 1
        self.mono += 0.12
        if self.raw_failure:
            raise RuntimeError('Missing required CAN feedback frames')
        result = raw(now=self.wall(), pose=self.node._last_alignment_target)
        if self.raw_fault:
            result['err_code'] = 1
        return result

    def commands(self, method):
        return [event for event in self.events if event[0] == method]

    def assert_no_release(self):
        self.assertFalse(self.node._master_publish_enabled)
        self.assertFalse(self.node.in_teach_mode)
        self.assertFalse(any(event[2].get('grag_teach_ctrl') == 1 for event in self.commands('MotionCtrl_1')))
        self.assertFalse(any(event[0] == 'ack' and event[1] == 0xFA for event in self.events))

    def test_managed_configuration_dispatches_to_native_entry_without_legacy_writes(self):
        calls = []
        self.node._enter_native_master_mode = lambda: calls.append('native-entry')
        self.node.linkage_config_callback(SimpleNamespace(data='master'))
        self.assertEqual(calls, ['native-entry'])
        self.assertFalse(self.commands('MasterSlaveConfig'))
        self.assertFalse(self.commands('JointCtrl') or self.commands('EnableArm'))
        self.assertEqual(self.node.current_linkage_config, 0xFC)
        self.assert_no_release()
        self.assertFalse(self.commands('DisableArm') or self.commands('ResetPiper'))

    def test_without_trusted_alignment_target_no_fa_no_default_zero_hold(self):
        self.refresh_hearts()
        self.node._last_alignment_target = None
        with self.assertRaises(RuntimeError):
            self.node._wait_for_follower_readiness()
        self.assertFalse(self.commands('MasterSlaveConfig') or self.commands('JointCtrl'))
        self.assert_no_release()

    def test_generic_launch_preserves_legacy_six_second_teach_sequence(self):
        self.node.require_follower_ready = False
        self.refresh_hearts()
        self.node.linkage_config_callback(SimpleNamespace(data='master'))
        release = [event for event in self.commands('MotionCtrl_1') if event[2].get('grag_teach_ctrl') == 1]
        self.assertEqual(len(release), 1)
        self.assertGreaterEqual(release[0][3], 1006.0)
        self.assertFalse(release[0][4])
        self.assertTrue(self.node._master_publish_enabled)
        self.assertTrue(self.node.in_teach_mode)
        self.assertEqual(self.node.current_linkage_config, 0xFA)
        self.assertIn(('ack', 0xFA, True), self.events)

    def test_managed_fc_recovery_verifies_fresh_native_pose_without_torque_toggle(self):
        self.node.current_linkage_config = 0xFA
        self.node._return_to_alignment_hold()
        configs = [event[2]['linkage_config'] for event in self.commands('MasterSlaveConfig')]
        self.assertEqual(configs, [0xFC])
        self.assertGreater(self.raw_calls, 0)
        self.assertEqual(self.node.current_linkage_config, 0xFC)
        self.assertEqual(self.node.new_config_, 0xFC)
        self.assert_no_release()
        self.assertFalse(self.commands('DisableArm') or self.commands('ResetPiper'))
        fc_index = next(i for i, event in enumerate(self.events)
                        if event[0] == 'MasterSlaveConfig' and event[2]['linkage_config'] == 0xFC)
        self.assertFalse(any(event[0] in ('EnableArm', 'DisableArm', 'ResetPiper')
                             for event in self.events[fc_index:]))

    def test_failed_fc_rollback_latches_unknown_state_instead_of_false_fc_ack(self):
        self.raw_failure = True
        with self.assertRaises(RuntimeError):
            self.node._return_to_alignment_hold()
        self.assertEqual(self.node.current_linkage_config, 0)
        self.assertIsNone(self.node.new_config_)
        self.assert_no_release()
        self.assertFalse(any(event[0] == 'ack' for event in self.events))

    def test_fc_rollback_fault_does_not_append_blind_sdk_control_writes(self):
        self.raw_fault = True
        with self.assertRaises(RuntimeError):
            self.node._return_to_alignment_hold()
        fc_index = next(i for i, event in enumerate(self.events)
                        if event[0] == 'MasterSlaveConfig' and event[2]['linkage_config'] == 0xFC)
        controls = [event[0] for event in self.events[fc_index:] if event[0] != 'error']
        self.assertEqual(controls, ['MasterSlaveConfig'])
        self.assertEqual(self.raw_calls, 1)
        self.assertEqual(self.node.current_linkage_config, 0)
        self.assert_no_release()

    def test_direct_teach_request_cannot_bypass_missing_ready_or_fc_configuration(self):
        self.node.teach_mode_callback(SimpleNamespace(data=1))
        self.assert_no_release()
        self.assertFalse(self.commands('MasterSlaveConfig'))

    def test_managed_direct_teach_helper_cannot_enable_or_bypass_native_handoff(self):
        self.node._follower_ready_rows = {'left': hearts()['left']}
        self.on_sleep = lambda _seconds: None
        with self.assertRaises(RuntimeError):
            self.node._init_master_teach_mode()
        self.assertFalse(self.commands('EnableArm'))
        self.assert_no_release()

    def test_alignment_commands_are_not_replayed_after_teaching_has_started(self):
        self.node.in_teach_mode = True
        self.node.joint_callback(SimpleNamespace(position=[0.0] * 7))
        self.assertEqual(self.events, [])

    def command(self, source=None):
        source = self.wall() if source is None else source
        return SimpleNamespace(position=[0, .3, -.4, 0, .2, 0, .04],
                               header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: source)))

    def test_ack_is_executed_source_stamp_instead_of_completion_time(self):
        self.node.mode = 1
        message = self.command(source=999.8)
        self.node.joint_callback(message)
        self.assertEqual(self.node._last_joint_command_wall, 999.8)
        self.assertTrue(self.node._last_joint_command_valid)
        self.assertEqual(len(self.commands('MotionCtrl_2')), 2)

    def test_failed_sdk_write_does_not_refresh_ack_and_marks_not_ready(self):
        self.node.mode = 1
        old = self.node._last_joint_command_wall
        self.fail_sdk = 'GripperCtrl'
        with self.assertRaises(RuntimeError):
            self.node.joint_callback(self.command())
        self.assertEqual(self.node._last_joint_command_wall, old)
        self.assertFalse(self.node._last_joint_command_valid)

    def test_successful_inflight_callback_keeps_last_completed_valid_ack_until_completion(self):
        self.node.mode = 1
        previous = self.node._last_joint_command_wall
        observed = []
        def gripper(*_args):
            observed.append((self.node._last_joint_command_valid, self.node._last_joint_command_wall))
        self.node.piper.GripperCtrl = gripper
        self.node.joint_callback(self.command(source=1000.0))
        self.assertEqual(observed, [(True, previous)])
        self.assertTrue(self.node._last_joint_command_valid)
        self.assertEqual(self.node._last_joint_command_wall, 1000.0)

    def test_zero_invalid_and_future_command_sources_do_not_ack_or_write(self):
        self.node.mode = 1
        for stamp in (0.0, float('nan'), self.wall() + 0.06):
            with self.subTest(stamp=stamp), self.assertRaises(RuntimeError):
                self.node.joint_callback(self.command(source=stamp))
            self.assertFalse(self.node._last_joint_command_valid)
        self.assertFalse(self.commands('JointCtrl'))

    def test_heartbeat_receipt_callback_uses_independent_lock_during_mode_transition(self):
        self.node._mode_transition_lock.acquire()
        done = threading.Event()
        message = twist()
        message.linear.x, message.linear.y, message.angular.x = 1.0, 1000.0, 1000.0
        worker = threading.Thread(target=lambda: (self.node.follower_ready_callback(message, 'left'), done.set()))
        try:
            worker.start()
            self.assertTrue(done.wait(0.5), 'heartbeat callback blocked behind transition lock')
        finally:
            self.node._mode_transition_lock.release()
            worker.join(timeout=1)
        self.assertEqual(self.node._follower_ready_rows['left']['command_time'], 1000.0)

    def test_generic_direct_teach_callback_cannot_overtake_config_transition(self):
        self.node.require_follower_ready = False
        self.refresh_hearts()
        attempted = threading.Event()
        finished = threading.Event()
        workers = []
        def request():
            attempted.set()
            self.node.teach_mode_callback(SimpleNamespace(data=1))
            finished.set()
        def during_sleep(_seconds):
            self.refresh_hearts()
            if not workers:
                worker = threading.Thread(target=request)
                workers.append(worker)
                worker.start()
                self.assertTrue(attempted.wait(0.5))
                self.assertFalse(finished.wait(0.05), 'teach callback bypassed the configuration lock')
                self.assertFalse(self.node._master_publish_enabled)
        self.on_sleep = during_sleep
        self.node.linkage_config_callback(SimpleNamespace(data='master'))
        workers[0].join(timeout=1)
        self.assertTrue(finished.is_set())
        releases = [event for event in self.commands('MotionCtrl_1') if event[2].get('grag_teach_ctrl') == 1]
        self.assertEqual(len(releases), 1)
        self.assertGreaterEqual(releases[0][3], 1006.0)

    def test_follower_monitor_is_passive_and_uses_the_live_loop_and_sample_publication_time(self):
        self.node.mode = 1
        published = []
        self.node._follower_ready_publisher = SimpleNamespace(publish=published.append)
        self.shutdown = lambda: bool(published)
        self.node._last_publish_loop_monotonic = 9.0  # Raw feedback alone is insufficient.
        self.node.FollowerReadinessThread()
        self.assertEqual(len(published), 1)
        self.assertEqual(published[0].linear.x, 0.0)
        self.assertAlmostEqual(published[0].angular.x, 1000.12)
        self.assertEqual(published[0].linear.y, 999.9)
        self.assertFalse(self.commands('JointCtrl') or self.commands('EnableArm') or self.commands('MotionCtrl_2'))

    def test_launch_turns_interlock_on_for_all_four_nodes_only_in_deployment(self):
        generic = ET.parse(ROOT / 'ros_ws/src/piper/launch/start_ms_piper_new.launch').getroot()
        self.assertEqual(next(arg for arg in generic.findall('arg')
                              if arg.attrib['name'] == 'require_follower_ready').attrib['default'], 'false')
        nodes = generic.findall('node')
        self.assertEqual(len(nodes), 4)
        for node in nodes:
            param = next(param for param in node.findall('param') if param.attrib['name'] == 'require_follower_ready')
            self.assertEqual(param.attrib['value'], '$(arg require_follower_ready)')
        deployment = ET.parse(ROOT / 'ros-launch/arms_official_wj_dev.launch').getroot()
        arg = next(arg for arg in deployment.find('include').findall('arg')
                   if arg.attrib['name'] == 'require_follower_ready')
        self.assertEqual(arg.attrib['value'], 'true')

    def test_explicit_disable_does_not_wait_for_mode_transition_lock(self):
        finished = threading.Event()
        self.node._mode_transition_lock.acquire()
        worker = threading.Thread(target=lambda: (self.node.master_enable_callback(SimpleNamespace(data=False)),
                                                   finished.set()))
        try:
            worker.start()
            self.assertTrue(finished.wait(0.5), 'manual disable waited for the mode lock')
        finally:
            self.node._mode_transition_lock.release()
            worker.join(timeout=1)
        self.assertTrue(self.node._manual_disable_requested.is_set())
        self.assertEqual(len(self.commands('DisableArm')), 1)
        self.assertFalse(self.node.GetEnableFlag())
        self.assertFalse(self.node.is_enabled)
        self.assert_no_release()

    def test_disable_during_generic_transition_cancels_later_automatic_writes(self):
        self.node.require_follower_ready = False
        self.refresh_hearts()
        sleeps = []
        def cancel_during_last_wait(seconds):
            sleeps.append(seconds)
            self.refresh_hearts()
            if len(sleeps) == 3:
                self.node.master_enable_callback(SimpleNamespace(data=False))
        self.on_sleep = cancel_during_last_wait
        self.node.linkage_config_callback(SimpleNamespace(data='master'))
        disable_index = next(i for i, event in enumerate(self.events) if event[0] == 'DisableArm')
        self.assertEqual(sum(sleeps), 6.0)
        self.assertFalse(any(event[0] not in ('error',) for event in self.events[disable_index + 1:]), self.events)
        self.assertEqual(self.node.current_linkage_config, 0)
        self.assertFalse(self.node.is_enabled)
        self.assert_no_release()

    def test_queued_true_cannot_clear_manual_disable_latch(self):
        attempted = threading.Event()
        finished = threading.Event()
        self.node._mode_transition_lock.acquire()
        def queued_enable():
            attempted.set()
            self.node.master_enable_callback(SimpleNamespace(data=True))
            finished.set()
        worker = threading.Thread(target=queued_enable)
        try:
            worker.start()
            self.assertTrue(attempted.wait(0.5))
            self.node.master_enable_callback(SimpleNamespace(data=False))
        finally:
            self.node._mode_transition_lock.release()
            worker.join(timeout=1)
        self.assertTrue(finished.is_set())
        self.assertTrue(self.node._manual_disable_requested.is_set())
        self.assertFalse(self.commands('EnableArm'))
        self.assertFalse(self.node.GetEnableFlag())
        self.assert_no_release()

    def test_disable_waits_only_for_one_inflight_sdk_write_and_prevents_later_writes(self):
        started = threading.Event()
        finish_write = threading.Event()
        disable_done = threading.Event()
        cancelled_writes = []
        def enable(_motor):
            self.events.append(('EnableArm', (), {}, self.wall(), False))
            started.set()
            if not finish_write.wait(1):
                raise RuntimeError('test SDK write timed out')
        self.node.piper.EnableArm = enable
        def automatic_write():
            try:
                self.node._checked_sdk_write('EnableArm', 7)
                self.node._checked_sdk_write('MotionCtrl_1', grag_teach_ctrl=1)
            except RuntimeError as exc:
                cancelled_writes.append(str(exc))
        writer = threading.Thread(target=automatic_write)
        stopper = threading.Thread(target=lambda: (self.node.master_enable_callback(SimpleNamespace(data=False)),
                                                    disable_done.set()))
        writer.start()
        try:
            self.assertTrue(started.wait(0.5))
            stopper.start()
            self.assertTrue(self.node._manual_disable_requested.wait(0.5))
            self.assertFalse(disable_done.is_set())
            finish_write.set()
            writer.join(timeout=1)
            stopper.join(timeout=1)
        finally:
            finish_write.set()
            writer.join(timeout=1)
            if stopper.ident is not None:
                stopper.join(timeout=1)
        self.assertTrue(disable_done.is_set())
        self.assertTrue(cancelled_writes)
        self.assertEqual([event[0] for event in self.events], ['EnableArm', 'DisableArm'])
        self.assert_no_release()


if __name__ == '__main__':
    unittest.main()
