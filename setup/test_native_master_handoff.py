"""Exercise physical FA release, native forwarding and rollback without hardware."""
import ast
from copy import deepcopy
import math
from pathlib import Path
import struct
import threading
from types import SimpleNamespace
import unittest

from master_native_forwarding import NativeMasterCache, ACTUAL_IDS, FA_IDS
from prepare_follower_enabled import checked_enabled_feedback
from prepare_follower_mode import REQUIRED_IDS
from teaching_readiness import require_pair_ready


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'ros_ws/src/piper/scripts/piper_start_ms_node_new.py'


def joint_state():
    return SimpleNamespace(header=SimpleNamespace(stamp=None), position=[], name=[])


class NativeHandoffTests(unittest.TestCase):
    def setUp(self):
        self.mono = 10.0
        self.events = []
        self.shutdown = False
        self.host_tx_only = False
        self.missing_fa = False
        self.missing_fa_id = None
        self.fa_delay_s = 0.0
        self.fa_stop_after_s = None
        self.fa_resume_after_s = None
        self.fa_started_at = None
        self.fa_ack_delay_s = 0.0
        self.fa_delayed_ack_side = 'right'
        self.reader_failure_after_fa_s = None
        self.cancel_after_fa_s = None
        self.shutdown_after_fa_s = None
        self.sdk_error_method = None
        self.reader_failure = False
        self.replay_old_epoch = False
        self.cached_actual = None
        self.ack_mode = 'normal'
        self.first_jump = False
        self.fa_joint_delta_mdeg = 300
        self.actual_fault = False
        self.actual_disabled = False
        self.actual_jump = False
        self.actual_stopped = False
        self.rollback_failure = False
        self.recovery_drift_mdeg = 0
        self.actual_gripper = 40000
        self.fa_gripper = 40000
        self.cancel_priming = False
        self.cancel_fa = False
        self.cancelled_once = False
        self.workers = []
        self.physical_released = False
        self.left_ack = 999.9
        self.target = [1000, 20000, -30000, 4000, 5000, 6000]
        self.current_pose = [21000, 35000, -44000, 9000, 14000, 12000]
        self.raw_mode = 0
        self.phase = None
        self.epoch = 0
        self.cache = NativeMasterCache(wall=self.wall)
        tree = ast.parse(SOURCE.read_text())
        cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'C_PiperRosNode')
        names = {'_native_stage', '_stop_native_forwarding', '_mark_native_mode_unknown',
                 '_arm_native_forwarding', '_native_forward_failed', '_native_forward_tick',
                 '_native_wait_for_data', '_publish_mode_status',
                 '_wait_native_first', '_wait_native_pair_ack', '_enter_native_master_mode',
                 '_return_to_current_native_hold', '_hold_last_alignment_pose',
                 '_return_to_alignment_hold',
                 'linkage_config_callback', '_set_linkage_config', '_raise_if_manual_disabled',
                 '_mark_manually_disabled_state', '_checked_sdk_write', 'master_enable_callback',
                 '_set_master_enable', 'GetEnableFlag', 'follower_ready_callback'}
        names.add('PublishMasterArmJointAndGripper')
        names.update({'_alignment_command_key', '_invalidate_native_preparation',
                      '_native_prepared_since', '_native_reader_diagnostics', '_native_handoff_error',
                      '_native_require_recent_pair',
                      '_prepare_native_master_mode', '_release_native_master_mode',
                      'joint_callback', '_apply_joint_command', '_publish_master_status'})
        names.add('_record_gripper_handoff')
        cls.body = [node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name in names]
        namespace = dict(math=math, threading=threading, JointState=joint_state,
                         String=lambda: SimpleNamespace(data=''),
                         time=SimpleNamespace(time=self.wall, monotonic=lambda: self.mono, sleep=self.sleep),
                         rospy=SimpleNamespace(is_shutdown=lambda: self.shutdown,
                                               Time=SimpleNamespace(from_sec=lambda source: source,
                                                                    now=lambda: SimpleNamespace(to_sec=self.wall)),
                                               loginfo=lambda *args: None, logwarn=lambda *args: None,
                                               logerr=lambda *args: self.events.append(('error', args))),
                         checked_enabled_feedback=checked_enabled_feedback,
                         Twist=lambda: SimpleNamespace(linear=SimpleNamespace(x=0,y=0,z=0),
                                                        angular=SimpleNamespace(x=0,y=0,z=0)),
                         require_pair_ready=require_pair_ready, sample_feedback=self.sample)
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(SOURCE), 'exec'), namespace)
        self.node = namespace['C_PiperRosNode']()
        node = self.node
        node.mode = 0
        node.can_port = 'can_left_mas'
        node.require_follower_ready = True
        node.follower_ready_timeout_s = 0.1
        node._readiness_lock = threading.Lock()
        node._mode_transition_lock = threading.RLock()
        node._torque_write_lock = threading.Lock()
        node._manual_disable_requested = threading.Event()
        node._native_forward_lock = threading.Lock()
        node._native_forward_phase = None
        node._native_last_actual = None
        node.native_handoff_report = None
        node._follower_ready_rows = {}
        node._last_alignment_target = tuple(self.target)
        node._last_alignment_command_gripper_um = 40000
        node._last_joint_command_valid = True
        node._transition_in_progress = False
        node._master_publish_enabled = False
        node.current_linkage_config = 0xFC
        node.new_config_ = 0xFC
        node.is_enabled = True
        node.in_teach_mode = False
        node._C_PiperRosNode__enable_flag = True
        node.joint_std_pub_master = SimpleNamespace(get_num_connections=lambda: 2, publish=self.publish)
        node._publish_mode_status = lambda: self.events.append(('mode-ack', node.current_linkage_config))
        node.master_status_pub = SimpleNamespace(publish=lambda message: self.events.append(('status', message)))
        node.mode_status_pub = SimpleNamespace(publish=lambda message: self.events.append(('mode-string', message.data)))
        node._native_master_reader = SimpleNamespace(wait_started=lambda timeout: True,
                                                    begin_phase=self.begin, snapshot=self.snapshot,
                                                    diagnostics=lambda after=None: self.cache.diagnostics(after))

        class SDK:
            def __getattr__(_self, method):
                return lambda *args, **kwargs: self.sdk(method, args, kwargs)
        node.piper = SDK()
        self.refresh_hearts()
        self.addCleanup(self.join_workers)

    def join_workers(self):
        for worker in self.workers:
            worker.join(timeout=1)
            self.assertFalse(worker.is_alive())

    def wall(self):
        return 1000 + self.mono - 10

    def begin(self, kind, since_timestamp):
        self.phase = kind
        if kind == 'fa':
            self.fa_started_at = self.mono
        self.epoch = self.cache.begin_phase(kind, since_timestamp)
        self.events.append(('armed', kind, since_timestamp))
        return self.epoch

    def snapshot(self, after=None):
        if (self.reader_failure or self.phase == 'fa' and self.reader_failure_after_fa_s is not None
                and self.mono - self.fa_started_at >= self.reader_failure_after_fa_s):
            raise RuntimeError('listener stopped')
        if self.phase == 'fa' and self.replay_old_epoch:
            return deepcopy(self.cached_actual)
        result = self.cache.snapshot(after=after)
        if result is not None and result['kind'] == 'actual':
            self.cached_actual = deepcopy(result)
        return result

    def frames(self):
        if self.phase is None or self.phase == 'fa' and self.missing_fa:
            return
        if self.phase == 'fa' and self.mono - self.fa_started_at < self.fa_delay_s:
            return
        if (self.phase == 'fa' and self.fa_stop_after_s is not None
                and self.mono - self.fa_started_at >= self.fa_stop_after_s
                and (self.fa_resume_after_s is None
                     or self.mono - self.fa_started_at < self.fa_resume_after_s)):
            return
        if self.phase == 'actual':
            if self.actual_stopped:
                return
            payloads = {key: bytes(8) for key in ACTUAL_IDS}
            payloads[0x2A1] = bytes([1, 0, 0, 0, 0, 0, 0, 0])
            if self.actual_fault:
                payloads[0x2A1] = bytes([1, 1, 0, 0, 0, 0, 0, 0])
            pose = list(self.target)
            if self.actual_jump:
                pose[0] += 2000
            for index, key in enumerate((0x2A5, 0x2A6, 0x2A7)):
                payloads[key] = struct.pack('>ii', *pose[index * 2:index * 2 + 2])
            payloads[0x2A8] = struct.pack('>i', self.actual_gripper) + bytes(4)
            for key in range(0x261, 0x267):
                payloads[key] = bytes([0, 0, 0, 0, 0, 0x40, 0, 0])
            if self.actual_disabled:
                payloads[0x264] = bytes(8)
        else:
            pose = list(self.target)
            pose[0] += 3000 if self.first_jump else self.fa_joint_delta_mdeg
            payloads = {key: struct.pack('>ii', *pose[index * 2:index * 2 + 2])
                        for index, key in enumerate(FA_IDS[:3])}
            payloads[0x159] = struct.pack('>i', self.fa_gripper) + bytes(4)
        for key, data in payloads.items():
            if self.phase == 'fa' and key == self.missing_fa_id:
                continue
            self.cache.ingest(SimpleNamespace(is_rx=not self.host_tx_only, is_error_frame=False,
                                             arbitration_id=key, data=data, dlc=8,
                                             timestamp=self.wall() - 0.001))

    def refresh_hearts(self):
        command = self.left_ack
        if self.ack_mode == 'old':
            command = 999.9
        if self.ack_mode == 'stalled':
            command = self.frozen_ack
        if self.ack_mode == 'fa-old' and self.physical_released:
            command = 999.9
        self.node._follower_ready_rows = {
            side: dict(ready=True, source_time=self.wall(), received_monotonic=self.mono,
                       command_time=command if side == 'left' else self.wall() - 0.001)
            for side in ('left', 'right')}
        if (self.phase == 'fa' and self.physical_released
                and self.mono - self.fa_started_at < self.fa_ack_delay_s):
            self.node._follower_ready_rows[self.fa_delayed_ack_side]['command_time'] = (
                self.cache.diagnostics()['since_timestamp'] - 0.01)
        if self.ack_mode == 'single':
            self.node._follower_ready_rows.pop('right')

    def sleep(self, seconds):
        self.mono += seconds
        if self.cancel_priming and not self.cancelled_once:
            self.cancelled_once = True
            self.node.master_enable_callback(SimpleNamespace(data=False))
        if (self.phase == 'fa' and self.cancel_after_fa_s is not None and not self.cancelled_once
                and self.mono - self.fa_started_at >= self.cancel_after_fa_s):
            self.cancelled_once = True
            self.node.master_enable_callback(SimpleNamespace(data=False))
        if (self.phase == 'fa' and self.shutdown_after_fa_s is not None
                and self.mono - self.fa_started_at >= self.shutdown_after_fa_s):
            self.shutdown = True
            return
        self.frames()
        self.node._native_forward_tick()
        self.refresh_hearts()

    def publish(self, message):
        self.events.append(('publish', self.phase, message.header.stamp, list(message.position),
                            self.node._transition_in_progress, self.node.in_teach_mode))
        self.left_ack = message.header.stamp

    def sdk(self, method, args, kwargs):
        self.events.append((method, args, kwargs, self.wall()))
        if self.sdk_error_method == method:
            raise RuntimeError('simulated command write failed')
        if method == 'MasterSlaveConfig' and kwargs['linkage_config'] == 0xFA:
            # Model the actual firmware boundary: FA itself releases torque
            # control. There is no later MotionCtrl_1 release in this model.
            actual = [event for event in self.events if event[:2] == ('publish', 'actual')]
            self.assertTrue(actual, 'FA physically released before FC actual tracking was primed')
            priming = next(event[2] for event in self.events if event[:2] == ('armed', 'actual'))
            require_pair_ready(self.node._follower_ready_rows, now_wall=self.wall(),
                               now_monotonic=self.mono, minimum_command_time=priming)
            self.assertEqual(self.node._native_forward_phase['kind'], 'fa')
            self.physical_released = True
            self.events.append(('physical-release', self.wall()))
            if self.cancel_fa:
                worker = threading.Thread(target=lambda: self.node.master_enable_callback(SimpleNamespace(data=False)))
                self.workers.append(worker)
                worker.start()
                self.assertTrue(self.node._manual_disable_requested.wait(0.5))
            else:
                self.sleep(0.005)  # The independent forwarding pump runs while SDK call is in flight.
            self.events.append(('fa-sdk-return', self.wall()))
        elif method == 'MasterSlaveConfig' and kwargs['linkage_config'] == 0xFC:
            self.physical_released = False
            self.raw_mode = 0
        elif method == 'MotionCtrl_2':
            self.raw_mode = 1

    def sample(self, _channel, since_timestamp=0):
        self.mono += 0.02
        self.events.append(('raw-read', self.wall()))
        if self.rollback_failure:
            raise RuntimeError('Driver fault observed during sampling')
        actual = list(self.current_pose)
        if self.raw_mode == 1:
            actual[0] += self.recovery_drift_mdeg
        return dict(ctrl_mode=self.raw_mode, arm_status=0, err_code=0, gripper_mode=0,
                    driver_enabled=[True] * 6, driver_foc_bytes=[0x40] * 6,
                    joint_mdeg=actual, timestamp_sec=self.wall(),
                    component_age_s={f'0x{key:03X}': 0.001 for key in REQUIRED_IDS})

    def enter(self):
        self.node.linkage_config_callback(SimpleNamespace(data='master'))

    def configs(self):
        return [event[2]['linkage_config'] for event in self.events if event[0] == 'MasterSlaveConfig']

    def alignment_message(self, *, gripper=None, joints=None):
        stamp = self.wall()
        values = self.target if joints is None else joints
        grip = self.node._last_alignment_command_gripper_um if gripper is None else gripper
        return SimpleNamespace(position=[value / 57324.840764 for value in values] + [grip / 1e6],
                               header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: stamp)))

    def prepare(self):
        self.node.linkage_config_callback(SimpleNamespace(data='prepare_master'))

    def release(self):
        self.node.linkage_config_callback(SimpleNamespace(data='release_master'))

    def test_prepare_tracks_fc_through_three_second_same_target_hold_without_releasing(self):
        self.prepare()
        phase = self.node._native_forward_phase
        self.assertTrue(phase['prepared'])
        self.assertFalse(self.physical_released)
        self.assertEqual(self.configs(), [])
        prepared_since = self.node._native_prepared_since()
        self.assertGreater(prepared_since, 0)
        for _ in range(30):
            self.sleep(0.1)
            self.node.joint_callback(self.alignment_message())
            self.assertIs(self.node._native_forward_phase, phase)
            self.assertTrue(phase['prepared'])
            self.assertEqual(self.node._native_prepared_since(), prepared_since)
            self.assertFalse(self.physical_released)
        self.node._publish_master_status()
        status = next(event[1] for event in reversed(self.events) if event[0] == 'status')
        self.assertEqual((status.angular.y, status.linear.z, status.linear.y), (0xFC, 0, 1))
        self.assertEqual(status.angular.z, prepared_since)
        self.assertEqual(self.configs(), [])
        self.release()
        self.assertTrue(self.node.in_teach_mode)
        self.assertEqual(self.configs(), [0xFA])
        self.assertEqual(len([event for event in self.events if event[:2] == ('armed', 'actual')]), 1)
        self.assertEqual(self.node._native_prepared_since(), 0)

    def test_release_without_preparation_never_sends_fa(self):
        self.release()
        self.assertEqual(self.configs(), [])
        self.assertFalse(self.physical_released)

    def test_actual_phase_invalidated_after_ready_check_cannot_arm_or_send_fa(self):
        self.prepare()
        actual = self.node._native_forward_phase
        def invalidate_before_arm(timeout):
            self.node._native_forward_failed(actual, RuntimeError('actual stream failed during arming'))
        self.node._native_master_reader.wait_started = invalidate_before_arm
        self.release()
        self.assertFalse(self.physical_released)
        self.assertEqual(self.configs(), [])
        self.assertFalse(any(event[:2] == ('armed', 'fa') for event in self.events))

    def test_fa_phase_cancelled_before_role_command_cannot_send_fa(self):
        self.prepare()
        original_stage = self.node._native_stage
        def cancel_before_write(name, source_time=None):
            original_stage(name, source_time)
            if name == 'fa-command-start':
                self.node._stop_native_forwarding()
        self.node._native_stage = cancel_before_write
        self.release()
        self.assertFalse(self.physical_released)
        self.assertEqual(self.configs(), [])

    def test_stale_prepared_actual_is_rejected_in_point_two_to_point_two_five_gap(self):
        self.prepare()
        self.assertGreater(self.node._native_prepared_since(), 0)
        self.actual_stopped = True
        self.mono += 0.21  # Older than native actual TTL, before 0.25s forwarding watchdog.
        self.assertEqual(self.node._native_prepared_since(), 0)
        self.release()
        self.assertEqual(self.configs(), [])
        self.assertFalse(self.physical_released)

    def test_transient_pair_heartbeat_gap_withholds_token_without_discarding_prepared_epoch(self):
        self.prepare()
        phase = self.node._native_forward_phase
        self.node._follower_ready_rows.pop('right')
        self.assertEqual(self.node._native_prepared_since(), 0)
        self.assertTrue(phase['prepared'])
        self.refresh_hearts()
        self.assertEqual(self.node._native_prepared_since(), phase['since'])
        self.release()
        self.assertEqual(self.configs(), [0xFA])

    def test_fresh_raw_and_heartbeats_cannot_hide_stalled_command_source_ack(self):
        self.prepare()
        phase = self.node._native_forward_phase
        self.frozen_ack = self.left_ack
        self.ack_mode = 'stalled'
        for _ in range(6):
            self.sleep(0.1)
        self.assertIsNotNone(self.cache.snapshot())
        self.assertTrue(phase['prepared'])
        self.assertEqual(self.node._native_prepared_since(), 0)
        self.release()
        self.assertEqual(self.configs(), [])

    def test_changed_joint_target_invalidates_prepared_phase_and_blocks_release(self):
        self.prepare()
        changed = list(self.target)
        changed[0] += 100
        self.node.joint_callback(self.alignment_message(joints=changed))
        self.assertIsNone(self.node._native_forward_phase)
        self.assertEqual(self.node._native_prepared_since(), 0)
        self.release()
        self.assertEqual(self.configs(), [])

    def test_changed_gripper_target_keeps_preparation_and_native_gripper_continues_following(self):
        self.prepare()
        phase = self.node._native_forward_phase
        self.node.joint_callback(self.alignment_message(gripper=45000))
        self.assertIs(self.node._native_forward_phase, phase)
        self.assertEqual(self.node._native_prepared_since(), phase['since'])
        self.assertEqual(self.node._last_alignment_command_gripper_um, 45000)
        self.fa_gripper = 70000
        self.release()
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertEqual(self.configs(), [0xFA])
        for raw, command in ((70000, 70000), (-1200, 0), (80001, 80000)):
            self.fa_gripper = raw
            self.sleep(0.01)
            latest = [event for event in self.events if event[:2] == ('publish', 'fa')][-1]
            self.assertEqual(latest[3][6], command / 1e6)

    def test_failed_repeated_alignment_write_invalidates_preparation(self):
        self.prepare()
        self.sdk_error_method = 'GripperCtrl'
        with self.assertRaises(RuntimeError):
            self.node.joint_callback(self.alignment_message())
        self.assertFalse(self.node._last_joint_command_valid)
        self.release()
        self.assertEqual(self.configs(), [])

    def test_cancellation_after_prepare_cannot_be_released(self):
        self.prepare()
        self.node.master_enable_callback(SimpleNamespace(data=False))
        self.assertEqual(self.node._native_prepared_since(), 0)
        self.release()
        self.assertEqual(self.configs(), [])

    def test_first_fa_preserves_raw_gripper_and_forwards_clamped_command(self):
        for actual, fa in ((-1200, -149000), (80001, 150000), (149000, 76000)):
            with self.subTest(actual=actual, fa=fa):
                self.setUp()
                self.actual_gripper, self.fa_gripper = actual, fa
                self.node._last_alignment_command_gripper_um = min(80000, max(0, actual))
                self.enter()
                self.assertTrue(self.node.native_handoff_report['passed'])
                self.assertEqual(self.node._native_last_actual['raw_gripper_um'], actual)
                outgoing = [event[3][6] for event in self.events if event[:2] == ('publish', 'fa')]
                self.assertTrue(outgoing)
                self.assertTrue(all(value == min(80000, max(0, fa)) / 1e6 for value in outgoing))

    def test_first_fa_gripper_difference_over_five_mm_is_allowed_and_forwarded(self):
        self.actual_gripper, self.fa_gripper = -1200, 80001
        self.node._last_alignment_command_gripper_um = 0
        self.enter()
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertEqual(self.configs(), [0xFA])
        outgoing = [event[3][6] for event in self.events if event[:2] == ('publish', 'fa')]
        self.assertTrue(outgoing)
        self.assertTrue(all(value == 0.08 for value in outgoing))

    def test_native_reader_fault_report_identifies_exact_missing_fa_id(self):
        self.missing_fa_id = 0x159
        self.reader_failure_after_fa_s = 3.0
        self.enter()
        diagnostics = self.node.native_handoff_report['native_diagnostics']
        self.assertEqual(diagnostics['missing_ids'], ['0x159'])
        self.assertGreater(diagnostics['rx_counts']['0x155'], 0)
        self.assertFalse(self.node.native_handoff_report['passed'])

    def test_fa_physically_releases_only_after_fc_tracking_and_pair_ack(self):
        self.enter()
        self.assertEqual(self.configs(), [0xFA])
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertTrue(self.node.in_teach_mode)
        self.assertLess(self.mono - 10.0, 0.1)
        self.assertFalse(any(event[0] in ('EnableArm', 'DisableArm', 'MotionCtrl_1') for event in self.events))
        stages = [row['stage'] for row in self.node.native_handoff_report['stages']]
        self.assertLess(stages.index('actual-first-forwarded'), stages.index('fc-priming-acknowledged'))
        self.assertLess(stages.index('fc-priming-acknowledged'), stages.index('fa-command-start'))

    def test_native_fa_is_forwarded_before_configuration_call_returns_or_mode_ack(self):
        self.enter()
        first = next(i for i, event in enumerate(self.events) if event[:2] == ('publish', 'fa'))
        returned = next(i for i, event in enumerate(self.events) if event[0] == 'fa-sdk-return')
        acknowledged = next(i for i, event in enumerate(self.events) if event == ('mode-ack', 0xFA))
        self.assertLess(first, returned)
        self.assertLess(first, acknowledged)
        self.assertTrue(self.events[first][4])
        self.assertFalse(self.events[first][5])
        self.assertAlmostEqual(self.events[first][2], self.wall() - 0.001)

    def test_first_fa_group_can_arrive_after_two_seconds_without_a_startup_timeout(self):
        for delay in (0.3, 1.8, 3.0, 6.0):
            with self.subTest(delay=delay):
                self.setUp()
                self.fa_delay_s = delay
                self.enter()
                self.assertTrue(self.node.native_handoff_report['passed'])
                self.assertEqual(self.configs(), [0xFA])
                first = next(row for row in self.node.native_handoff_report['stages']
                             if row['stage'] == 'fa-first-forwarded')
                elapsed = first['monotonic'] - self.fa_started_at
                self.assertGreaterEqual(elapsed, delay)
                self.assertLess(elapsed, delay + 0.01)
                self.assertLess(first['source_latency_s'], 0.01)
                self.assertIsNone(self.node._native_forward_phase['startup_timeout_s'])
                self.assertIsNone(self.node._native_forward_phase['startup_deadline'])

    def test_fc_first_group_still_has_only_a_quarter_second_startup_window(self):
        self.actual_stopped = True
        self.enter()
        self.assertEqual(self.configs(), [])
        self.assertGreaterEqual(self.mono - 10.0, 0.25)
        self.assertLess(self.mono - 10.0, 0.26)
        self.assertIn('actual master pose within 0.25s startup window', self.node.native_handoff_report['error'])

    def test_missing_first_fa_group_waits_until_manual_disable_without_success_ack(self):
        self.missing_fa = True
        self.cancel_after_fa_s = 3.0
        self.enter()
        self.assertFalse(self.node.native_handoff_report['passed'])
        self.assertNotIn(('mode-ack', 0xFA), self.events)
        self.assertEqual(self.configs(), [0xFA])
        disabled_at = next(event[3] for event in self.events if event[0] == 'DisableArm')
        phase_start_wall = next(event[2] for event in self.events if event[:2] == ('armed', 'fa'))
        self.assertGreaterEqual(disabled_at - phase_start_wall, 3.0)
        self.assertLess(disabled_at - phase_start_wall, 3.01)
        self.assertEqual(self.node.native_handoff_report['native_diagnostics']['missing_ids'],
                         ['0x155', '0x156', '0x157', '0x159'])
        self.assertFalse(any(event[:2] == ('publish', 'fa') for event in self.events))

    def test_either_follower_ack_may_arrive_late_while_early_fa_stream_stays_fresh(self):
        for side in ('left', 'right'):
            with self.subTest(side=side):
                self.setUp()
                self.fa_delay_s = 0.3
                self.fa_ack_delay_s = 1.8
                self.fa_delayed_ack_side = side
                self.enter()
                self.assertTrue(self.node.native_handoff_report['passed'])
                self.assertEqual(self.configs(), [0xFA])
                acknowledged = next(row for row in self.node.native_handoff_report['stages']
                                    if row['stage'] == 'fa-following-acknowledged')
                elapsed = acknowledged['monotonic'] - self.fa_started_at
                self.assertGreaterEqual(elapsed, 1.8)
                self.assertLess(elapsed, 1.81)

    def test_pair_ack_may_arrive_after_two_point_five_seconds(self):
        self.fa_delay_s = 1.8
        self.fa_ack_delay_s = 6.0
        self.enter()
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertEqual(self.configs(), [0xFA])
        acknowledged = next(row for row in self.node.native_handoff_report['stages']
                            if row['stage'] == 'fa-following-acknowledged')
        elapsed = acknowledged['monotonic'] - self.fa_started_at
        self.assertGreaterEqual(elapsed, 6.0)
        self.assertLess(elapsed, 6.01)

    def test_stream_loss_after_first_group_waits_until_manual_disable_without_cached_replay(self):
        self.fa_delay_s = 0.3
        self.fa_stop_after_s = 0.4
        self.fa_ack_delay_s = 1.8
        self.cancel_after_fa_s = 3.0
        self.enter()
        self.assertFalse(self.node.native_handoff_report['passed'])
        self.assertEqual(self.configs(), [0xFA])
        disabled_at = next(event[3] for event in self.events if event[0] == 'DisableArm')
        phase_start_wall = next(event[2] for event in self.events if event[:2] == ('armed', 'fa'))
        last_source = max(event[2] for event in self.events if event[:2] == ('publish', 'fa'))
        self.assertGreaterEqual(disabled_at - phase_start_wall, 3.0)
        self.assertLess(disabled_at - phase_start_wall, 3.01)
        self.assertGreater(disabled_at - last_source, 2.5)
        self.assertLess(last_source - phase_start_wall, 0.4)
        stages = [row['stage'] for row in self.node.native_handoff_report['stages']]
        self.assertIn('fa-waiting-missing-data', stages)
        self.assertNotIn('fa-following-acknowledged', stages)

    def test_pre_ack_gap_resumes_fresh_data_in_same_epoch_without_cached_replay(self):
        self.prepare()
        actual_epoch = self.node._native_forward_phase['epoch']
        self.fa_delay_s = 0.3
        self.fa_stop_after_s = 0.4
        self.fa_resume_after_s = 4.0
        self.fa_ack_delay_s = 4.5
        self.release()
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertEqual(self.configs(), [0xFA])
        phase = self.node._native_forward_phase
        self.assertEqual(phase['epoch'], actual_epoch + 1)
        self.assertTrue(phase['handoff_confirmed'])
        self.assertFalse(phase['waiting'])
        self.assertEqual(len([event for event in self.events if event[:2] == ('armed', 'fa')]), 1)
        phase_start_wall = next(event[2] for event in self.events if event[:2] == ('armed', 'fa'))
        sources = [event[2] for event in self.events if event[:2] == ('publish', 'fa')]
        self.assertTrue(all(left < right for left, right in zip(sources, sources[1:])))
        self.assertFalse(any(0.4 <= source + 0.001 - phase_start_wall < 4.0 for source in sources))
        self.assertGreater(max(right - left for left, right in zip(sources, sources[1:])), 3.5)
        stages = self.node.native_handoff_report['stages']
        names = [row['stage'] for row in stages]
        self.assertLess(names.index('fa-waiting-missing-data'), names.index('fa-native-resumed'))
        self.assertLess(names.index('fa-native-resumed'), names.index('fa-following-acknowledged'))
        resumed = next(row for row in stages if row['stage'] == 'fa-native-resumed')
        acknowledged = next(row for row in stages if row['stage'] == 'fa-following-acknowledged')
        self.assertGreaterEqual(resumed['monotonic'] - self.fa_started_at, 4.0)
        self.assertLess(acknowledged['monotonic'] - self.fa_started_at, 4.51)

    def test_reader_fault_during_pre_ack_gap_still_fails_immediately(self):
        self.fa_delay_s = 0.3
        self.fa_stop_after_s = 0.4
        self.fa_ack_delay_s = 1.8
        self.reader_failure_after_fa_s = 0.8
        self.enter()
        self.assertFalse(self.node.native_handoff_report['passed'])
        self.assertEqual(self.configs(), [0xFA, 0xFC])
        recovered_at = next(event[3] for event in self.events
                            if event[0] == 'MasterSlaveConfig' and event[2]['linkage_config'] == 0xFC)
        phase_start_wall = next(event[2] for event in self.events if event[:2] == ('armed', 'fa'))
        self.assertGreaterEqual(recovered_at - phase_start_wall, 0.8)
        self.assertLess(recovered_at - phase_start_wall, 0.81)
        self.assertIn('listener stopped', self.node.native_handoff_report['error'])
        self.assertIn('fa-waiting-missing-data',
                      [row['stage'] for row in self.node.native_handoff_report['stages']])

    def test_manual_disable_during_pre_ack_gap_has_no_later_automatic_write(self):
        self.fa_delay_s = 0.3
        self.fa_stop_after_s = 0.4
        self.fa_ack_delay_s = 1.8
        self.cancel_after_fa_s = 0.8
        self.enter()
        self.assertFalse(self.node.native_handoff_report['passed'])
        self.assertEqual(self.configs(), [0xFA])
        self.assertIn('fa-waiting-missing-data',
                      [row['stage'] for row in self.node.native_handoff_report['stages']])
        disabled = next(index for index, event in enumerate(self.events) if event[0] == 'DisableArm')
        self.fa_stop_after_s = None
        self.sleep(0.005)
        self.assertIsNone(self.node._native_forward_phase)
        self.assertFalse(any(event[0] in ('MasterSlaveConfig', 'JointCtrl', 'EnableArm', 'publish')
                             for event in self.events[disabled + 1:]))

    def test_prepared_fc_stream_loss_still_fails_after_quarter_second(self):
        self.prepare()
        phase = self.node._native_forward_phase
        self.actual_stopped = True
        self.sleep(0.26)
        self.assertIsNone(self.node._native_forward_phase)
        self.assertIn('missing/stale', phase['error'])
        self.assertEqual(self.configs(), [])

    def test_reader_fault_during_first_fa_wait_still_fails_immediately(self):
        self.fa_delay_s = 1.8
        self.reader_failure_after_fa_s = 0.1
        self.enter()
        self.assertFalse(self.node.native_handoff_report['passed'])
        self.assertEqual(self.configs(), [0xFA, 0xFC])
        recovered_at = next(event[3] for event in self.events
                            if event[0] == 'MasterSlaveConfig' and event[2]['linkage_config'] == 0xFC)
        phase_start_wall = next(event[2] for event in self.events if event[:2] == ('armed', 'fa'))
        self.assertLess(recovered_at - phase_start_wall, 0.11)
        self.assertFalse(any(event[:2] == ('publish', 'fa') for event in self.events))

    def test_manual_disable_during_first_fa_wait_has_no_later_automatic_write(self):
        self.fa_delay_s = 1.8
        self.cancel_after_fa_s = 0.1
        self.enter()
        self.assertFalse(self.node.native_handoff_report['passed'])
        self.assertEqual(self.configs(), [0xFA])
        disabled = next(index for index, event in enumerate(self.events) if event[0] == 'DisableArm')
        self.assertFalse(any(event[0] in ('MasterSlaveConfig', 'JointCtrl', 'EnableArm', 'publish')
                             for event in self.events[disabled + 1:]))
        self.assertFalse(any(event[:2] == ('publish', 'fa') for event in self.events))

    def test_single_side_or_old_probe_ack_cannot_authorize_fa_release(self):
        for mode in ('single', 'old'):
            with self.subTest(mode=mode):
                self.setUp()
                self.ack_mode = mode
                self.enter()
                self.assertEqual(self.configs(), [])
                self.assertFalse(self.node.native_handoff_report['passed'])
                self.assertFalse(self.physical_released)

    def test_host_tx_actual_packets_cannot_prime_or_release_master(self):
        self.host_tx_only = True
        self.enter()
        self.assertEqual(self.configs(), [])
        self.assertFalse(any(event[0] == 'publish' for event in self.events))

    def test_missing_trusted_alignment_target_never_primes_or_releases(self):
        self.node._last_alignment_target = None
        self.enter()
        self.assertEqual(self.configs(), [])
        self.assertFalse(any(event[0] == 'publish' for event in self.events))

    def test_old_epoch_snapshot_cannot_acknowledge_or_publish_as_fa(self):
        self.replay_old_epoch = True
        self.reader_failure_after_fa_s = 3.0
        self.enter()
        self.assertFalse(any(event[:2] == ('publish', 'fa') for event in self.events))
        self.assertNotIn(('mode-ack', 0xFA), self.events)
        self.assertEqual(self.configs(), [0xFA, 0xFC])

    def test_managed_publish_never_uses_or_falls_back_to_sdk_cache(self):
        self.enter()
        before = len([event for event in self.events if event[0] == 'publish'])
        self.missing_fa = True
        self.mono += 0.02
        self.node.PublishMasterArmJointAndGripper()
        self.assertEqual(len([event for event in self.events if event[0] == 'publish']), before)
        self.assertFalse(any(event[0].startswith('GetArm') for event in self.events))

    def test_phase_change_and_publish_are_serialized_without_old_epoch_confirmation(self):
        self.node._arm_native_forwarding('actual')
        self.mono += 0.005
        self.frames()
        entered = threading.Event()
        release = threading.Event()
        armed = threading.Event()
        original = self.node._native_master_reader.snapshot
        def delayed_snapshot(after=None):
            entered.set()
            self.assertTrue(release.wait(1))
            return original(after=after)
        self.node._native_master_reader.snapshot = delayed_snapshot
        publisher = threading.Thread(target=self.node._native_forward_tick)
        switcher = threading.Thread(target=lambda: (self.node._arm_native_forwarding('fa'), armed.set()))
        self.workers.extend((publisher, switcher))
        publisher.start()
        try:
            self.assertTrue(entered.wait(0.5))
            switcher.start()
            self.assertFalse(armed.wait(0.05))
        finally:
            release.set()
            self.join_workers()
        self.assertTrue(armed.is_set())
        actual = next(i for i, event in enumerate(self.events) if event[:2] == ('publish', 'actual'))
        fa = next(i for i, event in enumerate(self.events) if event[:2] == ('armed', 'fa'))
        self.assertLess(actual, fa)
        self.assertIsNone(self.node._native_forward_phase['first_source'])
        self.assertIsNone(self.node._native_forward_phase['previous'])

    def test_bad_actual_fault_or_disabled_never_sends_sdk_control(self):
        for field in ('actual_fault', 'actual_disabled'):
            with self.subTest(field=field):
                self.setUp()
                setattr(self, field, True)
                self.enter()
                self.assertFalse(any(event[0] in ('MasterSlaveConfig', 'JointCtrl', 'MotionCtrl_1',
                                                 'MotionCtrl_2', 'EnableArm', 'DisableArm')
                                     for event in self.events))
                self.assertNotIn(('mode-ack', 0xFA), self.events)
                self.assertEqual(self.node.current_linkage_config, 0)

    def test_fc_priming_pose_difference_is_allowed_without_an_added_angle_gate(self):
        self.actual_jump = True
        self.enter()
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertEqual(self.configs(), [0xFA])

    def test_first_fa_pose_above_two_degrees_is_forwarded_without_an_added_angle_gate(self):
        self.fa_joint_delta_mdeg = 20000
        self.enter()
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertEqual(self.configs(), [0xFA])
        outgoing = [event for event in self.events if event[:2] == ('publish', 'fa')]
        self.assertAlmostEqual(outgoing[0][3][0], (self.target[0] + 20000) / 57324.840764)

    def test_missing_native_can_be_stopped_by_ros_shutdown_without_success_ack_or_recovery(self):
        self.missing_fa = True
        self.shutdown_after_fa_s = 3.0
        self.enter()
        self.assertFalse(self.node.native_handoff_report['passed'])
        self.assertNotIn(('mode-ack', 0xFA), self.events)
        self.assertEqual(self.configs(), [0xFA])
        self.assertIn('ROS shutdown', self.node.native_handoff_report['error'])

    def test_pair_ack_wait_can_be_stopped_by_ros_shutdown_without_success_ack_or_recovery(self):
        self.ack_mode = 'fa-old'
        self.shutdown_after_fa_s = 3.0
        self.enter()
        self.assertFalse(self.node.native_handoff_report['passed'])
        self.assertNotIn(('mode-ack', 0xFA), self.events)
        self.assertEqual(self.configs(), [0xFA])
        self.assertIn('ROS shutdown', self.node.native_handoff_report['error'])
        self.assertTrue(any(event[:2] == ('publish', 'fa') for event in self.events))

    def test_post_release_rollback_measures_new_pose_before_any_target_write(self):
        self.ack_mode = 'fa-old'
        self.reader_failure_after_fa_s = 0.1
        self.enter()
        self.assertEqual(self.configs(), [0xFA, 0xFC])
        fc = next(i for i, event in enumerate(self.events)
                  if event[0] == 'MasterSlaveConfig' and event[2]['linkage_config'] == 0xFC)
        controls = self.events[fc + 1:]
        first_target = next(i for i, event in enumerate(controls) if event[0] == 'JointCtrl')
        first_raw = next(i for i, event in enumerate(controls) if event[0] == 'raw-read')
        self.assertLess(first_raw, first_target)
        targets = [event[1] for event in controls if event[0] == 'JointCtrl']
        self.assertTrue(targets)
        self.assertEqual(set(targets), {tuple(self.current_pose)})
        self.assertNotEqual(tuple(self.current_pose), tuple(self.target))
        self.assertFalse(any(event[0] in ('EnableArm', 'DisableArm', 'ResetPiper', 'GripperCtrl') for event in controls))

    def test_unverifiable_rollback_stays_unknown_and_sends_no_blind_pose(self):
        self.missing_fa = True
        self.rollback_failure = True
        self.reader_failure_after_fa_s = 0.1
        self.enter()
        fc = next(i for i, event in enumerate(self.events)
                  if event[0] == 'MasterSlaveConfig' and event[2]['linkage_config'] == 0xFC)
        self.assertFalse(any(event[0] in ('JointCtrl', 'MotionCtrl_1', 'MotionCtrl_2') for event in self.events[fc + 1:]))
        self.assertEqual(self.node.current_linkage_config, 0)
        self.assertIsNone(self.node.new_config_)

    def test_fresh_current_fc_recovery_accepts_large_drift_and_keeps_one_measured_target(self):
        self.recovery_drift_mdeg = 5000
        self.node._return_to_current_native_hold()
        self.assertEqual(self.node.current_linkage_config, 0xFC)
        self.assertEqual(self.configs(), [0xFC])
        targets = [event[1] for event in self.events if event[0] == 'JointCtrl']
        self.assertEqual(set(targets), {tuple(self.current_pose)})
        self.assertFalse(any(event[0] in ('EnableArm', 'DisableArm', 'ResetPiper') for event in self.events))

    def test_generic_alignment_hold_recovery_accepts_actual_pose_difference(self):
        self.node.require_follower_ready = False
        self.recovery_drift_mdeg = 5000
        self.node._return_to_alignment_hold()
        self.assertEqual(self.node.current_linkage_config, 0xFC)
        self.assertEqual(self.configs(), [0xFC])
        targets = [event[1] for event in self.events if event[0] == 'JointCtrl']
        self.assertEqual(set(targets), {tuple(self.target)})
        self.assertFalse(any(event[0] in ('EnableArm', 'DisableArm', 'ResetPiper') for event in self.events))

    def test_pause_following_stops_fa_publication_without_changing_mode_or_native_phase(self):
        self.enter()
        phase = self.node._native_forward_phase
        before = deepcopy(phase)
        events_before = list(self.events)
        reader = self.node._native_master_reader
        now = self.mono
        self.node.linkage_config_callback(SimpleNamespace(data='pause_following'))
        self.assertEqual(self.mono, now)
        self.assertEqual(self.events, events_before)
        self.assertEqual(phase, dict(before, following_enabled=False))
        self.fa_joint_delta_mdeg = 25000
        self.fa_gripper = 73000
        for _ in range(5):
            self.sleep(0.1)
        self.assertEqual(self.events, events_before)
        self.assertEqual(phase, dict(before, following_enabled=False))
        self.assertIs(self.node._native_forward_phase, phase)
        self.assertIs(self.node._native_master_reader, reader)
        self.assertEqual(self.node.current_linkage_config, 0xFA)
        self.assertTrue(self.node.in_teach_mode)
        self.assertTrue(self.node.GetEnableFlag())
        self.assertGreater(self.cache.diagnostics()['rx_counts']['0x155'], 1)

        now = self.mono
        self.node.linkage_config_callback(SimpleNamespace(data='resume_following'))
        self.assertEqual(self.mono, now)
        self.assertEqual(self.events, events_before)
        self.sleep(0.005)
        outgoing = [event for event in self.events if event[:2] == ('publish', 'fa')]
        self.assertEqual(len(self.events), len(events_before) + 1)
        self.assertAlmostEqual(outgoing[-1][2], self.wall() - 0.001)
        self.assertAlmostEqual(outgoing[-1][3][0], (self.target[0] + 25000) / 57324.840764)
        self.assertEqual(outgoing[-1][3][6], 0.073)
        self.assertIs(self.node._native_forward_phase, phase)
        self.assertEqual(self.configs(), [0xFA])

    def test_resume_following_waits_for_fresh_physical_data_without_replaying_stale_data(self):
        self.enter()
        phase = self.node._native_forward_phase
        previous = dict(phase['previous'])
        self.node.linkage_config_callback(SimpleNamespace(data='pause_following'))
        self.missing_fa = True
        self.sleep(1.0)
        outgoing_before = len([event for event in self.events if event[0] == 'publish'])
        self.node.linkage_config_callback(SimpleNamespace(data='resume_following'))
        self.node._native_forward_tick()
        self.assertEqual(len([event for event in self.events if event[0] == 'publish']), outgoing_before)
        self.assertEqual(phase['previous'], previous)
        self.missing_fa = False
        self.fa_joint_delta_mdeg = 18000
        self.sleep(0.005)
        self.assertEqual(len([event for event in self.events if event[0] == 'publish']), outgoing_before + 1)
        outgoing = [event for event in self.events if event[:2] == ('publish', 'fa')][-1]
        self.assertAlmostEqual(outgoing[2], self.wall() - 0.001)
        self.assertAlmostEqual(outgoing[3][0], (self.target[0] + 18000) / 57324.840764)
        self.assertIs(self.node._native_forward_phase, phase)
        self.assertEqual(self.configs(), [0xFA])

    def test_pause_following_does_not_stop_fc_actual_preparation(self):
        self.prepare()
        phase = self.node._native_forward_phase
        before = len([event for event in self.events if event[:2] == ('publish', 'actual')])
        self.node.linkage_config_callback(SimpleNamespace(data='pause_following'))
        self.assertFalse(phase['following_enabled'])
        self.sleep(0.005)
        after = len([event for event in self.events if event[:2] == ('publish', 'actual')])
        self.assertEqual(after, before + 1)
        self.assertTrue(phase['prepared'])
        self.release()
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertIsNot(self.node._native_forward_phase, phase)
        self.assertTrue(self.node._native_forward_phase['following_enabled'])

    def test_pause_following_without_a_native_phase_is_a_software_noop(self):
        before = list(self.events)
        now = self.mono
        for command in ('pause_following', 'resume_following'):
            self.node.linkage_config_callback(SimpleNamespace(data=command))
            self.assertIsNone(self.node._native_forward_phase)
            self.assertEqual(self.events, before)
            self.assertEqual(self.mono, now)
        self.prepare()
        self.assertTrue(self.node._native_forward_phase['following_enabled'])

    def test_paused_following_uses_original_fc_exit_and_new_fa_phase_starts_enabled(self):
        self.enter()
        self.node.linkage_config_callback(SimpleNamespace(data='pause_following'))
        paused = self.node._native_forward_phase
        self.node.linkage_config_callback(SimpleNamespace(data='slave'))
        self.assertEqual(self.configs(), [0xFA, 0xFC])
        self.assertIsNone(self.node._native_forward_phase)
        self.assertEqual(self.node.current_linkage_config, 0xFC)
        self.assertFalse(self.node.in_teach_mode)
        self.assertTrue(self.node.GetEnableFlag())
        self.assertFalse(any(event[0] in ('EnableArm', 'DisableArm', 'ResetPiper', 'GripperCtrl')
                             for event in self.events))
        self.node.joint_callback(self.alignment_message())
        self.prepare()
        self.release()
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertTrue(self.node._native_forward_phase['following_enabled'])
        self.assertIsNot(self.node._native_forward_phase, paused)
        self.assertEqual(self.configs(), [0xFA, 0xFC, 0xFA])

    def test_manual_disable_while_paused_cannot_be_undone_by_resume_following(self):
        self.enter()
        self.node.linkage_config_callback(SimpleNamespace(data='pause_following'))
        self.node.master_enable_callback(SimpleNamespace(data=False))
        disabled = next(index for index, event in enumerate(self.events) if event[0] == 'DisableArm')
        self.node.linkage_config_callback(SimpleNamespace(data='resume_following'))
        self.sleep(0.005)
        self.assertIsNone(self.node._native_forward_phase)
        self.assertTrue(self.node._manual_disable_requested.is_set())
        self.assertFalse(any(event[0] in ('MasterSlaveConfig', 'JointCtrl', 'EnableArm', 'publish')
                             for event in self.events[disabled + 1:]))

    def test_runtime_silence_waits_same_epoch_without_replay_and_any_fresh_valid_pose_resumes(self):
        for duration in (1.0, 10.0, 30.0):
            with self.subTest(duration=duration):
                self.setUp()
                self.enter()
                phase = self.node._native_forward_phase
                previous = dict(phase['previous'])
                outgoing_before = len([event for event in self.events if event[0] == 'publish'])
                controls_before = [event for event in self.events
                                   if event[0] in ('MasterSlaveConfig', 'JointCtrl', 'MotionCtrl_1',
                                                   'MotionCtrl_2', 'EnableArm', 'DisableArm')]
                self.frozen_ack = self.left_ack
                self.ack_mode = 'stalled'
                self.missing_fa = True
                self.sleep(duration)
                self.assertIs(self.node._native_forward_phase, phase)
                self.assertTrue(phase['waiting'])
                self.assertEqual(phase['previous'], previous)
                self.assertEqual(self.node.current_linkage_config, 0xFA)
                self.assertTrue(self.node.in_teach_mode)
                self.assertEqual(self.node._follower_ready_rows['left']['command_time'], self.frozen_ack)
                self.assertEqual(len([event for event in self.events if event[0] == 'publish']), outgoing_before)
                type(self.node)._publish_mode_status(self.node)
                self.assertIn('等待主臂新数据', self.events[-1][1])
                self.missing_fa = False
                self.fa_joint_delta_mdeg = 31000
                self.fa_gripper = 80001  # Gripper keeps normal clamped following after a gap.
                self.sleep(0.005)
                self.assertIs(self.node._native_forward_phase, phase)
                self.assertFalse(phase['waiting'])
                outgoing = [event for event in self.events if event[:2] == ('publish', 'fa')]
                self.assertAlmostEqual(outgoing[-1][2], self.wall() - 0.001)
                self.assertAlmostEqual(outgoing[-1][3][0], (self.target[0] + 31000) / 57324.840764)
                self.assertEqual(outgoing[-1][3][6], 0.08)
                self.assertEqual(self.node._follower_ready_rows['left']['command_time'], self.frozen_ack)
                controls_after = [event for event in self.events
                                  if event[0] in ('MasterSlaveConfig', 'JointCtrl', 'MotionCtrl_1',
                                                  'MotionCtrl_2', 'EnableArm', 'DisableArm')]
                self.assertEqual(controls_after, controls_before)
                stages = [row['stage'] for row in self.node.native_handoff_report['stages']]
                self.assertIn('fa-waiting-missing-data', stages)
                self.assertIn('fa-native-resumed', stages)

    def test_runtime_gap_resumes_large_fresh_targets_without_an_angle_or_realign_gate(self):
        self.enter()
        phase = self.node._native_forward_phase
        self.missing_fa = True
        self.sleep(1.0)
        outgoing_before = len([event for event in self.events if event[0] == 'publish'])
        self.missing_fa = False
        self.fa_joint_delta_mdeg = 4301
        for step in range(5):
            self.fa_joint_delta_mdeg += 5000
            self.sleep(0.01)
            self.assertFalse(phase['waiting'])
            self.assertIsNone(phase['waiting_reason'])
            self.assertEqual(len([event for event in self.events if event[0] == 'publish']), outgoing_before + step + 1)
        self.assertEqual(self.node.current_linkage_config, 0xFA)
        self.assertIsNone(phase['error'])
        type(self.node)._publish_mode_status(self.node)
        self.assertNotIn('2度', self.events[-1][1])
        self.assertEqual(self.configs(), [0xFA])

    def test_short_runtime_gap_keeps_normal_forwarding_without_recovery_gate(self):
        self.enter()
        phase = self.node._native_forward_phase
        self.missing_fa = True
        self.sleep(0.1)
        self.assertFalse(phase['waiting'])
        self.fa_joint_delta_mdeg = 4301
        self.missing_fa = False
        self.sleep(0.005)
        outgoing = [event for event in self.events if event[:2] == ('publish', 'fa')]
        self.assertAlmostEqual(outgoing[-1][3][0], (self.target[0] + 4301) / 57324.840764)
        self.assertFalse(phase['waiting'])

    def test_waiting_requires_new_physical_complete_group_without_old_sample_replay(self):
        self.enter()
        phase = self.node._native_forward_phase
        self.missing_fa = True
        self.sleep(1.0)
        outgoing_before = len([event for event in self.events if event[0] == 'publish'])
        self.missing_fa = False
        self.host_tx_only = True
        self.sleep(0.01)
        self.assertTrue(phase['waiting'])
        self.assertEqual(len([event for event in self.events if event[0] == 'publish']), outgoing_before)
        self.host_tx_only = False
        self.missing_fa_id = 0x159
        self.sleep(0.01)
        self.assertTrue(phase['waiting'])
        self.assertEqual(len([event for event in self.events if event[0] == 'publish']), outgoing_before)
        self.missing_fa_id = None
        self.sleep(0.005)
        self.assertFalse(phase['waiting'])
        self.assertEqual(len([event for event in self.events if event[0] == 'publish']), outgoing_before + 1)

    def test_runtime_waiting_still_hard_stops_on_reader_failure(self):
        self.enter()
        self.missing_fa = True
        self.sleep(1.0)
        self.assertTrue(self.node._native_forward_phase['waiting'])
        self.reader_failure = True
        self.node._native_forward_tick()
        self.assertIsNone(self.node._native_forward_phase)
        self.assertEqual(self.node.current_linkage_config, 0)
        self.assertEqual(self.configs(), [0xFA])

    def test_runtime_waiting_still_hard_stops_on_malformed_native_gripper(self):
        self.enter()
        self.missing_fa = True
        self.sleep(1.0)
        self.missing_fa = False
        self.fa_gripper = 150001
        self.mono += 0.005
        with self.assertRaises(RuntimeError):
            self.frames()
        self.node._native_forward_tick()
        self.assertIsNone(self.node._native_forward_phase)
        self.assertEqual(self.node.current_linkage_config, 0)
        self.assertEqual(self.configs(), [0xFA])

    def test_manual_disable_while_waiting_cannot_resume_on_new_native_frames(self):
        self.enter()
        self.missing_fa = True
        self.sleep(1.0)
        self.assertTrue(self.node._native_forward_phase['waiting'])
        self.node.master_enable_callback(SimpleNamespace(data=False))
        disabled = next(index for index, event in enumerate(self.events) if event[0] == 'DisableArm')
        self.missing_fa = False
        self.sleep(0.005)
        self.assertIsNone(self.node._native_forward_phase)
        self.assertFalse(any(event[0] in ('MasterSlaveConfig', 'JointCtrl', 'EnableArm', 'publish')
                             for event in self.events[disabled + 1:]))

    def test_manual_disable_during_priming_cancels_without_release_or_recovery(self):
        self.cancel_priming = True
        self.enter()
        self.assertEqual(self.configs(), [])
        disabled = next(i for i, event in enumerate(self.events) if event[0] == 'DisableArm')
        self.assertFalse(any(event[0] == 'publish' for event in self.events[disabled + 1:]))
        self.assertTrue(self.node._manual_disable_requested.is_set())

    def test_manual_disable_during_inflight_fa_has_no_later_automatic_write(self):
        self.cancel_fa = True
        self.enter()
        self.join_workers()
        disabled = next(i for i, event in enumerate(self.events) if event[0] == 'DisableArm')
        self.assertFalse(any(event[0] in ('MasterSlaveConfig', 'JointCtrl', 'EnableArm', 'publish')
                             for event in self.events[disabled + 1:]))
        self.assertEqual(self.configs(), [0xFA])

    def test_manual_disable_does_not_wait_behind_inflight_native_ros_publication(self):
        self.node._arm_native_forwarding('actual')
        self.mono += 0.005
        self.frames()
        started = threading.Event()
        release = threading.Event()
        disabled = threading.Event()
        original = self.node.joint_std_pub_master.publish
        def blocked_publish(message):
            started.set()
            self.assertTrue(release.wait(1))
            original(message)
        self.node.joint_std_pub_master.publish = blocked_publish
        publisher = threading.Thread(target=self.node._native_forward_tick)
        stopper = threading.Thread(target=lambda: (self.node.master_enable_callback(SimpleNamespace(data=False)),
                                                   disabled.set()))
        self.workers.extend((publisher, stopper))
        publisher.start()
        try:
            self.assertTrue(started.wait(0.5))
            stopper.start()
            self.assertTrue(disabled.wait(0.5), 'DisableArm waited behind native forwarding publication')
            self.assertIsNone(self.node._native_forward_phase)
        finally:
            release.set()
            self.join_workers()
        published = len([event for event in self.events if event[0] == 'publish'])
        self.node._native_forward_tick()
        self.assertEqual(len([event for event in self.events if event[0] == 'publish']), published)

    def test_managed_fc_resume_uses_current_pose_without_old_target_or_enable(self):
        self.enter()
        self.node.linkage_config_callback(SimpleNamespace(data='slave'))
        self.assertEqual(self.node.current_linkage_config, 0xFC)
        self.assertIsNone(self.node._native_forward_phase)
        self.assertIsNone(self.node._last_alignment_target)
        self.assertFalse(any(event[0] == 'EnableArm' for event in self.events))


if __name__ == '__main__':
    unittest.main()
