"""Exercise d -> real measured hold -> teach acknowledgement -> automatic recording.

The production methods and raw-feedback verifier run with a deterministic clock
and stand-in publishers/readers. No ROS, camera, CAN, or SDK is initialized.
"""
from collections import deque
from contextlib import nullcontext, redirect_stdout
import io
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest

from test_compatibility import load_official_methods

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / 'setup'))
from master_alignment_feedback import alignment_targets, COMPONENTS, wait_for_master_alignment


class StopLoop(Exception):
    pass


class AutoDAggerRecordingTests(unittest.TestCase):
    def setUp(self):
        self.output = redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.namespace, operator_class = load_official_methods()
        self.operator = operator_class.__new__(operator_class)
        self.operator.start_dagger_preroll = lambda: None
        self.operator.finish_dagger_preroll = lambda _ready_at: None
        self.operator.consume_dagger_preroll = lambda: None
        self.operator.args = SimpleNamespace(dataset_dir=temporary.name, dataset_name='auto_record')
        self.operator.stop_flag = False
        self.operator.puppet_arm_left_deque = deque([SimpleNamespace(position=[0.1] * 6 + [0.04])])
        self.operator.puppet_arm_right_deque = deque([SimpleNamespace(position=[0.2] * 6 + [0.03])])
        self.clock = 0.0
        self.shutdown = False
        self.phase = None
        self.phases = []
        self.events = []
        self.keys = []
        self.published = []
        self.hold_failure = None
        self.shutdown_after_hold = False
        self.ack_failure = False
        self.acknowledged = False
        self.model_tick = 0
        self.on_tick = lambda tick: None
        self.record_count = 0

        def log(message, *args):
            self.events.append(('log', self.clock, message % args if args else message))

        def rate(hz):
            def sleep():
                self.clock += 1 / hz
                if hz != 10:
                    self.model_tick += 1
                    self.assertLess(self.model_tick, 20, 'model simulation failed to terminate')
                    self.on_tick(self.model_tick)
            return SimpleNamespace(sleep=sleep)

        self.ros = SimpleNamespace(
            is_shutdown=lambda: self.shutdown, Rate=rate,
            Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: 1000 + self.clock)), loginfo=log, logerr=log)
        self.namespace.update(
            rospy=self.ros, time=SimpleNamespace(monotonic=lambda: self.clock,
                                                time=lambda: 1000 + self.clock),
            math=math, os=os, JointState=SimpleNamespace, Header=SimpleNamespace,
            String=SimpleNamespace, alignment_targets=alignment_targets,
            dagger_mode_active=False, resume_inference_requested=False,
            save_data_requested=False, collection_active=False,
            waiting_for_user_confirm=False, delete_data_requested=False,
            dagger_mode_lock=threading.Lock(), collection_lock=threading.Lock(),
            save_data_lock=threading.Lock(), delete_data_lock=threading.Lock())

        def save(_args, _frames, _actions, path):
            self.events.append(('save-inference', self.clock))
            Path(path + '.hdf5').touch()

        self.namespace['save_data'] = save
        self.operator.stop_inference_collection = lambda **kwargs: (['observation'], ['action'])
        self.operator.enable_master_arms = lambda: self.events.append(('enable', self.clock))
        self.operator.release_prepared_masters = self.request_teach
        def ready_preparation(started_at, timeout=15):
            done = threading.Event()
            done.set()
            state = dict(done=done, cancel=threading.Event(), success=True, error=None,
                         hold_passed=False, deadline=started_at + timeout,
                         thread=SimpleNamespace(join=lambda **kwargs: None))
            self.operator.parallel_following_preparation = state
            self.events.append(('prepare-start', self.clock))
            return state
        self.operator._start_following_preparation = ready_preparation
        self.operator._following_preparation_guard = lambda state: None
        self.operator._following_preparation_ready = lambda state: True
        self.operator.wait_for_master_control = self.acknowledge
        self.operator.wait_for_fresh_policy_inputs = lambda: None
        self.operator.puppet_arm_publish_continuous = lambda *args: None
        self.operator.get_synced_frame = lambda: None
        self.operator.dagger_recording_ready = lambda: True
        self.operator.start_inference_collection = lambda: self.events.append(('restart-inference', self.clock))
        for side in ('left', 'right'):
            publisher = SimpleNamespace(publish=lambda message, side=side:
                                        self.published.append((self.clock, self.phase, side, list(message.position))))
            setattr(self.operator, 'master_' + side + '_joint_pub', publisher)
            setattr(self.operator, 'master_' + side + '_config_pub',
                    SimpleNamespace(publish=lambda _message, side=side:
                                    self.events.append(('slave-config', self.clock, side))))

        collector = self.namespace['SimpleDAggerCollector']([], dataset_dir=temporary.name)
        real_start, real_stop = collector.start_collection, collector.stop_collection
        def start():
            self.assertTrue(self.acknowledged, 'recording started before both master acknowledgements')
            self.record_count += 1
            self.events.append(('record', self.clock))
            real_start()
        def stop():
            self.events.append(('stop-human', self.clock))
            real_stop()
        collector.start_collection, collector.stop_collection = start, stop
        self.operator.data_collector = collector
        self.namespace['wait_for_master_alignment'] = self.verify

    def pause(self, seconds):
        self.clock += seconds

    def verify(self, targets, **options):
        name = 'hold' if options['stable_seconds'] == 3.0 else 'alignment'
        phase = {'name': name, 'started': self.clock, 'options': options, 'targets': targets}
        self.phases.append(phase)
        self.phase = name
        converted = alignment_targets(targets)
        def reader(channel, **_unused):
            self.clock += 0.06
            side = 'left' if 'left' in channel else 'right'
            row = dict(ctrl_mode=1, arm_status=0, err_code=0, gripper_mode=0,
                       driver_enabled=[True] * 6, driver_foc_bytes=[64] * 6,
                       joint_mdeg=list(converted[side]['joint_mdeg']),
                       gripper_um=converted[side]['gripper_um'],
                       timestamp_sec=1000 + self.clock,
                       component_age_s=dict.fromkeys(COMPONENTS, 0.0))
            if name == 'hold':
                if self.hold_failure == 'missing':
                    raise RuntimeError('Missing required CAN feedback frames')
                if self.hold_failure == 'disabled':
                    row['driver_enabled'][2] = False
                if self.hold_failure == 'stale':
                    row['component_age_s']['0x2A6'] = 0.5
                if self.hold_failure == 'fault':
                    row['driver_foc_bytes'][0] = 65
                if self.hold_failure == 'drift':
                    row['joint_mdeg'][1] += 5000
                if self.hold_failure == 'shutdown' and self.clock-phase['started'] >= 0.5:
                    self.shutdown = True
                if self.hold_failure == 'interrupt':
                    raise KeyboardInterrupt
            return row
        report = wait_for_master_alignment(
            targets, reader=reader, clock=lambda: self.clock, wall=lambda: 1000+self.clock,
            pause=self.pause, **options)
        phase.update(finished=self.clock, report=report)
        self.events.append((name + '-finished', self.clock, report['passed']))
        self.phase = None
        if name == 'hold' and self.shutdown_after_hold:
            self.shutdown = True
        return report

    def request_teach(self):
        holds = [phase for phase in self.phases if phase['name'] == 'hold']
        self.assertTrue(holds and holds[-1]['report']['passed'])
        self.assertGreaterEqual(holds[-1]['finished'] - holds[-1]['started'], 3.0)
        self.events.append(('teach', self.clock))

    def acknowledge(self, config, teaching, since, **_options):
        self.events.append(('ack', self.clock, config, teaching))
        if teaching and self.ack_failure:
            raise RuntimeError('Master handoff was not acknowledged')
        self.acknowledged = teaching

    def key(self, char):
        self.keys.append(char)
        previous = self.namespace['rospy']
        loops = [0]
        def done():
            loops[0] += 1
            return loops[0] > 1
        stdin = SimpleNamespace(fileno=lambda: 0, read=lambda _count: char)
        self.namespace.update(
            rospy=SimpleNamespace(is_shutdown=done), sys=SimpleNamespace(stdin=stdin),
            termios=SimpleNamespace(tcgetattr=lambda _stream: [], tcsetattr=lambda *args: None, TCSADRAIN=1),
            tty=SimpleNamespace(setcbreak=lambda _fd: None),
            select=SimpleNamespace(select=lambda *args: ([stdin], [], [])))
        try:
            self.namespace['keyboard_monitor_thread']()
        finally:
            self.namespace['rospy'] = previous

    def run_model(self, on_tick=None):
        if on_tick is not None:
            self.on_tick = on_tick
        buffer = SimpleNamespace(lock=threading.Lock(), cur_chunk=deque(), chunks=deque(),
                                 last_action=None, k=0, pop_next_action=lambda: None)
        self.namespace.update(
            lang_embeddings='test', np=SimpleNamespace(zeros=lambda _shape: []),
            torch=SimpleNamespace(inference_mode=nullcontext),
            websocket_client_policy=SimpleNamespace(WebsocketClientPolicy=lambda *args:
                                                    SimpleNamespace(get_server_metadata=lambda: {})),
            SimpleKalmanFilter=lambda: None, stream_buffer=buffer)
        args = SimpleNamespace(host='fake', port=0, publish_rate=30, use_temporal_smoothing=True)
        config = dict(state_dim=14, episode_len=1000, chunk_size=1)
        self.namespace['model_inference'](args, config, self.operator, keyboard_thread_started=True)

    def test_d_without_space_automatically_records_after_real_three_second_hold(self):
        self.key('d')
        def check(_tick):
            self.assertTrue(self.operator.data_collector.is_collecting)
            self.assertTrue(self.namespace['collection_active'])
            raise StopLoop
        with self.assertRaises(StopLoop):
            self.run_model(check)
        self.assertEqual(self.keys, ['d'])
        self.assertEqual(self.record_count, 1)
        self.assertEqual([phase['name'] for phase in self.phases], ['alignment', 'alignment', 'hold'])
        hold = self.phases[-1]
        self.assertGreaterEqual(hold['report']['stable_elapsed_s'], 3.0)
        self.assertGreaterEqual(hold['started'], self.phases[-2]['finished'])
        events = [event[0] for event in self.events]
        self.assertLess(events.index('hold-finished'), events.index('teach'))
        self.assertLess(events.index('teach'), events.index('ack'))
        self.assertLess(events.index('ack'), events.index('record'))
        published = [item for item in self.published if item[1] == 'hold']
        self.assertGreater(len(published), 4)
        self.assertEqual({item[2] for item in published}, {'left', 'right'})

    def test_hold_targets_are_current_follower_poses_not_fixed_intermediate_pose(self):
        self.key('d')
        self.on_tick = lambda _tick: (_ for _ in ()).throw(StopLoop())
        with self.assertRaises(StopLoop):
            self.run_model()
        hold = self.phases[-1]
        self.assertEqual(hold['targets']['left'], [0.1] * 6 + [0.04])
        self.assertEqual(hold['targets']['right'], [0.2] * 6 + [0.03])
        self.assertNotEqual(hold['targets'], self.phases[0]['targets'])

    def test_hold_failures_block_teaching_and_recording(self):
        for failure in ('missing', 'disabled', 'stale', 'fault', 'drift', 'shutdown'):
            with self.subTest(failure=failure):
                # Each subcase gets an independent collector and fake clock.
                case = AutoDAggerRecordingTests()
                case.setUp()
                try:
                    case.hold_failure = failure
                    case.key('d')
                    with case.assertRaisesRegex(RuntimeError, 'Failed to enter DAgger'):
                        case.run_model()
                    case.assertEqual(case.record_count, 0)
                    case.assertFalse(any(event[0] == 'teach' for event in case.events))
                    case.assertFalse(case.operator.master_alignment_reports['slave pose hold']['passed'])
                finally:
                    case.doCleanups()

    def test_keyboard_interrupt_during_hold_does_not_teach_or_record(self):
        self.hold_failure = 'interrupt'
        self.key('d')
        with self.assertRaises(KeyboardInterrupt):
            self.run_model()
        self.assertEqual(self.record_count, 0)
        self.assertFalse(any(event[0] == 'teach' for event in self.events))

    def test_shutdown_after_hold_before_mode_switch_blocks_teach(self):
        self.shutdown_after_hold = True
        self.key('d')
        with self.assertRaisesRegex(RuntimeError, 'Failed to enter DAgger'):
            self.run_model()
        self.assertEqual(self.record_count, 0)
        self.assertFalse(any(event[0] == 'teach' for event in self.events))

    def test_failed_master_mode_ack_does_not_start_recording(self):
        self.ack_failure = True
        self.key('d')
        with self.assertRaisesRegex(RuntimeError, 'not acknowledged'):
            self.run_model()
        self.assertEqual(self.record_count, 0)
        self.assertFalse(self.namespace['collection_active'])

    def test_two_d_r_cycles_each_record_automatically_without_space(self):
        self.key('d')
        def events(tick):
            if tick in (1, 3):
                self.assertTrue(self.operator.data_collector.is_collecting)
                self.key('r')
            elif tick == 2:
                self.assertFalse(self.namespace['dagger_mode_active'])
                self.assertFalse(self.namespace['collection_active'])
                self.key('d')
            else:
                self.assertFalse(self.namespace['dagger_mode_active'])
                raise StopLoop
        with self.assertRaises(StopLoop):
            self.run_model(events)
        self.assertEqual(self.record_count, 2)
        self.assertEqual(sum(event[0] == 'restart-inference' for event in self.events), 2)
        self.assertEqual(sum(phase['name'] == 'hold' for phase in self.phases), 2)
        self.assertNotIn(' ', self.keys)

    def test_s_stops_recording_without_automatically_restarting_it(self):
        self.operator.data_collector.has_data = lambda: True
        def save(**_kwargs):
            self.events.append(('save-human', self.clock))
            self.operator.data_collector.is_collecting = False
        self.operator.data_collector.save_current_episode = save
        self.key('d')
        def events(tick):
            if tick == 1:
                self.key('s')
            else:
                self.assertEqual(self.record_count, 1)
                self.assertFalse(self.operator.data_collector.is_collecting)
                self.assertFalse(self.namespace['collection_active'])
                raise StopLoop
        with self.assertRaises(StopLoop):
            self.run_model(events)
        self.assertTrue(any(event[0] == 'save-human' for event in self.events))


if __name__ == '__main__':
    unittest.main()
