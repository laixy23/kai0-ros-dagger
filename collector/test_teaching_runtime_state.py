"""Check idle teaching and soft data pause without ROS, SDK or hardware."""
from collections import deque
import collections
from contextlib import nullcontext, redirect_stdout
import io
import math
import threading
from types import SimpleNamespace
import unittest

from test_compatibility import load_official_methods


class StopModel(Exception):
    pass


def stamp(value):
    return SimpleNamespace(to_sec=lambda: value)


def mode_message(source, *, config=0xFA, teaching=1, enabled=1):
    return SimpleNamespace(angular=SimpleNamespace(x=source, y=config, z=0),
                           linear=SimpleNamespace(x=1, y=enabled, z=teaching))


class RuntimeStatusTests(unittest.TestCase):
    def setUp(self):
        self.ns, cls = load_official_methods()
        self.node = cls.__new__(cls)
        self.node.master_handoff_lock = threading.Lock()
        self.node.master_control_feedback = {}
        self.node.master_prepared_feedback = {}
        self.node.master_command_seen_at = {}
        self.mono = 10.0
        self.ns.update(time=SimpleNamespace(monotonic=lambda: self.mono,
                                            time=lambda: 990 + self.mono,
                                            sleep=lambda seconds: setattr(self, 'mono', self.mono + seconds)),
                       math=math, rospy=SimpleNamespace(is_shutdown=lambda: False))
        self.refresh()

    def refresh(self, **patch):
        for side in ('left', 'right'):
            self.node.master_control_status_callback(mode_message(990 + self.mono, **patch), side)

    def test_healthy_fa_with_no_new_commands_is_recordable_idle(self):
        ready, reason = self.node.dagger_recording_status()
        self.assertTrue(ready)
        self.assertIn('Waiting', reason)
        self.node.master_command_seen_at = {'left': 1.0, 'right': 1.0}
        self.assertTrue(self.node.dagger_recording_status()[0])

    def test_missing_old_disabled_or_non_fa_mode_pauses_without_latching(self):
        baseline = dict(self.node.master_prepared_feedback['right'])
        for patch in ({'config': 0}, {'config': 0xFC}, {'teaching': 0}, {'enabled': 0},
                      {'source_time': 999.4}, {'received_monotonic': 9.4},
                      {'source_time': 1000.06}, {'source_time': float('nan')}):
            with self.subTest(patch=patch):
                self.node.master_prepared_feedback['right'] = dict(baseline, **patch)
                self.assertFalse(self.node.dagger_recording_status()[0])
                self.node.master_prepared_feedback['right'] = dict(baseline)
                self.assertTrue(self.node.dagger_recording_status()[0])
        self.node.master_prepared_feedback.pop('right')
        self.assertFalse(self.node.dagger_recording_status()[0])

    def test_mode_wait_rejects_stale_source_or_disabled_status_despite_fresh_tuple(self):
        self.node.master_command_seen_at = {'left': 10.0, 'right': 10.0}
        for patch in ({'source_time': 999.0}, {'enabled': 0}):
            with self.subTest(patch=patch):
                self.mono = 10.0
                self.refresh()
                self.node.master_prepared_feedback['right'].update(patch)
                with self.assertRaises(RuntimeError):
                    self.node.wait_for_master_control(0xFA, True, since=10.0, timeout=0.1)

    def test_notice_changes_once_and_healthy_mode_can_resume(self):
        output = io.StringIO()
        with redirect_stdout(output):
            for _ in range(5):
                self.assertTrue(self.node.dagger_recording_ready())
            self.refresh(config=0, teaching=0)
            for _ in range(5):
                self.assertFalse(self.node.dagger_recording_ready())
            self.refresh()
            self.assertTrue(self.node.dagger_recording_ready())
        self.assertEqual(output.getvalue().count('Data append paused'), 1)
        self.assertEqual(output.getvalue().count('Recording status'), 2)


class SyncedIdleFrameTests(unittest.TestCase):
    def setUp(self):
        self.ns, cls = load_official_methods()
        self.node = cls.__new__(cls)
        self.node._dagger_live_after_source_time = None
        self.node.bridge = SimpleNamespace(imgmsg_to_cv2=lambda message, encoding: message.payload)
        for side in ('left', 'right', 'front'):
            setattr(self.node, 'img_' + side + '_deque', deque([
                SimpleNamespace(header=SimpleNamespace(stamp=stamp(1000)), payload=side)]))
        for side in ('left', 'right'):
            setattr(self.node, 'puppet_arm_' + side + '_deque', deque([
                SimpleNamespace(header=SimpleNamespace(stamp=stamp(1000.1)), position=[0.1] * 7)]))
            setattr(self.node, 'master_arm_' + side + '_deque', deque())

    def test_new_camera_and_actual_follower_data_do_not_need_master_packets(self):
        frame = self.node.get_synced_frame()
        self.assertEqual(frame[:3], ('front', 'left', 'right'))
        self.assertEqual(frame[3].position, [0.1] * 7)
        self.assertEqual(frame[4].position, [0.1] * 7)
        self.assertEqual(frame[5:], (None, None))
        self.assertIsNone(self.node.get_synced_frame(), 'consumed camera/follower frames must not be invented again')

    def test_optional_old_master_observation_is_not_popped_or_retimestamped(self):
        old = SimpleNamespace(header=SimpleNamespace(stamp=stamp(1)), position=[0.2] * 7)
        self.node.master_arm_left_deque.append(old)
        frame = self.node.get_synced_frame()
        self.assertIs(frame[5], old)
        self.assertIs(self.node.master_arm_left_deque[-1], old)
        self.assertEqual(old.header.stamp.to_sec(), 1)


class RuntimeModelTests(unittest.TestCase):
    def setUp(self):
        self.output = redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)
        self.ns, cls = load_official_methods()
        self.node = cls.__new__(cls)
        self.node._dagger_live_after_source_time = None
        self.node.consume_dagger_preroll = lambda: None
        self.node.master_handoff_lock = threading.Lock()
        self.node.master_control_feedback = {}
        self.node.master_prepared_feedback = {}
        self.node.master_command_seen_at = {}
        self.tick = 0
        self.start_calls = 0
        self.saved = []
        self.actions = []
        self.following = []
        self.on_tick = lambda tick: None
        self.on_advance = lambda: None
        self.mode = dict(config=0xFA, teaching=1, enabled=1)
        self.ns.update(
            time=SimpleNamespace(monotonic=lambda: 10 + self.tick / 30,
                                 time=lambda: 1000 + self.tick / 30), math=math,
            dagger_mode_active=True, resume_inference_requested=False, save_data_requested=False,
            collection_active=False, waiting_for_user_confirm=False, delete_data_requested=False,
            dagger_mode_lock=threading.Lock(), collection_lock=threading.Lock(),
            save_data_lock=threading.Lock(), delete_data_lock=threading.Lock(),
            rospy=SimpleNamespace(is_shutdown=lambda: False,
                                  Rate=lambda hz: SimpleNamespace(sleep=self.advance)),
            np=SimpleNamespace(zeros=lambda shape: [],
                               concatenate=lambda sequences: [value for values in sequences for value in values]),
            torch=SimpleNamespace(inference_mode=nullcontext), collections=collections,
            dm_env=SimpleNamespace(TimeStep=lambda **values: values,
                                   StepType=SimpleNamespace(FIRST=0, MID=1)),
            websocket_client_policy=SimpleNamespace(WebsocketClientPolicy=lambda *args:
                                                    SimpleNamespace(get_server_metadata=lambda: {})),
            SimpleKalmanFilter=lambda: None)
        self.node.wait_for_fresh_policy_inputs = lambda: None
        self.node.puppet_arm_publish_continuous = lambda *args: self.actions.append('startup-only')
        self.node.enter_dagger_mode = lambda: True
        self.node.set_dagger_following = self.following.append
        self.node.exit_dagger_mode = lambda: self.actions.append('explicit-exit')
        def restart():
            self.actions.append('explicit-restart')
            raise StopModel
        self.node.start_inference_collection = restart
        self.collector = self.ns['SimpleDAggerCollector']([])
        original_start = self.collector.start_collection
        def start():
            self.start_calls += 1
            original_start()
        self.collector.start_collection = start
        self.node.data_collector = self.collector
        state = SimpleNamespace(position=[0.1] * 7, velocity=[0.0] * 7, effort=[0.0] * 7)
        self.node.get_synced_frame = lambda: ('front', 'left', 'right', state, state, None, None)
        self.refresh()

    def refresh(self):
        for side in ('left', 'right'):
            self.node.master_control_status_callback(mode_message(1000 + self.tick / 30, **self.mode), side)

    def advance(self):
        self.tick += 1
        self.on_tick(self.tick)
        self.refresh()
        self.on_advance()

    def key(self, char):
        previous = self.ns['rospy']
        loops = [0]
        def done():
            loops[0] += 1
            return loops[0] > 1
        self.ns.update(
            rospy=SimpleNamespace(is_shutdown=done),
            sys=SimpleNamespace(stdin=SimpleNamespace(fileno=lambda: 0, read=lambda count: char)),
            termios=SimpleNamespace(tcgetattr=lambda stream: [], tcsetattr=lambda *args: None, TCSADRAIN=1),
            tty=SimpleNamespace(setcbreak=lambda fd: None), select=SimpleNamespace(select=lambda *args: ([0], [], [])))
        try:
            self.ns['keyboard_monitor_thread']()
        finally:
            self.ns['rospy'] = previous

    def run_model(self):
        self.ns['model_inference'](SimpleNamespace(host='fake', port=0, publish_rate=30),
                                   dict(state_dim=14, episode_len=1000, chunk_size=1),
                                   self.node, keyboard_thread_started=True)

    def test_runtime_invalid_mode_pauses_append_preserves_buffer_and_recovers_without_reset(self):
        first = []
        def tick(count):
            if count == 1:
                first.append(self.collector.timesteps[0])
                self.mode.update(config=0, teaching=0)
            elif count in (2, 3):
                self.assertEqual(self.collector.frame_count, 1)
                self.assertTrue(self.collector.is_collecting)
                self.assertTrue(self.ns['collection_active'])
                self.key(' ')
                if count == 3:
                    self.mode.update(config=0xFA, teaching=1)
            elif count == 4:
                self.assertEqual(self.collector.frame_count, 2)
                self.assertIs(self.collector.timesteps[0], first[0])
                raise StopModel
        self.on_tick = tick
        with self.assertRaises(StopModel):
            self.run_model()
        self.assertEqual(self.start_calls, 1)
        self.assertEqual(self.actions, ['startup-only'])
        self.assertTrue(self.ns['dagger_mode_active'])

    def test_legal_idle_without_control_packets_continues_recording_static_actual_pose(self):
        # Exercise the real five-stream synchronization, with a fresh actual
        # follower/camera sample each cycle and no master observations at all.
        self.node.get_synced_frame = self.ns['RosOperator'].get_synced_frame.__get__(self.node)
        self.node.bridge = SimpleNamespace(imgmsg_to_cv2=lambda message, encoding: message.payload)
        for side in ('left', 'right', 'front'):
            setattr(self.node, 'img_' + side + '_deque', deque())
        for side in ('left', 'right'):
            setattr(self.node, 'puppet_arm_' + side + '_deque', deque())
            setattr(self.node, 'master_arm_' + side + '_deque', deque())
        def fresh_data():
            source = 1000 + self.tick / 30
            for side in ('left', 'right', 'front'):
                getattr(self.node, 'img_' + side + '_deque').append(SimpleNamespace(
                    header=SimpleNamespace(stamp=stamp(source)), payload=side))
            for side in ('left', 'right'):
                getattr(self.node, 'puppet_arm_' + side + '_deque').append(SimpleNamespace(
                    header=SimpleNamespace(stamp=stamp(source)), position=[0.1] * 7,
                    velocity=[0.0] * 7, effort=[0.0] * 7))
        fresh_data()
        self.on_advance = fresh_data
        self.on_tick = lambda tick: (_ for _ in ()).throw(StopModel()) if tick == 4 else None
        with self.assertRaises(StopModel):
            self.run_model()
        self.assertEqual(self.collector.frame_count, 4)
        self.assertEqual(len(self.collector.actions), 3)
        self.assertTrue(all(action == [0.1] * 14 for action in self.collector.actions))
        self.assertEqual(self.start_calls, 1)
        self.assertEqual(self.actions, ['startup-only'])

    def test_save_keeps_valid_prefix_and_space_waits_for_mode_before_one_restart(self):
        def save(**kwargs):
            self.saved.append(list(self.collector.actions))
            self.collector.stop_collection()
            self.collector.actions = []
            self.collector.timesteps = []
        self.collector.save_current_episode = save
        def tick(count):
            if count == 2:
                self.mode.update(config=0, teaching=0)
                self.key('s')
            elif count == 3:
                self.assertEqual(len(self.saved), 1)
                self.assertEqual(len(self.saved[0]), 1)
                self.key(' ')
            elif count in (4, 5):
                self.assertEqual(self.start_calls, 1)
                self.assertFalse(self.collector.is_collecting)
                if count == 5:
                    self.mode.update(config=0xFA, teaching=1)
            elif count == 7:
                self.assertEqual(self.collector.frame_count, 2)
                raise StopModel
        self.on_tick = tick
        with self.assertRaises(StopModel):
            self.run_model()
        self.assertEqual(self.start_calls, 2)
        self.assertEqual(len(self.saved[0]), 1)
        self.assertEqual(self.actions, ['startup-only'])

    def test_explicit_r_keeps_existing_handoff_path_even_when_data_is_paused(self):
        def tick(count):
            self.mode.update(config=0, teaching=0)
            self.key('r')
        self.on_tick = tick
        with self.assertRaises(StopModel):
            self.run_model()
        self.assertEqual(self.actions, ['startup-only', 'explicit-exit', 'explicit-restart'])


if __name__ == '__main__':
    unittest.main()
