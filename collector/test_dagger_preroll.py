"""Exercise the real pre-roll worker and episode handoff without ROS or hardware."""
import ast
import collections
from contextlib import redirect_stdout
import io
import math
import os
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

SOURCE = Path(__file__).with_name('agilex_openpi_dagger_collect.py')


def load_methods():
    tree = ast.parse(SOURCE.read_text())
    nodes = [node for node in tree.body if isinstance(node, ast.ClassDef)
             and node.name in ('SimpleDAggerCollector', 'RosOperator')]
    namespace = dict(os=os, math=math, time=time, threading=threading,
                     collections=collections, deque=collections.deque,
                     process_thread=None, result_container={}, timesteps=[], actions=[])
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])),
                 str(SOURCE), 'exec'), namespace)
    return namespace


class DaggerPrerollTests(unittest.TestCase):
    def setUp(self):
        output = redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.ns = load_methods()
        self.shutdown = threading.Event()
        self.ros_clock = [100.0]
        self.ns.update(
            rospy=SimpleNamespace(is_shutdown=self.shutdown.is_set, loginfo=lambda *args: None,
                                  Time=SimpleNamespace(now=lambda: SimpleNamespace(
                                      to_sec=lambda: self.ros_clock[0]))),
            np=SimpleNamespace(concatenate=lambda values: list(values[0]) + list(values[1]),
                               uint8='uint8'),
            dm_env=SimpleNamespace(StepType=SimpleNamespace(FIRST=0, MID=1),
                                   TimeStep=lambda **values: SimpleNamespace(**values)))
        cls = self.ns['RosOperator']
        self.node = cls.__new__(cls)
        self.node.args = SimpleNamespace(publish_rate=500, dataset_name='preroll')
        self.node._dagger_preroll = None
        self.node._pending_dagger_preroll = []
        self.node._dagger_live_after_source_time = None
        self.node.data_collector = self.ns['SimpleDAggerCollector'](
            ['cam_high', 'cam_left_wrist', 'cam_right_wrist'], dataset_dir=temporary.name)
        self.node.bridge = SimpleNamespace(imgmsg_to_cv2=lambda message, _encoding: message.image)
        for name in ('img_left', 'img_right', 'img_front', 'puppet_arm_left',
                     'puppet_arm_right', 'master_arm_left', 'master_arm_right'):
            setattr(self.node, name + '_deque', collections.deque())
        self.addCleanup(self.cleanup_worker)

    def cleanup_worker(self):
        state = self.node._dagger_preroll
        if state is not None:
            state['stop'].set()
            state['thread'].join(timeout=1)
            self.assertFalse(state['thread'].is_alive(), 'pre-roll capture worker leaked')

    def wait_until(self, predicate):
        deadline = time.monotonic() + 1
        while not predicate() and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertTrue(predicate(), 'pre-roll worker did not produce the expected sample')

    def messages(self, source, value=None):
        value = source if value is None else value
        stamp = SimpleNamespace(to_sec=lambda: source)
        image = SimpleNamespace(dtype='uint8', source=source)
        camera = SimpleNamespace(header=SimpleNamespace(stamp=stamp), image=image)
        joint = SimpleNamespace(header=SimpleNamespace(stamp=stamp),
                                position=[value] * 7, velocity=[value + 1] * 7,
                                effort=[value + 2] * 7)
        return camera, joint

    def append_streams(self, source, value=None):
        camera, joint = self.messages(source, value)
        for name in ('img_left', 'img_right', 'img_front'):
            getattr(self.node, name + '_deque').append(camera)
        for name in ('puppet_arm_left', 'puppet_arm_right'):
            getattr(self.node, name + '_deque').append(joint)

    def sample(self, source, phase=None):
        camera, joint = self.messages(source)
        observation, action = self.node._build_dagger_sample(
            (camera.image, camera.image, camera.image, joint, joint, None, None))
        observation['phase'] = phase
        return source, observation, action

    def pending_sample(self, sampled_at, source):
        return (sampled_at,) + self.sample(source)

    def test_snapshot_preserves_probe_feedback_and_uses_actual_follower_schema(self):
        for source in (1.0, 2.0, 3.0):
            self.append_streams(source, source / 100)
        names = ('img_left', 'img_right', 'img_front', 'puppet_arm_left', 'puppet_arm_right')
        before = {name: tuple(getattr(self.node, name + '_deque')) for name in names}
        source, observation, action = self.node._snapshot_dagger_sample()
        self.assertEqual(source, 3.0)
        self.assertEqual(observation['qpos'], [0.03] * 14)
        self.assertEqual(action, [0.03] * 14)
        self.assertEqual(observation['qvel'], [1.03] * 14)
        self.assertEqual(observation['effort'], [2.03] * 14)
        self.assertEqual(set(observation['images']), {'cam_high', 'cam_left_wrist', 'cam_right_wrist'})
        for name in names:
            self.assertEqual(tuple(getattr(self.node, name + '_deque')), before[name])
        self.assertIs(self.node.puppet_arm_left_deque[-1], before['puppet_arm_left'][-1])
        self.assertIs(self.node.puppet_arm_right_deque[-1], before['puppet_arm_right'][-1])

    def test_real_worker_keeps_only_the_second_before_ready_and_deduplicates_sources(self):
        clock, calls = [0.0], [0]
        self.ns['time'] = SimpleNamespace(monotonic=lambda: clock[0])
        def snapshot():
            calls[0] += 1
            source = min(calls[0], 25) / 10
            clock[0] = 1000.0 + source * 3
            return self.sample(source)
        self.node._snapshot_dagger_sample = snapshot
        self.node.start_dagger_preroll()
        state = self.node._dagger_preroll
        self.wait_until(lambda: calls[0] >= 28)
        self.node.finish_dagger_preroll(ready_at=2.5)
        self.assertFalse(state['thread'].is_alive())
        self.assertIsNone(self.node._dagger_preroll)
        samples = self.node._pending_dagger_preroll
        self.assertEqual([row[1] for row in samples], [value / 10 for value in range(15, 26)])
        self.assertTrue(all(1.5 <= row[1] <= 2.5 for row in samples))
        self.assertTrue(all(row[0] > 1000 for row in samples), 'ROS and monotonic clocks were mixed')
        self.assertEqual(len({row[1] for row in samples}), len(samples))

    def test_finish_anchors_to_original_ready_time_not_first_release_or_latest_sample(self):
        joined = []
        self.node._dagger_preroll = {
            'stop': threading.Event(),
            'thread': SimpleNamespace(join=lambda: joined.append(True)),
            'samples': collections.deque(self.pending_sample(1000 + source, source)
                                         for source in (0.5, 1.0, 1.5, 2.0, 2.5, 2.6))}
        self.node.finish_dagger_preroll(ready_at=2.5)
        self.assertEqual(joined, [True])
        self.assertEqual([row[1] for row in self.node._pending_dagger_preroll], [1.5, 2.0, 2.5])

    def test_prefix_is_consumed_once_and_live_handoff_preserves_next_state_pairing(self):
        self.node._pending_dagger_preroll = [self.pending_sample(10.0, 1.0),
                                             self.pending_sample(10.5, 2.0)]
        collector = self.node.data_collector
        collector.start_collection()
        self.node.consume_dagger_preroll()
        self.assertEqual(collector.frame_count, 2)
        self.assertEqual(len(collector.actions), 1)
        self.assertEqual(self.node._pending_dagger_preroll, [])
        self.assertEqual(self.node._dagger_live_after_source_time, 2.0)
        self.node.consume_dagger_preroll()
        self.assertEqual(collector.frame_count, 2)
        self.append_streams(2.0)
        self.assertIsNone(self.node.get_synced_frame(), 'prefix last source was duplicated live')
        self.append_streams(3.0)
        synced = self.node.get_synced_frame()
        observation, action = self.node._build_dagger_sample(synced)
        collector.add_frame(observation, action)
        qpos = [frame.observation['qpos'] for frame in collector.timesteps[:-1]]
        self.assertEqual(qpos, [[1.0] * 14, [2.0] * 14])
        self.assertEqual(collector.actions, [[2.0] * 14, [3.0] * 14])
        self.assertEqual(collector.actions[:-1], qpos[1:])

    def prepare_entry(self):
        events, phase, count = [], ['before-entry'], [0]
        def snapshot():
            count[0] += 1
            self.ros_clock[0] = 100.0 + count[0] / 100
            return self.sample(self.ros_clock[0], phase[0])
        self.node._snapshot_dagger_sample = snapshot
        self.node.stop_inference_collection = lambda **kwargs: (
            events.append(('inference-stop', kwargs)) or (['observation'], ['action']))
        self.node.save_inference_collection = lambda *args: events.append(('inference-save',))
        self.node.enable_master_arms = lambda: events.append(('enable',))
        self.node.move_masters_to_safe_pose = lambda **kwargs: events.append(('safe', kwargs)) or True
        def align(**kwargs):
            events.append(('align', kwargs))
            phase[0] = 'final-hold'
            current = count[0]
            self.wait_until(lambda: count[0] >= current + 3)
            return True
        self.node.move_masters_to_slave_pose = align
        self.node.release_prepared_masters = lambda: events.append(('release',))
        def acknowledge(*args, **kwargs):
            events.append(('ack-wait', args, kwargs))
            phase[0] = 'FA-wait'
            current = count[0]
            self.wait_until(lambda: count[0] >= current + 3)
            events.append(('ready',))
        self.node.wait_for_master_control = acknowledge
        return events

    def test_entry_samples_during_existing_hold_and_fa_wait_without_resetting_episode(self):
        events = self.prepare_entry()
        self.assertTrue(self.node.enter_dagger_mode())
        self.assertIsNone(self.node._dagger_preroll)
        samples = self.node._pending_dagger_preroll
        phases = [row[2]['phase'] for row in samples]
        self.assertIn('final-hold', phases)
        self.assertIn('FA-wait', phases)
        self.assertEqual([event[0] for event in events],
                         ['inference-stop', 'inference-save', 'enable', 'safe', 'align',
                          'release', 'ack-wait', 'ready'])
        self.assertEqual(events[0][1], {'tail_seconds': 1.0})
        self.assertEqual(events[4][1], {'duration': 3.0, 'prepare_following': True})
        self.assertEqual(events[6][2], {'timeout': None})
        self.assertFalse(self.node.data_collector.is_collecting)
        self.node.data_collector.start_collection()
        self.node.consume_dagger_preroll()
        self.assertEqual(self.node.data_collector.frame_count, len(samples))
        self.assertEqual(len(self.node.data_collector.actions), len(samples) - 1)

    def test_failed_entry_discards_prefix_and_joins_worker(self):
        self.prepare_entry()
        self.node.move_masters_to_slave_pose = lambda **kwargs: False
        self.assertFalse(self.node.enter_dagger_mode())
        self.assertIsNone(self.node._dagger_preroll)
        self.assertEqual(self.node._pending_dagger_preroll, [])
        self.assertFalse(self.node.data_collector.is_collecting)

    def test_keyboard_interrupt_during_entry_discards_prefix_and_joins_worker(self):
        self.prepare_entry()
        def interrupt():
            raise KeyboardInterrupt
        self.node.release_prepared_masters = interrupt
        with self.assertRaises(KeyboardInterrupt):
            self.node.enter_dagger_mode()
        self.assertIsNone(self.node._dagger_preroll)
        self.assertEqual(self.node._pending_dagger_preroll, [])

    def test_shutdown_stops_worker_and_failed_entry_does_not_commit_prefix(self):
        self.prepare_entry()
        original = self.node.move_masters_to_slave_pose
        def align(**kwargs):
            result = original(**kwargs)
            self.shutdown.set()
            return result
        self.node.move_masters_to_slave_pose = align
        self.assertFalse(self.node.enter_dagger_mode())
        self.assertIsNone(self.node._dagger_preroll)
        self.assertEqual(self.node._pending_dagger_preroll, [])

    def test_stopped_then_restarted_episode_does_not_replay_the_entry_prefix(self):
        self.node._pending_dagger_preroll = [self.pending_sample(10.0, 1.0),
                                             self.pending_sample(10.5, 2.0)]
        collector = self.node.data_collector
        collector.start_collection()
        self.node.consume_dagger_preroll()
        collector.stop_collection()
        # The existing s/Space restart starts this same collector again; it
        # must not launch another pre-roll worker or reuse consumed samples.
        collector.start_collection()
        self.node.consume_dagger_preroll()
        self.assertIsNone(self.node._dagger_preroll)
        self.assertEqual(collector.frame_count, 0)
        self.assertEqual(collector.actions, [])
        _, observation, action = self.sample(3.0)
        collector.add_frame(observation, action)
        self.assertEqual(collector.frame_count, 1)
        self.assertEqual(collector.actions, [])


if __name__ == '__main__':
    unittest.main()
