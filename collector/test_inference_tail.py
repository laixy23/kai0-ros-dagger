"""Real capture-worker checks for the d-only inference recording tail.

Execute production methods without importing ROS or opening cameras/CAN. The
production save_data writer uses an in-memory HDF5 stand-in, so its destructive
pairing of timesteps/actions is tested without optional h5py dependencies.
"""
import ast
import collections
from contextlib import redirect_stdout
import io
import os
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

from test_compatibility import load_official_methods


class MemoryHDF5:
    def __init__(self):
        self.attrs = {}
        self.datasets = {}

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def create_group(self, name):
        return SimpleNamespace(create_dataset=lambda key, shape:
                               self.create_dataset(name + '/' + key, shape))

    def create_dataset(self, name, shape):
        rows = SimpleNamespace(shape=shape, values=None)
        self.datasets['/' + name.lstrip('/')] = rows
        return rows

    def __getitem__(self, name):
        rows = self.datasets['/' + name.lstrip('/')]

        class Dataset:
            def __setitem__(self, key, values):
                rows.values = [list(value) for value in values]

        return Dataset()


class InferenceTailTests(unittest.TestCase):
    def setUp(self):
        self.output = redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.namespace, operator_class = load_official_methods()
        self.operator = operator_class.__new__(operator_class)
        self.operator.start_dagger_preroll = lambda: None
        self.operator.finish_dagger_preroll = lambda _ready_at: None
        self.operator.stop_flag = False
        self.operator.args = SimpleNamespace(
            dataset_dir=self.temporary.name, dataset_name='tail', camera_names=['front', 'left', 'right'],
            export_video=True, frame_rate=100, max_timesteps=10000,
            use_depth_image=False, use_robot_base=False, video_fps=30,
            video_codec='libx264', video_quality=23)
        self.shutdown = threading.Event()
        self.started = threading.Event()
        self.sample_count = 0
        self.events = []
        self.file = None
        self.namespace.update(
            rospy=SimpleNamespace(is_shutdown=self.shutdown.is_set,
                                  Rate=lambda hz: SimpleNamespace(sleep=lambda: time.sleep(1 / hz)),
                                  Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=time.time)),
                                  loginfo=lambda *args: None),
            collections=collections,
            np=SimpleNamespace(concatenate=lambda values: list(values[0]) + list(values[1]),
                               array_equal=lambda left, right: left == right, uint8='uint8'),
            cv2=SimpleNamespace(cvtColor=lambda image, conversion: image, COLOR_BGR2RGB=0),
            create_video_from_images=lambda *args, **kwargs: None,
            dm_env=SimpleNamespace(StepType=SimpleNamespace(FIRST=0, MID=1),
                                   TimeStep=lambda kind, reward, discount, obs:
                                   SimpleNamespace(observation=obs)),
            h5py=SimpleNamespace(File=self.open_hdf5))
        source = Path(__file__).with_name('agilex_openpi_dagger_collect.py')
        operator_node = next(node for node in ast.parse(source.read_text()).body
                             if isinstance(node, ast.ClassDef) and node.name == 'RosOperator')
        process = next(node for node in operator_node.body
                       if isinstance(node, ast.FunctionDef) and node.name == 'process')
        writer = Path(__file__).with_name('collect_data.py')
        save = next(node for node in ast.parse(writer.read_text()).body
                    if isinstance(node, ast.FunctionDef) and node.name == 'save_data')
        exec(compile(ast.fix_missing_locations(ast.Module(body=[process, save], type_ignores=[])),
                     str(source), 'exec'), self.namespace)
        operator_class.process = self.namespace['process']
        self.operator.get_frame = self.get_frame
        self.operator.enable_master_arms = lambda: self.events.append(('enable', time.monotonic()))
        self.operator.move_masters_to_safe_pose = lambda **kwargs: self.mark('intermediate')
        self.operator.move_masters_to_slave_pose = lambda **kwargs: self.mark('align')
        self.operator.release_prepared_masters = lambda: self.events.append(('release', time.monotonic()))
        self.operator.wait_for_master_control = lambda *args, **kwargs: self.events.append(('ack', time.monotonic()))
        self.operator.puppet_arm_publish = lambda *args: self.fail('tail emitted a follower target')
        self.operator.endpose_publish = lambda *args: self.fail('tail emitted an end pose')
        self.addCleanup(self.cleanup_worker)

    def mark(self, name):
        self.events.append((name, time.monotonic()))
        return True

    def open_hdf5(self, *args, **kwargs):
        self.assertTrue(self.operator.stop_flag)
        self.assertFalse(self.namespace['process_thread'].is_alive())
        self.events.append(('save', time.monotonic()))
        self.file = MemoryHDF5()
        return self.file

    def get_frame(self):
        self.sample_count += 1
        value = self.sample_count / 1000
        joint = SimpleNamespace(position=[value] * 7, velocity=[0.0] * 7, effort=[0.0] * 7)
        if self.sample_count >= 3:
            self.started.set()
        image = SimpleNamespace(dtype='uint8')
        return image, image, image, None, None, None, joint, joint, None

    def start_worker(self):
        self.operator.start_inference_collection()
        self.assertTrue(self.started.wait(1), 'capture worker did not produce observations')

    def cleanup_worker(self):
        self.operator.stop_flag = True
        thread = self.namespace.get('process_thread')
        if isinstance(thread, threading.Thread):
            thread.join(timeout=1)
            self.assertFalse(thread.is_alive(), 'capture worker leaked')

    def test_d_samples_one_second_of_new_states_before_save_and_master_preparation(self):
        self.start_worker()
        samples_before_d = self.sample_count
        requested = time.monotonic()
        self.assertTrue(self.operator.enter_dagger_mode())
        names = [name for name, _ in self.events]
        self.assertEqual(names, ['save', 'enable', 'intermediate', 'align', 'release', 'ack'])
        self.assertGreaterEqual(self.events[0][1] - requested, 0.99)
        self.assertGreater(self.sample_count, samples_before_d)
        qpos = self.file.datasets['/observations/qpos'].values
        actions = self.file.datasets['/action'].values
        self.assertEqual(len(actions), self.sample_count - 1)
        self.assertEqual(actions[:-1], qpos[1:])
        self.assertEqual(actions[-1], [self.sample_count / 1000] * 14)
        self.assertGreater(actions[-1][0], samples_before_d / 1000)
        self.assertFalse(self.namespace['process_thread'].is_alive())

    def test_default_stop_is_immediate_and_keeps_the_real_prefix(self):
        self.start_worker()
        requested = time.monotonic()
        frames, actions = self.operator.stop_inference_collection()
        self.assertLess(time.monotonic() - requested, 0.2)
        self.assertTrue(self.operator.stop_flag)
        self.assertEqual(len(frames), len(actions) + 1)
        self.assertTrue(actions)
        self.assertEqual(self.events, [])

    def test_already_finished_worker_does_not_wait_or_invent_frames(self):
        self.operator.args.max_timesteps = 2
        self.operator.start_inference_collection()
        self.namespace['process_thread'].join(timeout=1)
        self.assertFalse(self.namespace['process_thread'].is_alive())
        requested = time.monotonic()
        frames, actions = self.operator.stop_inference_collection(tail_seconds=1.0)
        self.assertLess(time.monotonic() - requested, 0.2)
        self.assertEqual(len(frames), 3)
        self.assertEqual(len(actions), 2)

    def test_worker_failure_aborts_tail_without_save_or_master_commands(self):
        def fail():
            raise ValueError('camera failed')
        self.operator.process = fail
        self.operator.start_inference_collection()
        self.namespace['process_thread'].join(timeout=1)
        requested = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, 'Inference collection failed'):
            self.operator.enter_dagger_mode()
        self.assertLess(time.monotonic() - requested, 0.2)
        self.assertTrue(self.operator.stop_flag)
        self.assertEqual(self.events, [])

    def test_shutdown_interrupts_active_tail_and_blocks_save_or_master_commands(self):
        self.start_worker()
        cancellation = threading.Timer(0.05, self.shutdown.set)
        cancellation.start()
        self.addCleanup(cancellation.join)
        requested = time.monotonic()
        self.assertFalse(self.operator.enter_dagger_mode())
        self.assertLess(time.monotonic() - requested, 0.3)
        self.assertTrue(self.operator.stop_flag)
        self.assertFalse(self.namespace['process_thread'].is_alive())
        self.assertEqual(self.events, [])

    def test_shutdown_during_save_blocks_master_commands(self):
        self.operator.args.max_timesteps = 2
        self.operator.start_inference_collection()
        self.namespace['process_thread'].join(timeout=1)
        writer = self.namespace['save_data']
        def save_and_shutdown(*args):
            writer(*args)
            self.shutdown.set()
        self.namespace['save_data'] = save_and_shutdown
        self.assertFalse(self.operator.enter_dagger_mode())
        self.assertEqual([name for name, _ in self.events], ['save'])

    def test_keyboard_interrupt_during_tail_sets_stop_and_propagates_without_commands(self):
        self.start_worker()
        def interrupt(seconds):
            raise KeyboardInterrupt
        self.namespace['time'] = SimpleNamespace(monotonic=time.monotonic, sleep=interrupt)
        with self.assertRaises(KeyboardInterrupt):
            self.operator.enter_dagger_mode()
        self.assertTrue(self.operator.stop_flag)
        self.assertEqual(self.events, [])

    def test_stuck_worker_still_has_bounded_join_and_never_enters_teaching(self):
        clock = [0.0]
        joins = []
        self.namespace['process_thread'] = SimpleNamespace(is_alive=lambda: True,
                                                         join=lambda timeout: joins.append(timeout))
        self.namespace['time'] = SimpleNamespace(monotonic=lambda: clock[0],
                                                sleep=lambda seconds: clock.__setitem__(0, clock[0] + seconds))
        with self.assertRaisesRegex(RuntimeError, 'did not stop'):
            self.operator.enter_dagger_mode()
        self.assertEqual(joins, [5.0])
        self.assertAlmostEqual(clock[0], 1.0)
        self.assertTrue(self.operator.stop_flag)
        self.assertEqual(self.events, [])


if __name__ == '__main__':
    unittest.main()
