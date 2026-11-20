"""Exit-save checks with the real capture loop and destructive HDF5/video writer.

Only camera/ROS/HDF5 endpoints are inert stand-ins; no hardware is initialized.
"""
import ast
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest

import test_inference_tail as tail


class InferenceExitSaveTests(unittest.TestCase):
    setUp = tail.InferenceTailTests.setUp
    mark = tail.InferenceTailTests.mark
    get_frame = tail.InferenceTailTests.get_frame
    cleanup_worker = tail.InferenceTailTests.cleanup_worker
    start_worker = tail.InferenceTailTests.start_worker

    def open_hdf5(self, path, *args, **kwargs):
        Path(path).touch()
        result = tail.InferenceTailTests.open_hdf5(self, path, *args, **kwargs)
        if not hasattr(self, 'saved'):
            self.saved = []
        self.saved.append((path, self.file))
        return result

    def assert_saved_prefix(self, index=0):
        file = self.saved[index][1]
        qpos = file.datasets['/observations/qpos'].values
        actions = file.datasets['/action'].values
        self.assertTrue(actions)
        self.assertEqual(actions[:-1], qpos[1:])
        self.assertEqual(len(qpos), len(actions))
        return qpos, actions

    def run_main(self, model):
        source = Path(__file__).with_name('agilex_openpi_dagger_collect.py')
        main = next(node for node in ast.parse(source.read_text()).body
                    if isinstance(node, ast.FunctionDef) and node.name == 'main')
        self.operator.args.seed = None
        self.namespace.update(get_arguments=lambda: self.operator.args,
                              RosOperator=lambda args: self.operator,
                              get_config=lambda args: {}, model_inference=model)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[main], type_ignores=[])),
                     str(source), 'exec'), self.namespace)
        self.namespace['main']()

    def test_ctrl_c_in_main_saves_the_live_prefix_and_propagates_interrupt(self):
        def interrupt(*args):
            self.assertTrue(self.started.wait(1))
            raise KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            self.run_main(interrupt)
        qpos, actions = self.assert_saved_prefix()
        self.assertEqual(len(actions), self.sample_count - 1)
        self.assertEqual(actions[-1], [self.sample_count / 1000] * 14)
        self.assertEqual([name for name, _ in self.events], ['save'])
        self.assertFalse(self.namespace['process_thread'].is_alive())
        self.assertFalse(self.operator._inference_unsaved)
        self.assertIn('Saved inference:', self.output._new_target.getvalue())
        self.assertIn(f'({len(actions)} frames)', self.output._new_target.getvalue())

    def test_normal_main_return_saves_without_an_extra_sampling_tail(self):
        elapsed = []
        def finish(*args):
            self.assertTrue(self.started.wait(1))
            elapsed.append(time.monotonic())
        self.run_main(finish)
        self.assert_saved_prefix()
        self.assertLess(time.monotonic() - elapsed[0], 0.2)
        self.assertEqual([name for name, _ in self.events], ['save'])

    def test_ros_sleep_shutdown_preserves_prefix_and_saves_without_live_ros(self):
        class ROSInterrupt(Exception):
            pass
        sleeping = threading.Event()
        def sleep():
            sleeping.set()
            self.shutdown.wait(1)
            raise ROSInterrupt('ROS shutdown')
        self.namespace['rospy'].ROSInterruptException = ROSInterrupt
        self.namespace['rospy'].Rate = lambda hz: SimpleNamespace(sleep=sleep)
        def shutdown(*args):
            self.assertTrue(sleeping.wait(1))
            self.shutdown.set()
        self.run_main(shutdown)
        _, actions = self.assert_saved_prefix()
        self.assertEqual(len(actions), 1)
        self.assertNotIn('error', self.namespace['result_container'])
        self.assertTrue(self.shutdown.is_set())
        self.assertEqual([name for name, _ in self.events], ['save'])

    def test_three_d_saves_then_final_resume_and_exit_produce_four_segments(self):
        self.operator.args.max_timesteps = 2
        for cycle in range(3):
            self.operator.start_inference_collection()
            self.namespace['process_thread'].join(timeout=1)
            self.assertTrue(self.operator.enter_dagger_mode())
            self.assertIsNone(self.operator.stop_and_save_inference())
        self.operator.start_inference_collection()  # Same fresh recording as r.
        self.namespace['process_thread'].join(timeout=1)
        final_path = self.operator.stop_and_save_inference()
        self.assertEqual([Path(path).name for path, _ in self.saved],
                         [f'episode_{i}.hdf5' for i in range(4)])
        self.assertEqual(Path(final_path).name, 'episode_3')
        self.assertIsNone(self.operator.stop_and_save_inference())
        for index in range(4):
            self.assert_saved_prefix(index)
        self.assertEqual(len(self.saved), 4)

    def test_exit_during_teaching_does_not_resave_inference_or_save_correction(self):
        self.operator.args.max_timesteps = 2
        self.operator.start_inference_collection()
        self.namespace['process_thread'].join(timeout=1)
        self.assertTrue(self.operator.enter_dagger_mode())
        correction = SimpleNamespace(actions=['unsaved correction'], timesteps=['teacher'])
        self.operator.data_collector = correction
        previous_events = list(self.events)
        self.assertIsNone(self.operator.stop_and_save_inference())
        self.assertEqual(len(self.saved), 1)
        self.assertEqual(self.events, previous_events)
        self.assertEqual(correction.actions, ['unsaved correction'])
        self.assertEqual(correction.timesteps, ['teacher'])
        self.assertEqual(list(Path(self.temporary.name).glob('*dagger_hdf5')), [])

    def test_empty_and_single_observation_do_not_create_empty_hdf5(self):
        for limit in (-1, 0):
            with self.subTest(max_timesteps=limit):
                self.operator.args.max_timesteps = limit
                self.operator.start_inference_collection()
                self.namespace['process_thread'].join(timeout=1)
                self.assertIsNone(self.operator.stop_and_save_inference())
                self.assertEqual(list(Path(self.temporary.name).rglob('*.hdf5')), [])
                self.assertFalse(self.operator._inference_unsaved)

    def test_interrupting_d_tail_still_saves_the_unconsumed_prefix_on_exit(self):
        self.start_worker()
        def interrupt(seconds):
            raise KeyboardInterrupt
        self.namespace['time'] = SimpleNamespace(monotonic=time.monotonic, time=time.time, sleep=interrupt)
        with self.assertRaises(KeyboardInterrupt):
            self.operator.enter_dagger_mode()
        self.operator.stop_and_save_inference()
        self.assert_saved_prefix()
        self.assertEqual([name for name, _ in self.events], ['save'])

    def test_interrupted_destructive_d_export_reuses_path_and_all_frames_on_exit(self):
        self.operator.args.max_timesteps = 2
        self.operator.start_inference_collection()
        self.namespace['process_thread'].join(timeout=1)
        original_frames = list(self.namespace['timesteps'])
        original_actions = list(self.namespace['actions'])
        writer = self.namespace['save_data']
        attempts = []
        def interrupted_writer(args, frames, actions, path):
            attempts.append(path)
            Path(path + '.hdf5').touch()  # A partially written prior attempt.
            frames.pop(0)
            actions.pop(0)
            raise KeyboardInterrupt
        self.namespace['save_data'] = interrupted_writer
        with self.assertRaises(KeyboardInterrupt):
            self.operator.enter_dagger_mode()
        self.assertEqual(self.namespace['timesteps'], original_frames)
        self.assertEqual(self.namespace['actions'], original_actions)
        self.assertTrue(self.operator._inference_unsaved)
        def retry(args, frames, actions, path):
            attempts.append(path)
            writer(args, frames, actions, path)
        self.namespace['save_data'] = retry
        self.operator.stop_and_save_inference()
        self.assertEqual(attempts[0], attempts[1])
        self.assertEqual(len(self.saved), 1)
        self.assertEqual(len(self.saved[0][1].datasets['/action'].values), len(original_actions))
        self.assert_saved_prefix()
        self.assertFalse(self.operator._inference_unsaved)
        self.assertEqual(len(list(Path(self.temporary.name).rglob('*.hdf5'))), 1)
        self.assertEqual([name for name, _ in self.events], ['save'])

    def test_real_capture_error_is_reported_and_not_silently_saved_as_normal_exit(self):
        def fail():
            raise ValueError('camera failed')
        self.operator.process = fail
        self.operator.start_inference_collection()
        self.namespace['process_thread'].join(timeout=1)
        with self.assertRaisesRegex(RuntimeError, 'Inference collection failed') as error:
            self.operator.stop_and_save_inference()
        self.assertIsInstance(error.exception.__cause__, ValueError)
        self.assertEqual(list(Path(self.temporary.name).rglob('*.hdf5')), [])
        self.assertEqual(self.events, [])

    def test_live_capture_is_never_written_without_joining(self):
        self.start_worker()
        with self.assertRaisesRegex(RuntimeError, 'Stop inference collection'):
            self.operator.save_inference_collection(self.namespace['timesteps'], self.namespace['actions'])
        self.assertEqual(self.events, [])
        self.assertEqual(list(Path(self.temporary.name).rglob('*.hdf5')), [])


if __name__ == '__main__':
    unittest.main()
