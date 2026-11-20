"""Save and restart exercise the real collector loop, controls, and config helper.

Only in-memory ROS publishers, a fake policy, and a temporary save are used.
No robot, ROS node, CAN device, or SDK is initialized.
"""
from collections import deque
from contextlib import nullcontext, redirect_stdout
import io
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from test_compatibility import load_official_methods


class StopModel(Exception):
    pass


class SaveFollowingPauseTests(unittest.TestCase):
    def setUp(self):
        output = redirect_stdout(io.StringIO())
        output.__enter__()
        self.addCleanup(output.__exit__, None, None, None)
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.ns, operator_class = load_official_methods()
        self.node = operator_class.__new__(operator_class)
        self.node.consume_dagger_preroll = lambda: None
        self.events = []
        self.tick = 0
        self.on_tick = lambda _tick: None
        self.confirm_action = ('x', 'keyboard')
        self.ready = True
        self.ns.update(
            dagger_mode_active=True, resume_inference_requested=False,
            save_data_requested=False, collection_active=False,
            waiting_for_user_confirm=False, delete_data_requested=False,
            dagger_mode_lock=threading.Lock(), collection_lock=threading.Lock(),
            save_data_lock=threading.Lock(), delete_data_lock=threading.Lock(),
            lang_embeddings='test', String=SimpleNamespace,
            np=SimpleNamespace(zeros=lambda _shape: []),
            torch=SimpleNamespace(inference_mode=nullcontext),
            rospy=SimpleNamespace(is_shutdown=lambda: False,
                                  Rate=lambda _hz: SimpleNamespace(sleep=self.advance)),
            websocket_client_policy=SimpleNamespace(WebsocketClientPolicy=lambda *args:
                                                    SimpleNamespace(get_server_metadata=lambda: {})),
            SimpleKalmanFilter=lambda: None,
            stream_buffer=SimpleNamespace(lock=threading.Lock(), cur_chunk=deque(),
                                          chunks=deque(), last_action=None, k=0,
                                          pop_next_action=lambda: None))
        for side in ('left', 'right'):
            setattr(self.node, 'master_' + side + '_config_pub', SimpleNamespace(
                publish=lambda message, side=side: self.events.append((message.data, side))))
        self.node.wait_for_fresh_policy_inputs = lambda: None
        self.node.puppet_arm_publish_continuous = lambda *args: None
        self.node.enter_dagger_mode = lambda: self.events.append(('enter',)) or True
        self.node.dagger_recording_ready = lambda: self.ready
        self.node.dagger_recording_status = lambda: (self.ready, 'status')
        self.node.get_synced_frame = lambda: None
        self.node.exit_dagger_mode = lambda: self.events.append(('exit',))
        self.node.start_inference_collection = lambda: self.events.append(('inference-start',))

        self.collector = self.ns['SimpleDAggerCollector'](['front'], dataset_dir=directory.name)
        self.collector.full_dataset_dir = directory.name
        self.node.data_collector = self.collector
        start, stop, has_data = (self.collector.start_collection,
                                 self.collector.stop_collection, self.collector.has_data)
        def start_collection():
            self.events.append(('record-start',))
            start()
        def stop_collection():
            self.events.append(('record-stop',))
            stop()
        def has_episode():
            self.events.append(('has-data',))
            return has_data()
        self.collector.start_collection = start_collection
        self.collector.stop_collection = stop_collection
        self.collector.has_data = has_episode
        self.ns['save_data'] = self.save_data

    def save_data(self, _args, _timesteps, _actions, path):
        self.assert_paused()
        self.events.append(('save',))
        Path(path + '.hdf5').touch()

    def action(self, char, source='keyboard'):
        self.ns['handle_operator_action'](char, source, self.node)

    def seed_data(self):
        self.collector.actions[:] = ['action']
        self.collector.timesteps[:] = ['first', 'second']
        self.collector.frame_count = 2

    def assert_paused(self):
        self.assertFalse(self.collector.is_collecting)
        self.assertFalse(self.ns['collection_active'])
        self.assertEqual([event for event in self.events if event[0].endswith('_following')],
                         [('pause_following', 'left'), ('pause_following', 'right')])

    def confirm_save(self, _seconds):
        self.assertTrue(self.ns['waiting_for_user_confirm'])
        self.assert_paused()
        self.events.append(('confirmation',))
        self.action(*self.confirm_action)
        self.assert_paused()

    def advance(self):
        self.tick += 1
        self.assertLess(self.tick, 10, 'model simulation did not terminate')
        self.on_tick(self.tick)

    def run_model(self, on_tick):
        self.on_tick = on_tick
        args = SimpleNamespace(host='fake', port=0, publish_rate=30, use_temporal_smoothing=True)
        with patch('time.sleep', self.confirm_save), self.assertRaises(StopModel):
            self.ns['model_inference'](args, dict(state_dim=14, episode_len=100, chunk_size=1),
                                       self.node, keyboard_thread_started=True)

    def request_save(self, source='keyboard', data=True):
        self.assertEqual(self.events, [('enter',), ('record-start',)])
        self.events.clear()
        if data:
            self.seed_data()
        self.action('s' if source == 'keyboard' else 'save', source)
        # Requests alone never change forwarding or stop the active collector.
        self.assertEqual(self.events, [])
        self.assertTrue(self.collector.is_collecting)

    def test_helper_sends_the_existing_config_command_to_both_masters(self):
        self.node.set_dagger_following(False)
        self.node.set_dagger_following(True)
        self.assertEqual(self.events, [('pause_following', 'left'), ('pause_following', 'right'),
                                       ('resume_following', 'left'), ('resume_following', 'right')])

    def test_keyboard_and_middle_pedal_pause_before_stop_data_check_and_save(self):
        for source in ('keyboard', 'pedal'):
            with self.subTest(source=source):
                self.setUp()
                def tick(count):
                    if count == 1:
                        self.request_save(source)
                    else:
                        self.assert_paused()
                        self.assertEqual(self.events[:5], [('pause_following', 'left'),
                            ('pause_following', 'right'), ('record-stop',), ('has-data',), ('has-data',)])
                        self.assertLess(self.events.index(('record-stop',)), self.events.index(('save',)))
                        self.assertLess(self.events.index(('save',)), self.events.index(('confirmation',)))
                        self.assertFalse(self.ns['save_data_requested'])
                        raise StopModel
                self.run_model(tick)

    def test_confirmation_or_delete_keeps_following_paused_and_does_not_restart(self):
        for confirmation in ((' ', 'keyboard'), ('left', 'pedal'), ('delete', 'pedal')):
            with self.subTest(confirmation=confirmation):
                self.setUp()
                self.confirm_action = confirmation
                def tick(count):
                    if count == 1:
                        self.request_save()
                    elif count == 4:
                        self.assert_paused()
                        self.assertFalse(self.ns['waiting_for_user_confirm'])
                        self.assertFalse(self.collector.has_data())
                        self.assertEqual(self.collector.episode_idx, 0 if confirmation[0] == 'delete' else 1)
                        self.assertEqual(list(Path(self.collector.full_dataset_dir).glob('*.hdf5')) != [],
                                         confirmation[0] != 'delete')
                        self.assertNotIn(('record-start',), self.events)
                        raise StopModel
                self.run_model(tick)

    def test_empty_save_also_pauses_and_ends_recording(self):
        def tick(count):
            if count == 1:
                self.request_save(data=False)
            elif count == 3:
                self.assert_paused()
                self.assertNotIn(('save',), self.events)
                self.assertNotIn(('record-start',), self.events)
                raise StopModel
        self.run_model(tick)

    def test_space_and_left_pedal_resume_only_when_the_new_recording_starts(self):
        for source in ('keyboard', 'pedal'):
            with self.subTest(source=source):
                self.setUp()
                def tick(count):
                    if count == 1:
                        self.request_save()
                    elif count == 2:
                        self.assert_paused()
                        self.action(' ' if source == 'keyboard' else 'left', source)
                        self.assertTrue(self.ns['collection_active'])
                        self.assertFalse(self.collector.is_collecting)
                        self.assertNotIn(('resume_following', 'left'), self.events)
                    elif count == 3:
                        self.assertEqual(self.events[-3:], [('resume_following', 'left'),
                            ('resume_following', 'right'), ('record-start',)])
                        self.assertTrue(self.collector.is_collecting)
                        self.assertTrue(self.ns['collection_active'])
                        raise StopModel
                self.run_model(tick)

    def test_unsaved_data_rejects_restart_without_resuming_following(self):
        def failed_save(**_kwargs):
            self.assert_paused()
            self.events.append(('save-failed',))
            return False
        self.collector.save_current_episode = failed_save
        def tick(count):
            if count == 1:
                self.request_save()
            elif count == 2:
                self.action(' ')
                self.assertTrue(self.ns['collection_active'])
            elif count == 4:
                self.assert_paused()
                self.assertEqual(self.collector.actions, ['action'])
                self.assertNotIn(('record-start',), self.events)
                raise StopModel
        self.run_model(tick)

    def test_pending_space_waits_for_teaching_status_before_resuming(self):
        def tick(count):
            if count == 1:
                self.request_save()
            elif count == 2:
                self.ready = False
                self.action(' ')
            elif count in (3, 4):
                self.assertTrue(self.ns['collection_active'])
                self.assertFalse(self.collector.is_collecting)
                self.assertNotIn(('resume_following', 'left'), self.events)
                if count == 4:
                    self.ready = True
            elif count == 5:
                self.assertEqual(self.events[-3:], [('resume_following', 'left'),
                    ('resume_following', 'right'), ('record-start',)])
                raise StopModel
        self.run_model(tick)

    def test_r_uses_the_existing_fc_and_policy_resume_path_without_gate_resume(self):
        def tick(count):
            if count == 1:
                self.request_save()
            elif count == 2:
                self.assert_paused()
                self.action('r')
            elif count == 3:
                self.assertIn(('exit',), self.events)
                self.assertIn(('inference-start',), self.events)
                self.assertFalse(self.ns['dagger_mode_active'])
                self.assertFalse(self.ns['collection_active'])
                self.assertNotIn(('resume_following', 'left'), self.events)
                self.assertNotIn(('resume_following', 'right'), self.events)
                self.assertEqual(self.ns['inference_generation'], 1)
                raise StopModel
        self.run_model(tick)


if __name__ == '__main__':
    unittest.main()
