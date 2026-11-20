"""Private pedal command transport and shared action semantics, without hardware."""
from contextlib import redirect_stdout
import io
import os
import queue
import select
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from test_compatibility import load_official_methods


class PedalActionTests(unittest.TestCase):
    def setUp(self):
        self.ns, _cls = load_official_methods()
        self.ns.update(dagger_mode_active=False, collection_active=False,
                       resume_inference_requested=False, resume_inference_in_progress=False,
                       save_data_requested=False, delete_data_requested=False, waiting_for_user_confirm=False,
                       dagger_mode_lock=threading.Lock(), collection_lock=threading.Lock(),
                       save_data_lock=threading.Lock(), delete_data_lock=threading.Lock())
        self.healthy = True
        self.operator = SimpleNamespace(dagger_recording_status=lambda: (self.healthy, 'status'))
        self.output = redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)

    def pedal(self, token):
        self.ns['handle_operator_action'](token, source='pedal', ros_operator=self.operator)

    def test_left_enters_dagger_then_can_request_new_recording_without_another_mode_change(self):
        self.pedal('left')
        self.assertTrue(self.ns['dagger_mode_active'])
        self.assertFalse(self.ns['collection_active'])
        self.pedal('left')
        self.assertTrue(self.ns['collection_active'])
        self.assertFalse(self.ns['resume_inference_requested'])
        self.assertFalse(self.ns['save_data_requested'])
        self.pedal('left')
        self.assertTrue(self.ns['collection_active'])

    def test_left_stopped_recording_requires_confirmed_fa_but_keyboard_d_keeps_original_meaning(self):
        self.ns['dagger_mode_active'] = True
        self.healthy = False
        self.pedal('left')
        self.assertFalse(self.ns['collection_active'])
        self.healthy = True
        self.ns['handle_operator_action']('d', source='keyboard', ros_operator=self.operator)
        self.assertFalse(self.ns['collection_active'])
        self.pedal('left')
        self.assertTrue(self.ns['collection_active'])

    def test_each_single_pedal_only_confirms_save_window_once(self):
        for token in ('left', 'save', 'resume'):
            with self.subTest(token=token):
                self.ns.update(dagger_mode_active=True, collection_active=False,
                               resume_inference_requested=False, save_data_requested=False,
                               waiting_for_user_confirm=True, delete_data_requested=False)
                self.pedal(token)
                self.assertFalse(self.ns['waiting_for_user_confirm'])
                self.assertFalse(self.ns['collection_active'])
                self.assertFalse(self.ns['resume_inference_requested'])
                self.assertFalse(self.ns['save_data_requested'])
                self.assertFalse(self.ns['delete_data_requested'])
        self.pedal('resume')
        self.assertTrue(self.ns['resume_inference_requested'])

    def test_double_middle_delete_token_is_only_a_confirmation_window_action(self):
        self.ns['dagger_mode_active'] = True
        self.pedal('delete')
        self.assertFalse(self.ns['delete_data_requested'])
        self.assertFalse(self.ns['save_data_requested'])
        self.ns['waiting_for_user_confirm'] = True
        self.pedal('delete')
        self.assertTrue(self.ns['delete_data_requested'])
        self.assertFalse(self.ns['waiting_for_user_confirm'])
        self.ns['delete_data_requested'] = False  # The save routine consumes it.
        self.pedal('delete')
        self.assertFalse(self.ns['delete_data_requested'])

    def test_unknown_tokens_including_stop_do_not_confirm_resume_or_act(self):
        self.ns['waiting_for_user_confirm'] = True
        for token in ('stop', 'ctrlc', 'resume\n', 'unknown', '\x03'):
            self.pedal(token)
        self.assertTrue(self.ns['waiting_for_user_confirm'])
        self.assertFalse(self.ns['resume_inference_requested'])
        self.assertFalse(self.ns['delete_data_requested'])

    def test_keyboard_confirmation_and_ctrl_q_remain_available(self):
        self.ns['waiting_for_user_confirm'] = True
        self.ns['handle_operator_action']('w')
        self.assertTrue(self.ns['delete_data_requested'])
        self.ns['handle_operator_action']('\x11')
        self.assertTrue(self.ns['dagger_mode_active'])


class PrivatePedalPipeTests(unittest.TestCase):
    def setUp(self):
        self.ns, _cls = load_official_methods()
        self.actions = queue.Queue()
        self.stop = threading.Event()
        self.cbreak = threading.Event()
        self.errors = []
        self.descriptors = set()
        stdin_r, self.stdin_w = os.pipe()
        self.descriptors.add(self.stdin_w)
        self.stdin = os.fdopen(stdin_r, 'r')
        self.ns.update(sys=SimpleNamespace(stdin=self.stdin), select=select,
                       termios=SimpleNamespace(tcgetattr=lambda stream: [], tcsetattr=lambda *args: None, TCSADRAIN=1),
                       tty=SimpleNamespace(setcbreak=lambda fd: self.cbreak.set()),
                       rospy=SimpleNamespace(is_shutdown=self.stop.is_set),
                       handle_operator_action=lambda action, **kwargs: self.actions.put((action, kwargs['source'])))
        self.env = patch.dict(os.environ)
        self.env.start()
        os.environ.pop('DAGGER_PEDAL_COMMAND_FD', None)
        os.environ.pop('DAGGER_PEDAL_READY_FD', None)
        self.worker = None
        self.addCleanup(self.cleanup)

    def cleanup(self):
        self.stop.set()
        if self.worker is not None:
            self.worker.join(timeout=1)
            self.assertFalse(self.worker.is_alive())
        self.stdin.close()
        for fd in self.descriptors:
            try:
                os.close(fd)
            except OSError:
                pass
        self.env.stop()
        self.assertEqual(self.errors, [])

    def start(self, *, pipes=True, prequeued=b''):
        if pipes:
            command_r, self.command_w = os.pipe()
            self.ready_r, ready_w = os.pipe()
            self.descriptors.update((command_r, self.command_w, self.ready_r, ready_w))
            os.environ['DAGGER_PEDAL_COMMAND_FD'] = str(command_r)
            os.environ['DAGGER_PEDAL_READY_FD'] = str(ready_w)
            if prequeued:
                os.write(self.command_w, prequeued)
        def run():
            try:
                self.ns['keyboard_monitor_thread']()
            except BaseException as exc:
                self.errors.append(exc)
        self.worker = threading.Thread(target=run)
        self.worker.start()
        self.assertTrue(self.cbreak.wait(1))
        if pipes:
            self.assertTrue(select.select([self.ready_r], [], [], 1)[0], 'collector never reported keyboard readiness')
            self.assertEqual(os.read(self.ready_r, 64), b'ready\n')
            self.assertEqual(os.read(self.ready_r, 64), b'', 'readiness writer was not closed')

    def test_ready_then_fragmented_and_batched_private_tokens_reach_same_handler(self):
        self.start()
        os.write(self.command_w, b'le')
        with self.assertRaises(queue.Empty):
            self.actions.get(timeout=0.05)
        os.write(self.command_w, b'ft\nsave\nresume\ndelete\n')
        self.assertEqual([self.actions.get(timeout=1) for _ in range(4)],
                         [(token, 'pedal') for token in ('left', 'save', 'resume', 'delete')])
        os.write(self.stdin_w, b'r')
        self.assertEqual(self.actions.get(timeout=1), ('r', 'keyboard'))

    def test_pre_ready_commands_and_unknown_tokens_never_execute(self):
        self.start(prequeued=b'resume\nleft\n')
        os.write(self.command_w, b'unknown\nstop\nctrlc\n\xffleft\nleft\r\n')
        with self.assertRaises(queue.Empty):
            self.actions.get(timeout=0.05)
        os.write(self.command_w, b'left\n')
        self.assertEqual(self.actions.get(timeout=1), ('left', 'pedal'))

    def test_pipe_eof_keeps_existing_keyboard_controls_usable(self):
        self.start()
        os.close(self.command_w)
        self.descriptors.discard(self.command_w)
        os.write(self.stdin_w, b's')
        self.assertEqual(self.actions.get(timeout=1), ('s', 'keyboard'))

    def test_without_private_fds_only_original_keyboard_input_is_used(self):
        self.start(pipes=False)
        os.write(self.stdin_w, b'd')
        self.assertEqual(self.actions.get(timeout=1), ('d', 'keyboard'))


if __name__ == '__main__':
    unittest.main()
