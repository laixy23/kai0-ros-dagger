"""Hardware-free checks of the patched official methods (stdlib only).

AST extraction deliberately avoids importing ROS, pynput, or the driver entrypoint.
No fake module is installed into the runnable collector's import path.
"""
import ast
from collections import deque
from contextlib import nullcontext, redirect_stdout
import hashlib
import io
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / "agilex_openpi_dagger_collect.py"


def load_official_methods():
    tree = ast.parse(SOURCE.read_text())
    wanted = {"__init__", "start_inference_collection", "stop_inference_collection", "save_inference_collection", "stop_and_save_inference", "enter_dagger_mode", "get_frame", "get_frame_peek", "get_synced_frame", "wait_for_master_control", "exit_dagger_mode", "master_command_callback", "master_control_status_callback"}
    wanted.update({"record_policy_input_stamp", "require_fresh_policy_inputs", "wait_for_fresh_policy_inputs",
                   "puppet_arm_publish", "endpose_publish", "robot_base_publish", "puppet_arm_publish_continuous",
                   "img_left_callback", "img_right_callback", "img_front_callback", "puppet_arm_left_callback", "puppet_arm_right_callback"})
    wanted.update({"move_masters_to_safe_pose", "move_masters_to_slave_pose", "_move_masters_to_pose"})
    wanted.update({'_masters_prepared_current', '_check_preparation_cancelled', '_prepare_following_worker',
                   '_start_following_preparation', '_following_preparation_guard', '_following_preparation_ready',
                   '_hold_final_pose_and_prepare', 'release_prepared_masters'})
    wanted.update({'dagger_recording_status', 'dagger_recording_ready', '_master_handoff_diagnostics'})
    wanted.add('set_dagger_following')
    wanted.update({'_build_dagger_sample', '_snapshot_dagger_sample', 'start_dagger_preroll',
                   'finish_dagger_preroll', 'consume_dagger_preroll'})
    nodes = []
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "SimpleDAggerCollector":
            nodes.append(node)
        elif isinstance(node, ast.ClassDef) and node.name == "RosOperator":
            node.body = [method for method in node.body if isinstance(method, ast.FunctionDef) and method.name in wanted]
            nodes.append(node)
        elif isinstance(node, ast.FunctionDef) and node.name in {"model_inference", "reset_inference_after_intervention", "inference_fn_non_blocking_fast", "keyboard_monitor_thread", "handle_operator_action", "_pedal_pipe_from_environment"}:
            nodes.append(node)
    namespace = dict(os=os, time=time, math=math, threading=threading, process_thread=None, result_container={}, timesteps=[], actions=[], inference_generation=0, inference_state_lock=threading.Lock(), observation_window=None, stream_buffer=None, resume_inference_in_progress=False)
    exec(compile(ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[])), str(SOURCE), "exec"), namespace)
    operator_class = namespace["RosOperator"]
    operator_class.init = lambda self: None
    operator_class.init_ros = lambda self: None
    return namespace, operator_class


class CompatibilityTests(unittest.TestCase):
    def setUp(self):
        self.namespace, self.operator_class = load_official_methods()
        self.operator = self.operator_class.__new__(self.operator_class)
        self.operator.stop_flag = False
        self.operator._dagger_live_after_source_time = None
        self.operator.start_dagger_preroll = lambda: None
        self.operator.finish_dagger_preroll = lambda _ready_at: None
        self.namespace["rospy"] = SimpleNamespace(is_shutdown=lambda: False)
        self.addCleanup(self.cleanup_worker)
        self.output = redirect_stdout(io.StringIO())
        self.output.__enter__()
        self.addCleanup(self.output.__exit__, None, None, None)

    def cleanup_worker(self):
        self.operator.stop_flag = True
        thread = self.namespace.get("process_thread")
        if isinstance(thread, threading.Thread):
            thread.join(timeout=1)
            self.assertFalse(thread.is_alive(), "test capture worker leaked")

    def test_cli_dataset_root_is_used_for_both_recordings(self):
        with tempfile.TemporaryDirectory() as data_dir:
            operator = self.operator_class(SimpleNamespace(dataset_dir=data_dir, dataset_name="mixed_dagger_hdf5"))
            dagger = Path(data_dir) / "mixed_dagger_hdf5" / "aloha_mobile_dummy"
            inference = Path(data_dir) / "mixed_inference_hdf5" / "aloha_mobile_dummy"
            self.assertEqual(operator.data_collector.dataset_dir, data_dir)
            self.assertEqual(Path(operator.data_collector.full_dataset_dir), dagger)
            self.assertEqual(Path(operator.inference_dataset_dir), inference)
            self.assertTrue(dagger.is_dir())
            self.assertTrue(inference.is_dir())

    def test_stop_sets_flag_before_bounded_join(self):
        operator = self.operator
        calls = []
        class Thread:
            def join(self, timeout):
                self_test.assertTrue(operator.stop_flag)
                calls.append(timeout)
            def is_alive(self):
                return False
        self_test = self
        expected = (["frame"], ["action"])
        self.namespace.update(process_thread=Thread(), result_container={"res": expected})
        self.assertIs(operator.stop_inference_collection(), expected)
        self.assertEqual(calls, [5.0])

    def test_stuck_capture_does_not_enable_or_move_arms(self):
        calls = []
        self.operator.enable_master_arms = lambda: calls.append("enable")
        self.namespace["process_thread"] = SimpleNamespace(join=lambda timeout: None, is_alive=lambda: True)
        with self.assertRaisesRegex(RuntimeError, "did not stop"):
            self.operator.enter_dagger_mode()
        self.assertTrue(self.operator.stop_flag)
        self.assertEqual(calls, [])

    def test_empty_capture_does_not_enable_or_move_arms(self):
        calls = []
        self.operator.enable_master_arms = lambda: calls.append("enable")
        self.assertIs(self.operator.enter_dagger_mode(), False)
        self.assertEqual(calls, [])

    def test_repeat_capture_gets_fresh_lists_result_and_stop_flag(self):
        started = threading.Event()
        captures = []
        def process():
            frames = self.namespace["timesteps"]
            actions = self.namespace["actions"]
            captures.append((frames, actions, self.namespace["result_container"]))
            frames.extend(["first", "next"])
            actions.append("action")
            started.set()
            while not self.operator.stop_flag:
                threading.Event().wait(0.001)
            return frames, actions
        self.operator.process = process
        for _ in range(2):
            started.clear()
            self.operator.start_inference_collection()
            self.assertTrue(started.wait(1))
            self.assertFalse(self.operator.stop_flag)
            frames, actions = self.operator.stop_inference_collection()
            self.assertEqual(frames, ["first", "next"])
            self.assertEqual(actions, ["action"])
            frames.clear()  # Official save_data consumes these lists.
            actions.clear()
        for index in range(3):
            self.assertIsNot(captures[0][index], captures[1][index])

    def test_duplicate_start_is_rejected_without_resetting_active_capture(self):
        old_frames = self.namespace["timesteps"]
        self.namespace["process_thread"] = SimpleNamespace(is_alive=lambda: True)
        with self.assertRaisesRegex(RuntimeError, "already running"):
            self.operator.start_inference_collection()
        self.assertIs(self.namespace["timesteps"], old_frames)

    def test_two_d_r_cycles_save_separate_inference_segments(self):
        started = threading.Event()
        captures, saved, motions = [], [], []
        self.namespace["rospy"] = SimpleNamespace(loginfo=lambda *args: None, is_shutdown=lambda: False,
                                                Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=time.time)))
        def save_data(args, frames, actions, path):
            saved.append((list(frames), list(actions), path))
            Path(path + ".hdf5").touch()
            frames.clear()
            actions.clear()
        self.namespace["save_data"] = save_data
        self.operator.enable_master_arms = lambda: motions.append("enable")
        self.operator.move_masters_to_safe_pose = lambda duration: motions.append("safe") or True
        self.operator.move_masters_to_slave_pose = lambda duration, **kwargs: motions.append("align") or True
        self.operator.release_prepared_masters = lambda: motions.append("teach")
        self.operator.wait_for_master_control = lambda *args, **kwargs: None
        def process():
            frames, actions = self.namespace["timesteps"], self.namespace["actions"]
            captures.append((frames, actions))
            frames.append(f"observation-{len(captures)}")
            actions.append(f"action-{len(captures)}")
            started.set()
            while not self.operator.stop_flag:
                threading.Event().wait(0.001)
            return frames, actions
        self.operator.process = process
        with tempfile.TemporaryDirectory() as data_dir:
            self.operator.args = SimpleNamespace(dataset_dir=data_dir, dataset_name="mixed")
            self.operator.start_inference_collection()
            for cycle in range(2):
                self.assertTrue(started.wait(1))
                self.assertTrue(self.operator.enter_dagger_mode())  # d
                self.assertTrue(self.operator.stop_flag)
                started.clear()
                self.operator.start_inference_collection()  # r (same method the model loop invokes)
            self.assertTrue(started.wait(1))
            self.operator.stop_inference_collection()
            self.assertEqual([record[0] for record in saved], [["observation-1"], ["observation-2"]])
            self.assertEqual([record[1] for record in saved], [["action-1"], ["action-2"]])
            for record in saved:
                self.assertTrue(record[2].startswith(data_dir + os.sep))
            self.assertEqual([Path(record[2]).name for record in saved], ["episode_0", "episode_1"])
            self.assertEqual(motions, ["enable", "safe", "align", "teach"] * 2)

    def test_capture_exception_is_reported_before_teaching(self):
        def process():
            raise ValueError("camera decode failed")
        self.operator.process = process
        self.operator.start_inference_collection()
        self.namespace["process_thread"].join(timeout=1)
        with self.assertRaisesRegex(RuntimeError, "Inference collection failed") as error:
            self.operator.enter_dagger_mode()
        self.assertIsInstance(error.exception.__cause__, ValueError)

    def model_namespace(self):
        ns = self.namespace
        ns.update(
            lang_embeddings="test", dagger_mode_active=True, resume_inference_requested=False, save_data_requested=False,
            collection_active=False, dagger_mode_lock=threading.Lock(),
            save_data_lock=threading.Lock(), collection_lock=threading.Lock(),
            np=SimpleNamespace(zeros=lambda shape: []),
            torch=SimpleNamespace(inference_mode=nullcontext),
            rospy=SimpleNamespace(is_shutdown=lambda: False, Rate=lambda rate: SimpleNamespace(sleep=lambda: None)),
            websocket_client_policy=SimpleNamespace(WebsocketClientPolicy=lambda *args: SimpleNamespace(get_server_metadata=lambda: {})),
            SimpleKalmanFilter=lambda: None,
        )
        args = SimpleNamespace(host="fake", port=0, publish_rate=30)
        config = dict(state_dim=14, episode_len=1, chunk_size=1)
        return args, config

    def test_model_refuses_failed_dagger_entry(self):
        args, config = self.model_namespace()
        operator = SimpleNamespace(wait_for_fresh_policy_inputs=lambda: None, puppet_arm_publish_continuous=lambda *args: None, enter_dagger_mode=lambda: False)
        with self.assertRaisesRegex(RuntimeError, "Failed to enter DAgger"):
            self.namespace["model_inference"](args, config, operator, keyboard_thread_started=True)

    def test_model_r_transition_clears_human_capture_and_restarts_inference_capture(self):
        args, config = self.model_namespace()
        calls = []
        self.namespace["collection_active"] = True
        collector = SimpleNamespace(is_collecting=True, start_collection=lambda: None,
                                    stop_collection=lambda: calls.append("stop-human"))
        def synced_frame():
            self.namespace["resume_inference_requested"] = True  # Simulate the TTY r handler.
        class ResumeReached(Exception):
            pass
        def restart():
            self.assertFalse(self.namespace["collection_active"])
            self.assertTrue(self.namespace["dagger_mode_active"])
            calls.append("restart-inference")
            raise ResumeReached
        operator = SimpleNamespace(
            wait_for_fresh_policy_inputs=lambda: None, puppet_arm_publish_continuous=lambda *args: None, enter_dagger_mode=lambda: True,
            data_collector=collector, get_synced_frame=synced_frame, start_inference_collection=restart,
            dagger_recording_ready=lambda: True,
            consume_dagger_preroll=lambda: None,
            exit_dagger_mode=lambda: calls.append("handoff"),
        )
        with self.assertRaises(ResumeReached):
            self.namespace["model_inference"](args, config, operator, keyboard_thread_started=True)
        self.assertEqual(calls, ["stop-human", "handoff", "restart-inference"])

    def test_handoff_requires_fresh_mode_ack_and_both_publishers_quiet(self):
        operator = self.operator
        operator.master_handoff_lock = threading.Lock()
        operator.master_control_feedback = {}
        operator.master_command_seen_at = {}
        clock = [1.0]
        def sleep(seconds):
            clock[0] += seconds
            for side in ('left', 'right'):
                operator.master_control_feedback[side] = (clock[0], 0xFC, 0)
                if clock[0] < 1.5:
                    operator.master_command_seen_at[side] = clock[0]
        self.namespace['time'] = SimpleNamespace(monotonic=lambda: clock[0], sleep=sleep)
        self.namespace['rospy'] = SimpleNamespace(is_shutdown=lambda: False)
        operator.wait_for_master_control(0xFC, False, since=1.0, timeout=2, quiet_period=0.25)
        self.assertGreaterEqual(clock[0], 1.73)
        operator.master_control_feedback = {'left': (0.0, 0xFC, 0), 'right': (0.0, 0xFC, 0)}
        self.namespace['time'] = SimpleNamespace(monotonic=lambda: clock[0], sleep=lambda seconds: clock.__setitem__(0, clock[0] + seconds))
        with self.assertRaisesRegex(RuntimeError, 'policy stays paused'):
            operator.wait_for_master_control(0xFC, False, since=clock[0], timeout=0.1)

    def test_tty_r_requests_handoff_while_inference_stays_paused(self):
        iterations = [0]
        def shutdown():
            iterations[0] += 1
            return iterations[0] > 1
        stdin = SimpleNamespace(fileno=lambda: 0, read=lambda count: 'r')
        self.namespace.update(
            dagger_mode_active=True, resume_inference_requested=False,
            dagger_mode_lock=threading.Lock(), delete_data_lock=threading.Lock(),
            waiting_for_user_confirm=False,
            rospy=SimpleNamespace(is_shutdown=shutdown), sys=SimpleNamespace(stdin=stdin),
            termios=SimpleNamespace(tcgetattr=lambda stream: [], tcsetattr=lambda *args: None, TCSADRAIN=1),
            tty=SimpleNamespace(setcbreak=lambda fd: None),
            select=SimpleNamespace(select=lambda *args: ([stdin], [], [])),
        )
        self.namespace['keyboard_monitor_thread']()
        self.assertTrue(self.namespace['dagger_mode_active'])
        self.assertTrue(self.namespace['resume_inference_requested'])

    def test_handoff_rejects_still_publishing_master_even_with_slave_ack(self):
        operator = self.operator
        operator.master_handoff_lock = threading.Lock()
        operator.master_control_feedback = {}
        operator.master_command_seen_at = {}
        clock = [1.0]
        def sleep(seconds):
            clock[0] += seconds
            for side in ('left', 'right'):
                operator.master_control_feedback[side] = (clock[0], 0xFC, 0)
                operator.master_command_seen_at[side] = clock[0]
        self.namespace['time'] = SimpleNamespace(monotonic=lambda: clock[0], sleep=sleep)
        self.namespace['rospy'] = SimpleNamespace(is_shutdown=lambda: False)
        with self.assertRaisesRegex(RuntimeError, 'policy stays paused'):
            operator.wait_for_master_control(0xFC, False, since=1.0, timeout=0.5, quiet_period=0.25)

    def test_reset_discards_queued_actions_and_inflight_old_generation(self):
        integrated = []
        buffer = SimpleNamespace(lock=threading.Lock(), cur_chunk=deque(['old']), chunks=deque(['old']), last_action='old', k=4,
                                 integrate_new_chunk=lambda actions, **kwargs: integrated.extend(actions))
        self.namespace.update(stream_buffer=buffer, observation_window=['old'])
        self.namespace['reset_inference_after_intervention']()
        self.assertEqual(list(buffer.cur_chunk), [])
        self.assertIsNone(buffer.last_action)
        self.assertIsNone(self.namespace['observation_window'])
        self.assertEqual(self.namespace['inference_generation'], 1)
        shutdown = [False]
        image = SimpleNamespace(transpose=lambda *args: 'image')
        def update(*args):
            self.namespace['observation_window'] = [{'qpos': [], 'images': dict.fromkeys(['front', 'left', 'right'], image)}]
        self.namespace.update(
            dagger_mode_active=False, lang_embeddings='test', update_observation_window=update,
            np=SimpleNamespace(array=lambda images: images),
            cv2=SimpleNamespace(COLOR_BGR2RGB=1, cvtColor=lambda image, code: image),
            image_tools=SimpleNamespace(resize_with_pad=lambda images, *args: images),
            rospy=SimpleNamespace(is_shutdown=lambda: shutdown[0], Rate=lambda rate: SimpleNamespace(sleep=lambda: None)),
        )
        def infer(payload):
            self.namespace['reset_inference_after_intervention']()  # Resume while this request was in flight.
            shutdown[0] = True
            return {'actions': ['stale']}
        self.namespace['inference_fn_non_blocking_fast'](SimpleNamespace(), {'camera_names': ['front', 'left', 'right']}, SimpleNamespace(infer=infer), None)
        self.assertEqual(integrated, [])
        shutdown[0] = False
        def fresh_infer(payload):
            shutdown[0] = True
            return {'actions': ['fresh']}
        self.namespace['inference_fn_non_blocking_fast'](SimpleNamespace(), {'camera_names': ['front', 'left', 'right']}, SimpleNamespace(infer=fresh_infer), None)
        self.assertEqual(integrated, ['fresh'])

    def freshness_operator(self):
        operator = self.operator
        operator.policy_input_lock = threading.Lock()
        operator.policy_input_stamps = dict.fromkeys(['camera_front', 'camera_left', 'camera_right', 'follower_left', 'follower_right'], 1000.0)
        now = SimpleNamespace(to_sec=lambda: 1000.0)
        self.namespace['rospy'] = SimpleNamespace(Time=SimpleNamespace(now=lambda: now), is_shutdown=lambda: False,
                                                Rate=lambda rate: SimpleNamespace(sleep=lambda: None))
        commands = []
        publisher = SimpleNamespace(publish=lambda message: commands.append(message))
        for name in ['puppet_arm_left_publisher', 'puppet_arm_right_publisher', 'endpose_left_publisher', 'endpose_right_publisher', 'robot_base_publisher']:
            setattr(operator, name, publisher)
        self.namespace.update(JointState=lambda: SimpleNamespace(), Header=lambda: SimpleNamespace(),
                              PosCmd=lambda: SimpleNamespace(), Twist=lambda: SimpleNamespace(linear=SimpleNamespace(), angular=SimpleNamespace()))
        return operator, commands

    def test_missing_stale_future_nonfinite_input_prevents_all_policy_commands(self):
        for name in ['camera_front', 'camera_left', 'camera_right', 'follower_left', 'follower_right']:
            for stamp in [None, 999.49 if name.startswith('camera') else 999.74, 1000.1, float('nan'), float('inf')]:
                with self.subTest(name=name, stamp=stamp):
                    operator, commands = self.freshness_operator()
                    if stamp is None:
                        del operator.policy_input_stamps[name]
                    else:
                        operator.policy_input_stamps[name] = stamp
                    for method, args in [('puppet_arm_publish', ([0]*7, [0]*7)), ('endpose_publish', ([0]*7, [0]*7)), ('robot_base_publish', ([0, 0],))]:
                        with self.assertRaises(RuntimeError):
                            getattr(operator, method)(*args)
                    self.assertEqual(commands, [])

    def test_fresh_inputs_allow_policy_commands_and_deque_consumption_does_not_stale_them(self):
        operator, commands = self.freshness_operator()
        for callback, deque_name, key in [('img_front_callback', 'img_front_deque', 'camera_front'),
                                         ('img_left_callback', 'img_left_deque', 'camera_left'),
                                         ('img_right_callback', 'img_right_deque', 'camera_right'),
                                         ('puppet_arm_left_callback', 'puppet_arm_left_deque', 'follower_left'),
                                         ('puppet_arm_right_callback', 'puppet_arm_right_deque', 'follower_right')]:
            setattr(operator, deque_name, deque())
            getattr(operator, callback)(SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(to_sec=lambda: 999.9))))
            getattr(operator, deque_name).clear()
            self.assertEqual(operator.policy_input_stamps[key], 999.9)
        operator.puppet_arm_publish([0]*7, [0]*7)
        operator.endpose_publish([0]*7, [0]*7)
        operator.robot_base_publish([0, 0])
        self.assertEqual(len(commands), 5)

    def test_startup_wait_blocks_initial_pose_when_inputs_never_become_fresh(self):
        args, config = self.model_namespace()
        operator, commands = self.freshness_operator()
        operator.policy_input_stamps.clear()
        clock = [1.0]
        self.namespace['time'] = SimpleNamespace(monotonic=lambda: clock[0], sleep=lambda seconds: clock.__setitem__(0, clock[0] + seconds))
        operator.puppet_arm_publish_continuous = lambda *args: commands.append('initial-pose')
        with self.assertRaisesRegex(RuntimeError, 'Policy startup requires fresh'):
            self.namespace['model_inference'](args, config, operator, keyboard_thread_started=True)
        self.assertEqual(commands, [])

    def test_initial_pose_checks_freshness_again_before_every_interpolation_step(self):
        operator, commands = self.freshness_operator()
        operator.args = SimpleNamespace(publish_rate=30, arm_steps_length=[0.1]*7)
        operator.puppet_arm_left_deque = deque([SimpleNamespace(position=[0.0]*7)])
        operator.puppet_arm_right_deque = deque([SimpleNamespace(position=[0.0]*7)])
        operator.puppet_arm_publish_lock = threading.Lock()
        operator.puppet_arm_publish_lock.acquire()
        def sleep():
            operator.policy_input_stamps['camera_right'] = 999.0
        self.namespace['rospy'].Rate = lambda rate: SimpleNamespace(sleep=sleep)
        operator.wait_for_fresh_policy_inputs()
        with self.assertRaisesRegex(RuntimeError, 'Stale/future policy input'):
            operator.puppet_arm_publish_continuous([1.0]*7, [1.0]*7)
        self.assertEqual(len(commands), 2)  # Exactly the first fresh left/right step; no stale second step.

    def test_headless_combined_collector_does_not_import_pynput(self):
        collector = ast.parse(SOURCE.read_text())
        for node in ast.walk(collector):
            if isinstance(node, ast.ImportFrom):
                self.assertNotEqual(node.module, "pynput")
        standalone = ast.parse((ROOT / "collect_data.py").read_text())
        for node in standalone.body:
            self.assertFalse(isinstance(node, ast.ImportFrom) and node.module == "pynput")
        listeners = [node for node in ast.walk(standalone) if isinstance(node, ast.FunctionDef) and node.name == "keyboard_listener"]
        self.assertEqual(len(listeners), 1)
        self.assertTrue(any(isinstance(node, ast.ImportFrom) and node.module == "pynput" for node in ast.walk(listeners[0])))

    def test_all_color_boundaries_are_bgr_and_depth_stays_passthrough(self):
        counts = {"color": 0, "depth": 0}
        for path in [SOURCE, ROOT / "collect_data.py"]:
            for node in ast.walk(ast.parse(path.read_text())):
                if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "imgmsg_to_cv2":
                    is_depth = "depth" in ast.unparse(node.args[0])
                    self.assertEqual(node.args[1].value, "passthrough" if is_depth else "bgr8")
                    counts["depth" if is_depth else "color"] += 1
        self.assertEqual(counts, {"color": 13, "depth": 9})

    def test_rgb8_and_bgr8_frames_reach_policy_and_video_with_correct_colors(self):
        rgb = [(255, 0, 0), (0, 255, 0), (0, 0, 255)]
        bgr = [pixel[::-1] for pixel in rgb]
        depth = [100, 200, 300]
        class Bridge:
            def imgmsg_to_cv2(self, message, desired):
                if desired == "passthrough":
                    return message.pixels
                self_test.assertEqual(desired, "bgr8")
                return [pixel[::-1] for pixel in message.pixels] if message.encoding == "rgb8" else message.pixels
        self_test = self
        cv2 = SimpleNamespace(COLOR_BGR2RGB=1, cvtColor=lambda image, code: [pixel[::-1] for pixel in image])
        tree = ast.parse(SOURCE.read_text())
        inference = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "inference_fn")
        conversion = next(node.value for node in ast.walk(inference) if isinstance(node, ast.Assign) and any(isinstance(call, ast.Call) and isinstance(call.func, ast.Attribute) and call.func.attr == "cvtColor" for call in ast.walk(node.value)))
        save_tree = ast.parse((ROOT / "collect_data.py").read_text())
        video_conversion = next(node.value for node in ast.walk(save_tree) if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == "video_img" for target in node.targets))
        for encoding in ["rgb8", "bgr8"]:
            for getter in ["get_frame", "get_frame_peek", "get_synced_frame"]:
                with self.subTest(encoding=encoding, getter=getter):
                    stamp = SimpleNamespace(to_sec=lambda: 1.0)
                    image = SimpleNamespace(header=SimpleNamespace(stamp=stamp), encoding=encoding, pixels=rgb if encoding == "rgb8" else bgr)
                    depth_image = SimpleNamespace(header=SimpleNamespace(stamp=stamp), encoding="16UC1", pixels=depth)
                    joint = SimpleNamespace(header=SimpleNamespace(stamp=stamp))
                    operator = self.operator
                    operator.args = SimpleNamespace(use_depth_image=True, use_robot_base=False)
                    operator.bridge = Bridge()
                    for camera in ["left", "right", "front"]:
                        setattr(operator, f"img_{camera}_deque", deque([image]))
                        setattr(operator, f"img_{camera}_depth_deque", deque([depth_image]))
                    for role in ["puppet", "master"]:
                        for side in ["left", "right"]:
                            setattr(operator, f"{role}_arm_{side}_deque", deque([joint]))
                    frames = getattr(operator, getter)()
                    self.assertEqual(list(frames[:3]), [bgr] * 3)
                    if getter != "get_synced_frame":
                        self.assertEqual(list(frames[3:6]), [depth] * 3)
                    policy_rgb = eval(compile(ast.Expression(conversion), str(SOURCE), "eval"), {"image_arrs": list(frames[:3]), "cv2": cv2})
                    video_rgb = eval(compile(ast.Expression(video_conversion), str(ROOT / "collect_data.py"), "eval"), {"img": frames[0], "cv2": cv2})
                    self.assertEqual(policy_rgb, [rgb] * 3)
                    self.assertEqual(video_rgb, rgb)

    def test_original_sources_still_match_recorded_upstream_hashes(self):
        manifest = json.loads((ROOT / "upstream-manifest.json").read_text())
        for record in manifest["files"].values():
            source = Path(record["upstream_path"])
            if not source.exists():
                self.skipTest("original checkout is unavailable on this host")
            self.assertEqual(hashlib.sha256(source.read_bytes()).hexdigest(), record["upstream_sha256"])


if __name__ == "__main__":
    unittest.main()
