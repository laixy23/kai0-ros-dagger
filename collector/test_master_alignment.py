"""AST tests of the real alignment methods without importing ROS or a driver."""
import ast
from collections import deque
from contextlib import redirect_stdout
import io
import math
import os
from pathlib import Path
import sys
import tempfile
import threading
from types import SimpleNamespace
import unittest

SOURCE = Path(__file__).with_name("agilex_openpi_dagger_collect.py")
sys.path.insert(0, str(SOURCE.parent.parent / "setup"))
from master_alignment_feedback import alignment_targets


class CollectorAlignmentTests(unittest.TestCase):
    def setUp(self):
        self.t = 0.0
        self.published = []
        self.logs = []
        self.verified = []
        self.results = [True]
        tree = ast.parse(SOURCE.read_text())
        node = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "RosOperator")
        wanted = {"_move_masters_to_pose", "move_masters_to_safe_pose", "move_masters_to_slave_pose",
                  "_hold_final_pose_and_prepare", "save_inference_collection", "enter_dagger_mode"}
        node.body = [method for method in node.body if isinstance(method, ast.FunctionDef) and method.name in wanted]
        def sleep():
            self.t += 0.1
        def log(message, *values):
            self.logs.append(message % values if values else message)
        def verify(targets, **options):
            self.verified.append((self.t, targets, options))
            options["publish"]()
            passed = self.results.pop(0)
            self.t += options["stable_seconds"] if passed else 0
            sides = {side: {"error_degrees": [0, 0 if side == "left" or passed else 5, 0, 0, 0, 0],
                            "gripper_error_um": -15100}
                     for side in ("left", "right")}
            return {"passed": passed, "sides": sides, "error": "right J2 did not converge"}
        ros = SimpleNamespace(
            is_shutdown=lambda: False, Rate=lambda hz: SimpleNamespace(sleep=sleep),
            Time=SimpleNamespace(now=lambda: 1000 + self.t), loginfo=log, logerr=log)
        namespace = dict(
            rospy=ros, time=SimpleNamespace(monotonic=lambda: self.t, time=lambda: 1000 + self.t),
            math=math, os=os, alignment_targets=alignment_targets,
            wait_for_master_alignment=verify, JointState=SimpleNamespace, Header=SimpleNamespace,
            process_thread=None, save_data=lambda *arguments: None)
        exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), str(SOURCE), "exec"), namespace)
        self.operator = namespace["RosOperator"]()
        self.operator.start_dagger_preroll = lambda: None
        self.operator.finish_dagger_preroll = lambda _ready_at: None
        self.operator._start_following_preparation = lambda started_at, timeout: dict(
            deadline=started_at + timeout, hold_passed=False, cancel=threading.Event(),
            thread=SimpleNamespace(join=lambda **kwargs: None))
        self.operator._following_preparation_guard = lambda state: None
        self.operator._following_preparation_ready = lambda state: True
        for side in ("left", "right"):
            publisher = SimpleNamespace(publish=lambda message, side=side:
                                        self.published.append((side, list(message.position))))
            setattr(self.operator, "master_" + side + "_joint_pub", publisher)
        self.operator.puppet_arm_left_deque = deque([SimpleNamespace(position=[0.1] * 7)])
        self.operator.puppet_arm_right_deque = deque([SimpleNamespace(position=[0.2] * 7)])
        self.namespace = namespace

    def test_intermediate_and_slave_paths_both_require_verification_after_three_seconds(self):
        self.results = [True, True, True]
        self.assertTrue(self.operator.move_masters_to_safe_pose(duration=0.1))
        first = self.t
        self.assertTrue(self.operator.move_masters_to_slave_pose(duration=0.1))
        self.assertGreaterEqual(self.verified[0][0], 3.0)
        self.assertGreaterEqual(self.verified[1][0] - first, 3.0)
        for _, _, options in self.verified[:2]:
            self.assertEqual(options["timeout"], 8.0)
            self.assertEqual(options["tolerance_mdeg"], 1000)
            self.assertEqual(options["stable_seconds"], 0.3)
        self.assertEqual(len(self.verified), 3)
        self.assertEqual(self.verified[2][2]["stable_seconds"], 3.0)
        self.assertEqual(self.verified[2][2]["timeout"], 4.5)
        self.assertGreaterEqual(len(self.published), 120)

    def test_failed_verification_reports_all_six_and_does_not_claim_success(self):
        self.results = [False]
        self.assertFalse(self.operator.move_masters_to_safe_pose())
        self.assertIn("intermediate pose", self.operator.master_alignment_reports)
        self.assertTrue(any("right master errors J1..J6" in message and "5" in message for message in self.logs))
        self.assertTrue(any("teaching remains blocked" in message for message in self.logs))
        self.assertFalse(any("six actual joints converged" in message for message in self.logs))

    def test_bad_targets_are_rejected_before_publishing_or_waiting(self):
        for value in (float("nan"), float("inf"), "bad"):
            with self.subTest(value=value):
                self.assertFalse(self.operator._move_masters_to_pose([value] + [0] * 6, [0] * 7, 3, "invalid"))
        self.assertEqual(self.published, [])
        self.assertEqual(self.verified, [])

    def test_shutdown_during_official_phase_cannot_claim_alignment(self):
        self.namespace["rospy"].is_shutdown = lambda: self.t >= 0.3
        self.assertFalse(self.operator.move_masters_to_safe_pose())
        self.assertEqual(self.verified, [])

    def check_teach_blocked(self, results):
        self.results = results
        calls = []
        self.operator.stop_inference_collection = lambda **kwargs: (["observation"], ["action"])
        self.operator.enable_master_arms = lambda: calls.append("enable")
        self.operator.release_prepared_masters = lambda: calls.append("teach")
        self.operator.wait_for_master_control = lambda *arguments, **kwargs: calls.append("ack")
        output = io.StringIO()
        with tempfile.TemporaryDirectory() as directory, redirect_stdout(output):
            self.operator.args = SimpleNamespace(dataset_dir=directory, dataset_name="debug")
            self.assertFalse(self.operator.enter_dagger_mode())
        self.assertEqual(calls, ["enable"])
        self.assertTrue(self.operator.stop_flag)
        self.assertNotIn("DAGGER MODE ACTIVE", output.getvalue())

    def test_intermediate_failure_blocks_teach_entry(self):
        self.check_teach_blocked([False])

    def test_slave_alignment_failure_blocks_teach_even_after_intermediate_success(self):
        self.check_teach_blocked([True, False])

    def test_final_hold_failure_blocks_teach_after_both_alignments(self):
        self.check_teach_blocked([True, True, False])
        self.assertIn("slave pose hold", self.operator.master_alignment_reports)


if __name__ == "__main__":
    unittest.main()
