"""Offline tests of the isolated FA master-profile experiment on one follower.

SDK, CAN, recorder, ownership and clocks are stand-ins. No test opens hardware,
starts ROS, moves a robot, or sends an application command.
"""
import copy
import unittest

from follower_drag_probe import FEEDBACK_IDS
from follower_master_mode_probe import restore_follower, run_master_profile


class FakeSDK:
    def __init__(self, owner):
        self.owner = owner

    def ConnectPort(self, *, piper_init, start_thread):
        self.owner.assertIs(piper_init, False)
        self.owner.assertIs(start_thread, False)
        self.owner.calls.append(("connect",))
        if self.owner.stop_after_connect:
            self.owner.stopped = True

    def MasterSlaveConfig(self, *, linkage_config, feedback_offset, ctrl_offset, linkage_offset):
        self.owner.calls.append(("role", linkage_config, feedback_offset, ctrl_offset, linkage_offset))

    def EnableArm(self, value):
        self.owner.calls.append(("enable", value))

    def MotionCtrl_1(self, *, grag_teach_ctrl):
        self.owner.calls.append(("teach", grag_teach_ctrl))

    def DisconnectPort(self):
        self.owner.calls.append(("disconnect",))

    def __getattr__(self, name):
        raise AssertionError("Unexpected SDK method: " + name)


class FakeRecorder:
    def __init__(self, owner):
        self.owner = owner
        self.observations = 0

    def __enter__(self):
        self.owner.calls.append(("recorder-enter",))
        return self

    def __exit__(self, *error):
        self.owner.calls.append(("recorder-exit",))

    def observe(self, seconds, *, guard, stop_requested):
        self.observations += 1
        self.owner.calls.append(("observe", seconds))
        guard()
        if self.owner.fail_observation == self.observations:
            raise RuntimeError("raw observation failed")
        if self.owner.stop_observation == self.observations:
            self.owner.stopped = True

    def summary(self):
        return {"fake_observations": self.observations}


class FollowerMasterProfileTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        self.events = []
        self.sdk_count = 0
        self.recorder_count = 0
        self.read_count = 0
        self.stopped = False
        self.stop_after_connect = False
        self.stop_observation = None
        self.fail_observation = None
        self.initial_mode = 1
        self.mutate = lambda row: row
        self.fail_read = None
        self.now = 1000.0
        self.sdk = FakeSDK(self)
        self.recorder = FakeRecorder(self)

    def guard(self, channel):
        self.calls.append(("guard", channel))

    def idle(self):
        self.calls.append(("idle",))

    def factory(self, channel):
        self.sdk_count += 1
        self.calls.append(("factory", channel))
        return self.sdk

    def recorder_factory(self, channel):
        self.recorder_count += 1
        self.calls.append(("recorder", channel))
        return self.recorder

    def reader(self, channel, **options):
        self.read_count += 1
        self.now += 0.12
        if self.read_count == self.fail_read:
            raise RuntimeError("Missing required CAN feedback frames")
        return self.mutate({
            "ctrl_mode": self.initial_mode, "arm_status": 0, "err_code": 0,
            "gripper_mode": 0, "joint_mdeg": [0] * 6, "gripper_um": 0,
            "driver_enabled": [True] * 6, "driver_foc_bytes": [64] * 6,
            "component_age_s": dict.fromkeys(FEEDBACK_IDS, 0.001),
            "timestamp_sec": self.now,
        })

    def run_profile(self, channel="can_left_slave", **options):
        arguments = dict(
            apply=True, confirm_supported=True, duration=15,
            reader=self.reader, sdk_factory=self.factory,
            guard=self.guard, idle_guard=self.idle,
            recorder_factory=self.recorder_factory,
            wall=lambda: self.now,
            stop_requested=lambda: self.stopped,
            emit=lambda row: self.events.append(copy.deepcopy(row)),
        )
        arguments.update(options)
        return run_master_profile(channel, **arguments)

    def hardware_calls(self):
        return [call for call in self.calls if call[0] in ("connect", "role", "enable", "teach", "disconnect")]

    def assert_rejected_without_write(self, report):
        self.assertIn("error", report)
        self.assertFalse(any(call[0] in ("role", "enable", "teach") for call in self.calls))
        self.assertFalse(report["gravity_compensation_verified"])

    def test_default_is_passive_without_sdk_or_raw_recorder(self):
        report = self.run_profile(apply=False, confirm_supported=False)
        self.assertNotIn("error", report)
        self.assertEqual(self.sdk_count, 0)
        self.assertEqual(self.recorder_count, 0)
        self.assertEqual(self.hardware_calls(), [])
        self.assertFalse(report["gravity_compensation_verified"])

    def test_only_current_left_follower_is_allowed(self):
        for channel in ("can_right_slave", "can_left_mas", "can0"):
            with self.subTest(channel=channel):
                self.assert_rejected_without_write(self.run_profile(channel))
        self.assertEqual(self.sdk_count, 0)

    def test_apply_requires_supported_confirmation(self):
        self.assert_rejected_without_write(self.run_profile(confirm_supported=False))
        self.assertEqual(self.sdk_count, 0)

    def test_initial_fault_or_disabled_driver_cannot_write(self):
        for field, value in (("arm_status", 1), ("err_code", 1),
                             ("driver_enabled", [False] + [True] * 5),
                             ("driver_foc_bytes", [65] + [64] * 5)):
            with self.subTest(field=field):
                self.mutate = lambda row, field=field, value=value: dict(row, **{field: value})
                self.assert_rejected_without_write(self.run_profile())
        self.assertEqual(self.sdk_count, 0)

    def test_stale_individual_component_is_rejected(self):
        self.mutate = lambda row: dict(row, component_age_s=dict(row["component_age_s"], **{"0x2A5": 1.0}))
        self.assert_rejected_without_write(self.run_profile())
        self.assertEqual(self.sdk_count, 0)

    def test_full_profile_matches_fa_zero_offsets_enable_and_teach(self):
        report = self.run_profile()
        self.assertNotIn("error", report)
        self.assertEqual(self.hardware_calls(), [
            ("connect",), ("role", 0xFA, 0, 0, 0), ("enable", 7), ("teach", 1), ("disconnect",)])
        significant = [call for call in self.calls if call[0] in ("role", "enable", "teach", "observe")]
        self.assertEqual(significant, [
            ("role", 0xFA, 0, 0, 0), ("observe", 2),
            ("enable", 7), ("observe", 2), ("teach", 1), ("observe", 15)])
        self.assertFalse(report["gravity_compensation_verified"])
        self.assertEqual(len(report["commands"]), 3)
        self.assertTrue(any(event["record_type"] == "teaching_profile_requested" for event in self.events))

    def test_existing_fc_drag_mode_can_start_exact_master_profile(self):
        self.initial_mode = 2
        report = self.run_profile()
        self.assertNotIn("error", report)
        self.assertIn(("role", 0xFA, 0, 0, 0), self.hardware_calls())

    def test_stop_before_preparation_does_not_build_sdk(self):
        self.stopped = True
        self.assert_rejected_without_write(self.run_profile())
        self.assertEqual(self.sdk_count, 0)

    def test_stop_after_connect_cannot_change_role(self):
        self.stop_after_connect = True
        self.assert_rejected_without_write(self.run_profile())
        self.assertEqual(self.hardware_calls(), [("connect",), ("disconnect",)])

    def test_stop_after_role_never_enables_or_teaches(self):
        self.stop_observation = 1
        report = self.run_profile()
        self.assertIn("error", report)
        self.assertEqual(self.hardware_calls(), [("connect",), ("role", 0xFA, 0, 0, 0), ("disconnect",)])
        self.assertFalse(report["gravity_compensation_verified"])

    def test_error_after_enable_never_enters_teach_or_restores_automatically(self):
        self.fail_observation = 2
        report = self.run_profile()
        self.assertIn("error", report)
        self.assertEqual(self.hardware_calls(), [
            ("connect",), ("role", 0xFA, 0, 0, 0), ("enable", 7), ("disconnect",)])

    def test_fresh_feedback_failure_after_connect_blocks_role(self):
        self.fail_read = 2
        self.assert_rejected_without_write(self.run_profile())
        self.assertEqual(self.hardware_calls(), [("connect",), ("disconnect",)])


class RestoreClock:
    def __init__(self):
        self.now = 0.0

    def clock(self):
        return self.now

    def wall(self):
        return 1000.0 + self.now

    def pause(self, seconds):
        self.now += seconds


class RestoreSDK:
    def __init__(self, owner):
        self.owner = owner

    def ConnectPort(self, *, piper_init, start_thread):
        self.owner.assertIs(piper_init, False)
        self.owner.assertIs(start_thread, False)
        self.owner.calls.append(("connect",))
        if self.owner.stop_after_connect:
            self.owner.stopped = True

    def DisableArm(self, value):
        self.owner.calls.append(("disable", value))

    def MasterSlaveConfig(self, *, linkage_config, feedback_offset, ctrl_offset, linkage_offset):
        self.owner.calls.append(("role", linkage_config, feedback_offset, ctrl_offset, linkage_offset))

    def EnableArm(self, value):
        self.owner.calls.append(("enable", value))

    def MotionCtrl_2(self, **kwargs):
        self.owner.calls.append(("mode", kwargs))

    def DisconnectPort(self):
        self.owner.calls.append(("disconnect",))
        if self.owner.stop_after_disconnect:
            self.owner.stopped = True

    def __getattr__(self, name):
        raise AssertionError("Bootstrap must not send " + name)


class FollowerRestoreTests(unittest.TestCase):
    def setUp(self):
        self.time = RestoreClock()
        self.calls = []
        self.events = []
        self.read_count = 0
        self.stopped = False
        self.stop_after_connect = False
        self.stop_after_disconnect = False
        self.mode = 0
        self.enabled = False
        self.missing = False
        self.mutate = lambda row: row
        self.sdk = RestoreSDK(self)

    def guard(self, channel):
        self.calls.append(("guard", channel))

    def idle(self):
        self.calls.append(("idle",))

    def reader(self, channel, *, since_timestamp):
        self.read_count += 1
        self.time.now += 0.12
        if self.missing:
            raise RuntimeError("Missing required CAN feedback frames")
        self.assertGreaterEqual(self.time.wall(), since_timestamp)
        return self.mutate({
            "ctrl_mode": self.mode, "arm_status": 0, "err_code": 0,
            "gripper_mode": 0, "joint_mdeg": [0] * 6, "gripper_um": 0,
            "driver_enabled": [self.enabled] * 6,
            "driver_foc_bytes": [64 if self.enabled else 0] * 6,
            "component_age_s": dict.fromkeys(FEEDBACK_IDS, 0.001),
            "timestamp_sec": self.time.wall(),
        })

    def prepare(self, channel, *, apply, reader, sdk_factory):
        self.assertTrue(apply)
        self.assertEqual(reader, self.reader)
        self.assertTrue(callable(sdk_factory))
        prior_control = [call for call in self.calls if call[0] in ("connect", "disable", "role", "disconnect")]
        self.assertEqual(prior_control[-1], ("disconnect",), "Bootstrap SDK must disconnect before preparation")
        self.calls.append(("prepare", channel, apply))
        return {"passed": True}

    def run_restore(self, channel="can_left_slave", **options):
        arguments = dict(
            confirm_supported=True, reader=self.reader,
            sdk_factory=lambda channel: self.sdk,
            guard=self.guard, idle_guard=self.idle, prepare=self.prepare,
            clock=self.time.clock, pause=self.time.pause, wall=self.time.wall,
            stop_requested=lambda: self.stopped,
            emit=lambda row: self.events.append(copy.deepcopy(row)),
        )
        arguments.update(options)
        return restore_follower(channel, **arguments)

    def control_calls(self):
        return [call for call in self.calls if call[0] in ("connect", "disable", "role", "enable", "mode", "disconnect", "prepare")]

    def assert_no_prepare(self, report):
        self.assertFalse(report["passed"])
        self.assertIn("error", report)
        self.assertFalse(any(call[0] == "prepare" for call in self.calls))

    def test_restore_requires_support_before_any_sdk_connection(self):
        report = self.run_restore(confirm_supported=False)
        self.assert_no_prepare(report)
        self.assertEqual(self.control_calls(), [])

    def test_restore_restricts_target_to_the_tested_left_follower(self):
        report = self.run_restore("can_right_slave")
        self.assert_no_prepare(report)
        self.assertEqual(self.control_calls(), [])

    def test_fresh_all_disabled_mode_zero_feedback_allows_measured_pose_prepare(self):
        report = self.run_restore()
        self.assertTrue(report["passed"])
        self.assertNotIn("error", report)
        self.assertEqual(self.control_calls(), [
            ("connect",), ("disable", 7), ("disable", 7), ("disable", 7),
            ("role", 0xFC, 0, 0, 0), ("disable", 7), ("disconnect",),
            ("prepare", "can_left_slave", True)])
        self.assertEqual(report["disabled_feedback"]["ctrl_mode"], 0)
        self.assertEqual(report["disabled_feedback"]["driver_enabled"], [False] * 6)

    def test_missing_disabled_ack_does_not_prepare_or_enable(self):
        self.missing = True
        report = self.run_restore()
        self.assert_no_prepare(report)
        self.assertGreater(self.read_count, 1)
        self.assertEqual(self.control_calls()[-1], ("disconnect",))

    def test_drivers_staying_enabled_cannot_reach_measured_pose_prepare(self):
        self.enabled = True
        report = self.run_restore()
        self.assert_no_prepare(report)
        self.assertGreater(self.read_count, 1)
        self.assertEqual(self.control_calls()[-1], ("disconnect",))

    def test_arm_fault_after_fc_aborts_without_prepare_or_enable(self):
        self.mutate = lambda row: dict(row, arm_status=1)
        report = self.run_restore()
        self.assert_no_prepare(report)
        self.assertEqual(self.read_count, 1)
        self.assertEqual(self.control_calls()[-1], ("disconnect",))

    def test_driver_fault_after_fc_aborts_without_prepare_or_enable(self):
        self.mutate = lambda row: dict(row, driver_foc_bytes=[1] + [0] * 5)
        report = self.run_restore()
        self.assert_no_prepare(report)
        self.assertEqual(self.read_count, 1)
        self.assertEqual(self.control_calls()[-1], ("disconnect",))

    def test_stale_ack_after_fc_cannot_reach_prepare(self):
        self.mutate = lambda row: dict(row, timestamp_sec=self.time.wall() - 1)
        self.assert_no_prepare(self.run_restore())
        self.assertEqual(self.read_count, 1)

    def test_stop_after_connect_prevents_disable_and_role_writes(self):
        self.stop_after_connect = True
        self.assert_no_prepare(self.run_restore())
        self.assertEqual(self.control_calls(), [("connect",), ("disconnect",)])

    def test_stop_after_disabled_ack_prevents_prepare(self):
        self.stop_after_disconnect = True
        self.assert_no_prepare(self.run_restore())
        self.assertEqual(self.control_calls()[-1], ("disconnect",))

    def cancel_during_prepare(self, method, *args, **kwargs):
        def prepare(channel, *, apply, reader, sdk_factory):
            self.assertTrue(apply)
            self.assertEqual(reader, self.reader)
            self.calls.append(("prepare", channel, apply))
            sdk = sdk_factory(channel)
            sdk.ConnectPort(piper_init=False, start_thread=False)
            self.stopped = True  # Simulate SIGINT arriving inside the existing helper.
            try:
                getattr(sdk, method)(*args, **kwargs)
            finally:
                sdk.DisconnectPort()  # Disconnect must remain allowed after cancellation.
            return {"passed": True}
        report = self.run_restore(prepare=prepare)
        self.assertFalse(report["passed"])
        self.assertIn("Stop requested during measured-pose preparation", report["error"])
        self.assertFalse(any(call[0] in ("enable", "mode") for call in self.calls))
        self.assertEqual(self.control_calls()[-1], ("disconnect",))
        self.assertEqual(self.control_calls().count(("disconnect",)), 2)

    def test_cancellation_inside_prepare_blocks_enable_but_allows_disconnect(self):
        self.cancel_during_prepare("EnableArm", 7)

    def test_cancellation_inside_prepare_blocks_control_mode_change(self):
        self.cancel_during_prepare("MotionCtrl_2", ctrl_mode=1, move_mode=1)

    def test_prepare_cannot_report_success_if_cancellation_arrives_before_return(self):
        def prepare(channel, *, apply, reader, sdk_factory):
            self.calls.append(("prepare", channel, apply))
            self.stopped = True
            return {"passed": True}
        report = self.run_restore(prepare=prepare)
        self.assertFalse(report["passed"])
        self.assertIn("Stopped during measured-pose preparation", report["error"])


if __name__ == "__main__":
    unittest.main()
