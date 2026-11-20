"""Hardware-free checks of the isolated follower drag-teach experiment.

All CAN, SDK, ownership and clock boundaries are stand-ins. Importing this test
does not import a robot SDK, create ROS nodes, or open CAN/camera devices.
"""
import copy
import unittest

from follower_drag_probe import FEEDBACK_IDS, run_probe


class Clock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now

    def wall(self):
        return 1000.0 + self.now

    def pause(self, seconds):
        self.now += seconds


class FakeSDK:
    """The only permitted control write is entering drag teaching once."""
    def __init__(self, owner):
        self.owner = owner
        self.teaching = False

    def ConnectPort(self, *, piper_init, start_thread):
        self.owner.assertIs(start_thread, False, "Probe must not start SDK background threads")
        self.owner.calls.append(("connect", piper_init))
        if self.owner.stop_after_connect:
            self.owner.stopped = True

    def MotionCtrl_1(self, *, grag_teach_ctrl):
        self.owner.calls.append(("teach", grag_teach_ctrl))
        self.teaching = True
        if self.owner.stop_after_teach:
            self.owner.stopped = True

    def DisconnectPort(self):
        self.owner.calls.append(("disconnect",))

    def __getattr__(self, name):
        raise AssertionError("Unexpected SDK method: " + name)


class FollowerDragProbeTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.calls = []
        self.events = []
        self.read_count = 0
        self.sdk_count = 0
        self.stopped = False
        self.stop_after_connect = False
        self.stop_after_teach = False
        self.mode_after_teach = 2
        self.modify_snapshot = lambda row: row
        self.read_error_at = None
        self.guard_error_at = None
        self.guard_count = 0
        self.sdk = FakeSDK(self)

    def guard(self, channel):
        self.guard_count += 1
        self.calls.append(("guard", channel))
        if self.guard_count == self.guard_error_at:
            raise RuntimeError("Other arm controller detected")

    def idle(self):
        self.calls.append(("idle",))

    def factory(self, channel):
        self.sdk_count += 1
        self.calls.append(("factory", channel))
        return self.sdk

    def reader(self, channel, **_options):
        self.read_count += 1
        self.clock.now += 0.12
        if self.read_count == self.read_error_at:
            raise RuntimeError("Missing required CAN feedback frames")
        row = {
            "ctrl_mode": self.mode_after_teach if self.sdk.teaching else 1,
            "arm_status": 0,
            "err_code": 0,
            "gripper_mode": 0,
            "joint_mdeg": [1000 if self.sdk.teaching else 0] * 6,
            "gripper_um": 0,
            "driver_enabled": [True] * 6,
            "driver_foc_bytes": [64] * 6,
            "component_age_s": dict.fromkeys(FEEDBACK_IDS, 0.001),
            "timestamp_sec": self.clock.wall(),
        }
        return self.modify_snapshot(row)

    def run_probe(self, channel="can_left_slave", **options):
        arguments = dict(
            apply=True, confirm_supported=True, duration=1,
            reader=self.reader, sdk_factory=self.factory,
            guard=self.guard, idle_guard=self.idle,
            clock=self.clock.monotonic, wall=self.clock.wall, pause=self.clock.pause,
            stop_requested=lambda: self.stopped,
            emit=lambda row: self.events.append(copy.deepcopy(row)),
        )
        arguments.update(options)
        return run_probe(channel, **arguments)

    def control_calls(self):
        return [call for call in self.calls if call[0] in ("connect", "teach", "disconnect")]

    def assert_rejected_without_write(self, report):
        self.assertIn("error", report)
        self.assertEqual(report["application_commands_sent"], 0)
        self.assertFalse(any(call[0] == "teach" for call in self.calls))
        self.assertFalse(report["gravity_compensation_verified"])

    def test_default_plan_is_passive_and_creates_no_sdk(self):
        report = self.run_probe(apply=False, confirm_supported=False)
        self.assertNotIn("error", report)
        self.assertTrue(report["passive_only"])
        self.assertEqual(report["application_commands_sent"], 0)
        self.assertEqual(self.sdk_count, 0)
        self.assertEqual(self.control_calls(), [])

    def test_master_or_unmapped_channel_is_rejected(self):
        for channel in ("can_left_mas", "can_right_mas", "can0", "can_left_slave "):
            with self.subTest(channel=channel):
                report = self.run_probe(channel)
                self.assert_rejected_without_write(report)
        self.assertEqual(self.sdk_count, 0)

    def test_explicit_reassert_of_existing_teach_sends_only_the_same_command(self):
        self.modify_snapshot = lambda row: dict(row, ctrl_mode=2)
        report = self.run_probe(reassert_existing_teach=True)
        self.assertNotIn('error', report)
        self.assertTrue(report['initial_already_teaching'])
        self.assertTrue(report['command_acknowledged'])
        self.assertEqual(self.control_calls(), [('connect', False), ('teach', 1), ('disconnect',)])
        self.assertEqual(report['application_commands_sent'], 1)
        self.assertFalse(report['gravity_compensation_verified'])

    def test_passive_existing_teach_reports_it_without_connecting(self):
        self.modify_snapshot = lambda row: dict(row, ctrl_mode=2)
        report = self.run_probe(apply=False, confirm_supported=False)
        self.assertNotIn('error', report)
        self.assertTrue(report['initial_already_teaching'])
        self.assertTrue(report['teaching_may_still_be_active'])
        self.assertEqual(self.sdk_count, 0)

    def test_reassert_requires_apply(self):
        report = self.run_probe(apply=False, reassert_existing_teach=True)
        self.assert_rejected_without_write(report)
        self.assertEqual(self.sdk_count, 0)

    def test_apply_requires_explicit_physical_support(self):
        self.assert_rejected_without_write(self.run_probe(confirm_supported=False))
        self.assertEqual(self.sdk_count, 0)

    def test_invalid_duration_is_rejected_before_sdk_or_reader(self):
        for duration in (0, 61, float("nan"), float("inf"), "20"):
            with self.subTest(duration=duration):
                self.assert_rejected_without_write(self.run_probe(duration=duration))
        self.assertEqual(self.read_count, 0)
        self.assertEqual(self.sdk_count, 0)

    def test_initial_arm_and_motor_faults_prevent_teaching(self):
        for field, value in (("arm_status", 1), ("err_code", 1),
                             ("driver_foc_bytes", [65] + [64] * 5)):
            with self.subTest(field=field):
                self.modify_snapshot = lambda row, field=field, value=value: dict(row, **{field: value})
                self.assert_rejected_without_write(self.run_probe())
        self.assertEqual(self.sdk_count, 0)

    def test_disabled_initial_driver_does_not_trigger_an_enable(self):
        self.modify_snapshot = lambda row: dict(row, driver_enabled=[False] + [True] * 5)
        self.assert_rejected_without_write(self.run_probe())
        self.assertEqual(self.sdk_count, 0)

    def test_stale_snapshot_and_stale_individual_component_are_rejected(self):
        mutations = [
            lambda row: dict(row, timestamp_sec=self.clock.wall() - 1),
            lambda row: dict(row, component_age_s=dict(row["component_age_s"], **{"0x2A6": 0.3})),
            lambda row: dict(row, component_age_s={}),
        ]
        for mutate in mutations:
            self.modify_snapshot = mutate
            self.assert_rejected_without_write(self.run_probe())
        self.assertEqual(self.sdk_count, 0)

    def test_initial_teach_mode_is_not_reentered_or_changed(self):
        self.modify_snapshot = lambda row: dict(row, ctrl_mode=2)
        self.assert_rejected_without_write(self.run_probe())
        self.assertEqual(self.sdk_count, 0)

    def test_apply_sends_only_one_teach_command_then_disconnects(self):
        report = self.run_probe()
        self.assertNotIn("error", report)
        self.assertEqual(self.control_calls(), [("connect", False), ("teach", 1), ("disconnect",)])
        self.assertEqual(self.sdk_count, 1)
        self.assertEqual(report["application_commands_sent"], 1)
        self.assertTrue(report["command_acknowledged"])
        self.assertTrue(report["observation_completed"])
        self.assertFalse(report["automatic_restore"])
        self.assertTrue(report["teaching_may_still_be_active"])

    def test_motion_observation_does_not_claim_gravity_compensation(self):
        report = self.run_probe()
        self.assertNotIn("error", report)
        self.assertEqual(report["max_joint_change_deg"], [1.0] * 6)
        self.assertFalse(report["gravity_compensation_verified"])
        self.assertTrue(any(row["record_type"] == "teaching_ready" for row in self.events))
        self.assertTrue(any(row["record_type"] == "sample" for row in self.events))

    def test_stop_before_preparation_does_not_connect_or_send(self):
        self.stopped = True
        self.assert_rejected_without_write(self.run_probe())
        self.assertEqual(self.sdk_count, 0)

    def test_stop_after_connect_but_before_teach_disconnects_without_writing(self):
        self.stop_after_connect = True
        self.assert_rejected_without_write(self.run_probe())
        self.assertEqual(self.control_calls(), [("connect", False), ("disconnect",)])

    def test_ownership_change_after_connect_prevents_teach(self):
        self.guard_error_at = 3
        self.assert_rejected_without_write(self.run_probe())
        self.assertEqual(self.control_calls(), [("connect", False), ("disconnect",)])

    def test_feedback_failure_before_teach_cannot_send_command(self):
        self.read_error_at = 2
        self.assert_rejected_without_write(self.run_probe())
        self.assertEqual(self.control_calls(), [("connect", False), ("disconnect",)])

    def test_driver_disabling_during_teaching_stops_observation_without_mode_restore(self):
        def disabled(row):
            if self.sdk.teaching:
                row["driver_enabled"][1] = False
                row["driver_foc_bytes"][1] = 0
            return row
        self.modify_snapshot = disabled
        report = self.run_probe()
        self.assertIn("error", report)
        self.assertFalse(report["gravity_compensation_verified"])
        self.assertEqual(self.control_calls(), [("connect", False), ("teach", 1), ("disconnect",)])
        self.assertFalse(report["automatic_restore"])

    def test_missing_feedback_after_teach_does_not_trigger_enable_disable_or_reset(self):
        self.read_error_at = 4
        report = self.run_probe()
        self.assertIn("error", report)
        self.assertTrue(report["command_acknowledged"])
        self.assertEqual(self.control_calls(), [("connect", False), ("teach", 1), ("disconnect",)])
        self.assertFalse(report["automatic_restore"])

    def test_no_actual_teach_ack_is_rejected_by_three_second_deadline(self):
        self.mode_after_teach = 1
        report = self.run_probe(duration=4)
        self.assertIn("error", report)
        self.assertFalse(report["command_acknowledged"])
        self.assertLess(self.clock.now, 4)
        self.assertEqual(self.control_calls(), [("connect", False), ("teach", 1), ("disconnect",)])

    def test_short_observation_without_actual_ack_is_not_reported_as_success(self):
        self.mode_after_teach = 1
        report = self.run_probe(duration=1)
        self.assertIn("error", report)
        self.assertFalse(report["command_acknowledged"])
        self.assertFalse(report["gravity_compensation_verified"])
        self.assertEqual(self.control_calls(), [("connect", False), ("teach", 1), ("disconnect",)])

    def test_stop_after_command_does_not_exit_teach_or_add_targets(self):
        self.stop_after_teach = True
        report = self.run_probe()
        self.assertEqual(report["stop_reason"], "operator_stop")
        self.assertFalse(report["gravity_compensation_verified"])
        self.assertEqual(self.control_calls(), [("connect", False), ("teach", 1), ("disconnect",)])
        self.assertTrue(report["teaching_may_still_be_active"])


if __name__ == "__main__":
    unittest.main()
