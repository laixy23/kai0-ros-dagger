#!/usr/bin/env python3
"""State-machine and raw-frame tests; no CAN or real SDK imports/instances."""
import importlib.util
from pathlib import Path
import struct
import unittest


spec = importlib.util.spec_from_file_location("prepare_mode", Path(__file__).with_name("prepare_follower_mode.py"))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Plant:
    def __init__(self, *, refuse_can=False, ignore_disable=False, unstable=False,
                 enable_drift=False, drop_mode=False, ignore_disabled_targets=False):
        self.t = 1000.0
        self.mode = 2
        self.enabled = True
        self.joints = [1000, 30000, -35000, 0, 22000, 0]
        self.gripper = 17000
        self.accepted_target = [90000] * 6  # Deliberately stale, dangerous target.
        self.calls = []
        self.refuse_can, self.ignore_disable = refuse_can, ignore_disable
        self.unstable, self.enable_drift, self.drop_mode = unstable, enable_drift, drop_mode
        self.reads_in_can = 0
        self.ignore_disabled_targets = ignore_disabled_targets
        self.actuate_on_read = False

    def clock(self):
        return self.t

    def read(self, channel, since_timestamp=0):
        self.t += 0.13
        if self.actuate_on_read:
            self.joints = list(self.accepted_target)
            self.actuate_on_read = False
        if self.unstable and not self.enabled:
            self.joints[0] = 1000 if int(self.t * 100) % 2 else 2500
        if self.mode == 1 and not self.enabled:
            self.reads_in_can += 1
            if self.drop_mode and self.reads_in_can == 2:
                self.mode = 2
        return {"ctrl_mode": self.mode, "arm_status": 0, "err_code": 0,
                "joint_mdeg": list(self.joints), "gripper_um": self.gripper,
                "driver_enabled": [self.enabled] * 6,
                "driver_foc_bytes": [0x40 if self.enabled else 0] * 6,
                "component_age_s": {f"0x{key:03X}": 0.01 for key in module.REQUIRED_IDS},
                "timestamp_sec": self.t}

    def factory(self, channel):
        return self

    def ConnectPort(self, **kwargs):
        self.calls.append(("ConnectPort", kwargs))

    def DisconnectPort(self):
        self.calls.append(("DisconnectPort",))

    def DisableArm(self, motor):
        self.calls.append(("DisableArm", motor))
        if not self.ignore_disable:
            self.enabled = False

    def MotionCtrl_1(self, **kwargs):
        self.calls.append(("MotionCtrl_1", kwargs))
        self.mode = 0

    def ResetPiper(self):
        self.calls.append(("ResetPiper",))
        self.mode, self.enabled = 0, False
        self.joints[0] = 6200  # Supported arm settles differently after reset.
        self.accepted_target = [88000] * 6

    def MotionCtrl_2(self, **kwargs):
        self.calls.append(("MotionCtrl_2", kwargs))
        if not self.refuse_can:
            self.mode = 1

    def JointCtrl(self, *target):
        self.calls.append(("JointCtrl", tuple(target)))
        # Faithfully expose why preloading only before joint mode is insufficient.
        if self.mode == 1 and (self.enabled or not self.ignore_disabled_targets):
            self.accepted_target = list(target)

    def EnableArm(self, motor):
        self.calls.append(("EnableArm", motor))
        self.enabled = True
        if self.ignore_disabled_targets:
            self.actuate_on_read = True
            return
        self.joints = list(self.accepted_target)
        if self.enable_drift:
            self.joints[0] += 3000

    def run(self, **kwargs):
        return module.prepare("can_left_slave", reader=self.read, sdk_factory=self.factory,
                              clock=self.clock, wall=self.clock, step_timeout=0.8, **kwargs)


def frames():
    output = {key: (bytes(8), 100.0) for key in module.REQUIRED_IDS}
    output[0x2A1] = (bytes([2, 0, 1, 0, 0, 0, 0, 0]), 100.0)
    for key, pair in zip((0x2A5, 0x2A6, 0x2A7), ((1200, -1300), (0, 2200), (-3300, 4400))):
        output[key] = (struct.pack(">ii", *pair), 100.0)
    output[0x2A8] = (struct.pack(">ii", 17000, 0), 100.0)
    for key in range(0x261, 0x267):
        output[key] = (bytes([0, 0, 0, 0, 0, 0x40, 0, 0]), 100.0)
    return output


class RawFeedbackTests(unittest.TestCase):
    def test_raw_units_and_all_six_enable_bits(self):
        result = module.decode_feedback(frames(), 100.01)
        self.assertEqual(result["joint_mdeg"], [1200, -1300, 0, 2200, -3300, 4400])
        self.assertEqual(result["gripper_um"], 17000)
        self.assertEqual(result["driver_enabled"], [True] * 6)

    def test_one_stale_motor_is_not_masked_by_other_fresh_motors(self):
        values = frames()
        values[0x263] = (values[0x263][0], 99.0)
        with self.assertRaisesRegex(RuntimeError, "Stale"):
            module.decode_feedback(values, 100.01)

    def test_precommand_feedback_cannot_ack_a_transition(self):
        with self.assertRaisesRegex(RuntimeError, "pre-command"):
            module.decode_feedback(frames(), 100.01, since_timestamp=100.005)

    def test_missing_frame_and_driver_fault_are_rejected(self):
        values = frames()
        del values[0x2A7]
        with self.assertRaisesRegex(RuntimeError, "Missing"):
            module.decode_feedback(values, 100.01)
        values = frames()
        values[0x265] = (bytes([0, 0, 0, 0, 0, 0x41, 0, 0]), 100.0)
        with self.assertRaisesRegex(RuntimeError, "driver fault"):
            module.decode_feedback(values, 100.01)

    def test_v3_angle_gripper_feedback_is_rejected_without_mode_change(self):
        values = frames()
        values[0x2A8] = (struct.pack(">i", 90000) + bytes([0, 0, 0, 1]), 100.0)
        with self.assertRaisesRegex(RuntimeError, "WIDTH"):
            module.decode_feedback(values, 100.01)

    def test_arm_fault_and_error_code_are_rejected(self):
        for payload in (bytes([2, 1, 1, 0, 0, 0, 0, 0]), bytes([2, 0, 1, 0, 0, 0, 0, 1])):
            values = frames()
            values[0x2A1] = (payload, 100.0)
            with self.assertRaisesRegex(RuntimeError, "Arm fault"):
                module.decode_feedback(values, 100.01)


class PreparationTests(unittest.TestCase):
    def test_default_is_passive_and_never_constructs_sdk(self):
        plant = Plant()
        result = plant.run()
        self.assertTrue(result["passed"])
        self.assertEqual(result["hardware_control_writes"], 0)
        self.assertEqual(plant.calls, [])

    def test_noreset_success_reseeds_after_joint_mode_before_enable(self):
        plant = Plant()
        initial = list(plant.joints)
        result = plant.run(apply=True)
        self.assertTrue(result["passed"], result)
        self.assertEqual(plant.joints, initial)
        self.assertEqual(plant.gripper, 17000)
        names = [call[0] for call in plant.calls]
        mode_index, enable_index = names.index("MotionCtrl_2"), names.index("EnableArm")
        self.assertIn("JointCtrl", names[mode_index + 1:enable_index])
        self.assertNotIn("ResetPiper", names)
        self.assertNotIn("GripperCtrl", names)
        self.assertEqual(plant.calls[0], ("ConnectPort", {"piper_init": False}))
        self.assertEqual(names[-1], "DisconnectPort")

    def test_can_mode_failure_aborts_disabled_without_reset_or_enable(self):
        plant = Plant(refuse_can=True)
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertFalse(plant.enabled)
        self.assertFalse(any(call[0] in ("ResetPiper", "EnableArm") for call in plant.calls))

    def test_firmware_ignoring_disabled_targets_gets_immediate_postenable_hold(self):
        plant = Plant(ignore_disabled_targets=True)
        initial = list(plant.joints)
        result = plant.run(apply=True)
        self.assertTrue(result["passed"], result)
        names = [call[0] for call in plant.calls]
        enable_index = names.index("EnableArm")
        self.assertEqual(plant.calls[enable_index + 1], ("JointCtrl", tuple(initial)))
        self.assertEqual(plant.joints, initial)

    def test_disable_ack_failure_never_exits_teach_or_sets_target(self):
        plant = Plant(ignore_disable=True)
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertFalse(any(call[0] in ("MotionCtrl_1", "JointCtrl", "EnableArm") for call in plant.calls))

    def test_one_driver_still_enabled_prevents_mode_or_target_commands(self):
        plant = Plant()
        original_reader = plant.read
        def one_driver_enabled(channel, since_timestamp=0):
            value = original_reader(channel, since_timestamp)
            value["driver_enabled"][-1] = True
            return value
        result = module.prepare("can_right_slave", apply=True, reader=one_driver_enabled,
                                sdk_factory=plant.factory, clock=plant.clock, wall=plant.clock,
                                step_timeout=0.8)
        self.assertFalse(result["passed"])
        self.assertFalse(any(call[0] in ("MotionCtrl_1", "JointCtrl", "EnableArm") for call in plant.calls))

    def test_explicit_reset_occurs_once_and_rereads_settled_pose(self):
        plant = Plant()
        result = plant.run(apply=True, allow_reset=True)
        self.assertTrue(result["passed"], result)
        self.assertEqual(sum(call[0] == "ResetPiper" for call in plant.calls), 1)
        self.assertEqual(result["holding_target_mdeg"][0], 6200)
        self.assertEqual(plant.joints[0], 6200)

    def test_reset_flag_requires_apply(self):
        with self.assertRaisesRegex(RuntimeError, "requires"):
            Plant().run(allow_reset=True)

    def test_unstable_supported_pose_prevents_enable(self):
        plant = Plant(unstable=True)
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertFalse(any(call[0] == "EnableArm" for call in plant.calls))

    def test_mode_loss_after_ack_prevents_enable(self):
        plant = Plant(drop_mode=True)
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertFalse(any(call[0] == "EnableArm" for call in plant.calls))

    def test_enable_drift_holds_fresh_pose_and_never_auto_disables(self):
        plant = Plant(enable_drift=True)
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertTrue(result["failure_measured_hold_requested"])
        names = [call[0] for call in plant.calls]
        self.assertNotIn("DisableArm", names[names.index("EnableArm") + 1:])
        self.assertTrue(plant.enabled)
        self.assertEqual(plant.calls[-2], ("JointCtrl", tuple(plant.joints)))

    def test_fault_feedback_after_enable_does_not_blind_hold_or_disable(self):
        plant = Plant()
        original_reader = plant.read
        def faulty_after_enable(channel, since_timestamp=0):
            result = original_reader(channel, since_timestamp)
            if any(call[0] == "EnableArm" for call in plant.calls):
                raise RuntimeError("Stale or faulted enabled feedback")
            return result
        result = module.prepare("can_left_slave", apply=True, reader=faulty_after_enable,
                                sdk_factory=plant.factory, clock=plant.clock, wall=plant.clock,
                                step_timeout=0.8)
        self.assertFalse(result["passed"])
        self.assertNotIn("failure_measured_hold_requested", result)
        names = [call[0] for call in plant.calls]
        self.assertEqual(names[names.index("EnableArm") + 1:], ["JointCtrl", "DisconnectPort"])

    def test_interrupt_after_enable_attempts_fresh_hold_without_disable(self):
        plant = Plant()
        original_reader = plant.read
        interrupted = False
        def interrupt_once(channel, since_timestamp=0):
            nonlocal interrupted
            result = original_reader(channel, since_timestamp)
            if not interrupted and any(call[0] == "EnableArm" for call in plant.calls):
                interrupted = True
                raise KeyboardInterrupt()
            return result
        result = module.prepare("can_left_slave", apply=True, reader=interrupt_once,
                                sdk_factory=plant.factory, clock=plant.clock, wall=plant.clock,
                                step_timeout=0.8)
        self.assertFalse(result["passed"])
        self.assertTrue(result["failure_measured_hold_requested"])
        names = [call[0] for call in plant.calls]
        self.assertNotIn("DisableArm", names[names.index("EnableArm") + 1:])

    def test_master_channel_is_rejected_without_read_or_sdk(self):
        with self.assertRaisesRegex(RuntimeError, "Only"):
            module.prepare("can_left_mas", sdk_factory=lambda channel: self.fail("SDK created"))


if __name__ == "__main__":
    unittest.main()
