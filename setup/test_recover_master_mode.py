#!/usr/bin/env python3
"""Fake-plant transition tests; no CAN or real SDK instance is constructed."""
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


sys.path.insert(0, str(Path(__file__).parent))
spec = importlib.util.spec_from_file_location("recover_master", Path(__file__).with_name("recover_master_mode.py"))
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class Plant:
    def __init__(self, *, silent_fa=False, always_silent=False, ignore_disable=False,
                 unstable=False, refuse_can=False, drop_can=False, enable_drift=False,
                 ignore_disabled_targets=False, faulty_after_enable=False,
                 reset_enables=False, reset_silent=False):
        self.t = 1000.0
        self.read_count = 0
        self.role = 0xFA
        self.mode = 2
        self.enabled = True
        self.joints = [1000, 30000, -35000, 100, 22000, -100]
        self.gripper = 54000
        self.target = [90000] * 6
        self.calls = []
        self.silent_fa, self.always_silent = silent_fa, always_silent
        self.ignore_disable, self.unstable = ignore_disable, unstable
        self.refuse_can, self.drop_can = refuse_can, drop_can
        self.enable_drift = enable_drift
        self.ignore_disabled_targets = ignore_disabled_targets
        self.faulty_after_enable = faulty_after_enable
        self.reset_enables, self.reset_silent = reset_enables, reset_silent
        self.reset_once = False
        self.enabled_once = False
        self.can_reads = 0
        self.actuate_on_read = False

    def clock(self):
        return self.t

    def pause(self, span):
        self.t += span

    def read(self, channel, since_timestamp=0):
        self.t += 0.13
        self.read_count += 1
        if self.always_silent or (self.silent_fa and self.role == 0xFA) or (self.reset_silent and self.reset_once):
            raise RuntimeError("Missing required CAN feedback frames")
        if self.faulty_after_enable and self.enabled_once:
            raise RuntimeError("Driver fault observed during sampling")
        if self.actuate_on_read:
            self.joints = list(self.target)
            self.actuate_on_read = False
        if self.unstable and not self.enabled:
            self.joints[0] = 1000 if self.read_count % 2 else 2500
        if self.mode == 1 and not self.enabled:
            self.can_reads += 1
            if self.drop_can and self.can_reads == 2:
                self.mode = 2
        return {"ctrl_mode": self.mode, "arm_status": 0, "err_code": 0,
                "joint_mdeg": list(self.joints), "driver_enabled": [self.enabled] * 6,
                "timestamp_sec": self.t, "gripper_um": self.gripper}

    def factory(self, channel):
        self.calls.append(("construct_sdk", channel))
        return self

    def ConnectPort(self, **kwargs):
        self.calls.append(("ConnectPort", kwargs))

    def DisconnectPort(self):
        self.calls.append(("DisconnectPort",))

    def DisableArm(self, motor):
        self.calls.append(("DisableArm", motor))
        if not self.ignore_disable:
            self.enabled = False

    def MasterSlaveConfig(self, **kwargs):
        self.calls.append(("MasterSlaveConfig", kwargs))
        self.role = kwargs["linkage_config"]

    def MotionCtrl_1(self, **kwargs):
        self.calls.append(("MotionCtrl_1", kwargs))
        self.mode = 0
        self.joints[1] += 1500  # Supported pose settles differently after exiting teach.

    def ResetPiper(self):
        self.calls.append(("ResetPiper", {"all_six_disabled": not self.enabled}))
        self.reset_once = True
        self.mode = 0
        self.joints[0] = 6200
        self.target = [88000] * 6  # Old holding target must not be reused.
        if self.reset_enables:
            self.enabled = True

    def MotionCtrl_2(self, **kwargs):
        self.calls.append(("MotionCtrl_2", kwargs))
        if not self.refuse_can:
            self.mode = 1

    def JointCtrl(self, *target):
        self.calls.append(("JointCtrl", target))
        if self.mode == 1 and (self.enabled or not self.ignore_disabled_targets):
            self.target = list(target)

    def EnableArm(self, motor):
        self.calls.append(("EnableArm", motor))
        self.enabled, self.enabled_once = True, True
        if self.ignore_disabled_targets:
            self.actuate_on_read = True
        else:
            self.joints = list(self.target)
            if self.enable_drift:
                self.joints[0] += 3000

    def run(self, **kwargs):
        return module.recover("can_left_mas", reader=self.read, sdk_factory=self.factory,
                              controller_reader=lambda: [], clock=self.clock, wall=self.clock,
                              pause=self.pause, step_timeout=0.8, **kwargs)


class MasterRecoveryTests(unittest.TestCase):
    def test_default_is_plan_without_sdk_even_when_fa_has_no_feedback(self):
        plant = Plant(silent_fa=True)
        report = plant.run()
        self.assertTrue(report["passed"])
        self.assertTrue(report["plan_only"])
        self.assertTrue(report["hardware_ready_not_verified"])
        self.assertEqual(report["hardware_control_writes"], 0)
        self.assertIn("initial_feedback_error", report)
        self.assertEqual(plant.calls, [])

    def test_only_exact_master_whitelist_is_allowed(self):
        for channel in ("can_left_slave", "can0", "can_right_mas ", "can_right_mas;echo hi"):
            with self.subTest(channel=channel), self.assertRaisesRegex(RuntimeError, "MASTER"):
                module.recover(channel, apply=True)

    def test_active_or_stopped_controller_refuses_any_sdk_construction(self):
        plant = Plant()
        report = module.recover("can_right_mas", apply=True, reader=plant.read,
                                sdk_factory=plant.factory,
                                controller_reader=lambda: [{"pid": 7707, "state": "T"}])
        self.assertFalse(report["passed"])
        self.assertEqual(plant.calls, [])

    def test_success_bootstraps_silent_fa_then_holds_actual_settled_pose(self):
        plant = Plant(silent_fa=True)
        report = plant.run(apply=True)
        self.assertTrue(report["passed"], report)
        self.assertEqual(plant.role, 0xFC)
        self.assertEqual(plant.mode, 1)
        self.assertTrue(plant.enabled)
        self.assertEqual(report["holding_target_mdeg"], [1000, 31500, -35000, 100, 22000, -100])
        self.assertEqual(plant.joints, report["holding_target_mdeg"])
        self.assertEqual(plant.gripper, 54000)
        self.assertEqual(plant.calls[1], ("ConnectPort", {"piper_init": False}))
        names = [call[0] for call in plant.calls]
        role_index = names.index("MasterSlaveConfig")
        self.assertEqual(names[role_index - 3:role_index], ["DisableArm"] * 3)
        self.assertEqual(names[role_index + 1], "DisableArm")
        self.assertEqual(plant.calls[role_index][1], {"linkage_config": 0xFC, "feedback_offset": 0,
                                                     "ctrl_offset": 0, "linkage_offset": 0})
        mode_index, enable_index = names.index("MotionCtrl_2"), names.index("EnableArm")
        self.assertIn("JointCtrl", names[mode_index + 1:enable_index])
        self.assertEqual(plant.calls[enable_index + 1], ("JointCtrl", tuple(report["holding_target_mdeg"])))
        self.assertEqual(report["enable_pose_max_drift_mdeg"], 0)
        self.assertTrue(set(names) <= {"construct_sdk", "ConnectPort", "DisableArm", "MasterSlaveConfig",
                                      "MotionCtrl_1", "MotionCtrl_2", "JointCtrl", "EnableArm", "DisconnectPort"})

    def test_no_feedback_after_fc_never_exits_teach_sets_target_or_enables(self):
        plant = Plant(always_silent=True)
        report = plant.run(apply=True)
        self.assertFalse(report["passed"])
        names = [call[0] for call in plant.calls]
        self.assertEqual(names.count("MasterSlaveConfig"), 1)
        self.assertTrue(all(name == "DisableArm" for name in names[names.index("MasterSlaveConfig") + 1:-1]))
        self.assertFalse(any(name in names for name in ("MotionCtrl_1", "MotionCtrl_2", "JointCtrl", "EnableArm")))

    def test_explicit_reset_occurs_once_disabled_and_reseeds_postreset_actual_pose(self):
        plant = Plant(silent_fa=True)
        report = plant.run(apply=True, allow_reset=True)
        self.assertTrue(report["passed"], report)
        self.assertTrue(report["reset_selected"])
        self.assertEqual(report["reset_commands_requested"], 1)
        names = [call[0] for call in plant.calls]
        self.assertEqual(names.count("ResetPiper"), 1)
        reset_index = names.index("ResetPiper")
        self.assertLess(names.index("MasterSlaveConfig"), reset_index)
        self.assertLess(reset_index, names.index("MotionCtrl_1"))
        self.assertEqual(plant.calls[reset_index], ("ResetPiper", {"all_six_disabled": True}))
        self.assertEqual(report["reset_disabled_feedback"]["driver_enabled"], [False] * 6)
        self.assertEqual(report["holding_target_mdeg"], [6200, 31500, -35000, 100, 22000, -100])
        self.assertEqual(plant.joints, report["holding_target_mdeg"])
        logged = [entry["method"] for entry in report["sdk_command_log"]]
        self.assertEqual(logged.count("ResetPiper"), 1)

    def test_reset_selection_requires_apply_and_never_constructs_sdk(self):
        plant = Plant()
        with self.assertRaisesRegex(RuntimeError, "requires --apply"):
            plant.run(allow_reset=True)
        self.assertEqual(plant.calls, [])

    def test_reset_never_runs_before_all_six_disabled_ack(self):
        plant = Plant(ignore_disable=True)
        report = plant.run(apply=True, allow_reset=True)
        self.assertFalse(report["passed"])
        self.assertFalse(any(call[0] in ("ResetPiper", "MotionCtrl_1", "JointCtrl", "EnableArm")
                             for call in plant.calls))

    def test_missing_feedback_after_reset_never_exits_teach_or_enables(self):
        plant = Plant(reset_silent=True)
        report = plant.run(apply=True, allow_reset=True)
        self.assertFalse(report["passed"])
        names = [call[0] for call in plant.calls]
        self.assertEqual(names.count("ResetPiper"), 1)
        self.assertFalse(any(name in names for name in ("MotionCtrl_1", "JointCtrl", "EnableArm")))

    def test_reset_unexpectedly_enabling_drivers_fails_without_target_or_enable(self):
        plant = Plant(reset_enables=True)
        report = plant.run(apply=True, allow_reset=True)
        self.assertFalse(report["passed"])
        names = [call[0] for call in plant.calls]
        self.assertEqual(names.count("ResetPiper"), 1)
        self.assertFalse(any(name in names for name in ("MotionCtrl_1", "JointCtrl", "EnableArm")))

    def test_control_mode_failure_after_explicit_reset_does_not_retry_or_enable(self):
        plant = Plant(refuse_can=True)
        report = plant.run(apply=True, allow_reset=True)
        self.assertFalse(report["passed"])
        self.assertFalse(plant.enabled)
        names = [call[0] for call in plant.calls]
        self.assertEqual(names.count("ResetPiper"), 1)
        self.assertNotIn("EnableArm", names)

    def test_failed_default_control_transition_never_selects_reset(self):
        plant = Plant(refuse_can=True)
        report = plant.run(apply=True)
        self.assertFalse(report["passed"])
        self.assertFalse(report["reset_selected"])
        self.assertEqual(report["reset_commands_requested"], 0)
        self.assertFalse(any(call[0] == "ResetPiper" for call in plant.calls))

    def test_disabled_ack_failure_blocks_all_later_mode_or_target_commands(self):
        plant = Plant(ignore_disable=True)
        report = plant.run(apply=True)
        self.assertFalse(report["passed"])
        self.assertFalse(any(call[0] in ("MotionCtrl_1", "JointCtrl", "EnableArm") for call in plant.calls))

    def test_initial_fault_or_bad_gripper_unit_does_not_begin_bootstrap(self):
        for message in ("Arm fault: status=1", "V3 gripper mode must be WIDTH(0)"):
            with self.subTest(message=message):
                plant = Plant()
                def faulty(*args, **kwargs):
                    raise RuntimeError(message)
                report = module.recover("can_left_mas", apply=True, reader=faulty,
                                        sdk_factory=plant.factory, controller_reader=lambda: [])
                self.assertFalse(report["passed"])
                self.assertEqual(plant.calls, [])

    def test_one_still_enabled_driver_blocks_exiting_teach(self):
        plant = Plant()
        original = plant.read
        def one_enabled(*args, **kwargs):
            snapshot = original(*args, **kwargs)
            snapshot["driver_enabled"][-1] = True
            return snapshot
        report = module.recover("can_right_mas", apply=True, reader=one_enabled,
                                sdk_factory=plant.factory, controller_reader=lambda: [],
                                clock=plant.clock, wall=plant.clock, pause=plant.pause, step_timeout=0.8)
        self.assertFalse(report["passed"])
        self.assertFalse(any(call[0] in ("MotionCtrl_1", "JointCtrl", "EnableArm") for call in plant.calls))

    def test_unstable_supported_pose_prevents_target_or_enable(self):
        plant = Plant(unstable=True)
        report = plant.run(apply=True)
        self.assertFalse(report["passed"])
        self.assertFalse(any(call[0] in ("JointCtrl", "EnableArm") for call in plant.calls))

    def test_can_mode_failure_leaves_disabled_without_enable(self):
        plant = Plant(refuse_can=True)
        report = plant.run(apply=True)
        self.assertFalse(report["passed"])
        self.assertFalse(plant.enabled)
        self.assertFalse(any(call[0] == "EnableArm" for call in plant.calls))

    def test_can_mode_loss_before_enable_aborts(self):
        plant = Plant(drop_can=True)
        report = plant.run(apply=True)
        self.assertFalse(report["passed"])
        self.assertFalse(any(call[0] == "EnableArm" for call in plant.calls))

    def test_ignored_disabled_target_is_reseeded_immediately_after_enable(self):
        plant = Plant(ignore_disabled_targets=True)
        report = plant.run(apply=True)
        self.assertTrue(report["passed"], report)
        self.assertEqual(plant.joints, report["holding_target_mdeg"])

    def test_enable_drift_fails_holds_fresh_actual_and_never_disables(self):
        plant = Plant(enable_drift=True)
        report = plant.run(apply=True)
        self.assertFalse(report["passed"])
        self.assertTrue(report["failure_measured_hold_requested"])
        names = [call[0] for call in plant.calls]
        self.assertNotIn("DisableArm", names[names.index("EnableArm") + 1:])
        self.assertEqual(plant.calls[-2], ("JointCtrl", tuple(plant.joints)))
        self.assertTrue(plant.enabled)

    def test_fault_after_enable_never_holds_untrusted_pose_or_disables(self):
        plant = Plant(faulty_after_enable=True)
        report = plant.run(apply=True)
        self.assertFalse(report["passed"])
        self.assertNotIn("failure_measured_hold_requested", report)
        names = [call[0] for call in plant.calls]
        self.assertEqual(names[names.index("EnableArm") + 1:], ["JointCtrl", "DisconnectPort"])


class ControllerOwnershipTests(unittest.TestCase):
    def test_python_slot_parser_excludes_shell_and_inline_strings(self):
        script = "piper_start_ms_node_new.py"
        self.assertEqual(module.script_argument(["python3", "-u", "/tmp/" + script]), script)
        self.assertIsNone(module.script_argument(["ssh", "host", "python3 /tmp/" + script]))
        self.assertIsNone(module.script_argument(["python3", "-c", "print('" + script + "')"]))

    def test_stopped_controller_blocks_but_zombie_and_embedded_string_do_not(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for pid, state, argv in ((1, "T", ["python3", "/tmp/piper_start_ms_node_new.py"]),
                                     (2, "Z", ["python3", "/tmp/agilex_openpi_dagger_collect.py"]),
                                     (3, "S", ["bash", "-c", "piper_start_ms_node_new.py"])):
                path = root / str(pid)
                path.mkdir()
                (path / "cmdline").write_bytes(b"\0".join(part.encode() for part in argv))
                (path / "status").write_text("State:\t" + state + "\n")
            self.assertEqual(module.active_controllers(root), [{"pid": 1, "state": "T", "script": "piper_start_ms_node_new.py"}])


if __name__ == "__main__":
    unittest.main()
