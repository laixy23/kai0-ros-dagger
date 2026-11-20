"""Raw-feedback/ramp tests with a fake plant, no CAN or real SDK construction."""
from copy import deepcopy
from pathlib import Path
import json
import os
import signal
import sys
import tempfile
import unittest
from unittest import mock
from contextlib import redirect_stdout
import io

sys.path.insert(0, str(Path(__file__).parent))
import check_right_alignment as module


class Plant:
    def __init__(self):
        self.t = 1000.0
        self.master = [-9000, 6000, -300, -1300, 32000, 7700]
        self.slave = [-5000, 7200, -1400, -1900, 34500, 7600]
        self.master_mode = 1
        self.slave_mode = 1
        self.calls = []
        self.recoveries = []
        self.owners = []
        self.silent = False
        self.stuck_joint = None
        self.stale_slave_after_write = False
        self.move_slave_after_write = False
        self.master_fault_after_write = False
        self.nonfinite = False
        self.invalid_gripper = False
        self.writes = 0
        self.slave_read_delay = 0.0
        self.connect_shift = 0

    def clock(self):
        return self.t

    def pause(self, delay):
        self.t += delay

    def read(self, channel, since_timestamp=0):
        self.t += 0.03
        if channel == module.SLAVE:
            self.t += self.slave_read_delay
        if channel == module.MASTER and self.silent:
            raise RuntimeError("Missing required CAN feedback frames")
        if channel == module.MASTER and self.master_fault_after_write and self.writes:
            raise RuntimeError("Driver fault observed during sampling")
        q = list(self.master if channel == module.MASTER else self.slave)
        if self.nonfinite and channel == module.MASTER:
            q[1] = float("nan")
        if self.move_slave_after_write and self.writes and channel == module.SLAVE:
            q[1] += 1001
        ages = {f"0x{key:03X}": 0.001 for key in module.REQUIRED_IDS}
        if self.stale_slave_after_write and self.writes and channel == module.SLAVE:
            # Aggregate current timestamp with one older raw component is unsafe
            # even though its age is below the ordinary 0.2 second age limit.
            ages["0x2A6"] = 0.19
        return {"ctrl_mode": self.master_mode if channel == module.MASTER else self.slave_mode,
                "arm_status": 0, "err_code": 0, "gripper_mode": 1 if self.invalid_gripper else 0,
                "joint_mdeg": q, "gripper_um": -800 if channel == module.MASTER else 50000,
                "driver_enabled": [True] * 6, "component_age_s": ages, "timestamp_sec": self.t}

    def factory(self, channel):
        self.calls.append(("construct", channel))
        if channel != module.MASTER:
            raise AssertionError("Only the right master may get an SDK instance")
        return self

    def recover(self, channel, apply):
        self.recoveries.append((channel, apply))
        self.silent, self.master_mode = False, 1
        return {"passed": True, "enable_pose_max_drift_mdeg": 0}

    def ConnectPort(self, **kwargs):
        self.calls.append(("ConnectPort", kwargs))
        if self.connect_shift:
            self.t += 0.8
            self.master[1] += self.connect_shift

    def JointCtrl(self, *target):
        self.calls.append(("JointCtrl", list(target)))
        self.writes += 1
        for i in range(6):
            if i != self.stuck_joint:
                self.master[i] = target[i]

    def DisconnectPort(self):
        self.calls.append(("DisconnectPort",))

    def run(self, **kwargs):
        return module.diagnose(reader=self.read, sdk_factory=self.factory,
                               recovery=self.recover, controller_reader=lambda: self.owners,
                               clock=self.clock, wall=self.clock, pause=self.pause, **kwargs)


class RightAlignmentTests(unittest.TestCase):
    def test_passive_never_constructs_sdk_or_recovers_missing_fa(self):
        plant = Plant()
        plant.silent = True
        result = plant.run()
        self.assertFalse(result["passed"])
        self.assertEqual(plant.calls, [])
        self.assertEqual(plant.recoveries, [])

    def test_apply_requires_explicit_support_confirmation_before_any_read_or_sdk(self):
        plant = Plant()
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertEqual(plant.t, 1000)
        self.assertEqual(plant.calls, [])

    def test_existing_controller_refuses_any_recovery_or_sdk(self):
        plant = Plant()
        plant.owners = [{"pid": 55, "script": "piper_start_ms_node_new.py"}]
        result = plant.run(apply=True, confirm_supported=True)
        self.assertFalse(result["passed"])
        self.assertEqual(plant.calls, [])
        self.assertEqual(plant.recoveries, [])

    def test_all_six_reach_follower_only_master_joint_targets_with_bounded_ramp(self):
        plant = Plant()
        original_slave = list(plant.slave)
        result = plant.run(apply=True, confirm_supported=True)
        self.assertTrue(result["passed"], result.get("error"))
        self.assertEqual(plant.master, original_slave)
        self.assertEqual(plant.slave, original_slave)
        self.assertEqual(plant.recoveries, [])
        self.assertEqual(plant.calls[0], ("construct", module.MASTER))
        self.assertEqual(plant.calls[1], ("ConnectPort", {"piper_init": False}))
        self.assertTrue(all(call[0] in ("construct", "ConnectPort", "JointCtrl", "DisconnectPort")
                            for call in plant.calls))
        self.assertEqual(len(result["joint_comparison"]), 6)
        self.assertTrue(all(row["error_deg"] == 0 for row in result["joint_comparison"]))
        self.assertGreaterEqual(result["stable_duration_sec"], 0.3)
        commands = result["commands"]
        for before, after in zip(commands, commands[1:]):
            elapsed = after["monotonic_sec"] - before["monotonic_sec"]
            self.assertLessEqual(max(abs(a - b) for a, b in zip(after["target_mdeg"], before["target_mdeg"]))
                                 / elapsed, module.MAX_RATE_MDEG_S + 1e-6)

    def test_stuck_j2_cannot_pass_when_other_five_joints_follow(self):
        plant = Plant()
        plant.stuck_joint = 1
        result = plant.run(apply=True, confirm_supported=True, convergence_timeout=0.8)
        self.assertFalse(result["passed"])
        self.assertIn("convergence timeout", result["error"])
        self.assertAlmostEqual(result["joint_comparison"][1]["error_deg"], -1.2)
        self.assertTrue(result["failure_measured_hold_requested"])
        self.assertEqual(result["commands"][-1]["target_mdeg"], plant.master)

    def test_starting_difference_over_45_degrees_refuses_alignment_sdk(self):
        plant = Plant()
        plant.master[1] = plant.slave[1] + 45001
        result = plant.run(apply=True, confirm_supported=True)
        self.assertFalse(result["passed"])
        self.assertIn("45 degrees", result["error"])
        self.assertEqual(plant.calls, [])

    def test_silent_fa_master_is_explicitly_recovered_then_actual_pose_used(self):
        plant = Plant()
        plant.silent = True
        result = plant.run(apply=True, confirm_supported=True)
        self.assertTrue(result["passed"], result.get("error"))
        self.assertEqual(plant.recoveries, [(module.MASTER, True)])
        self.assertEqual(result["post_recovery_feedback"][module.MASTER]["joint_mdeg"],
                         [-9000, 6000, -300, -1300, 32000, 7700])

    def test_unsupported_master_or_follower_mode_never_gets_sdk(self):
        for side in ("master", "slave"):
            with self.subTest(side=side):
                plant = Plant()
                setattr(plant, side + "_mode", 3)
                result = plant.run(apply=True, confirm_supported=True)
                self.assertFalse(result["passed"])
                self.assertEqual(plant.calls, [])
                self.assertEqual(plant.recoveries, [])

    def test_nonfinite_pose_and_wrong_gripper_unit_refuse_any_write(self):
        for setting in ("nonfinite", "invalid_gripper"):
            with self.subTest(setting=setting):
                plant = Plant()
                setattr(plant, setting, True)
                result = plant.run(apply=True, confirm_supported=True)
                self.assertFalse(result["passed"])
                self.assertEqual(plant.calls, [])
                self.assertEqual(plant.recoveries, [])

    def test_one_precommand_slave_component_stops_and_only_holds_fresh_master(self):
        plant = Plant()
        plant.stale_slave_after_write = True
        result = plant.run(apply=True, confirm_supported=True)
        self.assertFalse(result["passed"])
        self.assertIn("pre-command raw CAN component", result["error"])
        self.assertTrue(result["failure_measured_hold_requested"])
        self.assertEqual(len(result["commands"]), 2)
        self.assertEqual(result["commands"][-1]["stage"], "failure_measured_hold")

    def test_follower_motion_over_one_degree_stops_alignment(self):
        plant = Plant()
        plant.move_slave_after_write = True
        result = plant.run(apply=True, confirm_supported=True)
        self.assertFalse(result["passed"])
        self.assertIn("follower moved", result["error"])
        self.assertTrue(result["failure_measured_hold_requested"])

    def test_fault_after_write_does_not_hold_untrusted_pose_or_disable(self):
        plant = Plant()
        plant.master_fault_after_write = True
        result = plant.run(apply=True, confirm_supported=True)
        self.assertFalse(result["passed"])
        self.assertNotIn("failure_measured_hold_requested", result)
        self.assertEqual(len(result["commands"]), 1)
        self.assertEqual(plant.calls[-1], ("DisconnectPort",))

    def test_feedback_that_ages_while_reading_other_arm_cannot_be_written(self):
        plant = Plant()
        plant.slave_read_delay = 0.3
        result = plant.run(apply=True, confirm_supported=True)
        self.assertFalse(result["passed"])
        self.assertIn("aged before write", result["error"])
        self.assertEqual(plant.writes, 0)

    def test_first_hold_is_measured_after_sdk_connection_delay(self):
        plant = Plant()
        plant.connect_shift = 500
        result = plant.run(apply=True, confirm_supported=True)
        self.assertTrue(result["passed"], result.get("error"))
        self.assertEqual(result["commands"][0]["target_mdeg"][1], 6500)
        self.assertEqual(result["post_connect_feedback"][module.MASTER]["joint_mdeg"][1], 6500)
        self.assertEqual(result["post_recovery_feedback"][module.MASTER]["joint_mdeg"][1], 6000)

    def test_new_controller_during_alignment_does_not_get_competing_hold(self):
        plant = Plant()
        result = module.diagnose(apply=True, confirm_supported=True, reader=plant.read,
                                sdk_factory=plant.factory, recovery=plant.recover,
                                controller_reader=lambda: [{"pid": 8}] if plant.writes else [],
                                clock=plant.clock, wall=plant.clock, pause=plant.pause)
        self.assertFalse(result["passed"])
        self.assertEqual(plant.writes, 1)
        self.assertNotIn("failure_measured_hold_requested", result)

    def test_cli_term_and_hup_during_ramp_request_only_fresh_measured_hold(self):
        for signum in (signal.SIGTERM, signal.SIGHUP):
            with self.subTest(signal=signum), tempfile.TemporaryDirectory() as directory:
                plant = Plant()
                original = module.diagnose
                old_handlers = {s: signal.getsignal(s) for s in (signal.SIGTERM, signal.SIGHUP)}
                self.addCleanup(lambda handlers=old_handlers: [signal.signal(s, h) for s, h in handlers.items()])
                def pause(delay):
                    plant.pause(delay)
                    if plant.writes:
                        os.kill(os.getpid(), signum)
                def injected(**kwargs):
                    return original(reader=plant.read, sdk_factory=plant.factory,
                                    recovery=plant.recover, controller_reader=lambda: [],
                                    clock=plant.clock, wall=plant.clock, pause=pause, **kwargs)
                output = Path(directory) / "report.json"
                argv = ["check", "--apply", "--confirm-supported", "--output", str(output)]
                with mock.patch.object(module, "ROOT", Path(directory)), mock.patch.object(sys, "argv", argv), \
                        mock.patch.object(module, "diagnose", side_effect=injected), redirect_stdout(io.StringIO()):
                    self.assertEqual(module.main(), 1)
                result = json.loads(output.read_text())
                self.assertTrue(result["failure_measured_hold_requested"])
                self.assertEqual(result["commands"][-1]["stage"], "failure_measured_hold")
                self.assertEqual(plant.calls[-1], ("DisconnectPort",))


if __name__ == "__main__":
    unittest.main()
