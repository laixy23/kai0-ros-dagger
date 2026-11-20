"""Manual observer tests with fake raw feedback and no hardware imports."""
from pathlib import Path
import sys
import json
import os
import signal
import tempfile
import unittest
from unittest import mock
from contextlib import redirect_stdout, redirect_stderr
import io

sys.path.insert(0, str(Path(__file__).parent))
import watch_right_joint_difference as module
from test_right_alignment import Plant


class ManualPlant(Plant):
    def __init__(self):
        super().__init__()
        self.refuse_teach = False
        self.drop_manual_feedback = False
        self.move_slave_on_manual = False
        self.stale = False
        self.unpowered = False
        self.partial_enable = False

    def MotionCtrl_1(self, **kwargs):
        self.calls.append(("MotionCtrl_1", kwargs))
        if not self.refuse_teach:
            self.master_mode = 2

    def read(self, channel, since_timestamp=0):
        if self.drop_manual_feedback and self.master_mode == 2 and channel == module.MASTER:
            raise RuntimeError("Missing required CAN feedback frames")
        row = super().read(channel, since_timestamp)
        if self.unpowered and channel == module.MASTER:
            row["driver_enabled"] = [False] * 6
            if self.partial_enable:
                row["driver_enabled"][0] = True
        if self.stale:
            row["component_age_s"]["0x2A6"] = 0.5
        if self.move_slave_on_manual and self.master_mode == 2 and channel == module.SLAVE:
            row["joint_mdeg"][1] += 1001
        return row

    def observe(self, **kwargs):
        return module.watch(reader=self.read, sdk_factory=self.factory,
                            controller_reader=lambda: self.owners, clock=self.clock,
                            wall=self.clock, pause=self.pause, **kwargs)


class ManualObserverTests(unittest.TestCase):
    def test_unpowered_observation_never_constructs_sdk_or_changes_motor_state(self):
        plant = ManualPlant()
        plant.unpowered, plant.master_mode = True, 0
        rows = []
        report = plant.observe(duration=0.8, unpowered_master=True, emit=rows.append)
        self.assertTrue(report["passed"], report.get("error"))
        self.assertEqual(plant.calls, [])
        self.assertTrue(rows[0]["all_six_disabled"])
        self.assertEqual(rows[0]["record_type"], "ready")
        self.assertEqual(len(report["final_sample"]["difference_deg"]), 6)

    def test_unpowered_observation_rejects_partial_enable_and_mode_preparation(self):
        plant = ManualPlant()
        plant.unpowered, plant.master_mode, plant.partial_enable = True, 0, True
        report = plant.observe(duration=0.8, unpowered_master=True)
        self.assertFalse(report["passed"])
        self.assertEqual(plant.calls, [])
        report = plant.observe(duration=0.8, unpowered_master=True, prepare_manual=True,
                               confirm_supported=True)
        self.assertFalse(report["passed"])
        self.assertEqual(plant.calls, [])

    def test_readonly_constructs_no_sdk_and_records_all_six_joint_differences(self):
        plant = ManualPlant()
        rows = []
        report = plant.observe(duration=0.8, emit=rows.append)
        self.assertTrue(report["passed"])
        self.assertEqual(plant.calls, [])
        samples = [row for row in rows if row["record_type"] == "sample"]
        self.assertTrue(samples)
        self.assertEqual(len(samples[0]["difference_deg"]), 6)
        self.assertEqual(samples[0]["difference_deg"][1], -1.2)
        self.assertEqual(rows[-1]["record_type"], "summary")

    def test_manual_entry_only_sends_single_teach_command_then_disconnects(self):
        plant = ManualPlant()
        rows = []
        report = plant.observe(duration=0.8, prepare_manual=True, confirm_supported=True, emit=rows.append)
        self.assertTrue(report["passed"], report.get("error"))
        self.assertTrue(report["manual_entry_acknowledged"])
        self.assertEqual(plant.master_mode, 2)
        self.assertEqual([call for call in plant.calls if call[0] == "MotionCtrl_1"],
                         [("MotionCtrl_1", {"grag_teach_ctrl": 1})])
        self.assertTrue(set(call[0] for call in plant.calls) <= {"construct", "ConnectPort", "MotionCtrl_1", "DisconnectPort"})
        self.assertEqual(rows[0]["record_type"], "ready")
        self.assertFalse(report["automatic_mode_restore"])

    def test_prepare_without_support_or_from_non_can_mode_never_constructs_sdk(self):
        plant = ManualPlant()
        report = plant.observe(duration=1, prepare_manual=True)
        self.assertFalse(report["passed"])
        self.assertEqual(plant.calls, [])
        plant.master_mode = 2
        report = plant.observe(duration=1, prepare_manual=True, confirm_supported=True)
        self.assertFalse(report["passed"])
        self.assertEqual(plant.calls, [])

    def test_missing_fa_and_stale_raw_components_fail_readonly_without_sdk(self):
        for setting in ("silent", "stale"):
            plant = ManualPlant()
            setattr(plant, setting, True)
            report = plant.observe(duration=1)
            self.assertFalse(report["passed"])
            self.assertEqual(plant.calls, [])

    def test_unacknowledged_or_missing_manual_feedback_never_falls_back_to_fa(self):
        for setting in ("refuse_teach", "drop_manual_feedback"):
            plant = ManualPlant()
            setattr(plant, setting, True)
            report = plant.observe(duration=1, prepare_manual=True, confirm_supported=True, ack_timeout=0.3)
            self.assertFalse(report["passed"])
            self.assertFalse(report.get("manual_entry_acknowledged", False))
            self.assertEqual([call[0] for call in plant.calls].count("MotionCtrl_1"), 1)
            self.assertTrue(set(call[0] for call in plant.calls) <= {"construct", "ConnectPort", "MotionCtrl_1", "DisconnectPort"})

    def test_follower_drift_stops_without_restoring_manual_or_sending_targets(self):
        plant = ManualPlant()
        plant.move_slave_on_manual = True
        report = plant.observe(duration=1, prepare_manual=True, confirm_supported=True)
        self.assertFalse(report["passed"])
        self.assertIn("follower moved", report["error"])
        self.assertEqual(plant.master_mode, 2)
        self.assertEqual([call[0] for call in plant.calls].count("MotionCtrl_1"), 1)

    def test_controller_appearance_stops_observer_without_mode_writes(self):
        plant = ManualPlant()
        def emit(row):
            if row["record_type"] == "sample":
                plant.owners = [{"pid": 88}]
        report = plant.observe(duration=2, emit=emit)
        self.assertFalse(report["passed"])
        self.assertIn("controller appeared", report["error"])
        self.assertEqual(plant.calls, [])

    def test_cli_term_ends_observation_and_preserves_manual_mode(self):
        plant = ManualPlant()
        original = module.watch
        old = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
        self.addCleanup(lambda: [signal.signal(s, handler) for s, handler in old.items()])
        def pause(delay):
            plant.pause(delay)
            if plant.master_mode == 2:
                os.kill(os.getpid(), signal.SIGTERM)
        def injected(**kwargs):
            return original(reader=plant.read, sdk_factory=plant.factory,
                            controller_reader=lambda: [], clock=plant.clock, wall=plant.clock,
                            pause=pause, **kwargs)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "sample.jsonl"
            argv = ["watch", "--prepare-manual", "--confirm-supported", "--output", str(path)]
            with mock.patch.object(module, "ROOT", Path(directory)), mock.patch.object(sys, "argv", argv), \
                    mock.patch.object(module, "watch", side_effect=injected), redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
                self.assertEqual(module.main(), 0)
            rows = [json.loads(line) for line in path.read_text().splitlines()]
            self.assertEqual(rows[-1]["stop_reason"], "operator_stop")
            self.assertFalse(rows[-1]["automatic_mode_restore"])
        self.assertEqual(plant.master_mode, 2)
        self.assertEqual([call[0] for call in plant.calls].count("MotionCtrl_1"), 1)


if __name__ == "__main__":
    unittest.main()
