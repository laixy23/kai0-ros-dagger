#!/usr/bin/env python3
"""Raw-feedback convergence tests with a fake clock/reader and no hardware."""
import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parent))
import master_alignment_feedback as module


TARGETS = {side: [0.1, 0.32, -0.36, 0.05, 0.24, -0.1, 0.07]
           for side in ("left", "right")}


class Plant:
    def __init__(self, transform=None):
        self.t = 1000.0
        self.calls = []
        self.published = []
        self.transform = transform
        self.converted = module.alignment_targets(TARGETS)

    def clock(self):
        return self.t

    def pause(self, duration):
        self.t += duration

    def read(self, channel, since_timestamp=0):
        self.t += 0.06
        self.calls.append((channel, since_timestamp))
        side = next(side for side, name in module.CHANNELS.items() if name == channel)
        row = {"ctrl_mode": 1, "arm_status": 0, "err_code": 0, "gripper_mode": 0,
               "driver_enabled": [True] * 6, "driver_foc_bytes": [64] * 6,
               "joint_mdeg": list(self.converted[side]["joint_mdeg"]),
               "timestamp_sec": self.t, "gripper_um": 54900,
               "component_age_s": dict.fromkeys(module.COMPONENTS, 0.005)}
        if self.transform:
            self.transform(side, row, len(self.calls))
        return row

    def run(self, **overrides):
        arguments = dict(since_timestamp=1000.0, reader=self.read, clock=self.clock,
                         wall=self.clock, pause=self.pause, timeout=1.5,
                         publish=lambda: self.published.append(copy.deepcopy(TARGETS)))
        arguments.update(overrides)
        return module.wait_for_master_alignment(TARGETS, **arguments)


class AlignmentFeedbackTests(unittest.TestCase):
    def test_success_needs_both_six_joint_feedbacks_sustained_and_ignores_gripper_error(self):
        plant = Plant()
        report = plant.run()
        self.assertTrue(report["passed"], report)
        self.assertGreaterEqual(report["stable_elapsed_s"], 0.3)
        self.assertGreaterEqual(report["samples"], 3)
        self.assertTrue(all(row["within_tolerance"] for row in report["sides"].values()))
        self.assertEqual(report["sides"]["right"]["gripper_error_um"], -15100)
        self.assertFalse(report["gripper_matching_required"])
        self.assertEqual(report["hardware_api_calls"], 0)
        self.assertTrue(all(target == TARGETS for target in plant.published))
        self.assertTrue(all(since == 1000 for _, since in plant.calls))

    def test_target_wire_units_match_deployed_driver_factor_exactly(self):
        actual = module.alignment_targets(TARGETS)
        self.assertEqual(actual["right"]["joint_mdeg"],
                         [round(value * 57324.840764) for value in TARGETS["right"][:6]])
        self.assertNotEqual(actual["right"]["joint_mdeg"][1], round(0.32 * 180000 / math.pi))

    def test_right_j2_stuck_cannot_be_hidden_by_left_success(self):
        def stuck(side, row, _):
            if side == "right":
                row["joint_mdeg"][1] += 5000
        report = Plant(stuck).run()
        self.assertFalse(report["passed"])
        self.assertTrue(report["sides"]["left"]["within_tolerance"])
        self.assertEqual(report["sides"]["right"]["error_degrees"], [0, 5, 0, 0, 0, 0])

    def test_every_right_arm_joint_is_checked(self):
        for joint in range(6):
            with self.subTest(joint=joint + 1):
                def stuck(side, row, _, joint=joint):
                    if side == "right":
                        row["joint_mdeg"][joint] += 1001
                self.assertFalse(Plant(stuck).run()["passed"])

    def test_boundary_tolerance_is_inclusive(self):
        def edge(side, row, _):
            row["joint_mdeg"] = [value + 1000 for value in row["joint_mdeg"]]
        self.assertTrue(Plant(edge).run()["passed"])

    def test_brief_alignment_does_not_accumulate_across_out_of_tolerance_samples(self):
        def intermittent(side, row, count):
            if side == "right" and (count // 2) % 2 == 0:
                row["joint_mdeg"][1] += 3000
        report = Plant(intermittent).run()
        self.assertFalse(report["passed"])

    def test_one_old_component_rejects_fresh_aggregate_timestamp(self):
        for component in module.COMPONENTS:
            with self.subTest(component=component):
                def stale(side, row, _, component=component):
                    row["component_age_s"][component] = 0.15  # Fresh-looking aggregate, frame predates command.
                report = Plant(stale).run()
                self.assertFalse(report["passed"])
                self.assertIn(component, report["error"])

    def test_cached_raw_frames_cannot_gain_freshness_from_retimestamping(self):
        first_stamps = {}
        def cached(side, row, _):
            first_stamps.setdefault(side, row["timestamp_sec"] - 0.005)
            row["component_age_s"] = dict.fromkeys(module.COMPONENTS,
                                                   row["timestamp_sec"] - first_stamps[side])
        report = Plant(cached).run()
        self.assertFalse(report["passed"])
        self.assertIn("did not advance", report["error"])
        self.assertEqual(report["samples"], 1)

    def test_missing_stale_future_components_are_rejected(self):
        for bad_age in (None, 0.201, -0.051, float("nan")):
            with self.subTest(age=bad_age):
                def malformed(side, row, _):
                    if bad_age is None:
                        row["component_age_s"].pop("0x263")
                    else:
                        row["component_age_s"]["0x263"] = bad_age
                self.assertFalse(Plant(malformed).run()["passed"])

    def test_fault_mode_disabled_driver_and_bad_unit_fail_closed(self):
        mutations = [lambda row: row.update(ctrl_mode=2),
                     lambda row: row.update(arm_status=1),
                     lambda row: row.update(err_code=4),
                     lambda row: row.update(gripper_mode=1),
                     lambda row: row["driver_enabled"].__setitem__(5, False),
                     lambda row: row["driver_foc_bytes"].__setitem__(1, 65)]
        for index, mutation in enumerate(mutations):
            with self.subTest(case=index):
                report = Plant(lambda side, row, _: mutation(row)).run()
                self.assertFalse(report["passed"])

    def test_bad_actual_joint_vectors_and_timestamps_fail_closed(self):
        for field, bad in (("joint_mdeg", [0] * 5), ("joint_mdeg", [0, 0, 0, 0, 0, float("nan")]),
                           ("timestamp_sec", float("nan")), ("timestamp_sec", 999.0),
                           ("gripper_um", float("nan"))):
            with self.subTest(field=field, bad=bad):
                report = Plant(lambda side, row, _: row.update({field: bad})).run()
                self.assertFalse(report["passed"])

    def test_invalid_targets_never_publish_or_read(self):
        for bad in ([0] * 6, [0] * 6 + [float("nan")], [float("inf")] + [0] * 6,
                    ["0"] + [0] * 6, [True] + [0] * 6):
            with self.subTest(target=bad):
                plant = Plant()
                targets = dict(TARGETS, right=bad)
                report = module.wait_for_master_alignment(
                    targets, since_timestamp=1000, reader=plant.read, clock=plant.clock,
                    wall=plant.clock, publish=lambda: plant.published.append("unexpected"))
                self.assertFalse(report["passed"])
                self.assertEqual(plant.calls, [])
                self.assertEqual(plant.published, [])

    def test_reader_fault_stops_waiting_and_timeout_is_bounded(self):
        plant = Plant()
        def fault(*args, **kwargs):
            raise RuntimeError("Driver fault observed during sampling")
        report = plant.run(reader=fault)
        self.assertFalse(report["passed"])
        self.assertIn("Driver fault", report["error"])
        plant = Plant(lambda side, row, _: row["joint_mdeg"].__setitem__(0, 99999))
        report = plant.run(timeout=0.8)
        self.assertFalse(report["passed"])
        self.assertLessEqual(report["elapsed_s"], 0.8)

    def test_shutdown_or_ctrl_c_never_reports_success(self):
        plant = Plant()
        self.assertFalse(plant.run(shutdown=lambda: True)["passed"])
        self.assertEqual(plant.calls, [])
        def ctrl_c():
            raise KeyboardInterrupt
        with self.assertRaises(KeyboardInterrupt):
            plant.run(publish=ctrl_c)

    def test_shutdown_during_last_raw_sample_blocks_success(self):
        plant = Plant()
        report = plant.run(shutdown=lambda: len(plant.calls) >= 8)
        self.assertFalse(report["passed"])
        self.assertIn("shutdown", report["error"])

    def test_short_preparation_overlaps_the_full_three_second_hold(self):
        plant = Plant()
        report = plant.run(timeout=8, stable_seconds=3, initial_stable_since=1000.0,
                           completion_ready=lambda: plant.t >= 1000.2)
        self.assertTrue(report['passed'], report)
        self.assertGreaterEqual(report['elapsed_s'], 3.0)
        self.assertLess(report['elapsed_s'], 3.2)

    def test_slow_preparation_does_not_restart_or_append_the_hold(self):
        plant = Plant()
        report = plant.run(timeout=8, stable_seconds=3, initial_stable_since=1000.0,
                           completion_ready=lambda: plant.t >= 1004.2)
        self.assertTrue(report['passed'], report)
        self.assertGreaterEqual(report['elapsed_s'], 4.2)
        self.assertLess(report['elapsed_s'], 4.4)
        self.assertGreater(len(plant.published), 20)

    def test_worker_failure_guard_aborts_between_samples(self):
        plant = Plant()
        def guard():
            if plant.t >= 1000.25:
                raise RuntimeError('parallel preparation failed')
        report = plant.run(timeout=8, stable_seconds=3, initial_stable_since=1000,
                           completion_ready=lambda: False, guard=guard)
        self.assertFalse(report['passed'])
        self.assertIn('parallel preparation failed', report['error'])
        self.assertLess(report['elapsed_s'], 0.4)

    def test_stable_hold_cannot_pass_without_parallel_completion(self):
        report = Plant().run(timeout=4, stable_seconds=3, initial_stable_since=1000,
                             completion_ready=lambda: False)
        self.assertFalse(report['passed'])
        self.assertGreaterEqual(report['stable_elapsed_s'], 3)
        self.assertIn('parallel preparation was not ready', report['error'])

    def test_future_initial_stability_time_is_rejected_before_publication(self):
        plant = Plant()
        report = plant.run(initial_stable_since=1000.01)
        self.assertFalse(report['passed'])
        self.assertEqual(plant.published, [])
        self.assertEqual(plant.calls, [])


if __name__ == "__main__":
    unittest.main()
