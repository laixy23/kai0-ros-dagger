"""Enabled-mode transition tests using independent raw-feedback timestamps."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parent))
import prepare_follower_enabled as module


class Plant:
    def __init__(self, *, mode=2, disabled=False, refuse_can=False,
                 drop_enable_at=None, drift_at=None, creep=0,
                 stale_component=None, freeze_after=None, fault_at=None,
                 interrupt_at=None):
        self.t = 1000.0
        self.mode = mode
        self.enabled = not disabled
        self.pose = [1000, 30000, -35000, 0, 22000, 0]
        self.target = [90000] * 6  # A cached target must be replaced on transition.
        self.calls = []
        self.reads = 0
        self.refuse_can = refuse_can
        self.drop_enable_at, self.drift_at = drop_enable_at, drift_at
        self.creep = creep
        self.stale_component, self.freeze_after = stale_component, freeze_after
        self.fault_at, self.interrupt_at = fault_at, interrupt_at
        self.previous_row = None

    def clock(self):
        return self.t

    def reader(self, channel, since_timestamp=0.0):
        self.t += 0.12
        self.reads += 1
        if self.interrupt_at == self.reads:
            raise KeyboardInterrupt
        if self.drop_enable_at is not None and self.reads >= self.drop_enable_at:
            self.enabled = False
        if self.mode == 1:
            self.pose = list(self.target)
            self.pose[0] += self.creep * self.reads
        if self.drift_at is not None and self.reads >= self.drift_at:
            self.pose[0] += 1500
        row = {"ctrl_mode": self.mode, "arm_status": 0, "err_code": 0,
               "gripper_mode": 0, "driver_enabled": [self.enabled] * 6,
               "driver_foc_bytes": [0x40 if self.enabled else 0] * 6,
               "joint_mdeg": list(self.pose), "timestamp_sec": self.t,
               "component_age_s": {f"0x{key:03X}": 0.01 for key in module.REQUIRED_IDS}}
        if self.stale_component:
            row["component_age_s"][self.stale_component] = 0.3
        if self.fault_at == self.reads:
            row["err_code"] = 1
        if self.freeze_after is not None and self.reads > self.freeze_after:
            row = deepcopy(self.previous_row)
        self.previous_row = deepcopy(row)
        return row

    def factory(self, channel):
        self.calls.append(("factory", channel))
        return self

    def ConnectPort(self, **kwargs):
        self.calls.append(("ConnectPort", kwargs))

    def DisconnectPort(self):
        self.calls.append(("DisconnectPort",))

    def JointCtrl(self, *target):
        self.calls.append(("JointCtrl", target))
        if self.mode == 1:
            self.target = list(target)

    def MotionCtrl_1(self, **kwargs):
        self.calls.append(("MotionCtrl_1", kwargs))
        self.mode = 0

    def MotionCtrl_2(self, **kwargs):
        self.calls.append(("MotionCtrl_2", kwargs))
        if not self.refuse_can:
            self.mode = 1

    def run(self, **kwargs):
        return module.prepare_enabled(
            "can_left_slave", reader=self.reader, sdk_factory=self.factory,
            clock=self.clock, wall=self.clock, step_timeout=1.2, **kwargs)


class EnabledFollowerTests(unittest.TestCase):
    def assert_no_torque_commands(self, plant):
        names = [call[0] for call in plant.calls]
        self.assertFalse(set(names) & {"DisableArm", "EnableArm", "ResetPiper", "GripperCtrl", "MasterSlaveConfig"})

    def test_can1_ready_is_passive_even_when_apply_requested(self):
        plant = Plant(mode=1)
        result = plant.run(apply=True)
        self.assertTrue(result["passed"], result)
        self.assertTrue(result["already_ready"])
        self.assertEqual(plant.calls, [])
        self.assertEqual(result["hardware_control_writes"], 0)

    def test_default_teaching_plan_never_creates_sdk(self):
        plant = Plant()
        result = plant.run()
        self.assertTrue(result["passed"])
        self.assertTrue(result["plan_only"])
        self.assertEqual(plant.calls, [])

    def test_enabled_teaching_preloads_and_immediately_reseeds_fixed_measured_target(self):
        plant = Plant()
        initial = list(plant.pose)
        result = plant.run(apply=True)
        self.assertTrue(result["passed"], result)
        names = [call[0] for call in plant.calls]
        self.assertEqual(plant.calls[1], ("ConnectPort", {"piper_init": False}))
        first_joint = names.index("JointCtrl")
        self.assertEqual(names[first_joint:first_joint + 5],
                         ["JointCtrl", "MotionCtrl_1", "JointCtrl", "MotionCtrl_2", "JointCtrl"])
        for call in plant.calls:
            if call[0] == "JointCtrl":
                self.assertEqual(list(call[1]), initial)
        self.assertEqual(plant.target, initial)
        self.assertEqual(result["holding_target_mdeg"], initial)
        self.assertGreaterEqual(result["stable_elapsed_s"], 0.3)
        self.assertEqual(result["observed_max_drift_mdeg"], 0)
        self.assert_no_torque_commands(plant)

    def test_disabled_initial_feedback_aborts_without_connecting(self):
        plant = Plant(disabled=True)
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertIn("already have all six", result["error"])
        self.assertEqual(plant.calls, [])

    def test_partial_enable_aborts_without_connecting(self):
        plant = Plant()
        def partial(channel, since_timestamp=0.0):
            row = plant.reader(channel, since_timestamp)
            row["driver_enabled"][3] = False
            return row
        result = module.prepare_enabled("can_right_slave", apply=True, reader=partial,
                                        sdk_factory=plant.factory, wall=plant.clock, clock=plant.clock)
        self.assertFalse(result["passed"])
        self.assertEqual(plant.calls, [])

    def test_standby_and_other_modes_do_not_write(self):
        for mode in (0, 3):
            with self.subTest(mode=mode):
                plant = Plant(mode=mode)
                self.assertFalse(plant.run(apply=True)["passed"])
                self.assertEqual(plant.calls, [])

    def test_stale_individual_component_is_rejected_even_with_fresh_aggregate(self):
        plant = Plant(stale_component="0x261")
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertIn("0x261", result["error"])
        self.assertEqual(plant.calls, [])

    def test_nonadvancing_components_stop_before_first_control_write(self):
        plant = Plant(freeze_after=1)
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertIn("did not advance", result["error"])
        self.assertFalse(result["sdk_command_log"])
        self.assertEqual(plant.calls[-1], ("DisconnectPort",))

    def test_fresh_aggregate_with_pretransition_driver_stamp_cannot_ack_mode(self):
        plant = Plant()
        original = plant.reader
        def pretransition(channel, since_timestamp=0.0):
            row = original(channel, since_timestamp)
            if plant.reads >= 3:
                row["component_age_s"]["0x261"] = 0.15
            return row
        plant.reader = pretransition
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertIn("pre-command", result["error"])
        self.assertEqual(len(result["sdk_command_log"]), 5)
        self.assert_no_torque_commands(plant)

    def test_drift_during_sdk_connection_refreshes_target_once_without_rejection(self):
        plant = Plant(drift_at=2)
        result = plant.run(apply=True)
        self.assertTrue(result["passed"], result)
        self.assertTrue(result['drift_is_diagnostic_only'])
        expected = result['pre_write_feedback']['joint_mdeg']
        self.assertNotEqual(result['initial_feedback']['joint_mdeg'], expected)
        self.assertEqual(result['holding_target_mdeg'], expected)
        targets = [list(call[1]) for call in plant.calls if call[0] == 'JointCtrl']
        self.assertTrue(targets)
        self.assertTrue(all(target == expected for target in targets))
        self.assertGreater(result['observed_max_drift_mdeg'], 1000)
        self.assertEqual(plant.calls[-1], ("DisconnectPort",))
        self.assert_no_torque_commands(plant)

    def test_invalid_initial_feedback_does_not_construct_sdk(self):
        for patch in ({"arm_status": 1}, {"err_code": 1}, {"gripper_mode": 1},
                      {"driver_foc_bytes": [0x41] * 6}, {"joint_mdeg": [float("nan")] * 6}):
            with self.subTest(patch=patch):
                plant = Plant()
                def invalid(channel, since_timestamp=0.0):
                    row = plant.reader(channel, since_timestamp)
                    row.update(patch)
                    return row
                result = module.prepare_enabled("can_left_slave", apply=True, reader=invalid,
                                                sdk_factory=plant.factory, wall=plant.clock, clock=plant.clock)
                self.assertFalse(result["passed"])
                self.assertEqual(plant.calls, [])

    def test_missing_raw_feedback_blocks_without_commands(self):
        plant = Plant()
        def missing(channel, since_timestamp=0.0):
            raise RuntimeError("Missing required CAN feedback frames")
        result = module.prepare_enabled("can_left_slave", apply=True, reader=missing,
                                        sdk_factory=plant.factory)
        self.assertFalse(result["passed"])
        self.assertEqual(plant.calls, [])

    def test_enable_loss_after_mode_change_aborts_without_reenable_or_disable(self):
        plant = Plant(drop_enable_at=3)
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertIn("all six", result["error"])
        self.assertEqual(len(result["sdk_command_log"]), 5)
        self.assert_no_torque_commands(plant)

    def test_mode_refusal_times_out_without_fallback(self):
        plant = Plant(refuse_can=True)
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertTrue(result["error"].startswith("TimeoutError("), result)
        self.assert_no_torque_commands(plant)

    def test_large_pose_drift_is_reported_without_rejection_or_reanchoring(self):
        plant = Plant(drift_at=3)
        result = plant.run(apply=True)
        self.assertTrue(result["passed"], result)
        self.assertEqual(result["observed_max_drift_mdeg"], 1500)
        targets = [tuple(row['args']) for row in result['sdk_command_log'] if row['method'] == 'JointCtrl']
        self.assertEqual(set(targets), {tuple(result['holding_target_mdeg'])})
        self.assert_no_torque_commands(plant)

    def test_creeping_drift_is_diagnostic_and_does_not_reanchor_the_commands(self):
        plant = Plant(creep=300)
        result = plant.run(apply=True)
        self.assertTrue(result["passed"], result)
        self.assertGreater(result["observed_max_drift_mdeg"], 1000)
        targets = [tuple(row['args']) for row in result['sdk_command_log'] if row['method'] == 'JointCtrl']
        self.assertEqual(set(targets), {tuple(result['holding_target_mdeg'])})
        self.assert_no_torque_commands(plant)

    def test_fault_after_transition_does_not_request_blind_holding_or_torque_change(self):
        plant = Plant(fault_at=3)
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertEqual(len(result["sdk_command_log"]), 5)
        self.assert_no_torque_commands(plant)

    def test_interrupt_disconnects_without_additional_control_writes(self):
        plant = Plant(interrupt_at=3)
        result = plant.run(apply=True)
        self.assertFalse(result["passed"])
        self.assertTrue(result["interrupted"])
        self.assertEqual(len(result["sdk_command_log"]), 5)
        self.assertEqual(plant.calls[-1], ("DisconnectPort",))
        self.assert_no_torque_commands(plant)

    def test_master_channel_is_rejected_before_read_or_sdk(self):
        plant = Plant()
        result = module.prepare_enabled("can_left_mas", apply=True, reader=plant.reader,
                                        sdk_factory=plant.factory)
        self.assertFalse(result["passed"])
        self.assertEqual(plant.reads, 0)
        self.assertEqual(plant.calls, [])


if __name__ == "__main__":
    unittest.main()
