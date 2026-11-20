"""Current-pose startup enabling, verified against independent raw feedback."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parent))
import enable_arm_current_pose as module
from prepare_follower_enabled import checked_enabled_feedback
from prepare_follower_mode import REQUIRED_IDS


class Plant:
    def __init__(self, *, mode=1, enabled=False, enable_delay=0,
                 enable_effect=True, enable_return=None, refuse_can=False,
                 connection_drift=0, drift_after=None, interrupt_at=None,
                 raise_after_enable=False, patch=None):
        self.t = 1000.0
        self.mode = mode
        self.enabled = ([enabled] * 6 if type(enabled) is bool else list(enabled))
        self.pose = [1000, 30000, -35000, 4000, 22000, -2000]
        self.target = [90000] * 6
        self.calls = []
        self.reads = 0
        self.enable_delay = enable_delay
        self.enable_remaining = None
        self.enable_effect = enable_effect
        self.enable_return = enable_return
        self.refuse_can = refuse_can
        self.connection_drift = connection_drift
        self.drift_after = drift_after
        self.interrupt_at = interrupt_at
        self.raise_after_enable = raise_after_enable
        self.patch = patch or {}
        self.joint_written = False

    def clock(self):
        return self.t

    def pause(self, seconds):
        self.t += seconds

    def reader(self, channel, since_timestamp=0.0):
        self.t += 0.12
        self.reads += 1
        if self.reads == self.interrupt_at:
            raise KeyboardInterrupt
        if self.enable_remaining is not None:
            if self.enable_remaining <= 0 and self.enable_effect:
                self.enabled = [True] * 6
            self.enable_remaining -= 1
        if self.mode == 1 and all(self.enabled) and self.joint_written:
            self.pose = list(self.target)
        if self.drift_after is not None and self.reads >= self.drift_after:
            self.pose[0] += 2000
        row = {
            'ctrl_mode': self.mode, 'arm_status': 0, 'err_code': 0,
            'gripper_mode': 0, 'driver_enabled': list(self.enabled),
            'driver_foc_bytes': [0x40 if bit else 0 for bit in self.enabled],
            'joint_mdeg': list(self.pose), 'timestamp_sec': self.t,
            'component_age_s': {f'0x{key:03X}': 0.01 for key in REQUIRED_IDS},
        }
        row.update(deepcopy(self.patch))
        return row

    def factory(self, channel):
        self.calls.append(('factory', channel))
        return self

    def ConnectPort(self, **kwargs):
        self.calls.append(('ConnectPort', kwargs))
        self.pose[0] += self.connection_drift

    def DisconnectPort(self):
        self.calls.append(('DisconnectPort',))

    def JointCtrl(self, *target):
        self.calls.append(('JointCtrl', target))
        # A disabled motor may ignore the seed; the seed immediately after
        # EnableArm and subsequent fixed-pose commands must stand on their own.
        if self.mode == 1 and all(self.enabled):
            self.target = list(target)
            self.joint_written = True

    def MotionCtrl_1(self, **kwargs):
        self.calls.append(('MotionCtrl_1', kwargs))
        self.mode = 0

    def MotionCtrl_2(self, **kwargs):
        self.calls.append(('MotionCtrl_2', kwargs))
        if not self.refuse_can:
            self.mode = 1

    def EnableArm(self, motor):
        self.calls.append(('EnableArm', motor))
        self.enable_remaining = self.enable_delay
        if self.enable_delay == 0 and self.enable_effect:
            self.enabled = [True] * 6
        if self.raise_after_enable:
            raise RuntimeError('simulated transport failure after EnableArm')
        return self.enable_return

    def run(self, channel='can_left_mas', **kwargs):
        return module.enable_at_current_pose(
            channel, reader=self.reader, sdk_factory=self.factory,
            clock=self.clock, wall=self.clock, pause=self.pause,
            step_timeout=1.2, **kwargs)


class CurrentPoseEnableTests(unittest.TestCase):
    def assert_no_torque_removal_or_other_control(self, plant, report):
        names = {call[0] for call in plant.calls}
        self.assertFalse(names & {'DisableArm', 'ResetPiper', 'GripperCtrl',
                                 'MasterSlaveConfig', 'MotionCtrl_3'})
        self.assertFalse(report['torque_removal_requested'])
        self.assertFalse(report['reset_requested'])
        self.assertEqual(report['gripper_commands_requested'], 0)
        self.assertFalse(report['initial_pose_requested'])
        self.assertEqual(plant.calls[-1], ('DisconnectPort',))
        targets = [list(call[1]) for call in plant.calls if call[0] == 'JointCtrl']
        self.assertTrue(targets)
        self.assertTrue(all(target == report['holding_target_mdeg'] for target in targets))
        self.assertNotEqual(report['holding_target_mdeg'], [0] * 6)

    def test_all_four_enabled_can1_channels_are_passive(self):
        for channel in module.CHANNELS:
            for apply in (False, True):
                with self.subTest(channel=channel, apply=apply):
                    plant = Plant(enabled=True)
                    result = plant.run(channel, apply=apply)
                    self.assertTrue(result['passed'], result)
                    self.assertTrue(result['already_ready'])
                    self.assertEqual(result['sdk_instances_created'], 0)
                    self.assertEqual(result['hardware_control_writes'], 0)
                    self.assertEqual(plant.calls, [])

    def test_disabled_and_partial_plan_never_construct_sdk(self):
        for enabled in (False, [True, True, False, True, False, True]):
            with self.subTest(enabled=enabled):
                plant = Plant(enabled=enabled)
                result = plant.run()
                self.assertTrue(result['passed'], result)
                self.assertTrue(result['plan_only'])
                self.assertEqual(plant.calls, [])
                self.assertFalse(result['enable_requested'])

    def test_disabled_or_partial_can1_seeds_enables_and_immediately_reseeds(self):
        for channel in module.CHANNELS:
            for enabled in (False, [True, False, True, True, False, True]):
                with self.subTest(channel=channel, enabled=enabled):
                    plant = Plant(enabled=enabled)
                    result = plant.run(channel, apply=True)
                    self.assertTrue(result['passed'], result)
                    self.assertTrue(result['enable_requested'])
                    self.assertEqual(plant.calls[1], ('ConnectPort', {'piper_init': False}))
                    self.assertEqual([call[0] for call in plant.calls[2:5]],
                                     ['JointCtrl', 'EnableArm', 'JointCtrl'])
                    self.assertEqual(plant.calls[3], ('EnableArm', 7))
                    self.assertGreaterEqual(result['stable_elapsed_s'], 0.3)
                    self.assertTrue(all(result['final_feedback']['driver_enabled']))
                    self.assert_no_torque_removal_or_other_control(plant, result)

    def test_standby_and_teaching_use_existing_mode_sequence_before_enabling(self):
        for mode in (0, 2):
            for enabled in (False, True):
                with self.subTest(mode=mode, enabled=enabled):
                    plant = Plant(mode=mode, enabled=enabled)
                    result = plant.run(apply=True)
                    self.assertTrue(result['passed'], result)
                    names = [call[0] for call in plant.calls[2:]]
                    self.assertEqual(names[:5], ['JointCtrl', 'MotionCtrl_1',
                                                'JointCtrl', 'MotionCtrl_2', 'JointCtrl'])
                    self.assertEqual(plant.calls[3], ('MotionCtrl_1', {'grag_teach_ctrl': 0x02}))
                    self.assertEqual(plant.calls[5], ('MotionCtrl_2', {
                        'ctrl_mode': 1, 'move_mode': 1, 'move_spd_rate_ctrl': 10, 'is_mit_mode': 0}))
                    self.assertEqual(names.count('EnableArm'), int(not enabled))
                    if not enabled:
                        self.assertEqual(names[5:7], ['EnableArm', 'JointCtrl'])
                    self.assert_no_torque_removal_or_other_control(plant, result)

    def test_fresh_postconnection_pose_replaces_initial_once_without_drift_gate(self):
        plant = Plant(mode=2, connection_drift=2500, drift_after=3)
        result = plant.run(apply=True)
        self.assertTrue(result['passed'], result)
        self.assertNotEqual(result['initial_feedback']['joint_mdeg'], result['holding_target_mdeg'])
        self.assertEqual(result['pre_write_feedback']['joint_mdeg'], result['holding_target_mdeg'])
        self.assertGreaterEqual(result['observed_max_drift_mdeg'], 2000)
        self.assertTrue(result['drift_is_diagnostic_only'])
        self.assert_no_torque_removal_or_other_control(plant, result)

    def test_false_sdk_return_does_not_override_true_raw_feedback(self):
        plant = Plant(enable_return=False)
        result = plant.run(apply=True)
        self.assertTrue(result['passed'], result)
        self.assert_no_torque_removal_or_other_control(plant, result)

    def test_old_false_bits_after_enable_are_polled_until_actual_enable(self):
        plant = Plant(enable_delay=3)
        result = plant.run(apply=True)
        self.assertTrue(result['passed'], result)
        self.assertGreaterEqual(plant.reads, 8)
        self.assertEqual([call[0] for call in plant.calls].count('EnableArm'), 1)
        self.assert_no_torque_removal_or_other_control(plant, result)

    def test_enable_command_without_raw_effect_cannot_pass(self):
        for enable_return in (None, True):
            with self.subTest(enable_return=enable_return):
                plant = Plant(enable_effect=False, enable_return=enable_return)
                result = plant.run(apply=True)
                self.assertFalse(result['passed'], result)
                self.assertTrue(result['error'].startswith('TimeoutError('), result)
                self.assertEqual([call[0] for call in plant.calls].count('EnableArm'), 1)
                self.assert_no_torque_removal_or_other_control(plant, result)

    def test_software_enabled_flags_without_raw_foc_enable_cannot_pass(self):
        plant = Plant(patch={'driver_enabled': [True] * 6,
                             'driver_foc_bytes': [0] * 6})
        result = plant.run(apply=True)
        self.assertFalse(result['passed'], result)
        self.assertTrue(result['error'].startswith('TimeoutError('), result)
        self.assert_no_torque_removal_or_other_control(plant, result)

    def test_mode_refusal_cannot_pass_even_if_enable_succeeds(self):
        plant = Plant(mode=2, refuse_can=True)
        result = plant.run(apply=True)
        self.assertFalse(result['passed'], result)
        self.assertTrue(result['error'].startswith('TimeoutError('), result)
        self.assert_no_torque_removal_or_other_control(plant, result)

    def test_invalid_faults_units_and_freshness_still_reject_before_sdk(self):
        patches = [
            {'arm_status': 1}, {'err_code': 1}, {'gripper_mode': 1},
            {'driver_enabled': [1] * 6}, {'driver_foc_bytes': [1] * 6},
            {'joint_mdeg': [float('nan')] * 6}, {'joint_mdeg': [1] * 5},
            {'timestamp_sec': 999.0},
            {'component_age_s': {f'0x{key:03X}': 0.3 for key in REQUIRED_IDS}},
        ]
        for patch in patches:
            with self.subTest(patch=patch):
                plant = Plant(patch=patch)
                result = plant.run(apply=True)
                self.assertFalse(result['passed'], result)
                self.assertEqual(plant.calls, [])

    def test_unknown_channel_rejects_before_reading_or_connecting(self):
        plant = Plant()
        result = plant.run('can0', apply=True)
        self.assertFalse(result['passed'])
        self.assertEqual(plant.reads, 0)
        self.assertEqual(plant.calls, [])

    def test_interrupt_or_failure_after_enable_only_disconnects(self):
        for args in ({'interrupt_at': 3}, {'raise_after_enable': True}):
            with self.subTest(args=args):
                plant = Plant(**args)
                result = plant.run(apply=True)
                self.assertFalse(result['passed'])
                self.assertTrue(result['enable_requested'])
                self.assertEqual(result['interrupted'], 'interrupt_at' in args)
                self.assert_no_torque_removal_or_other_control(plant, result)

    def test_checked_feedback_default_still_rejects_disabled_or_partial(self):
        for enabled in (False, [True, False, True, True, True, True]):
            with self.subTest(enabled=enabled):
                plant = Plant(enabled=enabled)
                row = plant.reader('can_left_mas')
                with self.assertRaisesRegex(RuntimeError, 'all six'):
                    checked_enabled_feedback(row, now=plant.clock())
                joints, stamps = checked_enabled_feedback(
                    row, now=plant.clock(), require_enabled=False)
                self.assertEqual(joints, plant.pose)
                self.assertEqual(len(stamps), len(REQUIRED_IDS))


if __name__ == '__main__':
    unittest.main()
