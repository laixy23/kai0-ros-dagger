"""Native-feedback, role-bootstrap and no-torque-toggle recovery tests."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parent))
import recover_master_enabled as module
from prepare_follower_mode import REQUIRED_IDS


class Plant:
    def __init__(self, *, mode=2, silent=False, disabled=False, refuse_can=False,
                 never_feedback=False, delay_post_fc=0, disable_on_fc=False,
                 fault_valid_read=None, drift_valid_read=None, stale_valid_read=None,
                 precommand_after_fc=False, missing_after_anchor=False,
                 connection_drift=False, frozen_after_anchor=False,
                 interrupt_valid_read=None, busy_check=None, fc_mode=0):
        self.t = 1000.0
        self.mode = mode
        self.silent, self.enabled = silent, not disabled
        self.pose = [-18533, 4200, -64000, 5020, 22330, 0]
        self.fa_commands = [90000] * 6  # Never a measured pose.
        self.target = [88000] * 6  # Firmware may ignore targets before CAN1.
        self.refuse_can, self.never_feedback = refuse_can, never_feedback
        self.delay_post_fc, self.disable_on_fc = delay_post_fc, disable_on_fc
        self.fault_valid_read, self.drift_valid_read = fault_valid_read, drift_valid_read
        self.stale_valid_read, self.precommand_after_fc = stale_valid_read, precommand_after_fc
        self.missing_after_anchor, self.connection_drift = missing_after_anchor, connection_drift
        self.frozen_after_anchor, self.interrupt_valid_read = frozen_after_anchor, interrupt_valid_read
        self.busy_check = busy_check
        self.owner_checks = self.valid_reads = self.reads = self.post_fc_reads = 0
        self.fc_sent = False
        self.fc_mode = fc_mode
        self.can_command_sent = False
        self.calls = []
        self.first_native_pose = None
        self.first_native_time = None
        self.saved_row = None

    def clock(self):
        return self.t

    def pause(self, seconds):
        self.t += seconds

    def owners(self):
        self.owner_checks += 1
        return [{'pid': 123, 'script': 'piper_start_ms_node_new.py'}] if self.busy_check is not None and self.owner_checks >= self.busy_check else []

    def reader(self, channel, since_timestamp=0.0):
        self.t += 0.12
        self.reads += 1
        if self.never_feedback or self.silent and not self.fc_sent:
            raise RuntimeError(module.MISSING_PREFIX)
        if self.fc_sent:
            self.post_fc_reads += 1
            if self.post_fc_reads <= self.delay_post_fc:
                raise RuntimeError(module.MISSING_PREFIX)
        if self.missing_after_anchor and self.valid_reads:
            raise RuntimeError(module.MISSING_PREFIX)
        self.valid_reads += 1
        if self.interrupt_valid_read == self.valid_reads:
            raise KeyboardInterrupt
        if self.mode == 1 and (not self.fc_sent or self.can_command_sent):
            self.pose = list(self.target)
        if self.drift_valid_read == self.valid_reads:
            self.pose[0] += 1500
        row = {'ctrl_mode': self.mode, 'arm_status': 0, 'err_code': 0,
               'gripper_mode': 0, 'driver_enabled': [self.enabled] * 6,
               'driver_foc_bytes': [0x40 if self.enabled else 0] * 6,
               'joint_mdeg': list(self.pose), 'timestamp_sec': self.t,
               'component_age_s': {f'0x{key:03X}': 0.01 for key in REQUIRED_IDS}}
        if self.fault_valid_read == self.valid_reads:
            row['err_code'] = 1
        if self.stale_valid_read == self.valid_reads:
            row['component_age_s']['0x264'] = 0.3
        if self.precommand_after_fc and self.fc_sent and self.valid_reads == 1:
            row['component_age_s']['0x261'] = 0.15
        if self.first_native_pose is None:
            self.first_native_pose, self.first_native_time = list(self.pose), self.t
        if self.frozen_after_anchor and self.saved_row is not None:
            return deepcopy(self.saved_row)
        self.saved_row = deepcopy(row)
        self.calls.append(('native', self.t, channel))
        return row

    def factory(self, channel):
        self.calls.append(('factory', channel))
        return self

    def ConnectPort(self, **kwargs):
        self.calls.append(('ConnectPort', kwargs))
        if self.connection_drift:
            self.pose[0] += 1500

    def DisconnectPort(self):
        self.calls.append(('DisconnectPort',))

    def MasterSlaveConfig(self, **kwargs):
        self.calls.append(('MasterSlaveConfig', kwargs, self.t))
        self.fc_sent = True
        self.mode = self.fc_mode  # Normal FC semantics: standby; a firmware may report CAN1.
        if self.disable_on_fc:
            self.enabled = False

    def JointCtrl(self, *target):
        self.calls.append(('JointCtrl', target, self.t))
        if self.first_native_time is None:
            raise AssertionError('Target was sent before native actual pose was verified')
        if self.mode == 1:
            self.target = list(target)

    def MotionCtrl_1(self, **kwargs):
        self.calls.append(('MotionCtrl_1', kwargs, self.t))
        if self.first_native_time is None:
            raise AssertionError('Exited teaching before native actual pose was verified')
        self.mode = 0

    def MotionCtrl_2(self, **kwargs):
        self.calls.append(('MotionCtrl_2', kwargs, self.t))
        if self.first_native_time is None:
            raise AssertionError('Selected CAN1 before native actual pose was verified')
        if not self.refuse_can:
            self.mode = 1
            self.can_command_sent = True

    def GetArmJointCtrl(self):
        raise AssertionError('FA command frames must never supply the actual pose')

    def run(self, **kwargs):
        values = dict(firmware_version='S-V1.9-0260716', reader=self.reader,
                      sdk_factory=self.factory, controller_reader=self.owners,
                      clock=self.clock, wall=self.clock, pause=self.pause,
                      step_timeout=1.5)
        values.update(kwargs)
        return module.recover_enabled('can_left_mas', **values)


class MasterRecoveryTests(unittest.TestCase):
    def assert_only_allowed_writes(self, result):
        names = [row['method'] for row in result['sdk_command_log']]
        self.assertTrue(set(names) <= module.ALLOWED_COMMANDS)
        self.assertFalse(set(names) & {'EnableArm', 'DisableArm', 'ResetPiper', 'GripperCtrl'})
        self.assertFalse(result['torque_removal_requested'] or result['enable_requested'] or result['reset_requested'])
        for row in result['sdk_command_log']:
            if row['method'] == 'MasterSlaveConfig':
                self.assertEqual(row['kwargs'], dict(linkage_config=0xFC, feedback_offset=0, ctrl_offset=0, linkage_offset=0))
            if row['method'] == 'MotionCtrl_1':
                self.assertEqual(row['kwargs'], {'grag_teach_ctrl': 2})

    def test_fresh_enabled_can1_is_read_only_even_when_apply_true(self):
        plant = Plant(mode=1)
        result = plant.run(apply=True, firmware_version=None)
        self.assertTrue(result['passed'], result)
        self.assertTrue(result['already_ready'])
        self.assertEqual(result['hardware_control_writes'], 0)
        self.assertFalse(any(call[0] == 'factory' for call in plant.calls))

    def test_complete_enabled_teaching_and_standby_can_prepare_without_known_version(self):
        for mode in (0, 2):
            with self.subTest(mode=mode):
                plant = Plant(mode=mode)
                initial = list(plant.pose)
                result = plant.run(apply=True, firmware_version=None)
                self.assertTrue(result['passed'], result)
                self.assertFalse(result['bootstrap_attempted'])
                self.assertEqual([row['method'] for row in result['sdk_command_log']],
                                 ['JointCtrl', 'MotionCtrl_1', 'JointCtrl', 'MotionCtrl_2', 'JointCtrl'])
                self.assertEqual(plant.target, initial)
                self.assertGreaterEqual(result['stable_elapsed_s'], 0.3)
                self.assert_only_allowed_writes(result)

    def test_silent_fa_uses_one_fc_then_post_fc_native_pose_before_any_target_or_mode(self):
        plant = Plant(silent=True, delay_post_fc=2)
        expected = list(plant.pose)
        result = plant.run(apply=True)
        self.assertTrue(result['passed'], result)
        self.assertEqual([row['method'] for row in result['sdk_command_log']],
                         ['MasterSlaveConfig', 'JointCtrl', 'MotionCtrl_1', 'JointCtrl', 'MotionCtrl_2', 'JointCtrl'])
        self.assertEqual(result['post_fc_feedback']['ctrl_mode'], 0)
        self.assertEqual(result['holding_target_mdeg'], expected)
        self.assertNotEqual(expected, plant.fa_commands)
        self.assertEqual(plant.target, expected)
        self.assertIn(('ConnectPort', {'piper_init': False}), plant.calls)
        target_times = [call[2] for call in plant.calls if call[0] in ('JointCtrl', 'MotionCtrl_1', 'MotionCtrl_2')]
        self.assertTrue(all(stamp >= plant.first_native_time for stamp in target_times))
        self.assertTrue(all(row['args'] == expected for row in result['sdk_command_log'] if row['method'] == 'JointCtrl'))
        self.assertIn('not guaranteed', result['bootstrap_risk'])
        self.assertFalse(result['fa_command_frames_used_as_pose'])
        self.assert_only_allowed_writes(result)

    def test_plan_only_never_creates_sdk_even_for_supported_silent_fa(self):
        for plant in (Plant(mode=0), Plant(silent=True)):
            with self.subTest(silent=plant.silent):
                result = plant.run()
                self.assertTrue(result['passed'], result)
                self.assertTrue(result['plan_only'])
                self.assertEqual(result['sdk_instances_created'], 0)
                self.assertEqual(result['hardware_control_writes'], 0)
                self.assertFalse(any(call[0] == 'factory' for call in plant.calls))

    def test_fc_bootstrap_reporting_can1_still_replaces_old_cached_target_with_actual_pose(self):
        plant = Plant(silent=True, fc_mode=1)
        actual = list(plant.pose)
        stale = list(plant.target)
        result = plant.run(apply=True)
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['post_fc_feedback']['ctrl_mode'], 1)
        self.assertEqual(result['holding_target_mdeg'], actual)
        self.assertNotEqual(stale, actual)
        self.assertEqual(plant.target, actual)
        self.assertEqual([row['method'] for row in result['sdk_command_log']],
                         ['MasterSlaveConfig', 'JointCtrl', 'MotionCtrl_1', 'JointCtrl', 'MotionCtrl_2', 'JointCtrl'])
        self.assertTrue(all(row['args'] == actual for row in result['sdk_command_log'] if row['method'] == 'JointCtrl'))

    def test_old_unknown_and_malformed_firmware_block_silent_bootstrap_before_sdk(self):
        for version in (None, 'S-V1.8-8', 'S-V1.8-0260716', 'v1.9', 'S-V1.9-build', 'S-V1.9-123 junk'):
            with self.subTest(version=version):
                plant = Plant(silent=True)
                result = plant.run(apply=True, firmware_version=version)
                self.assertFalse(result['passed'])
                self.assertFalse(result['bootstrap_attempted'])
                self.assertEqual(result['sdk_instances_created'], 0)
                self.assertFalse(any(call[0] == 'factory' for call in plant.calls))

    def test_owner_blocks_read_and_all_sdk_construction(self):
        plant = Plant(silent=True, busy_check=1)
        result = plant.run(apply=True)
        self.assertFalse(result['passed'])
        self.assertEqual(plant.reads, 0)
        self.assertEqual(plant.calls, [])

    def test_controller_appearing_while_connecting_blocks_first_write(self):
        plant = Plant(silent=True, busy_check=3)
        result = plant.run(apply=True)
        self.assertFalse(result['passed'])
        self.assertEqual(result['hardware_control_writes'], 0)
        self.assertFalse(result['bootstrap_attempted'])
        self.assertEqual(plant.calls[-1], ('DisconnectPort',))

    def test_post_fc_missing_native_feedback_times_out_after_only_one_role_write(self):
        plant = Plant(silent=True, never_feedback=True)
        result = plant.run(apply=True)
        self.assertFalse(result['passed'])
        self.assertTrue(result['error'].startswith('TimeoutError('))
        self.assertEqual([row['method'] for row in result['sdk_command_log']], ['MasterSlaveConfig'])
        self.assertIsNone(plant.first_native_pose)
        self.assertLess(plant.t - 1000.0, 2.0)
        self.assert_only_allowed_writes(result)

    def test_disabled_or_faulty_post_fc_feedback_blocks_before_pose_commands(self):
        for patch in ({'disable_on_fc': True}, {'fault_valid_read': 1}, {'stale_valid_read': 1}):
            with self.subTest(patch=patch):
                plant = Plant(silent=True, **patch)
                result = plant.run(apply=True)
                self.assertFalse(result['passed'])
                self.assertEqual([row['method'] for row in result['sdk_command_log']], ['MasterSlaveConfig'])
                self.assert_only_allowed_writes(result)

    def test_pre_fc_individual_frame_cannot_be_hidden_in_fresh_post_fc_aggregate(self):
        plant = Plant(silent=True, precommand_after_fc=True)
        result = plant.run(apply=True)
        self.assertFalse(result['passed'])
        self.assertIn('pre-command', result['error'])
        self.assertEqual(result['hardware_control_writes'], 1)

    def test_disabled_initial_master_never_bootstraps_or_constructs_sdk(self):
        plant = Plant(disabled=True)
        result = plant.run(apply=True)
        self.assertFalse(result['passed'])
        self.assertEqual(result['sdk_instances_created'], 0)
        self.assertFalse(result['bootstrap_attempted'])

    def test_nonmissing_initial_fault_never_qualifies_for_role_bootstrap(self):
        plant = Plant()
        def fault(_channel, **_kwargs):
            raise RuntimeError('Arm fault observed during sampling')
        result = plant.run(apply=True, reader=fault)
        self.assertFalse(result['passed'])
        self.assertEqual(result['sdk_instances_created'], 0)
        self.assertFalse(result['bootstrap_attempted'])

    def test_master_connection_drift_refreshes_actual_target_once_without_rejection(self):
        plant = Plant(connection_drift=True)
        result = plant.run(apply=True)
        self.assertTrue(result['passed'], result)
        self.assertGreater(result['observed_max_drift_mdeg'], 1000)
        self.assertTrue(result['drift_is_diagnostic_only'])
        expected = result['pre_write_feedback']['joint_mdeg']
        self.assertNotEqual(result['initial_feedback']['joint_mdeg'], expected)
        self.assertEqual(result['holding_target_mdeg'], expected)
        targets = [tuple(row['args']) for row in result['sdk_command_log'] if row['method'] == 'JointCtrl']
        self.assertEqual(set(targets), {tuple(expected)})
        self.assert_only_allowed_writes(result)

    def test_after_anchor_large_drift_is_reported_without_rejection_or_reanchoring(self):
        plant = Plant(silent=True, drift_valid_read=2)
        result = plant.run(apply=True)
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['observed_max_drift_mdeg'], 1500)
        self.assertEqual(result['hardware_control_writes'], 6)
        targets = [tuple(row['args']) for row in result['sdk_command_log'] if row['method'] == 'JointCtrl']
        self.assertEqual(set(targets), {tuple(result['holding_target_mdeg'])})
        self.assert_only_allowed_writes(result)

    def test_after_anchor_fault_and_stale_feedback_never_append_control_writes(self):
        for patch in ({'fault_valid_read': 2}, {'stale_valid_read': 2}):
            with self.subTest(patch=patch):
                plant = Plant(silent=True, **patch)
                result = plant.run(apply=True)
                self.assertFalse(result['passed'])
                self.assertEqual(result['hardware_control_writes'], 6)
                self.assert_only_allowed_writes(result)

    def test_cached_pretransition_raw_feedback_cannot_acknowledge_can1(self):
        plant = Plant(silent=True, frozen_after_anchor=True)
        result = plant.run(apply=True)
        self.assertFalse(result['passed'])
        self.assertEqual(result['hardware_control_writes'], 6)

    def test_missing_after_anchor_wait_is_passive_and_times_out(self):
        plant = Plant(silent=True, missing_after_anchor=True)
        result = plant.run(apply=True)
        self.assertFalse(result['passed'])
        self.assertTrue(result['error'].startswith('TimeoutError('))
        self.assertEqual(result['hardware_control_writes'], 6)
        self.assert_only_allowed_writes(result)

    def test_can_mode_refusal_times_out_with_no_retry_or_reset(self):
        plant = Plant(silent=True, refuse_can=True)
        result = plant.run(apply=True)
        self.assertFalse(result['passed'])
        self.assertTrue(result['error'].startswith('TimeoutError('))
        self.assertEqual(result['hardware_control_writes'], 6)
        self.assert_only_allowed_writes(result)

    def test_interrupt_after_bootstrap_closes_without_any_fallback_write(self):
        plant = Plant(silent=True, interrupt_valid_read=1)
        result = plant.run(apply=True)
        self.assertFalse(result['passed'])
        self.assertTrue(result['interrupted'])
        self.assertEqual(result['hardware_control_writes'], 1)
        self.assertEqual(plant.calls[-1], ('DisconnectPort',))

    def test_follower_channel_rejected_before_feedback_or_sdk(self):
        plant = Plant()
        result = module.recover_enabled('can_left_slave', apply=True,
                                        reader=plant.reader, sdk_factory=plant.factory,
                                        controller_reader=plant.owners)
        self.assertFalse(result['passed'])
        self.assertEqual(plant.reads, 0)
        self.assertEqual(plant.owner_checks, 0)
        self.assertEqual(plant.calls, [])


class FirmwareTests(unittest.TestCase):
    def test_strict_supported_and_unsupported_firmware_formats(self):
        for version in ('S-V1.8-9', 'S-V1.8-10', 'S-V1.8.9', 'S-V1.8.9-0260716',
                        'S-V1.9-0260716', 'S-V1.9-1260713', 'S-V1.10-001', 'S-V2.0-1'):
            with self.subTest(version=version):
                self.assertTrue(module.firmware_supports_enabled_role_change(version))
        for version in (None, 19, '', 'S-V1.8', 'S-V1.8-8', 'S-V1.7-99',
                        'S-V1.8-0260716', 'S-V1.8-09', 'S-V1.9-dev', 'v1.9',
                        'S-V1.9-123junk', 'S-V01.9-123', 'S-V1.8.09'):
            with self.subTest(version=version):
                self.assertFalse(module.firmware_supports_enabled_role_change(version))

    def test_cached_receipt_reads_only_selected_master_string(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'firmware-compat').mkdir()
            path = root / 'firmware-compat/provenance.json'
            path.write_text(json.dumps({'firmwares': {'can_left_mas': 'S-V1.9-0260716',
                                                      'can_right_mas': 'S-V1.9-0260713',
                                                      'can_left_slave': 'S-V1.9-0260716'}}))
            self.assertEqual(module.cached_firmware_version(root, 'can_left_mas'), 'S-V1.9-0260716')
            self.assertEqual(module.cached_firmware_version(root, 'can_right_mas'), 'S-V1.9-0260713')
            self.assertIsNone(module.cached_firmware_version(root, 'can_left_slave'))
            path.write_text('{bad JSON')
            self.assertIsNone(module.cached_firmware_version(root, 'can_left_mas'))
            path.unlink()
            self.assertIsNone(module.cached_firmware_version(root, 'can_left_mas'))


if __name__ == '__main__':
    unittest.main()
