"""Minimal opt-in gripper command with the existing native handoff machinery."""
import struct
import threading
from types import SimpleNamespace
import unittest

import test_native_master_handoff as native
from prepare_follower_mode import decode_feedback
from test_prepare_follower_mode import frames as feedback_frames


class MasterGripperHandoffTests(unittest.TestCase):
    def setUp(self):
        native.NativeHandoffTests.setUp(self)
        self.node.can_port = 'can_right_mas'
        self.raw_mode = 1
        self.actual_gripper = 32000
        self.manual_stop_after_release = False

    def sdk(self, method, args, kwargs):
        native.NativeHandoffTests.sdk(self, method, args, kwargs)
        if method == 'GripperCtrl' and self.manual_stop_after_release:
            worker = threading.Thread(target=lambda: self.node.master_enable_callback(SimpleNamespace(data=False)))
            self.workers.append(worker)
            worker.start()
            self.assertTrue(self.node._manual_disable_requested.wait(.5))

    def experimental_release(self):
        self.node.linkage_config_callback(SimpleNamespace(data='release_master_gripper'))

    def gripper_commands(self):
        return [event[1] for event in self.events if event[0] == 'GripperCtrl']

    def test_default_release_adds_no_gripper_command(self):
        self.prepare()
        self.release()
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertEqual(self.gripper_commands(), [])

        left = MasterGripperHandoffTests()
        left.setUp()
        try:
            left.node.can_port = 'can_left_mas'
            left.prepare()
            left.release()
            self.assertTrue(left.node.native_handoff_report['passed'])
            self.assertEqual(left.gripper_commands(), [])
        finally:
            left.tearDown()

    def test_right_release_adds_one_command_between_original_fa_arm_and_role_write(self):
        self.prepare()
        self.actual_gripper = 27000
        self.sleep(.005)
        namespace = self.node._release_native_master_mode.__func__.__globals__
        namespace['sample_feedback'] = lambda *args, **kwargs: self.fail('New passive read during release')
        self.experimental_release()
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertEqual(self.gripper_commands(), [(27000, 0, 0, 0)])
        armed = next(i for i, event in enumerate(self.events) if event[:2] == ('armed', 'fa'))
        release = next(i for i, event in enumerate(self.events) if event[0] == 'GripperCtrl')
        role = next(i for i, event in enumerate(self.events)
                    if event[0] == 'MasterSlaveConfig' and event[2]['linkage_config'] == 0xFA)
        self.assertLess(armed, release)
        self.assertLess(release, role)
        self.assertFalse(any(event[0] in ('raw-read', 'EnableArm', 'DisableArm', 'ResetPiper',
                                         'JointCtrl', 'MotionCtrl_1', 'MotionCtrl_2')
                             for event in self.events))
        stages = self.node.native_handoff_report['gripper_stages']
        command = next(row for row in stages if row['stage'] == 'gripper-release-sent')
        self.assertEqual(command['raw_width_um'], 27000)
        self.assertEqual(command['requested_width_um'], 27000)

    def test_left_release_adds_one_gripper_command_at_its_prepared_width(self):
        self.node.can_port = 'can_left_mas'
        self.actual_gripper = 21000
        self.prepare()
        self.experimental_release()
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertEqual(self.configs(), [0xFA])
        self.assertEqual(self.gripper_commands(), [(21000, 0, 0, 0)])

    def test_both_master_instances_use_their_own_distinct_prepared_widths(self):
        left = MasterGripperHandoffTests()
        left.setUp()
        try:
            left.node.can_port = 'can_left_mas'
            left.actual_gripper = 17000
            self.actual_gripper = 53000
            left.prepare()
            self.prepare()
            left.experimental_release()
            self.experimental_release()
            self.assertTrue(left.node.native_handoff_report['passed'])
            self.assertTrue(self.node.native_handoff_report['passed'])
            self.assertEqual(left.gripper_commands(), [(17000, 0, 0, 0)])
            self.assertEqual(self.gripper_commands(), [(53000, 0, 0, 0)])
            self.assertEqual(left.configs(), [0xFA])
            self.assertEqual(self.configs(), [0xFA])
        finally:
            left.tearDown()

    def test_new_gripper_enable_metadata_is_not_an_extra_release_gate(self):
        self.prepare()
        for key in ('gripper_status_byte', 'gripper_effort_mnm', 'gripper_source_time', 'gripper_hex'):
            self.node._native_last_actual.pop(key, None)
        self.experimental_release()
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertEqual(self.gripper_commands(), [(32000, 0, 0, 0)])
        self.assertFalse(any(event[0] == 'raw-read' for event in self.events))

    def test_actual_zero_width_is_used_without_a_new_nonzero_or_difference_gate(self):
        self.actual_gripper = 0
        self.prepare()
        self.experimental_release()
        self.assertTrue(self.node.native_handoff_report['passed'])
        self.assertEqual(self.gripper_commands(), [(0, 0, 0, 0)])

    def test_explicit_return_to_fc_adds_no_gripper_restore_command(self):
        self.prepare()
        self.experimental_release()
        self.node.linkage_config_callback(SimpleNamespace(data='slave'))
        self.assertEqual(self.configs(), [0xFA, 0xFC])
        self.assertEqual(self.node.current_linkage_config, 0xFC)
        self.assertEqual(self.gripper_commands(), [(32000, 0, 0, 0)])

    def test_reader_fault_with_missing_fa_gripper_uses_existing_fc_recovery_without_extra_gripper_write(self):
        self.prepare()
        self.missing_fa_id = 0x159
        self.reader_failure_after_fa_s = 3.0
        self.experimental_release()
        self.assertFalse(self.node.native_handoff_report['passed'])
        self.assertEqual(self.configs(), [0xFA, 0xFC])
        self.assertEqual(self.gripper_commands(), [(32000, 0, 0, 0)])
        self.assertFalse(any(event[0] in ('EnableArm', 'DisableArm', 'ResetPiper') for event in self.events))

    def test_existing_manual_disable_rule_still_cancels_before_fa(self):
        self.prepare()
        self.manual_stop_after_release = True
        self.experimental_release()
        self.assertTrue(self.node._manual_disable_requested.is_set())
        self.assertEqual(self.gripper_commands(), [(32000, 0, 0, 0)])
        self.assertEqual(self.configs(), [])
        self.assertFalse(any(event[0] in ('EnableArm', 'ResetPiper') for event in self.events))


for _name, _method in native.NativeHandoffTests.__dict__.items():
    if callable(_method) and not _name.startswith('test_') and _name not in ('setUp', 'sdk'):
        setattr(MasterGripperHandoffTests, _name, _method)


class GripperWireFeedbackTests(unittest.TestCase):
    def test_enable_and_signed_effort_are_decoded_from_gripper_not_arm_bits(self):
        frames = feedback_frames()
        frames[0x2A8] = (struct.pack('>ihBB', 17000, -123, 0, 0), 100.0)
        row = decode_feedback(frames, 100.01)
        self.assertEqual(row['driver_enabled'], [True] * 6)
        self.assertFalse(row['gripper_enabled'])
        self.assertEqual(row['gripper_effort_mnm'], -123)
        self.assertEqual(row['gripper_source_time'], 100.0)
        self.assertEqual(row['gripper_hex'], frames[0x2A8][0].hex())
        frames[0x2A8] = (struct.pack('>ihBB', 17000, 1000, 0x40, 0), 100.0)
        self.assertTrue(decode_feedback(frames, 100.01)['gripper_enabled'])


if __name__ == '__main__':
    unittest.main()
