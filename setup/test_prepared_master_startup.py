"""Validate prepared-master startup without ROS imports, SDK or CAN instances."""
import ast
from copy import deepcopy
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest

from prepare_follower_enabled import checked_enabled_feedback
from prepare_follower_mode import REQUIRED_IDS


SOURCE = Path(__file__).resolve().parents[1] / 'ros_ws/src/piper/scripts/piper_start_ms_node_new.py'


def load_startup():
    tree = ast.parse(SOURCE.read_text())
    node = next(item for item in tree.body
                if isinstance(item, ast.ClassDef) and item.name == 'C_PiperRosNode')
    wanted = {'_initialize_master_arm_config', 'GetEnableFlag'}
    node.body = [item for item in node.body
                 if isinstance(item, ast.FunctionDef) and item.name in wanted]
    namespace = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])),
                 str(SOURCE), 'exec'), namespace)
    return namespace, namespace['C_PiperRosNode']


class PreparedMasterStartupTests(unittest.TestCase):
    def setUp(self):
        self.namespace, node_class = load_startup()
        self.node = node_class()
        self.node.require_follower_ready = True
        self.node.mode_master = False
        self.node.can_port = 'can_left_mas'
        self.node.current_linkage_config = 0xFC
        self.node.new_config_ = None
        self.node.is_enabled = False
        self.node.in_teach_mode = False
        self.node._master_publish_enabled = False
        self.node._last_alignment_target = None
        self.node._manual_disable_requested = threading.Event()
        self.node._C_PiperRosNode__enable_flag = False
        self.clock = 1000.0
        self.writes = []
        self.reads = []
        self.logs = []
        self.row = dict(
            ctrl_mode=1, arm_status=0, err_code=0, gripper_mode=0,
            driver_enabled=[True] * 6, driver_foc_bytes=[0x40] * 6,
            joint_mdeg=[12000, 18000, -21000, 4000, 13000, 3000],
            timestamp_sec=1000.12,
            component_age_s={f'0x{key:03X}': 0.01 for key in REQUIRED_IDS})

        def read(channel, since_timestamp=0):
            self.reads.append((channel, since_timestamp))
            self.clock += 0.12
            return deepcopy(self.row)

        self.namespace.update(
            time=SimpleNamespace(time=lambda: self.clock,
                                 sleep=lambda seconds: setattr(self, 'clock', self.clock + seconds)),
            sample_feedback=read,
            checked_enabled_feedback=checked_enabled_feedback,
            rospy=SimpleNamespace(loginfo=lambda message: self.logs.append(message),
                                  logerr=lambda message: self.logs.append(message)))
        self.node._checked_sdk_write = lambda name, *args, **kwargs: self.writes.append((name, args, kwargs))

    def test_managed_startup_only_validates_prepared_holding_state(self):
        self.node._initialize_master_arm_config()
        self.assertEqual(self.reads, [('can_left_mas', 1000.0)])
        self.assertEqual(self.writes, [])
        self.assertTrue(self.node.is_enabled)
        self.assertTrue(self.node.GetEnableFlag())
        self.assertEqual(self.node.current_linkage_config, 0xFC)
        self.assertEqual(self.node.new_config_, 0xFC)
        self.assertFalse(self.node.in_teach_mode)
        self.assertFalse(self.node._master_publish_enabled)
        self.assertIsNone(self.node._last_alignment_target)

    def test_stale_disabled_mode2_or_faults_abort_instead_of_legacy_startup(self):
        baseline = deepcopy(self.row)
        for failure in ('stale', 'disabled', 'mode2', 'standby', 'fault', 'units', 'driver-fault'):
            with self.subTest(failure=failure):
                self.row = deepcopy(baseline)
                self.clock = 1000.0
                if failure == 'stale':
                    self.row['component_age_s']['0x264'] = 0.3
                elif failure == 'disabled':
                    self.row['driver_enabled'][3] = False
                elif failure == 'mode2':
                    self.row['ctrl_mode'] = 2
                elif failure == 'standby':
                    self.row['ctrl_mode'] = 0
                elif failure == 'fault':
                    self.row['err_code'] = 1
                elif failure == 'units':
                    self.row['gripper_mode'] = 1
                else:
                    self.row['driver_foc_bytes'][2] = 0x41
                with self.assertRaises(RuntimeError):
                    self.node._initialize_master_arm_config()
                self.assertEqual(self.writes, [])
                self.assertFalse(self.node.GetEnableFlag())
                self.assertIsNone(self.node._last_alignment_target)

    def test_missing_or_preparation_old_feedback_aborts_without_any_control_write(self):
        self.row['component_age_s']['0x2A5'] = 0.13
        with self.assertRaisesRegex(RuntimeError, 'pre-command follower component'):
            self.node._initialize_master_arm_config()
        self.assertEqual(self.writes, [])
        self.assertFalse(self.node.GetEnableFlag())

    def test_generic_legacy_startup_remains_available(self):
        self.node.require_follower_ready = False
        self.node._initialize_master_arm_config()
        self.assertEqual(self.reads, [])
        self.assertEqual([row[0] for row in self.writes],
                         ['MasterSlaveConfig', 'EnableArm', 'MotionCtrl_2', 'MotionCtrl_1'])
        self.assertEqual(self.writes[0][2]['linkage_config'], 0xFC)
        self.assertTrue(self.node.GetEnableFlag())


if __name__ == '__main__':
    unittest.main()
