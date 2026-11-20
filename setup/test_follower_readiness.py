"""Verify release preflight prerequisites using stand-in feedback only."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).parent))
import follower_readiness as module
from prepare_follower_mode import REQUIRED_IDS


class FollowerReadinessTests(unittest.TestCase):
    def setUp(self):
        self.topics = {'left': '/master/joint_left', 'right': '/master/joint_right'}
        self.connections = {'left': 2, 'right': 2}
        self.subscriptions = [
            ['/master/joint_left', ['/collector', '/piper_slave_left_run_123']],
            ['/master/joint_right', ['/collector', '/piper_slave_right_run_456']],
        ]
        self.rows = {channel: dict(
            ctrl_mode=1, arm_status=0, err_code=0, gripper_mode=0,
            driver_enabled=[True] * 6, driver_foc_bytes=[0x40] * 6,
            joint_mdeg=[0, 18000, -20000, 0, 13000, 0],
            timestamp_sec=1000.1,
            component_age_s={f'0x{key:03X}': 0.01 for key in REQUIRED_IDS})
            for channel in module.CHANNELS.values()}
        self.times = iter((1000.0, 1000.11))

    def check(self):
        return module.check_followers_ready(
            self.topics, self.connections, self.subscriptions,
            reader=lambda channel, **_kwargs: deepcopy(self.rows[channel]),
            wall=lambda: next(self.times))

    def test_both_follower_command_paths_and_fresh_enabled_feedback_pass(self):
        result = self.check()
        self.assertTrue(result['passed'])
        self.assertEqual(result['hardware_control_writes'], 0)
        self.assertEqual(result['sdk_instances_created'], 0)

    def test_preflight_does_not_treat_subscriber_names_as_execution_proof(self):
        self.subscriptions[1][1] = ['/collector', '/diagnostic_probe']
        result = self.check()
        self.assertTrue(result['passed'])
        self.assertEqual(result['hardware_control_writes'], 0)
        self.assertEqual(result['sdk_instances_created'], 0)

    def test_renamed_follower_nodes_do_not_block_raw_feedback_preflight(self):
        self.subscriptions = [
            ['/master/joint_left', ['/left_arm_driver']],
            ['/master/joint_right', ['/right_arm_driver']],
        ]
        self.assertTrue(self.check()['passed'])

    def test_no_transport_connection_blocks_readiness(self):
        self.connections['left'] = 0
        with self.assertRaisesRegex(RuntimeError, 'no connection'):
            self.check()

    def test_disabled_teaching_fault_or_stale_feedback_blocks_readiness(self):
        baseline = deepcopy(self.rows)
        for failure in ('disabled', 'teaching', 'fault', 'stale'):
            with self.subTest(failure=failure):
                self.rows = deepcopy(baseline)
                self.times = iter((1000.0, 1000.11))
                row = self.rows['can_right_slave']
                if failure == 'disabled':
                    row['driver_enabled'][3] = False
                elif failure == 'teaching':
                    row['ctrl_mode'] = 2
                elif failure == 'fault':
                    row['driver_foc_bytes'][2] = 0x41
                else:
                    row['component_age_s']['0x264'] = 0.3
                with self.assertRaises(RuntimeError):
                    self.check()

    def test_all_components_must_postdate_this_preflight(self):
        self.rows['can_right_slave']['component_age_s']['0x2A5'] = 0.15
        with self.assertRaisesRegex(RuntimeError, 'pre-command follower component'):
            self.check()


if __name__ == '__main__':
    unittest.main()
