"""Verify gripper release request selection without ROS, CAN, or SDK imports."""
import os
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from test_compatibility import load_official_methods


ENVIRONMENT_KEY = 'DAGGER_TEST_MASTER_GRIPPER_RELEASE'


class GripperTestEntryTests(unittest.TestCase):
    def setUp(self):
        self.namespace, operator_class = load_official_methods()
        self.operator = operator_class.__new__(operator_class)
        self.operator.parallel_following_preparation = {'hold_passed': True}
        self.operator._following_preparation_ready = lambda _state: True
        self.cancelled = False
        self.events = []
        self.namespace['String'] = SimpleNamespace

        def check_cancelled(_state):
            if self.cancelled:
                raise RuntimeError('cancelled before release')

        self.operator._check_preparation_cancelled = check_cancelled
        for side in ('left', 'right'):
            publisher = SimpleNamespace(publish=lambda message, side=side:
                                        self.events.append((side, message.data)))
            setattr(self.operator, 'master_' + side + '_config_pub', publisher)

    def test_default_unset_releases_both_master_grippers(self):
        with patch.dict(os.environ):
            os.environ.pop(ENVIRONMENT_KEY, None)
            self.operator.release_prepared_masters()
        self.assertEqual(self.events,
                         [('left', 'release_master_gripper'), ('right', 'release_master_gripper')])

    def test_empty_opt_in_retains_the_normal_release_path(self):
        with patch.dict(os.environ, {ENVIRONMENT_KEY: ''}):
            self.operator.release_prepared_masters()
        self.assertEqual(self.events, [('left', 'release_master'), ('right', 'release_master')])

    def test_explicit_right_only_uses_the_test_token_for_the_right_master(self):
        with patch.dict(os.environ, {ENVIRONMENT_KEY: 'right'}):
            self.operator.release_prepared_masters()
        self.assertEqual(self.events,
                         [('left', 'release_master'), ('right', 'release_master_gripper')])

    def test_explicit_both_uses_the_test_token_for_each_master(self):
        with patch.dict(os.environ, {ENVIRONMENT_KEY: 'both'}):
            self.operator.release_prepared_masters()
        self.assertEqual(self.events,
                         [('left', 'release_master_gripper'), ('right', 'release_master_gripper')])

    def test_other_selections_fall_back_to_both_normal_release_requests(self):
        for selection in ('left', 'true', 'RIGHT', 'BOTH'):
            with self.subTest(selection=selection), patch.dict(os.environ, {ENVIRONMENT_KEY: selection}):
                self.events.clear()
                self.operator.release_prepared_masters()
                self.assertEqual(self.events,
                                 [('left', 'release_master'), ('right', 'release_master')])

    def test_test_opt_in_cannot_bypass_final_hold_and_preparation(self):
        self.operator.parallel_following_preparation['hold_passed'] = False
        for selection in ('right', 'both'):
            with self.subTest(selection=selection), patch.dict(os.environ, {ENVIRONMENT_KEY: selection}):
                with self.assertRaisesRegex(RuntimeError, 'hold and preparation'):
                    self.operator.release_prepared_masters()
        self.assertEqual(self.events, [])

    def test_cancelled_test_request_does_not_publish_either_release(self):
        self.cancelled = True
        for selection in ('right', 'both'):
            with self.subTest(selection=selection), patch.dict(os.environ, {ENVIRONMENT_KEY: selection}):
                with self.assertRaisesRegex(RuntimeError, 'cancelled before release'):
                    self.operator.release_prepared_masters()
        self.assertEqual(self.events, [])


if __name__ == '__main__':
    unittest.main()
