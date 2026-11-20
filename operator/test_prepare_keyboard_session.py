"""Fresh-round ownership/recovery tests; no SDK or CAN instances are created."""
from copy import deepcopy
from pathlib import Path
import os
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).parent))
from prepare_keyboard_session import CHANNELS, prepare_session, recovery_plan
import prepare_keyboard_session as preparation


def normal():
    return {name: {"ctrl_mode": 1, "arm_status": 0, "err_code": 0,
                   "gripper_mode": 0, "driver_enabled": [True] * 6} for name in CHANNELS}


class Backend:
    def __init__(self, rows=None):
        self.rows = normal() if rows is None else rows
        self.calls = []
        self.restore_ok = True
        self.foreign_controller = False
        self.ready_error = None

    def check_dependencies_and_ownership(self):
        self.calls.append("ownership")
        if self.foreign_controller:
            raise RuntimeError("Existing collector or foreign controller")

    def stop(self):
        self.calls.append("stop")
        return {"passed": True}

    def feedback(self):
        self.calls.append("read")
        return deepcopy(self.rows)

    def restore(self, name, *, enable=False):
        self.calls.append(("restore", name, True) if enable else ("restore", name))
        if self.restore_ok:
            self.rows[name] = normal()[name]
        return {"passed": self.restore_ok}

    def start(self):
        self.calls.append("start")

    def wait_ready(self):
        self.calls.append("ready")
        if self.ready_error:
            raise self.ready_error


class KeyboardPreparationTests(unittest.TestCase):
    def test_stop_option_only_dispatches_stop_helper_without_startup(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / ".ros-env/bin").mkdir(parents=True)
            (root / "operator").mkdir()
            (root / ".ros-env/bin/python").symlink_to(sys.executable)
            (root / "operator/stop_arm_drivers.py").write_text("print('stop-only')\n")
            wrapper = Path(__file__).resolve().parents[1] / "run_dagger.sh"
            result = subprocess.run(["bash", str(wrapper), "--stop"],
                                    env={**os.environ, "DAGGER_ROOT": str(root)},
                                    capture_output=True, text=True, timeout=5)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertEqual(result.stdout.strip(), "stop-only")
            self.assertFalse((root / "_runtime").exists())

    def test_clean_restart_never_prompts_or_reconfigures_motors(self):
        backend = Backend()
        def no_prompt(_):
            self.fail("CAN1 enabled arms must not require torque recovery")
        result = prepare_session(backend, no_prompt)
        self.assertTrue(result["passed"])
        self.assertEqual(backend.calls[:2], ["ownership", "stop"])
        self.assertFalse(any(isinstance(call, tuple) for call in backend.calls))
        self.assertEqual(backend.calls[-2:], ["start", "ready"])
        self.assertFalse(result["initial_pose_requested"])
        self.assertFalse(result["policy_execution_started"])

    def test_mixed_enabled_recovery_never_prompts_or_requests_torque_removal(self):
        rows = normal()
        rows["can_right_slave"]["ctrl_mode"] = 2
        rows["can_left_mas"] = {"error": "Missing required CAN feedback frames"}
        backend = Backend(rows)
        result = prepare_session(backend, lambda _: self.fail("enabled recovery must never ask to unload arms"))
        self.assertTrue(result["passed"])
        self.assertEqual([call for call in backend.calls if isinstance(call, tuple)],
                         [("restore", "can_right_slave"), ("restore", "can_left_mas")])
        self.assertFalse(result['support_confirmed'])
        self.assertFalse(result['torque_removal_requested'])
        self.assertFalse(result['master_torque_removal_requested'])
        self.assertFalse(result['follower_torque_removal_requested'])
        self.assertEqual(result['torque_removal_requested_by_channel'],
                         {name: False for name in CHANNELS})

    def test_enabled_teaching_followers_use_recovery_without_support_or_torque_prompt(self):
        rows = normal()
        rows["can_left_slave"]["ctrl_mode"] = 2
        rows["can_right_slave"]["ctrl_mode"] = 2
        backend = Backend(rows)
        result = prepare_session(backend, lambda _: self.fail("enabled followers must not request torque recovery"))
        self.assertTrue(result["passed"])
        self.assertFalse(result["support_confirmed"])
        self.assertFalse(result["follower_torque_removal_requested"])
        self.assertEqual([call for call in backend.calls if isinstance(call, tuple)],
                         [("restore", "can_left_slave"), ("restore", "can_right_slave")])

    def test_hardware_backend_dispatches_followers_only_to_enabled_preparation(self):
        backend = preparation.HardwareBackend(Path("/unused"))
        with patch.object(preparation, "active_controllers", return_value=[]), \
                patch.object(preparation, "prepare_enabled", return_value={"passed": True}) as enabled, \
                patch.object(preparation, "recover_enabled") as master:
            self.assertTrue(backend.restore("can_left_slave")["passed"])
            enabled.assert_called_once_with("can_left_slave", apply=True)
            master.assert_not_called()

    def test_hardware_backend_dispatches_master_only_to_enabled_helper_with_cached_version(self):
        backend = preparation.HardwareBackend(Path("/unused"))
        with patch.object(preparation, "active_controllers", return_value=[]), \
                patch.object(preparation, "prepare_enabled") as enabled, \
                patch.object(preparation, "cached_firmware_version", return_value='S-V1.9-0260713') as firmware, \
                patch.object(preparation, "recover_enabled", return_value={"passed": True}) as master, \
                patch('recover_master_mode.recover', side_effect=AssertionError('old torque-off helper must never run')):
            self.assertTrue(backend.restore("can_right_mas")["passed"])
            firmware.assert_called_once_with(Path('/unused'), 'can_right_mas')
            master.assert_called_once_with("can_right_mas", apply=True, firmware_version='S-V1.9-0260713')
            enabled.assert_not_called()

    def test_master_teaching_recovery_ignores_obsolete_confirmation_callback(self):
        rows = normal()
        rows["can_right_mas"]["ctrl_mode"] = 2
        backend = Backend(rows)
        result = prepare_session(backend, lambda _: False)
        self.assertTrue(result["passed"])
        self.assertIn(('restore', 'can_right_mas'), backend.calls)
        self.assertFalse(result['support_confirmed'])

    def test_ctrlc_during_enabled_recovery_cleans_up_without_resume(self):
        rows = normal()
        rows["can_right_mas"]["ctrl_mode"] = 2
        backend = Backend(rows)
        with patch.object(backend, 'restore', side_effect=KeyboardInterrupt):
            result = prepare_session(backend, lambda _: self.fail('no unloading confirmation'))
        self.assertTrue(result["interrupted"])
        self.assertNotIn("start", backend.calls)
        self.assertEqual(backend.calls[-1], "stop")

    def test_existing_collector_does_not_stop_or_take_over_its_drivers(self):
        backend = Backend()
        backend.foreign_controller = True
        result = prepare_session(backend, lambda _: True)
        self.assertFalse(result["passed"])
        self.assertEqual(backend.calls, ["ownership"])

    def test_failed_direct_enable_blocks_start_without_an_unloading_fallback(self):
        rows = normal()
        rows["can_left_slave"]["driver_enabled"][2] = False
        backend = Backend(rows)
        backend.restore_ok = False
        result = prepare_session(backend, lambda _: True)
        self.assertFalse(result["passed"])
        self.assertNotIn("start", backend.calls)
        self.assertEqual([call for call in backend.calls if isinstance(call, tuple)],
                         [("restore", "can_left_slave", True)])
        self.assertEqual(result['enable_requested'], ['can_left_slave'])
        self.assertEqual(backend.calls[-1], "stop")

    def test_failed_enabled_preparation_never_launches_driver_or_falls_back(self):
        rows = normal()
        rows["can_left_slave"]["ctrl_mode"] = 2
        backend = Backend(rows)
        backend.restore_ok = False
        result = prepare_session(backend, lambda _: self.fail("no torque recovery prompt"))
        self.assertFalse(result["passed"])
        self.assertNotIn("start", backend.calls)
        self.assertEqual([call for call in backend.calls if isinstance(call, tuple)],
                         [("restore", "can_left_slave")])
        self.assertEqual(backend.calls[-1], "stop")

    def test_driver_readiness_failure_stops_new_driver(self):
        backend = Backend()
        backend.ready_error = RuntimeError("No fresh ROS feedback")
        result = prepare_session(backend, lambda _: True)
        self.assertFalse(result["passed"])
        self.assertEqual(backend.calls[-3:], ["start", "ready", "stop"])

    def test_unrelated_modes_faults_units_and_missing_followers_fail_without_restore(self):
        bad_rows = []
        for patch in ({"ctrl_mode": 3}, {"arm_status": 1}, {"err_code": 1},
                      {"gripper_mode": 1}, {"driver_enabled": [True] * 5},
                      {"error": "Missing required CAN feedback frames"}):
            rows = normal()
            rows["can_left_slave"].update(patch)
            bad_rows.append(rows)
        for rows in bad_rows:
            with self.subTest(rows=rows):
                backend = Backend(rows)
                result = prepare_session(backend, lambda _: self.fail("unsafe state prompted recovery"))
                self.assertFalse(result["passed"])
                self.assertNotIn("start", backend.calls)
                self.assertFalse(any(isinstance(call, tuple) for call in backend.calls))

    def test_master_fault_is_not_treated_as_silent_fa(self):
        rows = normal()
        rows["can_left_mas"] = {"error": "Driver fault observed during sampling"}
        with self.assertRaises(RuntimeError):
            recovery_plan(rows)

    def test_enabled_master_standby_is_recovered_without_confirmation(self):
        rows = normal()
        rows["can_left_mas"]["ctrl_mode"] = 0
        rows["can_right_mas"]["ctrl_mode"] = 2
        self.assertEqual(recovery_plan(rows), ["can_left_mas", "can_right_mas"])
        backend = Backend(rows)
        result = prepare_session(backend, lambda _: self.fail('enabled masters must not ask to unload'))
        self.assertTrue(result['passed'])
        self.assertEqual([call for call in backend.calls if isinstance(call, tuple)],
                         [('restore', 'can_left_mas'), ('restore', 'can_right_mas')])

    def test_initial_disabled_or_partial_arms_are_enabled_but_strict_checks_still_reject_them(self):
        for name in CHANNELS:
            for enabled in ([False] * 6, [True] * 5 + [False]):
                with self.subTest(name=name, enabled=enabled):
                    rows = normal()
                    rows[name]['driver_enabled'] = enabled
                    with self.assertRaises(RuntimeError):
                        recovery_plan(rows)
                    self.assertEqual(recovery_plan(rows, allow_enable=True), [name])
                    backend = Backend(rows)
                    result = prepare_session(backend, lambda _: self.fail('no unloading fallback'))
                    self.assertTrue(result['passed'], result)
                    self.assertEqual(result['enable_requested'], [name])
                    self.assertEqual([call for call in backend.calls if isinstance(call, tuple)],
                                     [('restore', name, True)])
                    self.assertFalse(result['torque_removal_requested'])
                    self.assertFalse(result['initial_pose_requested'])

    def test_disabled_standby_or_teaching_arms_use_direct_enable(self):
        for name in CHANNELS:
            for mode in (0, 2):
                with self.subTest(name=name, mode=mode):
                    rows = normal()
                    rows[name].update(ctrl_mode=mode, driver_enabled=[False] * 6,
                                      driver_foc_bytes=[0] * 6)
                    backend = Backend(rows)
                    result = prepare_session(backend)
                    self.assertTrue(result['passed'], result)
                    self.assertEqual(result['enable_requested'], [name])
                    self.assertEqual([call for call in backend.calls if isinstance(call, tuple)],
                                     [('restore', name, True)])

    def test_missing_enable_bit_in_foc_feedback_selects_the_direct_enable_path(self):
        rows = normal()
        rows['can_left_mas']['driver_foc_bytes'] = [0x40] * 5 + [0]
        with self.assertRaises(RuntimeError):
            recovery_plan(rows)
        backend = Backend(rows)
        result = prepare_session(backend)
        self.assertTrue(result['passed'], result)
        self.assertEqual(result['enable_requested'], ['can_left_mas'])
        self.assertIn(('restore', 'can_left_mas', True), backend.calls)

    def test_helper_success_without_actual_enable_cannot_start_drivers(self):
        rows = normal()
        rows['can_left_mas']['driver_enabled'] = [False] * 6
        backend = Backend(rows)
        def unchanged_restore(name, *, enable=False):
            backend.calls.append(('restore', name, enable))
            return {'passed': True}
        backend.restore = unchanged_restore
        result = prepare_session(backend)
        self.assertFalse(result['passed'])
        self.assertNotIn('start', backend.calls)
        self.assertEqual(result['enable_requested'], ['can_left_mas'])
        self.assertEqual(backend.calls[-1], 'stop')

    def test_enable_requested_only_lists_the_commands_that_were_attempted(self):
        rows = normal()
        rows['can_left_slave']['driver_enabled'] = [False] * 6
        rows['can_right_mas']['driver_enabled'] = [False] * 6
        backend = Backend(rows)
        backend.restore_ok = False
        result = prepare_session(backend)
        self.assertFalse(result['passed'])
        self.assertEqual(result['enable_needed'], ['can_left_slave', 'can_right_mas'])
        self.assertEqual(result['enable_requested'], ['can_left_slave'])

    def test_missing_master_feedback_is_delegated_to_enabled_helper_without_old_fallback(self):
        rows = normal()
        rows['can_left_mas'] = {'error': 'Missing required CAN feedback frames'}
        backend = Backend(rows)
        result = prepare_session(backend, lambda _: self.fail('silent master must not trigger unloading prompt'))
        self.assertTrue(result['passed'])
        self.assertIn(('restore', 'can_left_mas'), backend.calls)

    def test_failed_master_enabled_helper_prevents_driver_start_and_any_fallback(self):
        rows = normal()
        rows['can_left_mas'] = {'error': 'Missing required CAN feedback frames'}
        backend = Backend(rows)
        backend.restore_ok = False
        result = prepare_session(backend, lambda _: self.fail('no torque recovery confirmation'))
        self.assertFalse(result['passed'])
        self.assertNotIn('start', backend.calls)
        self.assertEqual([call for call in backend.calls if isinstance(call, tuple)],
                         [('restore', 'can_left_mas')])

    def test_missing_feedback_label_cannot_hide_known_master_torque_loss_or_fault(self):
        for patch_value in ({'driver_enabled': [False] * 6}, {'driver_enabled': [True] * 5 + [False]},
                            {'arm_status': 1}, {'err_code': 1}, {'gripper_mode': 1},
                            {'driver_foc_bytes': [0x40] * 5 + [0x41]},
                            {'driver_foc_bytes': [0] * 6}):
            with self.subTest(patch_value=patch_value):
                rows = normal()
                rows['can_left_mas'] = {'error': 'Missing required CAN feedback frames', **patch_value}
                backend = Backend(rows)
                result = prepare_session(backend, lambda _: self.fail('known bad state must never prompt'))
                self.assertFalse(result['passed'])
                self.assertFalse(any(isinstance(call, tuple) for call in backend.calls))
                self.assertNotIn('start', backend.calls)

    def test_unknown_cached_firmware_is_passed_as_unknown_and_helper_failure_is_returned(self):
        backend = preparation.HardwareBackend(Path('/unused'))
        failure = {'passed': False, 'error': 'unknown firmware, no mode write'}
        with patch.object(preparation, 'active_controllers', return_value=[]), \
                patch.object(preparation, 'cached_firmware_version', return_value=None), \
                patch.object(preparation, 'recover_enabled', return_value=failure) as master, \
                patch('recover_master_mode.recover', side_effect=AssertionError('no old fallback')):
            self.assertIs(backend.restore('can_left_mas'), failure)
            master.assert_called_once_with('can_left_mas', apply=True, firmware_version=None)

    def test_new_controller_blocks_master_recovery_before_any_helper(self):
        backend = preparation.HardwareBackend(Path('/unused'))
        with patch.object(preparation, 'active_controllers', return_value=[{'pid': 123}]), \
                patch.object(preparation, 'recover_enabled') as master:
            with self.assertRaisesRegex(RuntimeError, '新的控制进程'):
                backend.restore('can_left_mas')
            master.assert_not_called()

    def test_noninteractive_preparation_has_no_unloading_input_or_tty_prompt(self):
        backend = Backend()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            with patch.object(preparation.sys, 'argv', ['prepare_keyboard_session.py', '--apply', '--root', directory]), \
                    patch.object(preparation.sys, 'stdin', object()), \
                    patch.object(preparation, 'HardwareBackend', return_value=backend), \
                    patch.object(preparation.signal, 'signal'), \
                    patch('builtins.input', side_effect=AssertionError('preparation must not unload or prompt')), \
                    patch('builtins.print'):
                self.assertEqual(preparation.main(), 0)
            report = preparation.json.loads((root / '_runtime/keyboard-prepare.json').read_text())
            self.assertFalse(report['torque_removal_requested'])
            self.assertFalse(report['support_confirmed'])

    def test_enabled_standby_followers_still_require_inspection_without_an_unloading_fallback(self):
        for patch_value in ({"ctrl_mode": 0},
                            {"ctrl_mode": 0, "driver_foc_bytes": [0x40] * 6}):
            with self.subTest(patch_value=patch_value):
                rows = normal()
                rows["can_left_slave"].update(patch_value)
                backend = Backend(rows)
                result = prepare_session(backend, lambda _: self.fail("unsafe follower must not prompt recovery"))
                self.assertFalse(result["passed"])
                self.assertFalse(any(isinstance(call, tuple) for call in backend.calls))
                self.assertNotIn("start", backend.calls)

    def test_disabled_faulted_bad_feedback_or_unknown_mode_still_cannot_request_enable(self):
        for patch_value in ({'ctrl_mode': 3}, {'arm_status': 1}, {'err_code': 1},
                            {'gripper_mode': 1}, {'driver_foc_bytes': [0] * 5 + [1]},
                            {'error': 'Stale/pre-command feedback: 0x2A5'},
                            {'driver_enabled': [False] * 5}):
            with self.subTest(patch_value=patch_value):
                rows = normal()
                rows['can_left_mas'].update(driver_enabled=[False] * 6)
                rows['can_left_mas'].update(patch_value)
                backend = Backend(rows)
                result = prepare_session(backend)
                self.assertFalse(result['passed'])
                self.assertEqual(result['enable_requested'], [])
                self.assertFalse(any(isinstance(call, tuple) for call in backend.calls))
                self.assertNotIn('start', backend.calls)

    def test_hardware_backend_dispatches_requested_enable_to_the_current_pose_helper(self):
        backend = preparation.HardwareBackend(Path('/unused'))
        for name in CHANNELS:
            with self.subTest(name=name), \
                    patch.object(preparation, 'active_controllers', return_value=[]), \
                    patch.object(preparation, 'enable_at_current_pose', return_value={'passed': True}) as direct, \
                    patch.object(preparation, 'prepare_enabled') as follower, \
                    patch.object(preparation, 'recover_enabled') as master, \
                    patch.object(preparation, 'cached_firmware_version') as firmware:
                self.assertTrue(backend.restore(name, enable=True)['passed'])
                direct.assert_called_once_with(name, apply=True)
                follower.assert_not_called()
                master.assert_not_called()
                firmware.assert_not_called()

    def test_new_controller_blocks_requested_enable_before_any_helper(self):
        backend = preparation.HardwareBackend(Path('/unused'))
        with patch.object(preparation, 'active_controllers', return_value=[{'pid': 123}]), \
                patch.object(preparation, 'enable_at_current_pose') as direct:
            with self.assertRaisesRegex(RuntimeError, '新的控制进程'):
                backend.restore('can_left_mas', enable=True)
            direct.assert_not_called()


if __name__ == "__main__":
    unittest.main()
