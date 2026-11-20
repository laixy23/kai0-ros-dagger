"""USB/evdev and gesture tests with fake files and descriptors; no real input I/O."""
import importlib.util
import errno
from contextlib import redirect_stdout
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch


SOURCE = Path(__file__).with_name('foot_pedal_device.py')
spec = importlib.util.spec_from_file_location('foot_pedal_device_test_module', SOURCE)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


BINDINGS = {'left': {'interface': 'keyboard', 'code': 30},
            'middle': {'interface': 'mouse', 'code': 272},
            'right': {'interface': 'consumer', 'code': 164}}


def event(name, value):
    row = BINDINGS[name]
    return (row['interface'], module.EV_KEY, row['code'], value)


def config():
    return {'version': 1, 'device': {'vendor': module.VENDOR, 'product': module.PRODUCT, 'serial': module.SERIAL},
            'bindings': {name: dict(row) for name, row in BINDINGS.items()},
            'calibration': {'method': 'press_count', 'required_presses': 3,
                            'measured_press_counts': {name: 3 for name in module.PEDALS}}}


class GestureTests(unittest.TestCase):
    def setUp(self):
        self.mapper = module.PedalGestures(BINDINGS)

    def click(self, name, down=0.0, up=0.1, *, mapper=None, eligible=True):
        mapper = mapper or self.mapper
        return (mapper.feed(event(name, 1), down, allow_short=eligible)
                + mapper.feed(event(name, 0), up, allow_short=eligible))

    def test_left_click_fires_once_on_release_without_waiting(self):
        self.assertEqual(self.mapper.feed(event('left', 1), 0), [])
        self.assertEqual(self.mapper.tick(10), [])
        self.assertEqual(self.mapper.feed(event('left', 0), 10.1), ['left'])
        self.assertEqual(self.mapper.feed(event('left', 0), 10.2), [])

    def test_middle_and_right_singles_wait_for_a_second_press(self):
        for name, token in (('middle', 'save'), ('right', 'resume')):
            mapper = module.PedalGestures(BINDINGS)
            self.assertEqual(self.click(name, mapper=mapper), [])
            self.assertEqual(mapper.tick(0.6), [])
            self.assertEqual(mapper.tick(0.61), [token])
            self.assertEqual(mapper.tick(10), [])

    def test_fixed_short_pulse_doubles_emit_only_delete_or_stop(self):
        for name, token in (('middle', 'delete'), ('right', 'stop')):
            mapper = module.PedalGestures(BINDINGS)
            outputs = self.click(name, 0.0, 0.1, mapper=mapper)
            outputs += self.click(name, 0.2, 0.3, mapper=mapper)
            outputs += mapper.tick(10)
            self.assertEqual(outputs, [token])

    def test_long_holds_do_not_select_delete_or_stop(self):
        for name, token in (('left', 'left'), ('middle', 'save'), ('right', 'resume')):
            mapper = module.PedalGestures(BINDINGS)
            self.assertEqual(mapper.feed(event(name, 1), 0), [])
            self.assertEqual(mapper.tick(30), [])
            output = mapper.feed(event(name, 0), 31)
            output += mapper.tick(31.6)
            self.assertEqual(output, [token])

    def test_second_down_at_window_boundary_cancels_single_until_release(self):
        self.assertEqual(self.click('right', 0, 0.1), [])
        self.assertEqual(self.mapper.feed(event('right', 1), 0.6), [])
        self.assertEqual(self.mapper.tick(9), [])
        self.assertEqual(self.mapper.feed(event('right', 0), 10), ['stop'])
        self.assertEqual(self.mapper.tick(20), [])

    def test_clicks_outside_window_are_two_singles(self):
        self.assertEqual(self.click('middle', 0, 0.1), [])
        self.assertEqual(self.mapper.feed(event('middle', 1), 0.7), ['save'])
        self.assertEqual(self.mapper.feed(event('middle', 0), 0.8), [])
        self.assertEqual(self.mapper.tick(1.4), ['save'])

    def test_duplicate_down_and_auto_repeat_do_not_count_as_more_clicks(self):
        self.mapper.feed(event('right', 1), 0)
        self.assertEqual(self.mapper.feed(event('right', 1), 0.1), [])
        self.assertEqual(self.mapper.feed(event('right', 2), 0.2), [])
        self.assertEqual(self.mapper.tick(10), [])
        self.assertEqual(self.mapper.feed(event('right', 0), 10.1), [])
        self.assertEqual(self.mapper.tick(10.7), ['resume'])
        self.assertEqual(self.mapper.feed(event('right', 0), 10.8), [])

    def test_middle_double_requires_both_presses_to_be_ready_eligible(self):
        for eligible in ((False, True), (True, False), (False, False)):
            mapper = module.PedalGestures(BINDINGS)
            output = self.click('middle', 0, 0.1, mapper=mapper, eligible=eligible[0])
            output += self.click('middle', 0.2, 0.3, mapper=mapper, eligible=eligible[1])
            output += mapper.tick(1)
            self.assertEqual(output, [], eligible)

    def test_pre_ready_right_double_still_stops_without_resume(self):
        output = self.click('right', 0, 0.1, eligible=False)
        output += self.click('right', 0.2, 0.3, eligible=False)
        output += self.mapper.tick(1)
        self.assertEqual(output, ['stop'])

    def test_pre_ready_singles_never_become_actions_after_ready(self):
        for name in module.PEDALS:
            mapper = module.PedalGestures(BINDINGS)
            mapper.feed(event(name, 1), 0, allow_short=False)
            output = mapper.feed(event(name, 0), 0.1, allow_short=True)
            self.assertEqual(output + mapper.tick(1), [])

    def test_press_eligibility_is_captured_on_down_not_changed_on_release(self):
        self.mapper.feed(event('middle', 1), 0, allow_short=True)
        self.assertEqual(self.mapper.feed(event('middle', 0), 0.1, allow_short=False), [])
        self.assertEqual(self.mapper.tick(1), ['save'])

    def test_physical_overlap_cancels_ordinary_actions_until_all_released(self):
        self.mapper.feed(event('middle', 1), 0)
        self.mapper.feed(event('left', 1), 0.1)
        self.assertEqual(self.mapper.feed(event('middle', 0), 0.2), [])
        self.assertEqual(self.mapper.feed(event('left', 0), 0.3), [])
        self.assertEqual(self.mapper.tick(1), [])
        self.assertEqual(self.click('left', 1.1, 1.2), ['left'])

    def test_overlapping_middle_double_never_deletes(self):
        self.mapper.feed(event('left', 1), 0)
        output = self.click('middle', 0.1, 0.2)
        output += self.click('middle', 0.3, 0.4)
        output += self.mapper.feed(event('left', 0), 0.5)
        self.assertEqual(output + self.mapper.tick(1), [])

    def test_right_double_has_stop_priority_during_a_chord(self):
        self.mapper.feed(event('middle', 1), 0)
        output = self.click('right', 0.1, 0.2, eligible=False)
        output += self.click('right', 0.3, 0.4, eligible=False)
        output += self.mapper.feed(event('middle', 0), 0.5)
        self.assertEqual(output + self.mapper.tick(1), ['stop'])

    def test_unknown_interfaces_and_nonkey_events_do_not_start_clicks(self):
        self.assertEqual(self.mapper.feed(('other-keyboard', 1, 30, 1), 0), [])
        self.assertEqual(self.mapper.feed(('keyboard', 2, 30, 1), 0.1), [])
        self.assertEqual(self.mapper.tick(10), [])

    def test_dropped_events_cancel_pressed_and_pending_clicks(self):
        self.click('right', 0, 0.1)
        self.mapper.feed(('consumer', 0, module.SYN_DROPPED, 0), 0.2)
        self.assertEqual(self.mapper.tick(1), [])
        self.assertEqual(self.click('right', 1.1, 1.2), [])
        self.assertEqual(self.mapper.tick(2), [])
        self.mapper.feed(('consumer', 0, 0, 0), 2.1)
        self.assertEqual(self.click('right', 2.2, 2.3), [])
        self.assertEqual(self.mapper.tick(3), ['resume'])

    def test_late_processing_uses_source_timestamps_for_double_clicks(self):
        for name, token in (('middle', 'delete'), ('right', 'stop')):
            mapper = module.PedalGestures(BINDINGS)
            output = []
            for value, stamp in ((1, 100.0), (0, 100.1), (1, 100.2), (0, 100.3)):
                output += mapper.feed((*event(name, value), stamp), 200)
            output += mapper.tick(200)
            self.assertEqual(output, [token])

    def test_late_processing_does_not_merge_source_clicks_outside_window(self):
        output = []
        for value, stamp in ((1, 100.0), (0, 100.1), (1, 101.0), (0, 101.1)):
            output += self.mapper.feed((*event('right', value), stamp), 200)
        self.assertEqual(output + self.mapper.tick(200), ['resume', 'resume'])

    def test_delayed_source_after_newer_processing_tick_is_still_valid(self):
        self.mapper.tick(200)
        output = []
        for value, stamp in ((1, 100.0), (0, 100.1), (1, 100.2), (0, 100.3)):
            output += self.mapper.feed((*event('right', value), stamp), float('nan'))
        self.assertEqual(output + self.mapper.tick(200), ['stop'])
        with self.assertRaises(ValueError):
            self.mapper.feed((*event('right', 1), float('nan')), 201)

    def test_invalid_binding_identity_duplicates_window_and_clock_are_rejected(self):
        duplicate = config()['bindings']
        duplicate['right'] = dict(duplicate['left'])
        with self.assertRaises(ValueError):
            module.PedalGestures(duplicate)
        self.mapper.tick(1)
        with self.assertRaises(ValueError):
            self.mapper.tick(0)
        for invalid in (0, -1, True, float('nan')):
            with self.assertRaises(ValueError):
                module.PedalGestures(BINDINGS, double_click_window_s=invalid)


class DeviceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.sysfs = self.root / 'class'
        self.sysfs.mkdir()
        self.dev = self.root / 'dev'
        self.dev.mkdir()

    def add_interface(self, number, name, keys, *, serial=module.SERIAL, event_number=None):
        device = self.root / ('usb-' + serial)
        device.mkdir(exist_ok=True)
        for key, value in (('idVendor', module.VENDOR), ('idProduct', module.PRODUCT), ('serial', serial)):
            (device / key).write_text(value)
        interface = device / f'1-3:1.{number}'
        interface.mkdir(exist_ok=True)
        (interface / 'bInterfaceNumber').write_text(f'{number:02x}')
        input_dir = interface / 'input' / f'input{100 + number}'
        (input_dir / 'capabilities').mkdir(parents=True, exist_ok=True)
        (input_dir / 'name').write_text(name)
        (input_dir / 'phys').write_text('usb-0000:00:14.0-3/input0')
        (input_dir / 'capabilities' / 'ev').write_text('120013')
        (input_dir / 'capabilities' / 'key').write_text(keys)
        path = input_dir / f'event{number if event_number is None else event_number}'
        path.mkdir()
        (self.sysfs / path.name).symlink_to(path, target_is_directory=True)
        return path

    def device(self, config=None):
        return module.FootPedalDevice(config, sysfs_root=self.sysfs, dev_root=self.dev)

    def test_discovery_matches_exact_serial_and_keeps_all_three_hid_interfaces(self):
        for index, name in enumerate(('keyboard', 'mouse', 'consumer')):
            self.add_interface(index, name, str(index + 1))
        self.add_interface(9, 'another identical pedal', 'ff', serial='wrong-serial')
        rows = module.discover_interfaces(sysfs_root=self.sysfs, dev_root=self.dev)
        self.assertEqual(len(rows), 3)
        self.assertEqual({row['name'] for row in rows}, {'keyboard', 'mouse', 'consumer'})
        self.assertEqual(len({row['interface'] for row in rows}), 3)

    def test_interface_identity_survives_event_number_changes(self):
        path = self.add_interface(0, 'keyboard', 'ab')
        before = module.discover_interfaces(sysfs_root=self.sysfs, dev_root=self.dev)[0]['interface']
        (self.sysfs / path.name).unlink()
        moved = path.with_name('event87')
        path.rename(moved)
        (self.sysfs / moved.name).symlink_to(moved, target_is_directory=True)
        after = module.discover_interfaces(sysfs_root=self.sysfs, dev_root=self.dev)[0]['interface']
        self.assertEqual(before, after)

    def test_read_only_open_grabs_every_target_and_close_releases_all(self):
        for index in range(3):
            self.add_interface(index, str(index), str(index))
        with patch.object(module.os, 'open', side_effect=[40, 41, 42]) as opened, \
                patch.object(module.fcntl, 'ioctl') as ioctl, patch.object(module.os, 'close') as closed:
            device = self.device().open()
            self.assertEqual(len(device.filenos), 3)
            for call in opened.call_args_list:
                self.assertEqual(call.args[1] & os.O_ACCMODE, os.O_RDONLY)
                self.assertTrue(call.args[1] & os.O_NONBLOCK)
            clock_calls = [call for call in ioctl.call_args_list if call.args[1] == module.EVIOCSCLOCKID]
            self.assertEqual(clock_calls, [unittest.mock.call(fd, module.EVIOCSCLOCKID,
                             module.struct.pack('@i', module.time.CLOCK_MONOTONIC)) for fd in (40, 41, 42)])
            grab_calls = [call for call in ioctl.call_args_list if call.args[1] == module.EVIOCGRAB]
            self.assertEqual(grab_calls, [unittest.mock.call(fd, module.EVIOCGRAB, 1) for fd in (40, 41, 42)])
            device.close()
            self.assertEqual(device.filenos, {})
            self.assertEqual(ioctl.call_args_list[-3:], [unittest.mock.call(fd, module.EVIOCGRAB, 0) for fd in (40, 41, 42)])
            self.assertEqual(closed.call_count, 3)
            device.close()
            self.assertEqual(closed.call_count, 3)

    def test_partial_grab_failure_releases_previous_grabs_and_closes_all_descriptors(self):
        for index in range(3):
            self.add_interface(index, str(index), str(index))
        def grabbing(fd, request, value):
            if fd == 41 and value == 1:
                raise PermissionError('cannot grab')
        with patch.object(module.os, 'open', side_effect=[40, 41, 42]), \
                patch.object(module.fcntl, 'ioctl', side_effect=grabbing) as ioctl, \
                patch.object(module.os, 'close') as closed:
            device = self.device()
            with self.assertRaises(PermissionError):
                device.open()
            self.assertEqual(device.filenos, {})
            self.assertIn(unittest.mock.call(40, module.EVIOCGRAB, 0), ioctl.call_args_list)
            self.assertEqual(closed.call_count, 3)

    def test_permission_failure_after_one_open_does_not_leak_a_descriptor(self):
        for index in range(2):
            self.add_interface(index, str(index), str(index))
        with patch.object(module.os, 'open', side_effect=[40, PermissionError('denied')]), \
                patch.object(module.fcntl, 'ioctl') as ioctl, patch.object(module.os, 'close') as closed:
            with self.assertRaises(PermissionError):
                self.device().open()
            closed.assert_called_once_with(40)
            self.assertEqual(len(ioctl.call_args_list), 1)
            self.assertEqual(ioctl.call_args.args[1], module.EVIOCSCLOCKID)

    def test_clock_configuration_failure_closes_all_opened_fds_without_wall_clock_fallback(self):
        for index in range(3):
            self.add_interface(index, str(index), str(index))
        def configure(fd, request, _value):
            if fd == 41 and request == module.EVIOCSCLOCKID:
                raise OSError(errno.EINVAL, 'clock unsupported')
        with patch.object(module.os, 'open', side_effect=[40, 41, 42]), \
                patch.object(module.fcntl, 'ioctl', side_effect=configure) as ioctl, \
                patch.object(module.os, 'close') as closed:
            device = self.device()
            with self.assertRaises(OSError):
                device.open()
            self.assertEqual(device.filenos, {})
            self.assertEqual(closed.call_args_list, [unittest.mock.call(40), unittest.mock.call(41)])
            self.assertFalse(any(call.args[1] == module.EVIOCGRAB for call in ioctl.call_args_list))

    def test_partial_event_buffers_decode_signed_values_and_disconnect_is_explicit(self):
        device = self.device()
        device._fds = {40: 'keyboard'}
        device._buffers = {40: b''}
        encoded = module.INPUT_EVENT.pack(100, 250, module.EV_KEY, 30, 1) + module.INPUT_EVENT.pack(100, 500, 2, 0, -3)
        with patch.object(module.os, 'read', side_effect=[encoded[:7], BlockingIOError(), encoded[7:],
                                                         BlockingIOError(), BlockingIOError(), b'']):
            self.assertEqual(device.read_events(40), [])
            self.assertEqual(device.read_events(40), [('keyboard', 1, 30, 1, 100.00025), ('keyboard', 2, 0, -3, 100.0005)])
            self.assertEqual(device.read_events(40), [])
            with self.assertRaises(OSError):
                device.read_events(40)

    def test_read_drains_fixed_pulse_double_to_eagain_before_returning(self):
        device = self.device()
        device._fds = {40: 'consumer'}
        device._buffers = {40: b''}
        packets = [module.INPUT_EVENT.pack(100, usec, 1, 164, value)
                   for usec, value in ((0, 1), (100000, 0), (200000, 1), (300000, 0))]
        with patch.object(module.os, 'read', side_effect=packets + [BlockingIOError()]) as read:
            events = device.read_events(40)
        self.assertEqual(read.call_count, 5)
        self.assertEqual(events, [('consumer', 1, 164, value, 100 + usec / 1000000)
                                 for usec, value in ((0, 1), (100000, 0), (200000, 1), (300000, 0))])
        mapper = module.PedalGestures(BINDINGS)
        tokens = [token for row in events for token in mapper.feed(row, 200)]
        self.assertEqual(tokens + mapper.tick(200), ['stop'])

    def test_invalid_native_source_time_is_rejected_instead_of_synthesized(self):
        device = self.device()
        device._fds = {40: 'consumer'}
        device._buffers = {40: b''}
        invalid = module.INPUT_EVENT.pack(100, 1000000, 1, 164, 1)
        with patch.object(module.os, 'read', side_effect=[invalid, BlockingIOError()]):
            with self.assertRaises(RuntimeError):
                device.read_events(40)

    def test_invalid_config_and_missing_calibrated_interface_do_not_open_inputs(self):
        wrong = config()
        wrong['device']['serial'] = 'other'
        with self.assertRaises(ValueError):
            self.device(wrong)
        self.add_interface(0, 'keyboard', 'ab')
        with patch.object(module.os, 'open') as opened:
            with self.assertRaises(RuntimeError):
                self.device(config()).open()
            opened.assert_not_called()


class ScriptedDevice:
    def __init__(self, script):
        self.script = list(script)
        self.t = 0.0
        self.filenos = {1: 'scripted'}

    def clock(self):
        return self.t

    def select(self, _read, _write, _error, timeout):
        if self.script and self.script[0][0] <= self.t + timeout:
            self.t = self.script[0][0]
            return [1], [], []
        self.t += timeout
        return [], [], []

    def read_events(self, _fd):
        result = []
        while self.script and self.script[0][0] <= self.t:
            result.append(self.script.pop(0)[1])
        return result


class CalibrationTests(unittest.TestCase):
    def script(self, start=0.1, names=module.PEDALS):
        rows = []
        for name in names:
            for _ in range(3):
                rows.extend(((start, event(name, 1)), (start + 0.1, event(name, 0))))
                start += 0.3
        return rows

    def run_calibration(self, script, timeout=20):
        device = ScriptedDevice(script)
        lines = []
        result = module.calibrate(device, timeout, clock=device.clock, select_fn=device.select, emit=lines.append)
        return result, lines

    def test_three_matching_fixed_short_pulses_per_pedal_calibrate_without_holds(self):
        result, lines = self.run_calibration(self.script())
        self.assertEqual(result['bindings'], BINDINGS)
        self.assertEqual(result['calibration']['method'], 'press_count')
        self.assertEqual(result['calibration']['required_presses'], 3)
        self.assertEqual(result['calibration']['measured_press_counts'], dict.fromkeys(module.PEDALS, 3))
        self.assertTrue(all(duration < 0.2 for values in result['calibration']['measured_press_durations_s'].values()
                            for duration in values))
        self.assertEqual(sum('keydown interface=' in line for line in lines), 9)
        self.assertEqual(sum('3/3' in line for line in lines), 3)
        self.assertFalse(any('时间不足' in line for line in lines))

    def test_all_nine_clicks_delivered_together_advance_without_losing_batch_remainder(self):
        script = []
        for index, (_, row) in enumerate(self.script()):
            script.append((200, (*row, 100 + index * 0.1)))
        device = ScriptedDevice(script)
        device.t = 199
        result = module.calibrate(device, 20, clock=device.clock, select_fn=device.select, emit=lambda line: None)
        self.assertEqual(result['bindings'], BINDINGS)
        self.assertEqual(result['calibration']['measured_press_counts'], dict.fromkeys(module.PEDALS, 3))

    def test_duplicate_down_repeat_and_release_do_not_count_as_separate_presses(self):
        script = [(0.1, event('left', 1)), (0.2, event('left', 1)),
                  (0.3, event('left', 2)), (0.4, event('left', 0)), (0.5, event('left', 0))]
        with self.assertRaises(TimeoutError):
            self.run_calibration(script, timeout=1)

    def test_changed_codes_require_three_consecutive_matching_clicks(self):
        script = [(0.1, event('left', 1)), (0.2, event('left', 0)),
                  (0.3, event('middle', 1)), (0.4, event('middle', 0))]
        script += self.script(0.5)
        result, lines = self.run_calibration(script)
        self.assertEqual(result['bindings'], BINDINGS)
        self.assertTrue(any('键码改变' in line for line in lines))

    def test_previous_pedal_binding_cannot_be_reused_for_the_next_pedal(self):
        script = self.script(names=('left',))
        script += [(1.1, event('left', 1)), (1.2, event('left', 0))]
        script += self.script(1.4, names=('middle', 'right'))
        result, lines = self.run_calibration(script)
        self.assertEqual(result['bindings'], BINDINGS)
        self.assertTrue(any('重复' in line for line in lines))

    def test_overlap_resets_current_pedal_count_before_valid_three_clicks(self):
        script = self.script(names=('left',))[:4]
        script += [(0.8, event('left', 1)), (0.9, event('middle', 1)),
                   (1, event('left', 0)), (1.1, event('middle', 0))]
        result, lines = self.run_calibration(script + self.script(1.3))
        self.assertEqual(result['bindings'], BINDINGS)
        self.assertTrue(any('多键' in line for line in lines))
        self.assertEqual(sum('left: 3/3' in line for line in lines), 1)

    def test_dropped_input_resets_count_and_waits_for_syn_report(self):
        script = self.script(names=('left',))[:4]
        script += [(0.8, ('keyboard', 0, module.SYN_DROPPED, 0)),
                   (0.9, event('left', 1)), (1.0, event('left', 0)),
                   (1.1, ('keyboard', 0, 0, 0))]
        result, lines = self.run_calibration(script + self.script(1.3))
        self.assertEqual(result['bindings'], BINDINGS)
        self.assertTrue(any('事件丢失' in line for line in lines))

    def test_two_clicks_cannot_finish_a_pedal_even_if_they_were_long(self):
        script = [(0.1, event('left', 1)), (3, event('left', 0)),
                  (4, event('left', 1)), (7, event('left', 0))]
        with self.assertRaises(TimeoutError):
            self.run_calibration(script, timeout=8)

    def test_write_requires_complete_counts_and_remains_atomic_private_json(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pedal.json'
            for invalid_count in (2, True, None):
                invalid = config()
                invalid['calibration']['measured_press_counts']['right'] = invalid_count
                with self.assertRaises(ValueError):
                    module.write_config(path, invalid)
                self.assertFalse(path.exists())
            module.write_config(path, config())
            self.assertEqual(json.loads(path.read_text())['bindings'], BINDINGS)
            self.assertEqual(path.stat().st_mode & 0o777, 0o600)
            self.assertEqual(list(Path(directory).iterdir()), [path])

    def test_existing_verified_legacy_hold_config_remains_usable(self):
        legacy = config()
        legacy['calibration'] = {'measured_hold_seconds': dict.fromkeys(module.PEDALS, 2.3)}
        self.assertEqual(module.load_config(legacy)['bindings'], BINDINGS)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'pedal.json'
            module.write_config(path, legacy)
            self.assertEqual(json.loads(path.read_text())['bindings'], BINDINGS)


class TestModeTests(unittest.TestCase):
    def test_test_mode_sorts_cross_interface_events_before_emitting_actions(self):
        class Device:
            filenos = {1: 'keyboard', 2: 'mouse'}
            closed = False

            def __enter__(self):
                return self

            def __exit__(self, *args):
                self.closed = True

            def read_events(self, descriptor):
                if descriptor == 1:
                    return [(*event('left', 1), 100.0), (*event('left', 0), 100.3)]
                return [(*event('middle', 1), 100.1), (*event('middle', 0), 100.2)]

        device = Device()
        output = io.StringIO()
        with patch.object(module, 'FootPedalDevice', return_value=device), \
                patch.object(module, 'load_config', return_value=config()), \
                patch.object(module.select, 'select', side_effect=[([1, 2], [], []), KeyboardInterrupt]), \
                patch.object(module.time, 'monotonic', return_value=200), redirect_stdout(output):
            self.assertEqual(module.main(['--test', 'unused.json']), 130)
        # Both pedals physically overlapped, even though each fd's buffer has a
        # complete pair. Reading one interface at a time would wrongly emit left.
        self.assertEqual(len(output.getvalue().splitlines()), 1)
        self.assertTrue(device.closed)


if __name__ == '__main__':
    unittest.main()
