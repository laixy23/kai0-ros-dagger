"""Native decoder/cache/listener tests using stand-in frames and a no-send bus."""
from copy import deepcopy
from pathlib import Path
import queue
import struct
import sys
import threading
import time
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).parent))
import master_native_forwarding as module


POSE = [-22000, 30000, -40000, 5000, 6000, 0]


def message(key, data, stamp, **flags):
    values = dict(arbitration_id=key, data=bytes(data), timestamp=stamp, dlc=len(data),
                  is_rx=True, is_error_frame=False, is_remote_frame=False,
                  is_extended_id=False, is_fd=False)
    values.update(flags)
    return SimpleNamespace(**values)


def frames(kind, stamp=1000.01, *, joints=None, gripper=30000):
    joints = POSE if joints is None else joints
    ids = module.ACTUAL_IDS if kind == 'actual' else module.FA_IDS
    output = {}
    if kind == 'actual':
        output[0x2A1] = bytes([1, 0, 1, 0, 0, 0, 0, 0])
        for key, pair in zip((0x2A5, 0x2A6, 0x2A7), (joints[:2], joints[2:4], joints[4:])):
            output[key] = struct.pack('>ii', *pair)
        output[0x2A8] = struct.pack('>i', gripper) + bytes(4)
        for key in range(0x261, 0x267):
            output[key] = bytes([0, 0, 0, 0, 0, 0x40, 0, 0])
    else:
        for key, pair in zip((0x155, 0x156, 0x157), (joints[:2], joints[2:4], joints[4:])):
            output[key] = struct.pack('>ii', *pair)
        output[0x159] = struct.pack('>i', gripper) + bytes([0x03, 0xE8, 1, 0])
    return {key: (output[key], stamp + index * 0.001) for index, key in enumerate(ids)}


def feed(cache, values, **flags):
    for key, (data, stamp) in values.items():
        cache.ingest(message(key, data, stamp, **flags))


class DecoderCacheTests(unittest.TestCase):
    def setUp(self):
        self.wall = 1000.1
        self.cache = module.NativeMasterCache(wall=lambda: self.wall)

    def phase(self, kind='fa', since=1000.0):
        return self.cache.begin_phase(kind, since)

    def test_fa_full_group_preserves_signed_units_and_oldest_native_source_time(self):
        epoch = self.phase()
        values = frames('fa')
        feed(self.cache, values)
        result = self.cache.snapshot()
        self.assertEqual(result['kind'], 'fa')
        self.assertEqual(result['epoch'], epoch)
        self.assertEqual(result['joint_mdeg'], POSE)
        self.assertEqual(result['gripper_um'], 30000)
        self.assertEqual(result['source_time'], min(stamp for _, stamp in values.values()))
        self.assertEqual(result['component_stamps'], {f'0x{key:03X}': stamp for key, (_, stamp) in values.items()})
        self.wall += 0.01
        self.assertEqual(self.cache.snapshot()['source_time'], result['source_time'])

    def test_actual_requires_all_eleven_physical_components(self):
        self.phase('actual')
        values = frames('actual')
        missing = values.pop(0x264)
        feed(self.cache, values)
        self.assertIsNone(self.cache.snapshot())
        feed(self.cache, {0x264: missing})
        result = self.cache.snapshot()
        self.assertEqual(result['joint_mdeg'], POSE)
        self.assertEqual(len(result['component_stamps']), 11)
        self.assertEqual(result['source_time'], values[0x2A5][1])

    def test_partial_fa_group_never_defaults_missing_joint_or_gripper_to_zero(self):
        self.phase()
        values = frames('fa')
        del values[0x159]
        feed(self.cache, values)
        self.assertIsNone(self.cache.snapshot())

    def test_real_all_zero_native_teaching_pose_is_legal_without_any_fallback(self):
        self.phase()
        feed(self.cache, frames('fa', joints=[0] * 6, gripper=0))
        result = self.cache.snapshot()
        self.assertEqual(result['joint_mdeg'], [0] * 6)
        self.assertEqual(result['gripper_um'], 0)

    def test_local_tx_echo_is_rejected_even_from_a_different_sdk_socket(self):
        self.phase()
        feed(self.cache, frames('fa'), is_rx=False)
        self.assertIsNone(self.cache.snapshot())
        feed(self.cache, frames('fa'), is_rx=True)
        first = self.cache.snapshot()
        feed(self.cache, frames('fa', stamp=1000.05, joints=[90000] * 6), is_rx=False)
        self.assertIsNone(self.cache.snapshot(after=first['component_stamps']))
        self.assertEqual(self.cache.snapshot()['joint_mdeg'], POSE)

    def test_missing_physical_rx_marker_does_not_pass(self):
        self.phase()
        feed(self.cache, frames('fa'), is_rx=None)
        self.assertIsNone(self.cache.snapshot())

    def test_phase_change_clears_old_actual_and_fa_buffers_and_increments_epoch(self):
        first = self.phase('actual')
        feed(self.cache, frames('actual'))
        self.assertIsNotNone(self.cache.snapshot())
        second = self.phase('fa', since=1000.04)
        self.assertGreater(second, first)
        self.assertIsNone(self.cache.snapshot())
        feed(self.cache, frames('fa', stamp=1000.03))
        self.assertIsNone(self.cache.snapshot())
        feed(self.cache, frames('fa', stamp=1000.05))
        self.assertEqual(self.cache.snapshot()['epoch'], second)

    def test_after_requires_every_relevant_fa_id_to_advance(self):
        self.phase()
        feed(self.cache, frames('fa'))
        previous = self.cache.snapshot()['component_stamps']
        newer = frames('fa', stamp=1000.05)
        feed(self.cache, {key: value for key, value in newer.items() if key != 0x159})
        self.assertIsNone(self.cache.snapshot(after=previous))
        feed(self.cache, {0x159: newer[0x159]})
        self.assertIsNotNone(self.cache.snapshot(after=previous))

    def test_after_requires_slow_actual_motor_status_to_advance_too(self):
        self.phase('actual')
        feed(self.cache, frames('actual'))
        previous = self.cache.snapshot()['component_stamps']
        newer = frames('actual', stamp=1000.05)
        feed(self.cache, {key: value for key, value in newer.items() if key != 0x261})
        self.assertIsNone(self.cache.snapshot(after=previous))
        feed(self.cache, {0x261: newer[0x261]})
        self.assertIsNotNone(self.cache.snapshot(after=previous))

    def test_after_from_wrong_phase_or_nonfinite_stamp_is_rejected(self):
        self.phase()
        feed(self.cache, frames('fa'))
        for previous in ({'0x2A5': 1000}, {'0x155': float('nan')}):
            with self.subTest(previous=previous), self.assertRaises(ValueError):
                self.cache.snapshot(after=previous)

    def test_out_of_order_native_frame_cannot_replace_newer_value(self):
        self.phase()
        feed(self.cache, frames('fa', stamp=1000.05))
        feed(self.cache, frames('fa', stamp=1000.04, joints=[80000] * 6))
        self.assertEqual(self.cache.snapshot()['joint_mdeg'], POSE)

    def test_stale_components_return_pending_without_refreshing_source_times(self):
        for kind in ('actual', 'fa'):
            with self.subTest(kind=kind):
                self.phase(kind)
                feed(self.cache, frames(kind))
                self.assertIsNotNone(self.cache.snapshot())
                self.wall = 1000.5
                self.assertIsNone(self.cache.snapshot())
                self.wall = 1000.1

    def test_stale_received_backlog_is_ignored(self):
        self.phase()
        feed(self.cache, frames('fa', stamp=999.9))
        self.assertIsNone(self.cache.snapshot())

    def test_each_pose_group_must_be_coherent_in_time(self):
        for kind, late in (('fa', 1000.04), ('actual', 1000.08)):
            with self.subTest(kind=kind):
                self.phase(kind)
                values = frames(kind)
                gripper_id = 0x159 if kind == 'fa' else 0x2A8
                values[gripper_id] = (values[gripper_id][0], late)
                feed(self.cache, values)
                self.assertIsNone(self.cache.snapshot())

    def test_rtr_fd_and_extended_messages_are_not_native_samples(self):
        for flag in ('is_remote_frame', 'is_fd', 'is_extended_id'):
            with self.subTest(flag=flag):
                self.phase()
                feed(self.cache, frames('fa'), **{flag: True})
                self.assertIsNone(self.cache.snapshot())

    def test_short_frames_error_frames_and_future_native_frames_fail_closed(self):
        for variant in ('short', 'error', 'future'):
            with self.subTest(variant=variant):
                self.phase()
                packet = message(0x155, bytes(8), 1000.05)
                if variant == 'short':
                    packet.data, packet.dlc = bytes(7), 7
                elif variant == 'error':
                    packet.is_error_frame = True
                else:
                    packet.timestamp = 1000.2
                with self.assertRaises(RuntimeError):
                    self.cache.ingest(packet)
                with self.assertRaises(RuntimeError):
                    self.cache.snapshot()

    def test_only_known_fa_width_status_codes_are_accepted(self):
        for status in (0, 1, 2, 3, 4, 5, 6, 7, 8, 0x10, 0xFF):
            values = frames('fa')
            payload = bytearray(values[0x159][0])
            payload[6] = status
            values[0x159] = (bytes(payload), values[0x159][1])
            self.phase()
            with self.subTest(status=status):
                if status < 4:
                    feed(self.cache, values)
                    self.assertIsNotNone(self.cache.snapshot())
                else:
                    with self.assertRaises(RuntimeError):
                        feed(self.cache, values)
                    with self.assertRaises(RuntimeError):
                        module.decode_native_snapshot('fa', values, since_timestamp=1000,
                                                      now=self.wall)

    def test_gripper_width_uses_sdk_signed_bounds_in_both_phases(self):
        for kind in ('actual', 'fa'):
            for width in (-150001, 150001):
                with self.subTest(kind=kind, width=width):
                    self.phase(kind)
                    if kind == 'fa':
                        with self.assertRaises(RuntimeError):
                            feed(self.cache, frames(kind, gripper=width))
                    else:
                        feed(self.cache, frames(kind, gripper=width))
                        with self.assertRaises(RuntimeError):
                            self.cache.snapshot()

    def test_signed_width_is_preserved_while_only_command_target_is_clamped(self):
        for kind in ('actual', 'fa'):
            for raw, command in ((-1200, 0), (80001, 80000), (-150000, 0), (150000, 80000)):
                with self.subTest(kind=kind, raw=raw):
                    self.phase(kind)
                    feed(self.cache, frames(kind, gripper=raw))
                    result = self.cache.snapshot()
                    self.assertEqual(result['raw_gripper_um'], raw)
                    self.assertEqual(result['gripper_um'], raw)
                    self.assertEqual(result['command_gripper_um'], command)

    def test_diagnostics_identify_missing_unadvanced_and_lingering_actual_ids(self):
        self.phase('fa')
        values = frames('fa')
        feed(self.cache, {key: value for key, value in values.items() if key != 0x159})
        feed(self.cache, frames('actual'))
        feed(self.cache, values, is_rx=False)
        feed(self.cache, frames('fa', stamp=999.99))
        diagnostics = self.cache.diagnostics(after={f'0x{key:03X}': stamp for key, (_, stamp) in values.items()})
        self.assertEqual(diagnostics['missing_ids'], ['0x159'])
        self.assertEqual(diagnostics['rx_counts']['0x155'], 1)
        self.assertEqual(set(diagnostics['unadvanced_ids']), {'0x155', '0x156', '0x157'})
        self.assertTrue(diagnostics['native_actual_rx_after_fa'])
        self.assertEqual(diagnostics['rejections']['host_tx_or_missing_rx_marker'], 4)
        self.assertGreater(diagnostics['rejections']['old_phase'], 0)
        self.assertEqual(diagnostics['last_rx']['0x155']['hex'], values[0x155][0].hex())

    def test_diagnostics_retain_bad_width_raw_hex_and_source_age(self):
        self.phase()
        values = frames('fa', gripper=150001)
        with self.assertRaisesRegex(RuntimeError, 'raw_gripper_um=150001'):
            feed(self.cache, values)
        diagnostics = self.cache.diagnostics()
        self.assertEqual(diagnostics['rx_counts']['0x159'], 1)
        self.assertEqual(diagnostics['last_rx']['0x159']['hex'], values[0x159][0].hex())
        self.assertEqual(diagnostics['rejections']['width_outlier'], 1)

    def test_actual_cannot_prime_from_standby_teach_disabled_fault_or_angle(self):
        for variant in ('standby', 'teach', 'disabled', 'fault', 'angle'):
            with self.subTest(variant=variant):
                self.phase('actual')
                values = frames('actual')
                key = 0x261 if variant in ('disabled', 'fault') else 0x2A8 if variant == 'angle' else 0x2A1
                payload = bytearray(values[key][0])
                if variant == 'standby':
                    payload[0] = 0
                elif variant == 'teach':
                    payload[0] = 2
                elif variant == 'disabled':
                    payload[5] = 0
                elif variant == 'fault':
                    payload[5] = 0x41
                else:
                    payload[7] = 1
                values[key] = (bytes(payload), values[key][1])
                feed(self.cache, values)
                with self.assertRaises(RuntimeError):
                    self.cache.snapshot()

    def test_invalid_phase_and_future_phase_start_are_rejected(self):
        for kind, since in (('other', 1000.0), ('fa', float('nan')), ('fa', 0), ('fa', 1000.2)):
            with self.subTest(kind=kind, since=since), self.assertRaises(ValueError):
                self.cache.begin_phase(kind, since)


class FakeBus:
    def __init__(self):
        self.queue = queue.Queue()
        self.close_count = self.send_count = 0

    def recv(self, timeout):
        try:
            value = self.queue.get(timeout=timeout)
        except queue.Empty:
            return None
        if isinstance(value, Exception):
            raise value
        return value

    def send(self, *_args, **_kwargs):
        self.send_count += 1
        raise AssertionError('Native listener must never send a CAN frame')

    def shutdown(self):
        self.close_count += 1

    def inject(self, values, **flags):
        for key, (data, stamp) in values.items():
            self.queue.put(message(key, data, stamp, **flags))


class ReaderTests(unittest.TestCase):
    def setUp(self):
        self.bus = FakeBus()
        self.factory_channels = []
        def factory(channel):
            self.factory_channels.append(channel)
            return self.bus
        self.reader = module.NativeMasterReader('can_left_mas', bus_factory=factory,
                                                wall=lambda: 1000.1, mono=lambda: 10.0)
        self.addCleanup(self.reader.stop)

    def await_result(self, predicate, timeout=0.5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            result = predicate()
            if result:
                return result
            time.sleep(0.002)
        self.fail('fake native reader did not produce expected result')

    def test_start_opens_one_passive_bus_and_frames_cross_phases_without_reopening(self):
        self.assertIs(self.reader.start(), self.reader)
        self.assertTrue(self.reader.wait_started())
        self.assertIs(self.reader.start(), self.reader)
        actual_epoch = self.reader.begin_phase('actual', 1000.0)
        self.bus.inject(frames('actual'))
        actual = self.await_result(self.reader.snapshot)
        self.assertEqual(actual['epoch'], actual_epoch)
        fa_epoch = self.reader.begin_phase('fa', 1000.04)
        self.assertIsNone(self.reader.snapshot())
        self.bus.inject(frames('fa', stamp=1000.05))
        result = self.await_result(self.reader.snapshot)
        self.assertEqual((result['kind'], result['epoch']), ('fa', fa_epoch))
        self.assertEqual(self.factory_channels, ['can_left_mas'])
        self.assertEqual(self.bus.send_count, 0)
        self.reader.stop()
        self.assertEqual(self.bus.close_count, 1)

    def test_recv_failure_raises_in_snapshot_and_closes_without_tx(self):
        self.reader.start()
        self.reader.wait_started()
        self.reader.begin_phase('fa', 1000.0)
        self.bus.queue.put(RuntimeError('receive failed'))
        self.await_result(lambda: self.bus.close_count)
        with self.assertRaisesRegex(RuntimeError, 'receive failed'):
            self.reader.snapshot()
        self.assertEqual(self.bus.send_count, 0)

    def test_native_validation_failure_stops_listener_instead_of_returning_old_group(self):
        self.reader.start()
        self.reader.wait_started()
        self.reader.begin_phase('fa', 1000.0)
        self.bus.inject(frames('fa'))
        self.await_result(self.reader.snapshot)
        self.bus.queue.put(message(0x159, bytes([0, 0, 0, 0, 0, 0, 0x10, 0]), 1000.06))
        self.await_result(lambda: self.bus.close_count)
        with self.assertRaisesRegex(RuntimeError, 'WIDTH'):
            self.reader.snapshot()
        self.assertEqual(self.bus.send_count, 0)

    def test_startup_failure_is_reported_by_wait_started_and_snapshot(self):
        def failed(_channel):
            raise OSError('could not open CAN socket')
        reader = module.NativeMasterReader('can_right_mas', bus_factory=failed)
        self.addCleanup(reader.stop)
        reader.start()
        with self.assertRaisesRegex(RuntimeError, 'could not open'):
            reader.wait_started()
        with self.assertRaisesRegex(RuntimeError, 'could not open'):
            reader.snapshot()

    def test_wait_started_timeout_does_not_claim_ready(self):
        release = threading.Event()
        def delayed(_channel):
            release.wait(0.5)
            return self.bus
        reader = module.NativeMasterReader('can_left_mas', bus_factory=delayed)
        self.addCleanup(reader.stop)
        reader.start()
        try:
            with self.assertRaises(TimeoutError):
                reader.wait_started(timeout=0.01)
        finally:
            release.set()
        self.assertTrue(reader.wait_started(timeout=0.5))
        reader.stop()
        self.assertEqual(self.bus.close_count, 1)

    def test_stop_is_idempotent_and_stopped_cache_cannot_be_consumed(self):
        self.reader.start()
        self.reader.wait_started()
        self.reader.begin_phase('fa', 1000.0)
        self.bus.inject(frames('fa'))
        self.await_result(self.reader.snapshot)
        self.reader.stop()
        self.reader.stop()
        self.assertEqual(self.bus.close_count, 1)
        with self.assertRaises(RuntimeError):
            self.reader.snapshot()
        with self.assertRaises(RuntimeError):
            self.reader.begin_phase('actual', 1000.0)

    def test_before_start_is_never_ready_and_wrong_channels_never_open(self):
        with self.assertRaises(RuntimeError):
            self.reader.wait_started()
        with self.assertRaises(ValueError):
            module.NativeMasterReader('can_left_slave', bus_factory=lambda _: self.fail('must not open'))
        self.assertEqual(self.factory_channels, [])


if __name__ == '__main__':
    unittest.main()
