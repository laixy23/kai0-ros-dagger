"""Offline protocol and guarded adjustment checks; no hardware or SDK imports."""
from collections import deque
from types import SimpleNamespace
import unittest

from joint_assistance_adjust import Transport, adjust


class Clock:
    def __init__(self):
        self.elapsed = 0.0

    def clock(self):
        return self.elapsed

    def wall(self):
        return 1000.0 + self.elapsed


class Bus:
    def __init__(self, clock):
        self.clock = clock
        self.pending = deque()
        self.sent = []
        self.levels = [0] * 6
        self.readback_override = None
        self.send_ack = True
        self.native = True
        self.stale = False
        self.short = False
        self.query_responses = deque()
        self.before_setting = lambda: None

    def message(self, ident, data):
        return SimpleNamespace(
            arbitration_id=ident, data=bytes(data), is_extended_id=False,
            is_error_frame=False, is_remote_frame=False, is_rx=self.native,
            timestamp=self.clock.wall() - (1 if self.stale else 0),
        )

    def send(self, message):
        key, data = message.arbitration_id, bytes(message.data)
        self.sent.append((key, data))
        if key == 0x477:
            response = self.query_responses.popleft() if self.query_responses else (
                self.readback_override if self.readback_override is not None else self.levels)
            payload = bytes(response) + b'\x00\x00'
            self.pending.append(self.message(0x488, payload[:7] if self.short else payload))
        elif key == 0x487:
            self.before_setting()
            self.levels = list(data[:6])
            if self.send_ack:
                self.pending.append(self.message(0x476, b'\x87\x00\x00\x00\x00\x00\x00\x00'))
        else:
            raise AssertionError('Unexpected CAN control frame: ' + hex(key))

    def recv(self, timeout=0):
        self.clock.elapsed += max(timeout or 0, 0.001)
        return self.pending.popleft() if self.pending else None


class AssistanceAdjustmentTests(unittest.TestCase):
    def setUp(self):
        self.time = Clock()
        self.bus = Bus(self.time)
        self.events = []
        self.guards = 0
        self.stopped = False
        self.transport = Transport(
            self.bus, lambda **kwargs: SimpleNamespace(**kwargs),
            clock=self.time.clock, wall=self.time.wall,
            stop_requested=lambda: self.stopped)

    def guard(self):
        self.guards += 1

    def adjust(self, **options):
        arguments = dict(
            expected=[0] * 6, apply=True, confirm_supported=True,
            guard=self.guard, stop_requested=lambda: self.stopped,
            emit=lambda row: self.events.append(row),
        )
        arguments.update(options)
        return adjust(self.transport, **arguments)

    def settings(self):
        return [frame for frame in self.bus.sent if frame[0] == 0x487]

    def rejected_adjustment(self, **options):
        try:
            report = self.adjust(**options)
        except RuntimeError:
            pass
        else:
            self.assertFalse(report.get('verified', False))
            self.assertIn('error', report)
        self.assertEqual(self.settings(), [])

    def test_query_uses_exact_query_frame_and_reads_full_native_vector(self):
        self.bus.levels = [0, 1, 2, 8, 9, 10]
        self.assertEqual(self.transport.query(), self.bus.levels)
        self.assertEqual(self.bus.sent, [(0x477, bytes([5, 0, 0, 0, 3, 0, 0, 0]))])

    def test_query_discards_stale_pending_before_sending_request(self):
        self.bus.pending.append(self.bus.message(0x488, bytes([9] * 6 + [0, 0])))
        self.assertEqual(self.transport.query(), [0] * 6)

    def test_query_rejects_tx_echo_stale_or_short_responses(self):
        for attribute in ('native', 'stale', 'short'):
            with self.subTest(attribute=attribute):
                self.bus.native = attribute != 'native'
                self.bus.stale = attribute == 'stale'
                self.bus.short = attribute == 'short'
                with self.assertRaises(RuntimeError):
                    self.transport.query()
        self.assertEqual(self.settings(), [])

    def test_query_rejects_a_level_outside_zero_to_ten(self):
        self.bus.levels = [0, 0, 0, 0, 0, 11]
        with self.assertRaises(RuntimeError):
            self.transport.query()
        self.assertEqual(self.settings(), [])

    def test_configure_requires_ack_and_exact_full_readback(self):
        target = [1, 2, 3, 4, 5, 6]
        self.transport.configure(target)
        self.assertEqual(self.bus.sent, [
            (0x487, bytes(target + [0, 0])),
            (0x477, bytes([5, 0, 0, 0, 3, 0, 0, 0]))])

    def test_missing_ack_does_not_retry_setting(self):
        self.bus.send_ack = False
        with self.assertRaises(RuntimeError):
            self.transport.configure([1] * 6)
        self.assertEqual(self.settings(), [(0x487, bytes([1] * 6 + [0, 0]))])

    def test_readback_mismatch_does_not_retry_or_send_other_control(self):
        self.bus.readback_override = [0] * 6
        with self.assertRaises(RuntimeError):
            self.transport.configure([1] * 6)
        self.assertEqual(self.settings(), [(0x487, bytes([1] * 6 + [0, 0]))])
        self.assertTrue(all(key in (0x477, 0x487) for key, _ in self.bus.sent))

    def test_default_readonly_double_query_sends_no_setting(self):
        report = self.adjust(apply=False, confirm_supported=False)
        self.assertNotIn('error', report)
        self.assertEqual(len([frame for frame in self.bus.sent if frame[0] == 0x477]), 2)
        self.assertEqual(self.settings(), [])

    def test_different_queries_or_unexpected_current_vector_prevent_write(self):
        self.bus.query_responses.extend(([0] * 6, [1] * 6))
        self.rejected_adjustment()
        self.bus.query_responses.clear()
        self.bus.levels = [2] * 6
        self.rejected_adjustment()

    def test_apply_requires_support_and_no_stop_request(self):
        self.rejected_adjustment(confirm_supported=False)
        self.stopped = True
        self.rejected_adjustment()

    def test_apply_increments_each_joint_once_with_ten_cap(self):
        before = [0, 1, 5, 9, 10, 0]
        self.bus.levels = before.copy()
        report = self.adjust(expected=before)
        self.assertNotIn('error', report)
        self.assertTrue(report['verified'])
        self.assertEqual(self.settings(), [(0x487, bytes([1, 2, 6, 10, 10, 1, 0, 0]))])
        self.assertGreaterEqual(self.guards, 1)

    def test_before_and_target_record_is_emitted_before_setting_frame(self):
        def verify_backup_event():
            self.assertTrue(any(row.get('before') == [0] * 6 and row.get('target') == [1] * 6
                                for row in self.events), 'Save complete before/target vectors before writing')
        self.bus.before_setting = verify_backup_event
        report = self.adjust()
        self.assertNotIn('error', report)
        self.assertTrue(report['verified'])

    def test_restore_writes_exact_saved_baseline_vector(self):
        baseline = [0, 1, 5, 9, 10, 0]
        expected = [1, 2, 6, 10, 10, 1]
        self.bus.levels = expected.copy()
        report = self.adjust(expected=expected, target_override=baseline)
        self.assertNotIn('error', report)
        self.assertTrue(report['verified'])
        self.assertEqual(self.settings(), [(0x487, bytes(baseline + [0, 0]))])


if __name__ == '__main__':
    unittest.main()
