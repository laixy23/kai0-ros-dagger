#!/usr/bin/env python3
"""Read/adjust only left-follower joint assistance, using official pyAgxArm wire format.

Default is a query-only plan. Apply changes each joint by at most one level;
restore uses the saved original vector. No mode, payload, motor enable, torque,
position, gripper, calibration, home or reset commands are sent.
"""
import argparse
import datetime
import fcntl
import json
import os
from pathlib import Path
import signal
import threading
import time

from follower_drag_probe import ROOT, CONTROLLERS, check_environment, check_idle_can, require

CHANNEL = 'can_left_slave'
QUERY = bytes([5, 0, 0, 0, 3, 0, 0, 0])
CONTROLLERS.add(Path(__file__).name)


def valid_levels(values):
    require(isinstance(values, (list, tuple)) and len(values) == 6
            and all(type(value) is int and 0 <= value <= 10 for value in values),
            'Assistance must contain six integer levels in [0,10]')
    return list(values)


class Transport:
    def __init__(self, bus, message_factory, *, clock=time.monotonic, wall=time.time,
                 stop_requested=lambda: False):
        self.bus, self.message_factory = bus, message_factory
        self.clock, self.wall, self.stop_requested = clock, wall, stop_requested
        self.query_count = self.configuration_writes = 0
        self.last_readback = None
        self.last_ack = False

    def drain(self):
        for _ in range(256):
            if self.bus.recv(timeout=0) is None:
                return

    def send(self, ident, payload):
        require(not self.stop_requested(), 'Stopped before CAN request')
        self.bus.send(self.message_factory(arbitration_id=ident, data=payload, is_extended_id=False))

    def native(self, message, ident, since):
        return (message is not None and message.arbitration_id == ident and message.is_rx
                and not message.is_extended_id and not message.is_error_frame
                and not message.is_remote_frame and len(message.data) == 8
                and message.timestamp >= since)

    def query(self, timeout=1.5):
        self.drain()
        since = self.wall()
        self.send(0x477, QUERY)
        self.query_count += 1
        until = self.clock() + timeout
        while self.clock() < until:
            require(not self.stop_requested(), 'Stopped while querying assistance')
            message = self.bus.recv(timeout=min(0.1, max(0, until-self.clock())))
            if self.native(message, 0x488, since):
                self.last_readback = valid_levels(list(message.data[:6]))
                return list(self.last_readback)
        raise RuntimeError('No fresh valid 0x488 assistance response; refusing settings change')

    def configure(self, levels):
        levels = valid_levels(levels)
        self.drain()
        since = self.wall()
        self.configuration_writes += 1
        self.send(0x487, bytes(levels) + b'\x00\x00')
        self.last_ack = False
        until = self.clock() + 1.5
        while self.clock() < until:
            require(not self.stop_requested(), 'Stopped after setting; state may have changed')
            message = self.bus.recv(timeout=min(0.1, max(0, until-self.clock())))
            if self.native(message, 0x476, since) and message.data[0] == 0x87:
                self.last_ack = True
                break
        observed = self.query()
        require(self.last_ack, 'No fresh 0x87 setting ACK; inspect readback before retrying')
        require(observed == levels, 'Assistance readback does not match requested levels')
        return observed


def adjust(transport, *, expected, target_override=None, apply=False, confirm_supported=False,
           guard=lambda: None, stop_requested=lambda: False, emit=lambda row: None):
    report = {'record_type': 'summary', 'channel': CHANNEL, 'verified': False,
              'unrelated_control_commands_sent': 0, 'automatic_retry_or_restore': False,
              'parameter_kind': 'joint_assistance_rating_not_linear_gravity_gain'}
    try:
        expected = valid_levels(expected)
        target = ([min(value+1, 10) for value in expected] if target_override is None
                  else valid_levels(target_override))
        require(all(abs(a-b) <= 1 for a, b in zip(target, expected)),
                'This tool permits at most one level per joint per adjustment')
        require(not apply or confirm_supported, 'Physical support is required before adjustment')
        require(not stop_requested(), 'Stopped before reading settings')
        guard()
        before = transport.query()
        require(transport.query() == before == expected,
                'Current levels differ from the saved expected vector; no settings changed')
        report.update(before=before, target=target)
        emit({'record_type': 'before_write', 'channel': CHANNEL, 'before': before, 'target': target})
        if not apply or target == before:
            report['passive_only'] = True
            return report
        guard()
        require(not stop_requested(), 'Stopped before writing assistance')
        report['readback'] = transport.configure(target)
        report['verified'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = repr(exc)
    finally:
        report['configuration_writes_attempted'] = transport.configuration_writes
        report['query_count'] = transport.query_count
        report['last_readback'] = transport.last_readback
        report['ack_received'] = transport.last_ack
        emit(report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--restore', action='store_true')
    parser.add_argument('--confirm-supported', action='store_true')
    parser.add_argument('--baseline-file', type=Path,
                        default=ROOT/'_runtime/left-follower-assistance-baseline-20261111.json')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.apply and not args.confirm_supported:
        parser.error('--apply requires --confirm-supported')
    baseline = json.loads(args.baseline_file.read_text())
    require(baseline.get('channel') == CHANNEL, 'Saved baseline belongs to another arm')
    original = valid_levels(baseline['original_levels'])
    increased = [min(value+1, 10) for value in original]
    expected, target = (increased, original) if args.restore else (original, None)
    stopped = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, lambda _s, _f: stopped.set())
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    path = args.output or ROOT/'_runtime'/('left-follower-assistance-'+stamp+'.jsonl')
    path.parent.mkdir(parents=True, exist_ok=True)
    with (ROOT/'_runtime/keyboard-dagger.lock').open('a+') as lock, path.open('x') as output:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        check_environment(CHANNEL)
        check_idle_can()
        import can
        filters = [{'can_id': ident, 'can_mask': 0x7ff, 'extended': False} for ident in (0x476, 0x488)]
        with can.Bus(channel=CHANNEL, interface='socketcan', receive_own_messages=False,
                     can_filters=filters) as bus:
            transport = Transport(bus, can.Message, stop_requested=stopped.is_set)
            def emit(row):
                line = json.dumps(row, separators=(',', ':'), allow_nan=False)
                output.write(line+'\n')
                output.flush()
                os.fsync(output.fileno())
                print(line, flush=True)
            report = adjust(transport, expected=expected, target_override=target, apply=args.apply,
                            confirm_supported=args.confirm_supported,
                            guard=lambda: check_environment(CHANNEL),
                            stop_requested=stopped.is_set, emit=emit)
    print('Report: '+str(path), flush=True)
    return 1 if report.get('error') else 0


if __name__ == '__main__':
    raise SystemExit(main())
