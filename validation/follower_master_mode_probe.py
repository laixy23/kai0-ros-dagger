#!/usr/bin/env python3
"""Temporarily use the official FA master profile on the supported LEFT follower.

This is independent of DAgger. FA, zero offsets, EnableArm(7), and drag-teach=1
match the deployed master's initialization. No position/gripper/home/reset target
is sent. FA can suppress actual feedback: native 15x output is not actual pose.
The profile stays active until an explicit, supported --restore-follower run.
"""
from __future__ import annotations

import argparse
import collections
import datetime
import fcntl
import json
from pathlib import Path
import signal
import sys
import threading
import time

from follower_drag_probe import (ROOT, CONTROLLERS, check_environment, check_idle_can,
                                 make_sdk, require, sample_feedback, validate_snapshot)
from prepare_follower_mode import prepare as prepare_follower

CONTROLLERS.add(Path(__file__).name)
CHANNEL = 'can_left_slave'


class RawRecorder:
    """Count native received frames, excluding echoes of our own SDK writes."""
    def __init__(self, channel):
        self.channel = channel
        self.counts = collections.Counter()
        self.latest = {}

    def __enter__(self):
        import can
        self.bus = can.Bus(channel=self.channel, interface='socketcan', receive_own_messages=False)
        return self

    def __exit__(self, *_):
        self.bus.shutdown()

    def observe(self, seconds, *, guard, stop_requested):
        until = time.monotonic() + seconds
        next_guard = 0.0
        while time.monotonic() < until:
            require(not stop_requested(), 'Stop requested during observation; no later control steps sent')
            if time.monotonic() >= next_guard:
                guard()
                next_guard = time.monotonic() + 0.5
            message = self.bus.recv(timeout=min(0.05, max(0, until-time.monotonic())))
            if message is None or not message.is_rx:
                continue
            require(not message.is_error_frame, 'CAN error frame observed')
            if message.is_remote_frame:
                continue
            key, data = message.arbitration_id, bytes(message.data)
            self.counts[f'0x{key:03X}'] += 1
            self.latest[f'0x{key:03X}'] = {'data': data.hex(), 'timestamp_sec': message.timestamp}
            if key == 0x2A1:
                require(len(data) == 8 and data[1] == 0 and int.from_bytes(data[6:8], 'big') == 0,
                        'Actual arm fault observed after profile request')
            elif 0x261 <= key <= 0x266:
                require(len(data) == 8 and not data[5] & 0xBF, 'Actual driver fault observed')

    def summary(self):
        return {'native_frame_counts': dict(self.counts), 'native_latest': self.latest,
                'control_15x_frames_are_not_actual_pose': True}


def run_master_profile(channel, *, apply=False, confirm_supported=False, duration=15,
                       reader=sample_feedback, sdk_factory=make_sdk,
                       guard=check_environment, idle_guard=check_idle_can,
                       recorder_factory=RawRecorder, wall=time.time,
                       stop_requested=lambda: False, emit=lambda row: None):
    report = {'record_type': 'summary', 'channel': channel, 'commands': [],
              'gravity_compensation_verified': False, 'automatic_restore': False,
              'position_gripper_home_reset_commands_sent': 0}
    sdk = None
    recorder = None
    def command(method, *args, **kwargs):
        guard(channel)
        require(not stop_requested(), 'Stop requested before ' + method)
        report['commands'].append({'method': method, 'args': list(args), 'kwargs': kwargs,
                                   'timestamp_sec': wall()})
        getattr(sdk, method)(*args, **kwargs)
    try:
        require(channel == CHANNEL, 'This profile is restricted to the mapped LEFT follower')
        require(isinstance(duration, (int, float)) and 1 <= duration <= 60,
                'Observation duration must be in [1,60] seconds')
        require(not apply or confirm_supported, 'Physical support confirmation is required')
        require(not stop_requested(), 'Stopped before preparation')
        guard(channel)
        idle_guard()
        initial = reader(channel)
        report['initial_feedback'] = validate_snapshot(initial, wall())
        if not apply:
            report['passive_only'] = True
            return report
        guard(channel)
        idle_guard()
        with recorder_factory(channel) as recorder:
            sdk = sdk_factory(channel)
            sdk.ConnectPort(piper_init=False, start_thread=False)
            initial = reader(channel)
            guard(channel)
            report['initial_feedback'] = validate_snapshot(initial, wall())
            command('MasterSlaveConfig', linkage_config=0xFA,
                    feedback_offset=0, ctrl_offset=0, linkage_offset=0)
            emit({'record_type': 'role_requested', 'role': 'FA', 'channel': channel,
                  'message': 'FA requested on LEFT follower hardware. Keep physical support.'})
            recorder.observe(2, guard=lambda: guard(channel), stop_requested=stop_requested)
            command('EnableArm', 7)
            recorder.observe(2, guard=lambda: guard(channel), stop_requested=stop_requested)
            command('MotionCtrl_1', grag_teach_ctrl=1)
            emit({'record_type': 'teaching_profile_requested', 'channel': channel,
                  'message': 'FA + EnableArm(7) + drag-teach=1 requested. This is the full master profile, not FC teaching.'})
            recorder.observe(duration, guard=lambda: guard(channel), stop_requested=stop_requested)
            report['profile_sequence_completed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = repr(exc)
    finally:
        if recorder is not None:
            report.update(recorder.summary())
        if sdk is not None:
            try:
                sdk.DisconnectPort()
            except Exception as exc:
                report['disconnect_error'] = repr(exc)
        report['FA_may_still_be_active'] = any(r['method'] == 'MasterSlaveConfig' for r in report['commands'])
        report['timestamp_sec'] = wall()
        emit(report)
    return report


def restore_follower(channel, *, confirm_supported=False, reader=sample_feedback,
                     sdk_factory=make_sdk, guard=check_environment, idle_guard=check_idle_can,
                     prepare=prepare_follower, clock=time.monotonic, pause=time.sleep,
                     wall=time.time, stop_requested=lambda: False, emit=lambda row: None):
    """Recover real disabled feedback before existing measured-pose preparation."""
    report = {'record_type': 'restore_summary', 'channel': channel, 'passed': False,
              'bootstrap_commands': [], 'home_reset_gripper_commands_sent': 0}
    sdk = None
    def command(method, *args, **kwargs):
        guard(channel)
        require(not stop_requested(), 'Stop requested before restore command')
        report['bootstrap_commands'].append({'method': method, 'args': list(args), 'kwargs': kwargs})
        getattr(sdk, method)(*args, **kwargs)
    try:
        require(channel == CHANNEL, 'Restore is restricted to the mapped LEFT follower')
        require(confirm_supported, 'Physical support is required throughout restoration')
        guard(channel)
        idle_guard()
        require(not stop_requested(), 'Stopped before restoration')
        sdk = sdk_factory(channel)
        sdk.ConnectPort(piper_init=False, start_thread=False)
        for _ in range(3):
            command('DisableArm', 7)
            pause(0.03)
        phase = wall()
        command('MasterSlaveConfig', linkage_config=0xFC,
                feedback_offset=0, ctrl_offset=0, linkage_offset=0)
        deadline = clock() + 3
        while clock() < deadline:
            # FC can re-enable internally. Disable immediately, not after a wait.
            command('DisableArm', 7)
            try:
                row = reader(channel, since_timestamp=phase)
            except RuntimeError as exc:
                if not str(exc).startswith('Missing required CAN feedback frames'):
                    raise
                pause(0.03)
                continue
            enabled = row.get('driver_enabled', [])
            require(len(enabled) == 6 and all(type(value) is bool for value in enabled),
                    'Missing actual six-driver acknowledgment')
            # Reuse all freshness/fault checks while permitting disabled drivers.
            require(row.get('ctrl_mode') in (0, 1, 2, 6), 'Unexpected post-FC control mode')
            validation = dict(row, driver_enabled=[True] * 6, ctrl_mode=1)
            validate_snapshot(validation, wall())
            require(row['timestamp_sec'] >= phase, 'Feedback predates FC role change')
            if not any(enabled) and all(not (value & 0x40) for value in row['driver_foc_bytes']):
                report['disabled_feedback'] = row
                break
        else:
            raise RuntimeError('No fresh all-six-disabled acknowledgment after FC; no prepare/enable requested')
        sdk.DisconnectPort()
        sdk = None
        guard(channel)
        idle_guard()
        require(not stop_requested(), 'Stopped before measured-pose preparation')
        def checked_factory(selected):
            require(selected == channel, 'Preparation attempted a different channel')
            real = sdk_factory(selected)
            class CheckedSDK:
                def __getattr__(self, name):
                    method = getattr(real, name)
                    if name == 'DisconnectPort':
                        return method
                    def checked(*args, **kwargs):
                        guard(channel)
                        require(not stop_requested(), 'Stop requested during measured-pose preparation')
                        return method(*args, **kwargs)
                    return checked
            return CheckedSDK()
        result = prepare(channel, apply=True, reader=reader, sdk_factory=checked_factory)
        report['measured_pose_preparation'] = result
        require(not stop_requested(), 'Stopped during measured-pose preparation')
        require(result.get('passed'), 'Measured-pose preparation did not pass')
        report['passed'] = True
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = repr(exc)
    finally:
        if sdk is not None:
            try:
                sdk.DisconnectPort()
            except Exception as exc:
                report['disconnect_error'] = repr(exc)
        emit(report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--confirm-supported', action='store_true')
    parser.add_argument('--restore-follower', action='store_true')
    parser.add_argument('--duration', type=float, default=15)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if (args.apply or args.restore_follower) and not args.confirm_supported:
        parser.error('Explicit --confirm-supported is required')
    if args.apply and args.restore_follower:
        parser.error('Select apply OR restore, not both')
    stopped = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, lambda _s, _f: stopped.set())
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    path = args.output or ROOT / '_runtime' / ('left-follower-master-profile-' + stamp + '.jsonl')
    path.parent.mkdir(parents=True, exist_ok=True)
    with (ROOT / '_runtime/keyboard-dagger.lock').open('a+') as lock, path.open('x') as output:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        def emit(row):
            line = json.dumps(row, separators=(',', ':'), allow_nan=False)
            print(line, flush=True)
            output.write(line + '\n')
            output.flush()
        if args.restore_follower:
            report = restore_follower(CHANNEL, confirm_supported=args.confirm_supported,
                                      stop_requested=stopped.is_set, emit=emit)
        else:
            report = run_master_profile(CHANNEL, apply=args.apply, confirm_supported=args.confirm_supported,
                                        duration=args.duration, stop_requested=stopped.is_set, emit=emit)
    print('Report: ' + str(path), flush=True)
    return 1 if report.get('error') or report.get('disconnect_error') else 0


if __name__ == '__main__':
    raise SystemExit(main())
