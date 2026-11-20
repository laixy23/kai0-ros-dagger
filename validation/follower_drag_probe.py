#!/usr/bin/env python3
"""Isolated follower test of the same drag-teach command used by the masters.

Default is passive. Apply requires an already enabled, fault-free, physically
supported follower. Only MotionCtrl_1(grag_teach_ctrl=1) is sent. FA/FC role,
offsets, load parameters, enables, targets, gripper and reset are not changed.
Stopping the observer leaves the teaching state in place: keep physical support.
No conclusion about gravity compensation is inferred from position alone.
"""
from __future__ import annotations

import argparse
import datetime
import fcntl
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'setup'))
from prepare_follower_mode import make_sdk, sample_feedback

SERIALS = {'can_left_slave': '0027002C5547571120343930',
           'can_right_slave': '004B002C4648570220393831'}
FEEDBACK_IDS = ('0x2A1', '0x2A5', '0x2A6', '0x2A7', '0x2A8') + tuple(
    f'0x{key:03X}' for key in range(0x261, 0x267))
CONTROLLERS = {'piper_start_ms_node_new.py', 'agilex_openpi_dagger_collect.py',
               'run_keyboard_session.py', 'prepare_keyboard_session.py',
               'run_dagger.sh', 'arms_official_wj_dev.launch', 'start_ms_piper_new.launch',
               'run_right_manual_follow.py', 'run_first_live_cycle.py',
               'recover_master_mode.py', 'prepare_follower_mode.py',
               'follower_drag_probe.py'}


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def check_environment(channel):
    require(channel in SERIALS, 'Only explicitly mapped follower channels are allowed')
    device = (Path('/sys/class/net') / channel / 'device').resolve()
    serial = next(((p / 'serial').read_text().strip()
                   for p in (device, *device.parents) if (p / 'serial').is_file()), None)
    require(serial == SERIALS[channel], 'Follower USB serial mismatch or missing adapter')
    link = json.loads(subprocess.check_output(['ip', '-j', '-d', 'link', 'show', channel], text=True))[0]
    info = link['linkinfo']['info_data']
    require('UP' in link['flags'] and info.get('state') == 'ERROR-ACTIVE'
            and info.get('bittiming', {}).get('bitrate') == 1_000_000,
            'Follower CAN must already be UP, error-active, at 1 Mbps')
    for directory in Path('/proc').iterdir():
        if not directory.name.isdecimal() or int(directory.name) == os.getpid():
            continue
        try:
            argv = [p.decode(errors='replace') for p in
                    (directory / 'cmdline').read_bytes().split(b'\0') if p]
        except FileNotFoundError:
            continue
        require(not any(Path(arg).name in CONTROLLERS for arg in argv),
                'Another arm controller exists, including stopped processes: PID ' + directory.name)


def check_idle_can():
    match = re.search(r'^\s*(\d+)\s+current receive list entries',
                      Path('/proc/net/can/stats').read_text(), re.M)
    require(match is not None and int(match.group(1)) == 0,
            'Other CAN receivers are active; refusing a second owner')


def validate_snapshot(row, now, *, initial=False):
    stamp = row.get('timestamp_sec')
    require(isinstance(stamp, (int, float)) and math.isfinite(stamp), 'Invalid feedback timestamp')
    lag = now - stamp
    require(-0.05 <= lag <= 0.2, 'Stale feedback snapshot')
    ages = row.get('component_age_s', {})
    for key in FEEDBACK_IDS:
        age = ages.get(key)
        require(isinstance(age, (int, float)) and math.isfinite(age)
                and -0.05 <= age + lag <= 0.2, 'Missing/stale actual feedback: ' + key)
    require(row.get('arm_status') == 0 and row.get('err_code') == 0
            and row.get('gripper_mode') == 0, 'Arm fault or unexpected gripper units')
    foc = row.get('driver_foc_bytes', [])
    require(len(foc) == 6 and all(type(v) is int and not v & 0xBF for v in foc),
            'Motor driver fault or missing actual driver state')
    enabled = row.get('driver_enabled', [])
    require(len(enabled) == 6 and all(v is True for v in enabled),
            'All six actual drivers must remain enabled; test sends no enable commands')
    joints = row.get('joint_mdeg', [])
    require(len(joints) == 6 and all(isinstance(v, (int, float)) and not isinstance(v, bool)
                                   and math.isfinite(v) for v in joints), 'Invalid actual joints')
    require(row.get('ctrl_mode') in ((1,) if initial else (1, 2)),
            'Initial CAN-position mode required' if initial else 'Unexpected control mode')
    return row


def run_probe(channel, *, apply=False, confirm_supported=False, duration=20,
              reassert_existing_teach=False,
              reader=sample_feedback, sdk_factory=make_sdk,
              guard=check_environment, idle_guard=check_idle_can,
              clock=time.monotonic, wall=time.time, pause=time.sleep,
              stop_requested=lambda: False, emit=lambda row: None):
    report = {'record_type': 'summary', 'channel': channel, 'application_commands_sent': 0,
              'command_acknowledged': False, 'gravity_compensation_verified': False,
              'position_targets_sent': 0, 'enable_disable_commands_sent': 0,
              'role_load_gripper_reset_commands_sent': 0, 'samples': 0,
              'automatic_restore': False, 'commands': []}
    sdk = None
    latest = None
    peak = [0.0] * 6
    try:
        require(channel in SERIALS, 'Only explicitly mapped follower channels are allowed')
        require(isinstance(duration, (int, float)) and math.isfinite(duration)
                and 1 <= duration <= 60, 'Duration must be in [1,60] seconds')
        require(not apply or confirm_supported, 'Apply requires confirmed physical support')
        require(not reassert_existing_teach or apply,
                'Reasserting an existing teaching state requires apply')
        require(not stop_requested(), 'Stopped before preparation')
        guard(channel)
        idle_guard()
        initial = reader(channel)
        report['initial_feedback'] = initial
        validate_snapshot(initial, wall(), initial=apply and not reassert_existing_teach)
        report['initial_already_teaching'] = initial['ctrl_mode'] == 2
        if not apply:
            report['passive_only'] = True
            return report
        guard(channel)
        idle_guard()
        require(not stop_requested(), 'Stopped before SDK connection')
        sdk = sdk_factory(channel)
        sdk.ConnectPort(piper_init=False, start_thread=False)
        initial = reader(channel)
        report['initial_feedback'] = initial
        guard(channel)
        validate_snapshot(initial, wall(), initial=not reassert_existing_teach)
        require(not stop_requested(), 'Stopped before teaching command')
        report['application_commands_sent'] = 1
        report['commands'].append({'method': 'MotionCtrl_1', 'grag_teach_ctrl': 1,
                                   'timestamp_sec': wall()})
        sdk.MotionCtrl_1(grag_teach_ctrl=1)
        started = clock()
        emit({'record_type': 'command_sent', 'channel': channel,
              'message': 'Drag-teach command sent. Keep physical support; no gravity conclusion yet.'})
        while clock() - started < duration and not stop_requested():
            guard(channel)
            latest = reader(channel)
            report['final_feedback'] = latest
            validate_snapshot(latest, wall())
            if latest['ctrl_mode'] == 2 and not report['command_acknowledged']:
                report['command_acknowledged'] = True
                emit({'record_type': 'teaching_ready', 'channel': channel,
                      'message': 'Fresh actual mode=2 and six enabled drivers. Keep support; observer does not command motion.'})
            require(report['command_acknowledged'] or clock() - started < 3,
                    'No actual teaching-mode acknowledgment within 3 seconds')
            require(not report['command_acknowledged'] or latest['ctrl_mode'] == 2,
                    'Teaching mode changed unexpectedly')
            delta = [(a - b) / 1000 for a, b in zip(latest['joint_mdeg'], initial['joint_mdeg'])]
            peak = [max(a, abs(b)) for a, b in zip(peak, delta)]
            report['samples'] += 1
            emit({'record_type': 'sample', 'timestamp_sec': wall(), 'feedback': latest,
                  'joint_change_deg': delta})
            pause(0.1)
        require(report['command_acknowledged'] or stop_requested(),
                'No actual teaching-mode acknowledgment before observation ended')
        report['stop_reason'] = 'operator_stop' if stop_requested() else 'observation_duration'
        report['observation_completed'] = report['command_acknowledged']
    except (Exception, KeyboardInterrupt) as exc:
        report['error'] = repr(exc)
    finally:
        if sdk is not None:
            try:
                sdk.DisconnectPort()
            except Exception as exc:
                report['disconnect_error'] = repr(exc)
        report['max_joint_change_deg'] = peak
        report['timestamp_sec'] = wall()
        report['teaching_may_still_be_active'] = (report['application_commands_sent'] > 0
                                                 or report.get('initial_already_teaching', False))
        emit(report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--channel', required=True, choices=tuple(SERIALS))
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--confirm-supported', action='store_true')
    parser.add_argument('--reassert-existing-teach', action='store_true',
                        help='Explicitly resend the same teach command when actual mode is already 2')
    parser.add_argument('--duration', type=float, default=20)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.apply and not args.confirm_supported:
        parser.error('--apply requires --confirm-supported')
    stopped = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, lambda _s, _f: stopped.set())
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    output_path = args.output or ROOT / '_runtime' / ('follower-drag-' + stamp + '.jsonl')
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lock_path = ROOT / '_runtime/keyboard-dagger.lock'
    with lock_path.open('a+') as lock, output_path.open('x') as output:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        console_at = 0.0
        def emit(row):
            nonlocal console_at
            line = json.dumps(row, allow_nan=False, separators=(',', ':'))
            output.write(line + '\n')
            output.flush()
            if row['record_type'] != 'sample' or time.monotonic() >= console_at:
                print(line, flush=True)
                console_at = time.monotonic() + 1
        report = run_probe(args.channel, apply=args.apply, confirm_supported=args.confirm_supported,
                           duration=args.duration, reassert_existing_teach=args.reassert_existing_teach,
                           stop_requested=stopped.is_set, emit=emit)
    print('Report: ' + str(output_path), flush=True)
    return 1 if report.get('error') or report.get('disconnect_error') else 0


if __name__ == '__main__':
    raise SystemExit(main())
