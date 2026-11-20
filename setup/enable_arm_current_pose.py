#!/usr/bin/env python3
"""Enable a mapped arm at its measured pose without removing torque or homing.

All-enabled CAN1 arms are inspected passively. A disabled or partially enabled
arm is seeded from fresh actual joints before and immediately after EnableArm.
Standby/teaching modes use the existing exit-teach and CAN1 preparation sequence.
There is no disable, reset, gripper command, zero pose, or angular drift gate.
"""
from __future__ import annotations

import math
import time

from prepare_follower_enabled import checked_enabled_feedback
from prepare_follower_mode import make_sdk, sample_feedback


CHANNELS = ('can_left_mas', 'can_right_mas', 'can_left_slave', 'can_right_slave')


def _all_enabled(row):
    return (all(value is True for value in row['driver_enabled'])
            and all(value & 0x40 for value in row['driver_foc_bytes']))


def enable_at_current_pose(channel, apply=False, *, reader=sample_feedback,
                           sdk_factory=make_sdk, clock=time.monotonic,
                           wall=time.time, pause=time.sleep, step_timeout=3.0,
                           stable_seconds=0.3):
    """Return a report after passive planning or current-pose enabling."""
    report = {'channel': channel, 'apply': apply, 'passed': False,
              'sdk_instances_created': 0, 'sdk_command_log': [],
              'torque_removal_requested': False, 'enable_requested': False,
              'reset_requested': False, 'gripper_commands_requested': 0,
              'initial_pose_requested': False, 'drift_is_diagnostic_only': True,
              'plan': ['read fresh fault-free actual joints and six enable bits',
                       'already enabled CAN1: no SDK or control command',
                       'seed the current measured pose before mode selection or enabling',
                       'standby/teach: exit teach, select CAN1 at speed 10 and reseed',
                       'disabled/partial: EnableArm(7), immediately reseed the same pose',
                       'require actual enabled CAN1 feedback using the existing stability bounds']}
    sdk = None
    previous = None
    anchor = None
    drifts = []

    def read(since=0.0):
        nonlocal previous
        row = reader(channel, since_timestamp=since)
        joints, previous = checked_enabled_feedback(
            row, now=wall(), since_timestamp=since, previous=previous,
            require_enabled=False)
        if anchor is not None:
            drifts.append(max(abs(actual - target) for actual, target in zip(joints, anchor)))
        return row, joints

    def command(name, *args, **kwargs):
        report['sdk_command_log'].append({'method': name, 'args': list(args),
                                          'kwargs': kwargs, 'timestamp_sec': wall()})
        return getattr(sdk, name)(*args, **kwargs)

    try:
        if channel not in CHANNELS:
            raise RuntimeError('Only the four mapped physical arm channels may use this helper')
        if (not math.isfinite(step_timeout) or step_timeout <= 0
                or not math.isfinite(stable_seconds) or stable_seconds < 0):
            raise ValueError('Invalid current-pose enable bounds')
        initial, initial_joints = read()
        report.update(initial_feedback=initial, holding_target_mdeg=initial_joints)
        if initial['ctrl_mode'] == 1 and _all_enabled(initial):
            report.update(passed=True, already_ready=True, final_feedback=initial,
                          observed_max_drift_mdeg=0)
            return report
        if not apply:
            report.update(passed=True, plan_only=True)
            return report

        sdk = sdk_factory(channel)
        report['sdk_instances_created'] = 1
        sdk.ConnectPort(piper_init=False)
        current, current_joints = read()
        anchor = list(current_joints)
        report.update(pre_write_feedback=current, holding_target_mdeg=anchor,
                      holding_target_refreshed_before_first_write=True)
        if current['ctrl_mode'] == 1 and _all_enabled(current):
            report.update(passed=True, already_ready=True, final_feedback=current,
                          observed_max_drift_mdeg=0)
            return report

        phase = wall()
        command('JointCtrl', *anchor)
        if current['ctrl_mode'] in (0, 2):
            command('MotionCtrl_1', grag_teach_ctrl=0x02)
            command('JointCtrl', *anchor)
            phase = wall()
            command('MotionCtrl_2', ctrl_mode=0x01, move_mode=0x01,
                    move_spd_rate_ctrl=10, is_mit_mode=0x00)
            command('JointCtrl', *anchor)
        if not _all_enabled(current):
            phase = wall()
            report['enable_requested'] = True
            command('EnableArm', 7)
            command('JointCtrl', *anchor)

        deadline = clock() + step_timeout
        stable_since = None
        while clock() < deadline:
            snapshot, _ = read(phase)
            now = clock()
            if now >= deadline:
                raise TimeoutError('Current-pose enabling exceeded its existing time limit')
            if snapshot['ctrl_mode'] == 1 and _all_enabled(snapshot):
                if stable_since is None:
                    stable_since = now
                report['stable_elapsed_s'] = now - stable_since
                if now - stable_since >= stable_seconds:
                    report.update(passed=True, final_feedback=snapshot,
                                  observed_max_drift_mdeg=max(drifts, default=0))
                    break
            else:
                stable_since = None
            command('JointCtrl', *anchor)
            pause(0.01)
        else:
            raise TimeoutError('Six raw motor enable bits and CAN1 were not confirmed; no torque-off fallback')
    except (Exception, KeyboardInterrupt) as exc:
        report.update(error=repr(exc), interrupted=isinstance(exc, KeyboardInterrupt),
                      observed_max_drift_mdeg=max(drifts, default=0))
    finally:
        report['hardware_control_writes'] = len(report['sdk_command_log'])
        if sdk is not None:
            try:
                sdk.DisconnectPort()
            except Exception as exc:
                report.update(passed=False, disconnect_error=repr(exc))
    return report
