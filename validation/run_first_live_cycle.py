#!/usr/bin/env python3
"""Bound one real policy trial, allow manual DAgger recording, then bound resume.

This supervisor only subscribes to ROS. Motion is performed by the official
collector child. Run explicitly in the prepared ROS environment on the robot.
"""
import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import select
import sys
import termios
import threading
import time
import tty

from validate_ros_dagger import CollectorTerminal, stop_process


INITIAL_POSE = (0.0, 0.32, -0.36, 0.0, 0.24, 0.0, 0.07)
RESUME_MARKER = 'DAGGER MODE DEACTIVATED - RESUMING INFERENCE'


class LiveCycleSupervisor:
    """Pure event/timer decisions; no ROS, subprocess, or actuator operations."""
    def __init__(self, started_at):
        self.lock = threading.Lock()
        self.started_at = started_at
        self.first_policy = None
        self.takeover_at = None
        self.takeover_reason = None
        self.dagger_active_at = None
        self.resume_requested_at = None
        self.resume_log_at = None
        self.resumed_policy = None
        self.stop_reason = None
        self.stop_at = None

    def command(self, source, positions, now, topic='', ros_stamp=None):
        if not source or len(positions) != 7 or not all(math.isfinite(value) for value in positions):
            return
        with self.lock:
            event = dict(callerid=source, monotonic_time=now, ros_stamp=ros_stamp, topic=topic)
            if self.first_policy is None and self.takeover_at is None:
                delta = max(abs(a - b) for a, b in zip(positions, INITIAL_POSE))
                if delta > 1e-4:
                    self.first_policy = dict(event, max_delta_from_initial=delta)
            if (self.resume_requested_at is not None and self.resume_log_at is not None
                    and self.first_policy is not None and self.resumed_policy is None
                    and source == self.first_policy['callerid'] and now >= self.resume_log_at):
                self.resumed_policy = event

    def output(self, text, now):
        with self.lock:
            if 'DAGGER MODE ACTIVE' in text and self.dagger_active_at is None:
                self.dagger_active_at = now
            if self.resume_requested_at is not None and RESUME_MARKER in text and self.resume_log_at is None:
                self.resume_log_at = now

    def forwarded_key(self, key, now):
        with self.lock:
            if key.lower() == 'r' and self.resume_requested_at is None:
                self.resume_requested_at = now

    def tick(self, now):
        with self.lock:
            if self.stop_reason is not None:
                return None
            if self.takeover_at is None:
                policy_due = self.first_policy is not None and now - self.first_policy['monotonic_time'] >= 2.0
                fallback_due = self.first_policy is None and now - self.started_at >= 15.0
                if policy_due or fallback_due:
                    self.takeover_at = now
                    self.takeover_reason = 'two_second_policy_trial' if policy_due else 'no_policy_startup_fallback'
                    return 'd'
            if self.takeover_at is not None and self.dagger_active_at is None and now - self.takeover_at >= 45:
                self.stop_reason = 'takeover_ack_timeout'
            elif self.resumed_policy is not None and now - self.resumed_policy['monotonic_time'] >= 1.0:
                self.stop_reason = 'completed_one_second_resume'
            elif self.resume_requested_at is not None and self.resumed_policy is None and now - self.resume_requested_at >= 40:
                self.stop_reason = 'resume_policy_timeout'
            if self.stop_reason is not None:
                self.stop_at = now
                return 'stop'
            return None

    def snapshot(self):
        with self.lock:
            return dict(first_policy_seen=self.first_policy is not None, first_policy=self.first_policy,
                        resume_policy_seen=self.resumed_policy is not None, resumed_policy=self.resumed_policy,
                        started_at=self.started_at, takeover_at=self.takeover_at, takeover_reason=self.takeover_reason,
                        dagger_active_at=self.dagger_active_at, resume_requested_at=self.resume_requested_at,
                        resume_log_at=self.resume_log_at, stop_at=self.stop_at, stop_reason=self.stop_reason)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, required=True)
    parser.add_argument('--collector', type=Path)
    parser.add_argument('--model-port', type=int, default=8014)
    args = parser.parse_args()
    root = args.root.resolve(strict=True)
    collector = (args.collector or root / 'collector' / 'agilex_openpi_dagger_collect.py').resolve(strict=True)
    token = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    dataset_name = 'cloth_first_live_cycle_' + token
    data_dir = root / 'data'
    run_dir = root / 'logs' / ('first-live-cycle-' + token)
    run_dir.mkdir(parents=True)
    data_dir.mkdir(parents=True, exist_ok=True)
    env = os.environ.copy()
    env.update(ROS_MASTER_URI='http://127.0.0.1:11311', PYTHONUNBUFFERED='1')
    os.environ['ROS_MASTER_URI'] = env['ROS_MASTER_URI']
    import rospy
    from sensor_msgs.msg import JointState
    rospy.init_node('bounded_first_live_cycle', anonymous=True, disable_signals=True)
    state = LiveCycleSupervisor(time.monotonic())
    def command(message, topic):
        state.command(getattr(message, '_connection_header', {}).get('callerid', ''), message.position,
                      time.monotonic(), topic, message.header.stamp.to_sec())
    subscribers = [rospy.Subscriber(topic, JointState, command, callback_args=topic, queue_size=10, tcp_nodelay=True)
                   for topic in ('/master/joint_left', '/master/joint_right')]
    child_command = [sys.executable, '-u', str(collector), '--host', '127.0.0.1', '--port', str(args.model_port),
                     '--ctrl_type', 'joint', '--use_temporal_smoothing', '--chunk_size', '50', '--publish_rate', '30',
                     '--inference_rate', '3', '--dataset_dir', str(data_dir), '--dataset_name', dataset_name]
    report = dict(validation_only=False, hardware_io=True, training_allowed=True, dataset_name=dataset_name,
                  dataset_dir=str(data_dir), run_dir=str(run_dir), command=child_command,
                  ros_master_uri=env['ROS_MASTER_URI'], started_utc=datetime.now(timezone.utc).isoformat(), passed=False)
    terminal = None
    old_terminal = None
    done = threading.Event()
    stop_lock = threading.Lock()
    watchdog = None
    try:
        terminal = CollectorTerminal(child_command, env, run_dir / 'collector.log')
        def stop_child():
            with stop_lock:
                stop_process(terminal.proc)
        def supervise():
            try:
                while not done.is_set():
                    with terminal.changed:
                        output = terminal.buffer
                    state.output(output, time.monotonic())
                    action = state.tick(time.monotonic())
                    if action == 'd':
                        terminal.key('d')
                    elif action == 'stop':
                        stop_child()
                        done.set()
                        break
                    if terminal.proc.poll() is not None:
                        report['child_exited_without_supervised_stop'] = True
                        done.set()
                        break
                    time.sleep(0.01)
            except Exception as exc:
                report['watchdog_error'] = repr(exc)
                stop_child()
                done.set()
        watchdog = threading.Thread(target=supervise, daemon=True)
        watchdog.start()
        printed = 0
        ready_printed = False
        while not done.is_set():
            with terminal.changed:
                output = terminal.buffer
            state_snapshot = state.snapshot()
            if state_snapshot['dagger_active_at'] is not None:
                if not ready_printed:
                    if sys.stdin.isatty():
                        old_terminal = termios.tcgetattr(sys.stdin)
                        tty.setcbreak(sys.stdin.fileno())
                    print('\nLIVE CYCLE READY: Space=record, s=save, y=confirm, r=resume for1s then stop.\n', flush=True)
                    ready_printed = True
                if select.select([sys.stdin], [], [], 0.01)[0]:
                    key = os.read(sys.stdin.fileno(), 1)
                    if not key:
                        raise RuntimeError('Operator input closed')
                    char = key.decode(errors='ignore')
                    state.forwarded_key(char, time.monotonic())
                    terminal.key(char)
            if len(output) > printed:
                sys.stdout.write(output[printed:])
                sys.stdout.flush()
                printed = len(output)
            time.sleep(0.01)
        report['passed'] = state.snapshot()['stop_reason'] == 'completed_one_second_resume'
    except KeyboardInterrupt:
        report['operator_interrupted'] = True
    except Exception as exc:
        report['error'] = repr(exc)
    finally:
        done.set()
        if terminal is not None:
            stop_child()
            report['collector_exit_code'] = terminal.proc.poll()
        if watchdog is not None:
            watchdog.join(timeout=1)
        if terminal is not None:
            terminal.close()
        if old_terminal is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_terminal)
        for subscriber in subscribers:
            subscriber.unregister()
        report.update(state.snapshot())
        report['ended_utc'] = datetime.now(timezone.utc).isoformat()
        (run_dir / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        print(f"\nLive cycle report: {run_dir / 'report.json'} (passed={report['passed']})", flush=True)
        print('Collector stopped; motors were not automatically disabled.', flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
