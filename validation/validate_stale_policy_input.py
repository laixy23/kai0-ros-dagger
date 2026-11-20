#!/usr/bin/env python3
"""Synthetic camera-loss failure test of the actual collector and model.

Uses only a dedicated loopback ROS master and fake messages; never launches any
hardware driver or opens CAN/cameras. Run separately from validate_ros_dagger.py.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import subprocess
import sys
import time
from types import SimpleNamespace
import xmlrpc.client

from validate_ros_dagger import (
    CollectorTerminal, FakeRosInputs, ModelRelay, MASTER_PORT, PROXY_PORT,
    require_unused_port, stop_process, wait_until,
)


HERE = Path(__file__).resolve().parent


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--collector', type=Path,
                        default=HERE.parent / 'collector-source' / 'agilex_openpi_dagger_collect.py')
    parser.add_argument('--model-port', type=int, default=8014)
    args = parser.parse_args()
    collector = args.collector.resolve(strict=True)
    for port in (MASTER_PORT, PROXY_PORT):
        require_unused_port(port)
    run_dir = HERE / 'runs' / ('stale-' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ'))
    run_dir.mkdir(parents=True)
    (run_dir / 'DO_NOT_TRAIN.txt').write_text('SYNTHETIC FAILURE VALIDATION ONLY. NO TRAINING DATA.\n')
    env = os.environ.copy()
    overrides = dict(ROS_MASTER_URI=f'http://127.0.0.1:{MASTER_PORT}', ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1',
                     ROS_HOME=str(run_dir / 'ros-home'), ROS_LOG_DIR=str(run_dir / 'ros-logs'), PYTHONUNBUFFERED='1')
    env.update(overrides)
    os.environ.update(overrides)
    report = dict(hardware_io=False, fake_ros_inputs=True, validation_only=True, training_allowed=False,
                  collector=str(collector), ros_master_uri=env['ROS_MASTER_URI'], passed=False)
    master = terminal = fixture = relay = None
    master_log = (run_dir / 'roscore.log').open('w')
    try:
        master = subprocess.Popen(['roscore', '-p', str(MASTER_PORT)], env=env, stdout=master_log,
                                  stderr=subprocess.STDOUT, start_new_session=True)
        def master_ready():
            if master.poll() is not None:
                raise RuntimeError('Dedicated roscore exited')
            try:
                return xmlrpc.client.ServerProxy(env['ROS_MASTER_URI']).getPid('stale_validation')[0] == 1
            except OSError:
                return False
        wait_until(master_ready, 'Dedicated roscore did not start', timeout=20)
        fixture = FakeRosInputs()
        relay = ModelRelay(args.model_port, fixture)
        command = [sys.executable, '-u', str(collector), '--host', '127.0.0.1', '--port', str(PROXY_PORT),
                   '--ctrl_type', 'joint', '--chunk_size', '50', '--publish_rate', '30', '--inference_rate', '3',
                   '--frame_rate', '30', '--dataset_dir', str(run_dir / 'synthetic_datasets'),
                   '--dataset_name', 'VALIDATION_ONLY_DO_NOT_TRAIN', '--max_timesteps', '100000', '--use_temporal_smoothing']
        report['command'] = command
        terminal = CollectorTerminal(command, env, run_dir / 'collector.log')
        terminal.expect('Keyboard monitor thread started')
        def policy_events(after=0):
            return [event for event in fixture.command_sources if event['topic'].startswith('/master/joint_')
                    and event['source'] != fixture.rospy.get_name() and event['received_time'] >= after]
        wait_until(lambda: len(relay.records) >= 2 and policy_events(relay.records[0]['request_time']),
                   'Actual model replies/policy commands did not start', timeout=45)
        mark = terminal.mark()
        stopped_camera = fixture.image_pubs[2]  # Retain advertised publisher; suppress only its messages.
        fixture.image_pubs[2] = SimpleNamespace(publish=lambda message: None)
        frozen_at = time.monotonic()
        frozen_sequence = fixture.sequence
        exit_code = terminal.proc.wait(timeout=2)
        exited_at = time.monotonic()
        expected = 'RuntimeError: Stale/future policy input: camera_right'
        wait_until(lambda: expected in terminal.buffer[mark:], 'Missing specific camera freshness RuntimeError', timeout=1)
        assert exit_code != 0, 'Collector did not fail closed with a nonzero exit'
        time.sleep(0.3)
        assert fixture.sequence > frozen_sequence, 'Other fake sensors stopped alongside the camera'
        assert not policy_events(exited_at + 0.1), 'Policy commands continued after collector exit'
        report.update(passed=True, stopped_camera='/camera_r/color/image_raw', expected_error=expected,
                      collector_exit_code=exit_code, camera_stop_to_exit_seconds=exited_at - frozen_at,
                      fixture_sequence_at_camera_stop=frozen_sequence, fixture_sequence_after_exit=fixture.sequence,
                      model_replies_before_camera_stop=len([record for record in relay.records if record['request_time'] < frozen_at]),
                      policy_commands_after_exit=0)
        del stopped_camera
    except Exception as exc:
        report['error'] = repr(exc)
        raise
    finally:
        if terminal is not None:
            terminal.close()
        if fixture is not None:
            fixture.timer.shutdown()
            fixture.rospy.signal_shutdown('Synthetic freshness validation complete')
            report['command_counts'] = dict(fixture.commands)
        if relay is not None:
            relay.server.shutdown()
            report['model_replies'] = relay.records
        stop_process(master)
        (run_dir / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        master_log.close()
        print(f"Freshness report: {run_dir / 'report.json'} (passed={report['passed']})", flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
