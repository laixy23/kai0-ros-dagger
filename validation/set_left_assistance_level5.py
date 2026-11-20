"""Requested level-5 experiment: one-level steps using the validated transport."""
import datetime
import fcntl
import json
import os
import signal
import threading
import time

from joint_assistance_adjust import CHANNEL, ROOT, Transport, adjust
from follower_drag_probe import check_environment, check_idle_can, require


def main():
    import can
    stopped = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, lambda _s, _f: stopped.set())
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%S.%fZ')
    path = ROOT/'_runtime'/('left-follower-assistance-level5-'+stamp+'.jsonl')
    with (ROOT/'_runtime/keyboard-dagger.lock').open('a+') as lock, path.open('x') as output:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        def emit(row):
            line = json.dumps(row, separators=(',', ':'), allow_nan=False)
            output.write(line+'\n')
            output.flush()
            os.fsync(output.fileno())
            print(line, flush=True)
        check_environment(CHANNEL)
        check_idle_can()
        filters = [{'can_id': ident, 'can_mask': 0x7ff, 'extended': False} for ident in (0x476, 0x488)]
        with can.Bus(channel=CHANNEL, interface='socketcan', receive_own_messages=False,
                     can_filters=filters) as bus:
            transport = Transport(bus, can.Message, stop_requested=stopped.is_set)
            try:
                before = transport.query()
                require(transport.query() == before, 'Initial assistance readings differ')
                emit({'record_type': 'batch_baseline', 'channel': CHANNEL,
                      'original_levels': before, 'requested_levels': [5]*6})
                require(before in ([1]*6, [5]*6),
                        'Current settings differ from the last verified level 1; no adjustment started')
                current = before
                for level in range(2, 6) if current == [1]*6 else ():
                    require(not stopped.is_set(), 'Stopped before next adjustment')
                    result = adjust(transport, expected=current, target_override=[level]*6,
                                    apply=True, confirm_supported=True,
                                    guard=lambda: check_environment(CHANNEL),
                                    stop_requested=stopped.is_set,
                                    emit=lambda row, level=level: emit(dict(row, step_level=level)))
                    require(result.get('verified'), 'Step not verified; no further settings sent')
                    current = result['readback']
                    stopped.wait(0.5)
                emit({'record_type': 'batch_complete', 'channel': CHANNEL,
                      'original_levels': before, 'final_levels': current,
                      'verified': current == [5]*6, 'parameter_queries': transport.query_count,
                      'configuration_writes_attempted': transport.configuration_writes,
                      'other_arm_or_mode_enable_target_commands_sent': 0})
            except (Exception, KeyboardInterrupt) as exc:
                emit({'record_type': 'batch_failed', 'error': repr(exc),
                      'last_readback': transport.last_readback,
                      'configuration_writes_attempted': transport.configuration_writes})
                return 1
    print('Report: '+str(path), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
