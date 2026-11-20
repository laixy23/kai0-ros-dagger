"""Read-only prerequisites before requesting master drag teaching.

This preflight is separate from the driver's release interlock. It makes no CAN
writes, does not construct an SDK, and cannot by itself authorize later release.
"""
from concurrent.futures import ThreadPoolExecutor
import time

from prepare_follower_enabled import checked_enabled_feedback
from prepare_follower_mode import sample_feedback


CHANNELS = {'left': 'can_left_slave', 'right': 'can_right_slave'}


def require_follower_connections(topics, connections, subscriptions=()):
    """Check transport presence; the later command probe proves execution.

    ROS node names do not identify a physical controller. Keep subscriptions as
    a compatibility argument, without using a naming convention as a gate.
    """
    if set(topics) != set(CHANNELS) or set(connections) != set(CHANNELS):
        raise RuntimeError('Both follower command connections must be checked')
    for side in CHANNELS:
        count = connections[side]
        if type(count) is not int or count < 1:
            raise RuntimeError(side + ' follower command publisher has no connection')


def check_followers_ready(topics, connections, subscriptions=(), *,
                          reader=sample_feedback, wall=time.time):
    """Check transport and fresh enabled CAN1 before the actual command probe."""
    require_follower_connections(topics, connections, subscriptions)
    since = wall()
    with ThreadPoolExecutor(max_workers=2) as pool:
        pending = {side: pool.submit(reader, channel, since_timestamp=since)
                   for side, channel in CHANNELS.items()}
        rows = {side: future.result() for side, future in pending.items()}
    now = wall()
    for side, row in rows.items():
        checked_enabled_feedback(row, now=now, since_timestamp=since)
        if row['ctrl_mode'] != 1:
            raise RuntimeError(side + ' follower is not in normal CAN position control')
    return {'passed': True, 'channels': rows, 'checked_at': now,
            'hardware_control_writes': 0, 'sdk_instances_created': 0}
