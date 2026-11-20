"""Pure validation for follower heartbeats and master teaching release gates.

Twist protocol: linear.x is readiness, angular.x is the source Unix timestamp,
and linear.y is the source JointState.header timestamp of the last command whose
SDK writes all completed. Completion time cannot acknowledge a pre-probe command.
Receipt monotonic time is recorded independently by every subscriber.
"""
from __future__ import annotations

import math

from prepare_follower_enabled import checked_enabled_feedback


MAX_HEARTBEAT_AGE_S = 0.35
MAX_COMMAND_AGE_S = 10.0
MAX_LOOP_AGE_S = 0.1
MAX_FUTURE_S = 0.05
SIDES = ("left", "right")


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def follower_readiness(raw, *, now_wall, now_monotonic, loop_monotonic,
                       enabled_flag, subscriber_ready, command_time,
                       previous_raw_stamps=None):
    """Combine raw CAN health with a live execution loop and installed callback."""
    result = {"ready": False, "source_time": now_wall,
              "command_time": command_time if finite(command_time) else 0.0,
              "reason": None}
    try:
        if not finite(now_wall) or not finite(now_monotonic):
            raise RuntimeError("Invalid heartbeat clocks")
        if raw is None:
            raise RuntimeError("No fresh raw follower feedback")
        _, stamps = checked_enabled_feedback(raw, now=now_wall, previous=previous_raw_stamps)
        result["raw_stamps"] = stamps
        if raw["ctrl_mode"] != 1:
            raise RuntimeError("Follower is not in CAN joint-control mode")
        if enabled_flag is not True or subscriber_ready is not True:
            raise RuntimeError("Follower control callback is not enabled/installed")
        if (not finite(command_time) or command_time <= 0
                or not -MAX_FUTURE_S <= now_wall - command_time <= MAX_COMMAND_AGE_S):
            raise RuntimeError("No recent successfully executed source command timestamp")
        if (not finite(loop_monotonic)
                or not 0 <= now_monotonic - loop_monotonic < MAX_LOOP_AGE_S):
            raise RuntimeError("Follower publishing loop stopped or is stale")
        result["ready"] = True
    except (RuntimeError, TypeError, ValueError, KeyError) as exc:
        result["reason"] = str(exc)
    return result


def require_pair_ready(rows, *, now_wall, now_monotonic, minimum_command_time=0.0):
    """Require fresh independently received heartbeats from both physical followers."""
    if not finite(now_wall) or not finite(now_monotonic) or not finite(minimum_command_time):
        raise RuntimeError("Invalid teaching readiness clocks")
    for side in SIDES:
        row = rows.get(side)
        if row is None or row.get("ready") is not True:
            raise RuntimeError(side + " follower readiness is missing/false")
        source = row.get("source_time")
        receipt = row.get("received_monotonic")
        command = row.get("command_time")
        if (not finite(source) or source <= 0
                or not -MAX_FUTURE_S <= now_wall - source < MAX_HEARTBEAT_AGE_S):
            raise RuntimeError(side + " follower source heartbeat is stale/future")
        if (not finite(receipt)
                or not -MAX_FUTURE_S <= now_monotonic - receipt < MAX_HEARTBEAT_AGE_S):
            raise RuntimeError(side + " follower heartbeat receipt is stale/future")
        if (not finite(command) or command <= 0 or command < minimum_command_time
                or not -MAX_FUTURE_S <= now_wall - command <= MAX_COMMAND_AGE_S):
            raise RuntimeError(side + " follower joint-command acknowledgement is missing/stale/pre-probe")
    return True
