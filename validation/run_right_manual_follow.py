#!/usr/bin/env python3
"""Right-only manual follow using actual, fresh master 2A feedback.

The master MUST already be in manual teach (mode 2), with all six drivers on.
This tool never creates a master SDK or changes its mode. Only the right slave
receives bounded position targets. No ROS, policy, gripper, reset or zero writes.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

from check_right_alignment import (MASTER, SLAVE, ROOT, active_controllers,
                                   fresh_at_write, make_sdk, require,
                                   sample_feedback, valid_snapshot)

RATE, STEP, INITIAL_GAP = 5000, 1000, 45000
LIMITS = [(-150000, 150000), (0, 180000), (-170000, 0),
          (-100000, 100000), (-70000, 70000), (-180000, 180000)]


def joints(values, limits, label):
    require(len(values) == len(limits) == 6, label + " needs six joints/limits")
    for i, (value, bounds) in enumerate(zip(values, limits)):
        require(len(bounds) == 2 and all(isinstance(x, (int, float)) and math.isfinite(x) for x in bounds)
                and bounds[0] < bounds[1], "Invalid queried joint limits")
        require(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                and bounds[0] <= value <= bounds[1],
                f"{label} J{i+1}={value} outside [{bounds[0]},{bounds[1]}] mdeg")
    return [int(round(x)) for x in values]


def follow(*, apply=False, feedback_master=False, confirm_supported=False, duration=90,
           reader=sample_feedback, sdk_factory=make_sdk, controller_reader=active_controllers,
           limits=None, clock=time.monotonic, wall=time.time, pause=time.sleep,
           stop_requested=lambda: False, emit=lambda row: None):
    limits = limits or {MASTER: LIMITS, SLAVE: LIMITS}
    report = {"record_type": "summary", "passed": False, "samples": 0,
              "pid": os.getpid(),
              "master_control_writes": 0, "gripper_writes": 0,
              "left_arm_writes": 0, "policy_execution": False,
              "enable_disable_reset_zero_writes": 0, "commands": [],
              "target_rate_limit_deg_s": 5, "target_step_limit_deg": 1,
              "master_source": "fresh_actual_2A_feedback"}
    sdk = None
    written = False
    latest = None

    def unowned():
        require(not controller_reader(), "Other arm/collector controller detected")

    def read_one(channel):
        row = valid_snapshot(reader(channel), require_can=channel == SLAVE)
        require(channel != MASTER or (row["ctrl_mode"] == 2 and all(row["driver_enabled"])),
                "Right master must already be in manual mode 2 with all six drivers enabled")
        joints(row["joint_mdeg"], limits[channel], channel + " actual")
        return row

    def read_pair():
        with ThreadPoolExecutor(max_workers=2) as pool:
            master, slave = list(pool.map(read_one, (MASTER, SLAVE)))
        fresh_at_write(master, wall()); fresh_at_write(slave, wall())
        return master, slave

    def send(target, actual, stage):
        nonlocal written
        unowned()
        if stage == "follow":
            require(not stop_requested(), "Stop requested before target write")
        target = joints(target, limits[SLAVE], "slave target")
        fresh_at_write(actual, wall())
        written = True
        sdk.MotionCtrl_2(ctrl_mode=1, move_mode=1, move_spd_rate_ctrl=10, is_mit_mode=0)
        if stage == "follow":
            require(not stop_requested(), "Stop requested before joint target")
        report["commands"].append({"channel": SLAVE, "method": "JointCtrl", "stage": stage,
                                   "target_mdeg": target, "timestamp_sec": wall(), "monotonic_sec": clock()})
        sdk.JointCtrl(*target)

    try:
        require(math.isfinite(duration) and 0 < duration <= 180, "Duration must be in (0,180] seconds")
        require(not apply or (feedback_master and confirm_supported),
                "Apply requires --feedback-master --confirm-supported")
        unowned()
        require(not stop_requested(), "Cancelled before preparation")
        master, slave = read_pair()
        report["initial_feedback"] = {MASTER: master, SLAVE: slave}
        require(max(abs(a-b) for a,b in zip(master["joint_mdeg"], slave["joint_mdeg"])) <= INITIAL_GAP,
                "Initial master/slave difference exceeds 45 degrees")
        if not apply:
            report.update(passed=True, passive_only=True)
            return report
        sdk = sdk_factory(SLAVE)  # Deliberately no master SDK or master writes.
        sdk.ConnectPort(piper_init=False)
        master, slave = read_pair()
        command = [int(x) for x in slave["joint_mdeg"]]
        require(max(abs(a-b) for a,b in zip(master["joint_mdeg"], command)) <= INITIAL_GAP,
                "Master moved beyond 45-degree initial cap while connecting")
        started = last_write = clock()
        soft_since = None
        aligned_since = None
        next_emit = started
        emit({"record_type": "ready", "pid": os.getpid(),
              "message": "Controller ready; bounded initial catch-up starts; wait for measured alignment before dragging", "timestamp_sec": wall(),
              "initial_master_minus_slave_deg": [(a-b)/1000 for a,b in zip(master["joint_mdeg"], command)]})
        while clock() - started < duration and not stop_requested():
            pause(0.02)
            unowned()
            master, slave = read_pair()
            desired, actual = master["joint_mdeg"], slave["joint_mdeg"]
            now = clock()
            tracking = max(abs(a-b) for a,b in zip(command, actual))
            require(tracking <= 5000, "Slave published-command tracking error exceeds 5 degrees")
            soft_since = now if tracking > 1000 and soft_since is None else soft_since if tracking > 1000 else None
            require(soft_since is None or now - soft_since <= 1,
                    "Slave published-command tracking error stayed above 1 degree for over 1 second")
            elapsed = now - last_write
            require(math.isfinite(elapsed) and elapsed > 0, "Invalid command clock")
            step = min(STEP, math.floor(RATE * elapsed))
            target = [int(old + max(-step, min(step, wanted-old))) for old,wanted in zip(command,desired)]
            require(max(abs(a-b) for a,b in zip(target, actual)) <= 5000,
                    "Next target would exceed measured 5-degree tracking cap")
            fresh_at_write(master, wall())
            send(target, slave, "follow")
            command, last_write = target, clock()
            latest = {"record_type": "sample", "timestamp_sec": wall(),
                      "master_actual_deg": [x/1000 for x in desired], "slave_actual_deg": [x/1000 for x in actual],
                      "master_minus_slave_deg": [(a-b)/1000 for a,b in zip(desired,actual)],
                      "command_tracking_deg": [(a-b)/1000 for a,b in zip(command,actual)]}
            aligned = max(abs(x) for x in latest["master_minus_slave_deg"]) <= 1
            aligned_since = now if aligned and aligned_since is None else aligned_since if aligned else None
            if aligned_since is not None and now - aligned_since >= 0.3 and not report.get("all_six_aligned"):
                report["all_six_aligned"] = True
                emit({"record_type": "all_six_aligned", "timestamp_sec": wall(), "errors_deg": latest["master_minus_slave_deg"]})
            report["samples"] += 1
            if now >= next_emit:
                emit(latest); next_emit = now + 0.5
        report.update(passed=report["samples"] > 0,
                      stop_reason="operator_stop" if stop_requested() else "duration_limit")
    except (Exception, KeyboardInterrupt) as exc:
        report.update(error=repr(exc), stop_reason="interrupted_or_rejected")
    finally:
        if written and sdk is not None:
            try:
                unowned()
                actual = read_one(SLAVE)
                send(actual["joint_mdeg"], actual, "exit_measured_hold")
                report["exit_slave_hold_requested"] = True
            except Exception as exc:
                report["exit_hold_error"] = repr(exc); report["passed"] = False
        if sdk is not None:
            try: sdk.DisconnectPort()
            except Exception as exc:
                report["disconnect_error"] = repr(exc); report["passed"] = False
        report["final_sample"] = latest
        emit(report)
    return report


def load_limits(path):
    data = json.loads(path.read_text())
    return {channel: [(round(data[channel]["limits"][str(j)]["min_deg"]*1000),
                       round(data[channel]["limits"][str(j)]["max_deg"]*1000)) for j in range(1,7)]
            for channel in (MASTER, SLAVE)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--feedback-master", action="store_true")
    parser.add_argument("--confirm-supported", action="store_true")
    parser.add_argument("--duration", type=float, default=90)
    parser.add_argument("--limits-file", type=Path, default=ROOT/"_runtime/right-joint-limits-query.json")
    parser.add_argument("--output", type=Path, default=ROOT/("_runtime/right-manual-follow-"+datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")+".jsonl"))
    args = parser.parse_args()
    if args.apply and not (args.feedback_master and args.confirm_supported):
        parser.error("--apply requires --feedback-master --confirm-supported")
    stopped = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, lambda s,f: stopped.set())
    lock_path = ROOT/"_runtime/keyboard-dagger.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    with lock_path.open("a+") as lock:
        try: fcntl.flock(lock, fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError: parser.error("Another controller owns the arms")
        limits = load_limits(args.limits_file)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w") as output:
            def emit(row):
                line=json.dumps(row,separators=(",",":"),allow_nan=False)
                print(line,flush=True); output.write(line+"\n"); output.flush()
            report=follow(apply=args.apply,feedback_master=args.feedback_master,confirm_supported=args.confirm_supported,
                          duration=args.duration,limits=limits,stop_requested=stopped.is_set,emit=emit)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
