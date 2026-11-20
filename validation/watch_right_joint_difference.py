#!/usr/bin/env python3
"""Watch right-pair raw joint angles; optionally enter master manual teach only.

No JointCtrl, follower write, gripper write, enable/disable, reset or role switch.
Exit only ends observation; it NEVER restores the master mode automatically.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import fcntl
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

from check_right_alignment import (MASTER, SLAVE, ROOT, active_controllers, comparison,
                                   fresh_at_write, make_sdk, require, sample_feedback,
                                   valid_snapshot)


def watch(*, duration=120.0, prepare_manual=False, confirm_supported=False, unpowered_master=False,
          reader=sample_feedback, sdk_factory=make_sdk, controller_reader=active_controllers,
          clock=time.monotonic, wall=time.time, pause=time.sleep,
          stop_requested=lambda: False, emit=lambda row: None, ack_timeout=3.0):
    report = {"record_type": "summary", "passed": False, "samples": 0,
              "prepare_manual": prepare_manual, "manual_entry_requested": False,
              "unpowered_master_observation": unpowered_master,
              "automatic_mode_restore": False, "joint_targets_sent": 0,
              "follower_control_writes": 0, "gripper_control_writes": 0,
              "enable_disable_reset_or_role_writes": 0}
    sdk = None
    latest = None
    minima = maxima = None

    def no_controllers():
        require(not controller_reader(), "Arm/collector controller appeared; observer stopped")

    def pair(since=0.0):
        master = valid_snapshot(reader(MASTER, since_timestamp=since), since=since, require_can=False)
        slave = valid_snapshot(reader(SLAVE, since_timestamp=since), since=since)
        if unpowered_master:
            require(master["ctrl_mode"] == 0 and not any(master["driver_enabled"]),
                    "Unpowered observation requires standby and all six master drivers disabled")
        else:
            require(master["ctrl_mode"] in (1, 2) and all(master["driver_enabled"]),
                    "Master must have fresh enabled CAN/manual feedback")
        fresh_at_write(master, wall())
        fresh_at_write(slave, wall())
        if "baseline_slave_mdeg" in report:
            require(max(abs(a - b) for a, b in zip(slave["joint_mdeg"], report["baseline_slave_mdeg"])) <= 1000,
                    "Right follower moved more than 1 degree; observer stopped")
        report["last_observed_master_ctrl_mode"] = master["ctrl_mode"]
        return master, slave

    try:
        require(math.isfinite(duration) and 0 < duration <= 180, "Duration must be finite and in (0,180] seconds")
        require(not prepare_manual or confirm_supported, "--prepare-manual requires --confirm-supported")
        require(not (prepare_manual and unpowered_master), "Unpowered observation cannot prepare or change modes")
        no_controllers()
        require(not stop_requested(), "Observer cancelled before preparation")
        master, slave = pair()
        report["initial_feedback"] = {MASTER: master, SLAVE: slave}
        report["baseline_slave_mdeg"] = list(slave["joint_mdeg"])
        if unpowered_master:
            emit({"record_type": "ready", "timestamp_sec": wall(), "master_ctrl_mode": 0,
                  "all_six_disabled": True, "manual_entry_requested": False,
                  "message": "READONLY READY: master is unpowered; support its weight; no follower movement"})
        since = 0.0
        if prepare_manual:
            require(master["ctrl_mode"] == 1, "Manual preparation requires master initially in enabled CAN mode")
            sdk = sdk_factory(MASTER)
            sdk.ConnectPort(piper_init=False)
            no_controllers()
            require(not stop_requested(), "Observer cancelled before manual entry")
            master, slave = pair()
            require(master["ctrl_mode"] == 1, "Master changed mode before manual entry")
            since = wall()
            report["manual_entry_requested"] = True
            sdk.MotionCtrl_1(grag_teach_ctrl=0x01)
            deadline = clock() + ack_timeout
            acknowledged = False
            while clock() < deadline and not stop_requested():
                no_controllers()
                master, slave = pair(since)
                if master["ctrl_mode"] == 2:
                    acknowledged = True
                    break
                pause(0.05)
            require(acknowledged, "No fresh manual-mode/full raw feedback acknowledgment; no role fallback attempted")
            report["manual_entry_acknowledged"] = True
            emit({"record_type": "ready", "timestamp_sec": wall(), "master_ctrl_mode": 2,
                  "all_six_enabled": True, "manual_entry_requested": True,
                  "message": "READY: move only the right master; right follower has no automatic following"})

        started = clock()
        next_due = started
        while clock() - started < duration and not stop_requested():
            if clock() < next_due:
                pause(min(next_due - clock(), 0.5))
            if stop_requested() or clock() - started >= duration:
                break
            no_controllers()
            master, slave = pair(since)
            if prepare_manual:
                require(master["ctrl_mode"] == 2, "Master left manual mode; observer stopped")
            latest = {"record_type": "sample", "timestamp_sec": wall(), "elapsed_sec": clock() - started,
                      "master_ctrl_mode": master["ctrl_mode"],
                      "master_deg": [value / 1000 for value in master["joint_mdeg"]],
                      "slave_deg": [value / 1000 for value in slave["joint_mdeg"]],
                      "difference_deg": [(a - b) / 1000 for a, b in zip(master["joint_mdeg"], slave["joint_mdeg"])]}
            emit(latest)
            if minima is None:
                minima = {key: list(latest[key]) for key in ("master_deg", "slave_deg", "difference_deg")}
                maxima = {key: list(latest[key]) for key in minima}
            else:
                for key in minima:
                    minima[key] = [min(a, b) for a, b in zip(minima[key], latest[key])]
                    maxima[key] = [max(a, b) for a, b in zip(maxima[key], latest[key])]
            report["samples"] += 1
            next_due = max(next_due + 0.5, clock())
        report.update(passed=report["samples"] > 0,
                      stop_reason="operator_stop" if stop_requested() else "duration_limit")
    except KeyboardInterrupt:
        report.update(passed=report["samples"] > 0, stop_reason="operator_interrupt")
    except Exception as exc:
        report.update(error=repr(exc), stop_reason="unsafe_or_missing_feedback")
    finally:
        if sdk is not None:
            try:
                sdk.DisconnectPort()
            except Exception as exc:
                report["disconnect_error"] = repr(exc)
        report["final_sample"] = latest
        report["min_deg"] = minima
        report["max_deg"] = maxima
        if minima is not None:
            report["range_deg"] = {key: [b - a for a, b in zip(minima[key], maxima[key])] for key in minima}
            report["master_movement_threshold_deg"] = 0.2
            report["master_movement_detected"] = max(report["range_deg"]["master_deg"]) > 0.2
        report["mode_left_unchanged_on_exit"] = True
        emit(report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare-manual", action="store_true")
    parser.add_argument("--confirm-supported", action="store_true")
    parser.add_argument("--unpowered-master", action="store_true",
                        help="Pure read-only comparison while all six master drivers are disabled; manually support the arm")
    parser.add_argument("--duration", type=float, default=120)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or not 0 < args.duration <= 180:
        parser.error("--duration must be finite and in (0,180]")
    if args.prepare_manual and not args.confirm_supported:
        parser.error("--prepare-manual requires --confirm-supported")
    if args.prepare_manual and args.unpowered_master:
        parser.error("--unpowered-master cannot change modes or prepare manual control")
    path = args.output or ROOT / ("_runtime/right-manual-comparison-"
                                 + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ") + ".jsonl")
    lock_path = ROOT / "_runtime/keyboard-dagger.lock"
    lock_path.parent.mkdir(parents=True, exist_ok=True)
    stopped = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, lambda s, f: stopped.set())
    with lock_path.open("a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("Another keyboard round or diagnostic owns the arms")
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w") as output:
            def emit(row):
                line = json.dumps(row, separators=(",", ":"), allow_nan=False)
                print(line, flush=True)
                output.write(line + "\n")
                output.flush()
            report = watch(duration=args.duration, prepare_manual=args.prepare_manual,
                           unpowered_master=args.unpowered_master,
                           confirm_supported=args.confirm_supported, stop_requested=stopped.is_set, emit=emit)
    if report["manual_entry_requested"]:
        print("Observer ended. Master mode was NOT restored automatically; no movement/enable/disable command was sent on exit.", file=sys.stderr)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
