#!/usr/bin/env python3
"""Measure J1-J6 of the right pair; optionally align ONLY the supported master.

Default is passive. Explicit --apply --confirm-supported permits a measured-pose
master recovery and slow JointCtrl targets. No follower, gripper, home, reset,
teach-mode activation, ROS publisher, inference, or automatic disable is used.
The rate limit describes the commanded target ramp, not guaranteed motor speed.
"""
from __future__ import annotations

import argparse
import fcntl
import json
import math
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "setup"))
from prepare_follower_mode import MAX_AGE_S, REQUIRED_IDS, make_sdk, sample_feedback
from recover_master_mode import active_controllers, recover

MASTER = "can_right_mas"
SLAVE = "can_right_slave"
MAX_START_MDEG = 45_000
MAX_RATE_MDEG_S = 5_000
MAX_SLAVE_DRIFT_MDEG = 1_000
TOLERANCE_MDEG = 1_000


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def valid_snapshot(row, *, since=0.0, require_can=True):
    require(row.get("arm_status") == 0 and row.get("err_code") == 0, "Arm fault/error feedback")
    require(row.get("gripper_mode") == 0, "Gripper feedback must be WIDTH")
    q = row.get("joint_mdeg", [])
    require(len(q) == 6 and all(isinstance(x, (int, float)) and not isinstance(x, bool)
                              and math.isfinite(x) for x in q), "Invalid/nonfinite six-joint actual feedback")
    enabled = row.get("driver_enabled", [])
    require(len(enabled) == 6 and all(type(x) is bool for x in enabled), "Incomplete six-driver feedback")
    stamp = row.get("timestamp_sec")
    require(isinstance(stamp, (int, float)) and math.isfinite(stamp) and stamp >= since,
            "Missing/stale post-command snapshot timestamp")
    ages = row.get("component_age_s", {})
    for key in REQUIRED_IDS:
        age = ages.get(f"0x{key:03X}")
        require(isinstance(age, (int, float)) and math.isfinite(age)
                and -0.05 <= age <= MAX_AGE_S and stamp - age + 1e-6 >= since,
                f"Stale/missing/pre-command raw CAN component 0x{key:03X}")
    if require_can:
        require(row.get("ctrl_mode") == 1 and all(enabled), "Expected fault-free CAN mode with all six drivers enabled")
    return row


def comparison(master, slave):
    return [{"joint": i + 1, "master_deg": master["joint_mdeg"][i] / 1000,
             "slave_deg": slave["joint_mdeg"][i] / 1000,
             "error_deg": (master["joint_mdeg"][i] - slave["joint_mdeg"][i]) / 1000}
            for i in range(6)]


def fresh_at_write(row, now):
    """Reading the other arm or opening an SDK must not hide elapsed age."""
    delay = now - row["timestamp_sec"]
    require(math.isfinite(delay) and delay >= -0.05, "Invalid/future raw feedback clock before write")
    for key in REQUIRED_IDS:
        age = row["component_age_s"][f"0x{key:03X}"] + delay
        require(-0.05 <= age <= MAX_AGE_S, f"Raw CAN component aged before write 0x{key:03X}")


def diagnose(*, apply=False, confirm_supported=False, reader=sample_feedback,
             sdk_factory=make_sdk, recovery=recover, controller_reader=active_controllers,
             clock=time.monotonic, wall=time.time, pause=time.sleep,
             convergence_timeout=8.0, stable_period=0.3, period=0.1):
    report = {"passed": False, "apply": apply, "supported_confirmed": confirm_supported,
              "master_channel": MASTER, "slave_channel": SLAVE,
              "joint_units": "0.001 degree", "commands": [], "initial_feedback": {},
              "follower_control_writes": 0, "gripper_control_writes": 0,
              "home_reset_or_teach_activation_commands": 0,
              "automatic_disable_after_enable": False,
              "target_ramp_limit_deg_s": MAX_RATE_MDEG_S / 1000,
              "target_rate_is_not_measured_motor_speed": True}
    sdk = None
    alignment_started = False

    def no_controllers():
        owners = controller_reader()
        require(not owners, "Other arm/collector controllers still running: " + str(owners))

    def read(channel, since=0.0, require_can=True):
        return valid_snapshot(reader(channel, since_timestamp=since), since=since, require_can=require_can)

    def send(target, stage, measured, follower):
        nonlocal alignment_started
        no_controllers()
        fresh_at_write(measured, wall())
        fresh_at_write(follower, wall())
        alignment_started = True
        mono, stamp = clock(), wall()
        report["commands"].append({"method": "JointCtrl", "channel": MASTER,
                                   "target_mdeg": list(target), "stage": stage,
                                   "monotonic_sec": mono, "timestamp_sec": stamp,
                                   "master_actual_mdeg": list(measured["joint_mdeg"])})
        sdk.JointCtrl(*target)
        return mono, stamp

    try:
        require(not apply or confirm_supported, "--apply requires explicit supported/clear-workspace confirmation")
        no_controllers()
        for channel in (SLAVE, MASTER):
            try:
                report["initial_feedback"][channel] = read(channel, require_can=False)
            except Exception as exc:
                report["initial_feedback"][channel] = {"error": str(exc)}
        initial_slave = report["initial_feedback"][SLAVE]
        require("error" not in initial_slave, "Missing/unsafe right follower feedback: " + str(initial_slave))
        valid_snapshot(initial_slave)
        baseline_slave = list(initial_slave["joint_mdeg"])
        initial_master = report["initial_feedback"][MASTER]
        if not apply:
            require("error" not in initial_master, "Missing/unsafe right master feedback: " + str(initial_master))
            valid_snapshot(initial_master)
            report["joint_comparison"] = comparison(initial_master, initial_slave)
            report["passed"] = all(abs(row["error_deg"]) <= TOLERANCE_MDEG / 1000
                                   for row in report["joint_comparison"])
            report["passive_only"] = True
            return report

        if "error" in initial_master:
            require(initial_master["error"].startswith("Missing required CAN feedback frames"),
                    "Unsafe master feedback; recovery refused: " + initial_master["error"])
            needs_recovery = True
        else:
            require(initial_master.get("ctrl_mode") in (0, 1, 2), "Unsupported master control mode")
            needs_recovery = initial_master["ctrl_mode"] != 1 or not all(initial_master["driver_enabled"])
        if needs_recovery:
            no_controllers()
            report["master_recovery"] = recovery(MASTER, apply=True)
            require(report["master_recovery"].get("passed"), "Measured-pose master recovery failed")
        no_controllers()
        master = read(MASTER)
        slave = read(SLAVE)
        report["post_recovery_feedback"] = {MASTER: master, SLAVE: slave}
        require(max(abs(a - b) for a, b in zip(slave["joint_mdeg"], baseline_slave))
                <= MAX_SLAVE_DRIFT_MDEG, "Right follower moved more than 1 degree during preparation")
        target = list(slave["joint_mdeg"])
        command = [int(round(x)) for x in master["joint_mdeg"]]
        starting_errors = [a - b for a, b in zip(command, target)]
        report["captured_target_mdeg"] = list(target)
        report["starting_error_mdeg"] = starting_errors
        require(max(map(abs, starting_errors)) <= MAX_START_MDEG,
                "Starting joint difference exceeds 45 degrees; no alignment trajectory sent")
        sdk = sdk_factory(MASTER)
        sdk.ConnectPort(piper_init=False)
        # Construction/connection can take time. The first holding command must
        # use actual feedback read after that delay, not the pre-connect cache.
        master, slave = read(MASTER), read(SLAVE)
        require(max(abs(a - b) for a, b in zip(slave["joint_mdeg"], baseline_slave))
                <= MAX_SLAVE_DRIFT_MDEG, "Right follower moved more than 1 degree while connecting")
        command = [int(round(x)) for x in master["joint_mdeg"]]
        starting_errors = [a - b for a, b in zip(command, target)]
        require(max(map(abs, starting_errors)) <= MAX_START_MDEG,
                "Starting joint difference exceeds 45 degrees after connection")
        report["post_connect_feedback"] = {MASTER: master, SLAVE: slave}
        report["initial_master_hold_mdeg"] = list(command)
        ramp_started = clock()
        ramp_deadline = ramp_started + max(map(abs, starting_errors)) / MAX_RATE_MDEG_S + 5.0
        last_command_mono, last_command_wall = send(command, "initial_measured_hold", master, slave)
        stage, stable_since = "ramp", None
        convergence_started = None
        previous_actual = list(master["joint_mdeg"])
        actual_step_max = 0
        command_rate_max = 0.0

        while True:
            pause(period)
            no_controllers()
            master = read(MASTER, last_command_wall)
            slave = read(SLAVE, last_command_wall)
            slave_drift = [a - b for a, b in zip(slave["joint_mdeg"], baseline_slave)]
            require(max(map(abs, slave_drift)) <= MAX_SLAVE_DRIFT_MDEG,
                    "Right follower moved more than 1 degree; trajectory stopped")
            actual_step_max = max(actual_step_max, max(abs(a - b) for a, b in
                                                       zip(master["joint_mdeg"], previous_actual)))
            previous_actual = list(master["joint_mdeg"])
            now = clock()
            report["final_feedback"] = {MASTER: master, SLAVE: slave}
            report["joint_comparison"] = comparison(master, slave)
            report["target_tracking_error_mdeg"] = [a - b for a, b in zip(master["joint_mdeg"], target)]
            report["slave_drift_mdeg"] = slave_drift
            report["max_measured_step_delta_mdeg"] = actual_step_max
            report["max_commanded_target_rate_deg_s"] = command_rate_max / 1000
            if stage == "converge":
                within = (max(abs(row["error_deg"]) for row in report["joint_comparison"]) <= 1
                          and max(map(abs, report["target_tracking_error_mdeg"])) <= TOLERANCE_MDEG)
                stable_since = now if within and stable_since is None else stable_since if within else None
                if stable_since is not None and now - stable_since >= stable_period:
                    report.update(passed=True, stable_duration_sec=now - stable_since)
                    break
                require(now - convergence_started <= convergence_timeout,
                        "Right master convergence timeout; J1-J6 errors_deg="
                        + str([row["error_deg"] for row in report["joint_comparison"]]))
                last_command_mono, last_command_wall = send(target, "converge", master, slave)
                continue

            require(now <= ramp_deadline, "Bounded right-master target ramp timed out")
            elapsed = now - last_command_mono
            require(math.isfinite(elapsed) and elapsed > 0, "Invalid monotonic ramp clock")
            step = math.floor(MAX_RATE_MDEG_S * min(elapsed, 0.25))
            next_command = [int(previous + max(-step, min(step, desired - previous)))
                            for previous, desired in zip(command, target)]
            actual_command_rate = max(abs(a - b) for a, b in zip(next_command, command)) / elapsed
            require(actual_command_rate <= MAX_RATE_MDEG_S + 1e-6, "Command target ramp rate exceeded")
            command_rate_max = max(command_rate_max, actual_command_rate)
            command = next_command
            last_command_mono, last_command_wall = send(command, "ramp", master, slave)
            if command == target:
                stage, convergence_started = "converge", clock()
    except (Exception, KeyboardInterrupt) as exc:
        report["error"] = repr(exc)
        if sdk is not None and alignment_started:
            try:
                no_controllers()
                snapshot = read(MASTER)
                fresh_at_write(snapshot, wall())
                report["failure_feedback"] = snapshot
                report["commands"].append({"method": "JointCtrl", "channel": MASTER,
                                           "stage": "failure_measured_hold",
                                           "target_mdeg": list(snapshot["joint_mdeg"]),
                                           "monotonic_sec": clock(), "timestamp_sec": wall()})
                sdk.JointCtrl(*snapshot["joint_mdeg"])
                report["failure_measured_hold_requested"] = True
            except Exception as hold_exc:
                report["failure_hold_error"] = repr(hold_exc)
    finally:
        if sdk is not None:
            try:
                sdk.DisconnectPort()
            except Exception as exc:
                report["disconnect_error"] = repr(exc)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--confirm-supported", action="store_true")
    parser.add_argument("--output", type=Path, default=ROOT / "_runtime/right-alignment-diagnostic.json")
    args = parser.parse_args()
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    lock = None
    if args.apply:
        if not args.confirm_supported:
            parser.error("--apply requires --confirm-supported")
        path = ROOT / "_runtime/keyboard-dagger.lock"
        path.parent.mkdir(parents=True, exist_ok=True)
        lock = path.open("a+")
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error("Another keyboard round or diagnostic owns the arms")
    try:
        report = diagnose(apply=args.apply, confirm_supported=args.confirm_supported)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report, indent=2))
        return 0 if report["passed"] else 1
    finally:
        if lock is not None:
            lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
