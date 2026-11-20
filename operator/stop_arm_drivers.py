#!/usr/bin/env python3
"""Stop the managed ROS arm processes without changing any motor/CAN state.

Freeze the complete service before killing it. A normal systemd stop can send
SIGCONT to paused nodes; resuming an old master node could replay stale targets.
This helper never resumes nodes or calls a hardware API. It is not an emergency
stop: an already accepted motor trajectory or holding torque can remain active.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import time


SERVICE = "kai0-official-arms.service"


def stop_arm_drivers(*, runner=subprocess.run, timeout=5.0):
    report = {"passed": False, "service": SERVICE, "commands": [], "errors": [],
              "hardware_api_calls": 0, "resume_requested": False}

    def call(arguments):
        command = ["systemctl", "--user", *arguments, SERVICE]
        report["commands"].append(command)
        try:
            result = runner(command, capture_output=True, text=True, check=False, timeout=timeout)
        except (OSError, subprocess.TimeoutExpired) as exc:
            report["errors"].append(repr(exc))
            return None
        if result.returncode:
            report["errors"].append(result.stderr.strip() or f"systemctl returned {result.returncode}")
        return result

    def state():
        result = call(["show", "--property=LoadState,ActiveState,SubState,MainPID,ControlGroup"])
        if result is None:
            return None
        values = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
        return values if values.get("LoadState") else None

    def stopped(values):
        if not values:
            return False
        if values.get("LoadState") == "not-found":
            return True
        if values.get("ActiveState") not in ("inactive", "failed") or values.get("MainPID", "0") != "0":
            return False
        group = values.get("ControlGroup", "")
        if group:
            # cgroup v2 is used on wj-dev. Refuse to claim success if members
            # remain, or if a nonempty group cannot be checked.
            path = Path("/sys/fs/cgroup") / group.lstrip("/") / "cgroup.procs"
            try:
                if path.read_text().strip():
                    return False
            except FileNotFoundError:
                pass  # Removed after the service became inactive.
            except OSError:
                return False
        return True

    initial = state()
    report["initial_state"] = initial
    if initial is None:
        report["errors"].append("Cannot determine the managed arm service state")
        return report
    if stopped(initial):
        report.update(passed=True, final_state=initial, already_stopped=True)
        return report
    # Try KILL even if the service exited during the freeze command. Never
    # fall back to SIGCONT, nor to a graceful stop while a node may be paused.
    call(["kill", "--kill-whom=all", "--signal=SIGSTOP"])
    call(["kill", "--kill-whom=all", "--signal=SIGKILL"])
    # Verify no live group remains before invoking systemd's normal stop,
    # because stop may SIGCONT members before terminating them.
    deadline = time.monotonic() + timeout
    after_kill = state()
    while not stopped(after_kill) and time.monotonic() < deadline:
        time.sleep(0.05)
        after_kill = state()
    report["after_kill_state"] = after_kill
    if not stopped(after_kill):
        report["errors"].append("Arm service still has live members after KILL; no SIGCONT/normal stop attempted")
        return report
    call(["stop"])
    final = state()
    report.update(final_state=final, passed=stopped(final))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--human", action="store_true", help="Print a short operator-facing result")
    args = parser.parse_args()
    report = stop_arm_drivers()
    if args.human and report["passed"]:
        print("[已停止] 四臂 ROS 驱动和转发已停止；电机状态未主动更改，未归位或恢复推理。")
    else:
        if args.human:
            print("[错误] 无法确认四臂 ROS 驱动全部停止，请检查以下状态。")
        print(json.dumps(report, indent=2))
    return 0 if report["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
