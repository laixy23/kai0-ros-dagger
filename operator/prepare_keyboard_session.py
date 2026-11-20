#!/usr/bin/env python3
"""Prepare a fresh keyboard round while retaining torque on all four arms.

No policy execution or initial-pose command is performed here. The collector
owns those actions after preparation succeeds. Ctrl+C never means resume.
"""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "setup"))
from prepare_follower_mode import sample_feedback
from prepare_follower_enabled import prepare_enabled
from enable_arm_current_pose import enable_at_current_pose
from recover_master_mode import active_controllers
from recover_master_enabled import cached_firmware_version, recover_enabled
from stop_arm_drivers import stop_arm_drivers

CHANNELS = ("can_left_slave", "can_right_slave", "can_left_mas", "can_right_mas")
SERVICES = ("kai0-official-mixed1", "kai0-official-roscore", "kai0-official-cameras")


def recovery_plan(rows, allow_enable=False):
    """Plan normal startup enables; strict checks still require all six motors enabled."""
    if set(rows) != set(CHANNELS):
        raise RuntimeError("四臂反馈不完整。")
    needed = []
    for name in CHANNELS:
        row = rows[name]
        # A missing-feedback label must never hide a known fault or disabled
        # motor. Only genuinely silent masters may use the firmware-gated path.
        for field in ('arm_status', 'err_code', 'gripper_mode'):
            if field in row and row[field] != 0:
                raise RuntimeError(name + " 存在故障或夹爪单位异常。")
        if 'driver_enabled' in row:
            enabled = row['driver_enabled']
            if len(enabled) != 6 or not all(type(value) is bool for value in enabled):
                raise RuntimeError(name + " 驱动反馈不完整。")
            if not all(enabled) and (not allow_enable or 'error' in row):
                raise RuntimeError(name + " 六驱动未全部使能，拒绝启动。")
        if 'driver_foc_bytes' in row:
            foc = row['driver_foc_bytes']
            if (len(foc) != 6 or not all(type(value) is int and 0 <= value <= 255 for value in foc)
                    or any(value & 0xBF for value in foc)
                    or (not all(value & 0x40 for value in foc) and (not allow_enable or 'error' in row))):
                raise RuntimeError(name + " 电机驱动故障或使能反馈不完整。")
        if 'ctrl_mode' in row and row['ctrl_mode'] not in (0, 1, 2):
            raise RuntimeError(name + " 处于其他控制模式，拒绝接管。")
        if "error" in row:
            if name.endswith("_mas") and row["error"].startswith("Missing required CAN feedback frames"):
                needed.append(name)
                continue
            raise RuntimeError(name + " 反馈异常：" + row["error"])
        if (row.get("arm_status") != 0 or row.get("err_code") != 0
                or row.get("gripper_mode") != 0):
            raise RuntimeError(name + " 存在故障或夹爪单位异常。")
        enabled = row.get("driver_enabled", [])
        if len(enabled) != 6 or not all(type(value) is bool for value in enabled):
            raise RuntimeError(name + " 驱动反馈不完整。")
        if row.get("ctrl_mode") not in (0, 1, 2):
            raise RuntimeError(name + " 处于其他控制模式，拒绝接管。")
        needs_enable = not all(enabled) or ('driver_foc_bytes' in row and
                                           not all(value & 0x40 for value in row['driver_foc_bytes']))
        if name.endswith("_slave"):
            if row["ctrl_mode"] not in (1, 2) and not (allow_enable and needs_enable):
                raise RuntimeError(name + " 从臂处于待机模式，需单独检查；不会自动执行卸力恢复。")
        if needs_enable or row["ctrl_mode"] != 1:
            needed.append(name)
    return needed


def prepare_session(backend, confirm=None):
    """Restore control and directly enable at actual pose, without unloading first."""
    report = {"passed": False, "recovery": {}, "policy_execution_started": False,
              "initial_pose_requested": False, "support_confirmed": False,
              "torque_removal_requested": False,
              "master_firmware_version_source": "cached_receipt",
              "master_torque_removal_requested": False,
              "follower_torque_removal_requested": False,
              "enable_requested": [],
              "torque_removal_requested_by_channel": {name: False for name in CHANNELS}}
    claimed = False
    try:
        backend.check_dependencies_and_ownership()
        claimed = True
        report["old_drivers_stopped"] = backend.stop()
        rows = backend.feedback()
        needed = recovery_plan(rows, allow_enable=True)
        enable_needed = [name for name in needed if
                         not all(rows[name].get('driver_enabled', [True] * 6)) or
                         ('driver_foc_bytes' in rows[name] and
                          not all(value & 0x40 for value in rows[name]['driver_foc_bytes']))]
        report.update(initial_feedback=rows, recovery_needed=needed, enable_needed=enable_needed)
        if needed:
            for name in needed:
                if name in enable_needed:
                    report['enable_requested'].append(name)
                    result = backend.restore(name, enable=True)
                else:
                    result = backend.restore(name)
                report["recovery"][name] = result
                if not result.get("passed"):
                    raise RuntimeError(name + " 恢复失败：" + result.get("error", "未知错误"))
        # Revalidate after every preparation sequence, before enabling ROS writes.
        for _ in range(3):
            if recovery_plan(backend.feedback()):
                raise RuntimeError("恢复后四臂未保持 CAN 模式及六驱动使能，拒绝启动。")
        backend.start()
        report["drivers_started"] = True
        backend.wait_ready()
        report["passed"] = True
    except (Exception, KeyboardInterrupt) as exc:
        report["error"] = repr(exc)
        report["interrupted"] = isinstance(exc, KeyboardInterrupt)
        if claimed:
            try:
                report["failure_cleanup"] = backend.stop()
            except Exception as cleanup_exc:
                report["cleanup_error"] = repr(cleanup_exc)
    return report


class HardwareBackend:
    def __init__(self, root):
        self.root = root

    @staticmethod
    def run(argv, timeout=10):
        return subprocess.run(argv, check=True, capture_output=True, text=True, timeout=timeout)

    def check_dependencies_and_ownership(self):
        for service in SERVICES:
            self.run(["systemctl", "--user", "is-active", "--quiet", service + ".service"])
        links = json.loads(self.run(["ip", "-j", "-details", "link", "show"]).stdout)
        by_name = {row.get("ifname"): row for row in links}
        for name in CHANNELS:
            row = by_name.get(name, {})
            data = row.get("linkinfo", {}).get("info_data", {})
            if ("UP" not in row.get("flags", [])
                    or data.get("bittiming", {}).get("bitrate") != 1_000_000
                    or data.get("state") in ("BUS-OFF", "ERROR-PASSIVE")
                    or any(str(mode).upper().replace("_", "-") in ("LOOPBACK", "LISTEN-ONLY")
                           for mode in data.get("ctrlmode", []))):
                raise RuntimeError(name + " 未配置为正常 UP / 1 Mbps。")
        unit = subprocess.run(["systemctl", "--user", "show", "kai0-official-arms.service",
                               "--property=ControlGroup", "--value"],
                              text=True, capture_output=True, timeout=5)
        group = unit.stdout.strip()
        for owner in active_controllers():
            if owner["script"] != "piper_start_ms_node_new.py":
                raise RuntimeError("已有采集器运行，PID=" + str(owner["pid"]))
            cgroups = Path(f'/proc/{owner["pid"]}/cgroup').read_text().splitlines()
            owned = bool(group) and any(line.split(":", 2)[-1] == group
                                       or line.split(":", 2)[-1].startswith(group + "/") for line in cgroups)
            if not owned:
                raise RuntimeError("发现独立四臂驱动，请先结束它，PID=" + str(owner["pid"]))

    def stop(self):
        result = stop_arm_drivers()
        if not result.get("passed"):
            raise RuntimeError("停止旧四臂驱动失败：" + json.dumps(result, ensure_ascii=False))
        return result

    def feedback(self):
        def one(name):
            try:
                return sample_feedback(name)
            except Exception as exc:
                return {"error": str(exc)}
        with ThreadPoolExecutor(max_workers=4) as pool:
            return dict(zip(CHANNELS, pool.map(one, CHANNELS)))

    def restore(self, name, *, enable=False):
        action = "按实测当前位置直接使能，不先失能或复位" if enable else "按实测当前位置恢复控制模式"
        print("[恢复] " + name + "：" + action + "……", flush=True)
        if active_controllers():
            raise RuntimeError("模式恢复前发现新的控制进程，已停止。")
        if enable:
            return enable_at_current_pose(name, apply=True)
        if name.endswith("_mas"):
            return recover_enabled(name, apply=True,
                                   firmware_version=cached_firmware_version(self.root, name))
        return prepare_enabled(name, apply=True)

    def start(self):
        if active_controllers():
            raise RuntimeError("启动前发现新的控制进程，已停止。")
        self.run(["systemd-run", "--user", "--unit=kai0-official-arms", "--collect",
                  "--property=WorkingDirectory=" + str(self.root), "--setenv=ROS_IP=127.0.0.1",
                  "/bin/bash", str(self.root / "ros-launch/run_ros.sh"), "roslaunch",
                  str(self.root / "ros-launch/arms_official_wj_dev.launch")])

    def wait_ready(self):
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            self.run(["systemctl", "--user", "is-active", "--quiet", "kai0-official-arms.service"])
            if not recovery_plan(self.feedback()):
                # Read actual ROS topics as well: active service alone is insufficient.
                self.run(["bash", str(self.root / "ros-launch/run_ros.sh"),
                          str(self.root / ".ros-env/bin/python"),
                          str(self.root / "ros-launch/probe_official_topics.py"),
                          "--samples", "3", "--timeout", "15"], timeout=20)
                return
            time.sleep(0.2)
        raise RuntimeError("四臂驱动初始化超时。")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(os.environ.get("DAGGER_ROOT", ROOT)))
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    if not args.apply:
        parser.error("仅由启动入口显式传入 --apply；只读检查请使用 run_dagger.sh --check")
    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    print("[准备] 四臂按实测当前位置恢复控制；未使能时直接使能，不先失能或复位。", flush=True)
    report = prepare_session(HardwareBackend(args.root))
    path = args.root / "_runtime/keyboard-prepare.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    if not report["passed"]:
        print("[阻塞] " + report["error"], file=sys.stderr)
        return 130 if report.get("interrupted") else 2
    print("[通过] 新一轮驱动已就绪。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
