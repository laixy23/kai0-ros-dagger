"""Receive Piper feedback only: never imports the SDK or calls Bus.send()."""
import argparse
import collections
import concurrent.futures
import json
from pathlib import Path
import struct
import time

import can


CHANNELS = ("can_left_slave", "can_right_slave", "can_left_mas", "can_right_mas")


def receive(channel, duration):
    counts, latest, stamps = collections.Counter(), {}, {}
    with can.Bus(channel=channel, interface="socketcan", receive_own_messages=False) as bus:
        started = time.monotonic()
        while time.monotonic() - started < duration:
            message = bus.recv(timeout=max(0, min(.1, duration - (time.monotonic() - started))))
            if message is None or message.is_error_frame or message.is_remote_frame:
                continue
            counts[f"0x{message.arbitration_id:03X}"] += 1
            if (message.arbitration_id in (0x2A1, 0x2A5, 0x2A6, 0x2A7, 0x2A8)
                or 0x261 <= message.arbitration_id <= 0x266) and len(message.data) == 8:
                latest[message.arbitration_id] = bytes(message.data)
                stamps[message.arbitration_id] = message.timestamp
        elapsed = time.monotonic() - started
    report = {"channel": channel, "application_frames_sent": 0, "duration_s": elapsed,
              "received_frames": sum(counts.values()), "frame_counts": dict(sorted(counts.items())),
              "component_age_s": {f"0x{k:03X}": time.time() - stamp for k, stamp in stamps.items()}}
    if all(key in latest for key in (0x2A5, 0x2A6, 0x2A7, 0x2A8)):
        joints = [v * .017444 / 1000 for key in (0x2A5, 0x2A6, 0x2A7)
                  for v in struct.unpack(">ii", latest[key])]
        report["qpos_rad_and_gripper_m"] = joints + [struct.unpack(">i", latest[0x2A8][:4])[0] / 1e6]
        report["joint_feedback_hz"] = counts["0x2A5"] / elapsed
    if 0x2A1 in latest:
        data = latest[0x2A1]
        report["arm_status"] = {"ctrl_mode": data[0], "arm_status": data[1],
                                "err_code": int.from_bytes(data[6:8], "big")}
    if all(key in latest for key in range(0x261, 0x267)):
        report["joint_driver_enabled"] = [bool(latest[key][5] & 0x40) for key in range(0x261, 0x267)]
        report["joint_driver_fault_masks"] = [latest[key][5] & 0xBF for key in range(0x261, 0x267)]
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=3)
    args = parser.parse_args()
    if not .5 <= args.seconds <= 30:
        parser.error("--seconds must be between .5 and 30")
    absent = [channel for channel in CHANNELS if not (Path("/sys/class/net") / channel).exists()]
    if absent:
        raise SystemExit("Missing CAN interfaces: " + ", ".join(absent))
    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        futures = {pool.submit(receive, channel, args.seconds): channel for channel in CHANNELS}
        reports = []
        for future, channel in futures.items():
            try:
                reports.append(future.result())
            except Exception as exc:
                reports.append({"channel": channel, "error": str(exc), "application_frames_sent": 0})
    result = {"timestamp_sec": time.time(), "sdk_opened": False, "application_frames_sent": 0,
              "channels": reports}
    output = Path(__file__).resolve().parent / "passive-can-report.json"
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
