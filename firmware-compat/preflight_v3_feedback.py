#!/usr/bin/env python3
"""Passively require fresh fault-free WIDTH feedback from all four V3 arms."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path

from prepare_follower_mode import sample_feedback


CHANNELS = ("can_left_slave", "can_right_slave", "can_left_mas", "can_right_mas")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = {"passed": False, "hardware_control_writes": 0,
              "sdk_instances_created": 0, "channels": {}}
    with ThreadPoolExecutor(max_workers=4) as pool:
        pending = {name: pool.submit(sample_feedback, name) for name in CHANNELS}
        for name, future in pending.items():
            try:
                report["channels"][name] = future.result()
            except Exception as exc:
                report["channels"][name] = {"error": repr(exc)}
    report["passed"] = all("error" not in value for value in report["channels"].values())
    text = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text)
    print(text, end="")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
