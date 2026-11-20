#!/usr/bin/env python3
"""Subscribe to official Kai0 camera/feedback topics without publishing commands."""
import argparse
import json
import threading
import time

import rospy
from sensor_msgs.msg import Image, JointState

TOPICS = {
    "/camera_f/color/image_raw": Image,
    "/camera_l/color/image_raw": Image,
    "/camera_r/color/image_raw": Image,
    "/puppet/joint_left": JointState,
    "/puppet/joint_right": JointState,
    "/puppet_master/joint_left": JointState,
    "/puppet_master/joint_right": JointState,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=10)
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--cameras-only", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    if args.timeout <= 0 or args.samples < 1:
        parser.error("timeout and samples must be positive")
    selected = {name: kind for name, kind in TOPICS.items()
                if not args.cameras_only or kind is Image}
    lock, counts, latest = threading.Lock(), {}, {}
    rospy.init_node("kai0_official_topic_probe", anonymous=True, disable_signals=True)

    def record(message, name):
        row = {"stamp_sec": message.header.stamp.to_sec(), "frame_id": message.header.frame_id}
        if isinstance(message, Image):
            row.update(width=message.width, height=message.height, encoding=message.encoding)
        else:
            row.update(position=list(message.position), velocity=list(message.velocity))
        with lock:
            counts[name] = counts.get(name, 0) + 1
            latest[name] = row

    subscribers = [rospy.Subscriber(name, kind, record, callback_args=name, queue_size=3)
                   for name, kind in selected.items()]
    deadline = time.monotonic() + args.timeout
    try:
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with lock:
                ready = all(counts.get(name, 0) >= args.samples for name in selected)
            if ready:
                break
            time.sleep(.02)
        with lock:
            report = {"passed": all(counts.get(name, 0) >= args.samples for name in selected),
                      "command_messages_published": 0,
                      "topics": {name: {"received": counts.get(name, 0), **latest.get(name, {})}
                                 for name in selected}}
        print(json.dumps(report, indent=2))
        return 0 if report["passed"] else 1
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        rospy.signal_shutdown("read-only topic probe complete")


if __name__ == "__main__":
    raise SystemExit(main())
