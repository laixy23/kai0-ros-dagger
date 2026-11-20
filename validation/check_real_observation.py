#!/usr/bin/env python3
"""Read real ROS cameras and passive CAN feedback; infer without publishing actions."""
from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import json
from pathlib import Path
import threading
import time


HERE = Path(__file__).resolve().parent
TOPICS = {
    "top_head": "/camera_f/color/image_raw",
    "hand_left": "/camera_l/color/image_raw",
    "hand_right": "/camera_r/color/image_raw",
}
CHANNELS = ("can_left_slave", "can_right_slave", "can_left_mas", "can_right_mas")
REQUIRED_IDS = ("0x2A1", "0x2A5", "0x2A6", "0x2A7", "0x2A8") + tuple(
    f"0x{value:03X}" for value in range(0x261, 0x267))


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--can-reader", type=Path, default=HERE.parent / "setup" / "read_can_passive.py")
    parser.add_argument("--output", type=Path, default=HERE / "real-observation-report.json")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8014)
    parser.add_argument("--can-seconds", type=float, default=1.0)
    parser.add_argument("--camera-timeout", type=float, default=10.0)
    args = parser.parse_args()
    require(0 < args.can_seconds <= 5 and args.camera_timeout > 0, "Invalid sampling duration")

    import cv2
    from cv_bridge import CvBridge
    import numpy as np
    from openpi_client import image_tools, websocket_client_policy
    import rospy
    from sensor_msgs.msg import Image

    report = {"passed": False, "timestamp_sec": time.time(),
              "model_uri": f"ws://{args.host}:{args.port}", "model_requests": [],
              "hardware_control_writes": 0, "ros_command_messages_published": 0,
              "can_application_frames_sent": 0, "piper_sdk_used": False,
              "image_max_age_s": 0.3, "can_max_age_s": 0.2,
              "prompt": "fold the cloth",
              "preprocessing": "CvBridge bgr8 -> JPEG encode/decode -> BGR2RGB -> resize_with_pad 224 -> CHW"}
    latest, lock, subscribers = {}, threading.Lock(), []
    policy = None
    try:
        spec = importlib.util.spec_from_file_location("passive_can_reader", args.can_reader.resolve(strict=True))
        passive = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(passive)
        rospy.init_node("dagger_real_observation_probe", anonymous=True, disable_signals=True)
        bridge = CvBridge()

        def on_image(message, name):
            with lock:
                latest[name] = (message, time.monotonic())

        subscribers = [rospy.Subscriber(topic, Image, on_image, callback_args=name,
                                       queue_size=1, buff_size=2 ** 24)
                       for name, topic in TOPICS.items()]
        deadline = time.monotonic() + args.camera_timeout
        ready = False
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with lock:
                ready = all(name in latest for name in TOPICS)
            if ready:
                break
            time.sleep(0.02)
        require(ready, "Did not receive all three actual ROS camera topics")
        policy = websocket_client_policy.WebsocketClientPolicy(host=args.host, port=args.port)
        report["model_metadata"] = policy.get_server_metadata()

        def read_can(channel):
            row = passive.receive(channel, args.can_seconds)
            return row, time.time()

        with ThreadPoolExecutor(max_workers=4) as pool:
            for index in range(3):
                futures = {channel: pool.submit(read_can, channel) for channel in CHANNELS}
                sampled = {channel: future.result() for channel, future in futures.items()}
                with lock:
                    snapshot = {name: latest[name] for name in TOPICS}
                rgb_images = []
                ordered = ("top_head", "hand_right", "hand_left")
                for name in ordered:
                    message, _ = snapshot[name]
                    bgr = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                    require(bgr.shape == (480, 640, 3), f"Unexpected {name} image shape: {bgr.shape}")
                    ok, encoded = cv2.imencode(".jpg", bgr)
                    require(ok, f"JPEG encoding failed for {name}")
                    bgr = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                    require(bgr is not None, f"JPEG decoding failed for {name}")
                    rgb_images.append(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                resized = image_tools.resize_with_pad(np.asarray(rgb_images), 224, 224)
                wall, mono, ros_now = time.time(), time.monotonic(), rospy.Time.now().to_sec()
                image_info = {}
                for name, (message, received) in snapshot.items():
                    stamp = message.header.stamp.to_sec()
                    header_age, receipt_age = ros_now - stamp, mono - received
                    require(stamp > 0 and -0.05 <= header_age <= 0.3 and receipt_age <= 0.3,
                            f"Stale/future {name} image: header={header_age:.4f}s receipt={receipt_age:.4f}s")
                    image_info[name] = {"topic": TOPICS[name], "encoding": message.encoding,
                                        "shape": [message.height, message.width, 3],
                                        "header_age_s": header_age, "receipt_age_s": receipt_age}
                can_info = {}
                for channel, (row, finished) in sampled.items():
                    require(row.get("application_frames_sent") == 0, f"CAN reader reported a write: {channel}")
                    ages = {key: float(value) + wall - finished
                            for key, value in row.get("component_age_s", {}).items()}
                    require(all(key in ages and -0.05 <= ages[key] <= 0.2 for key in REQUIRED_IDS),
                            f"Missing/stale CAN feedback: {channel} {ages}")
                    status = row.get("arm_status", {})
                    faults = row.get("joint_driver_fault_masks", [])
                    qpos = np.asarray(row.get("qpos_rad_and_gripper_m", []), dtype=float)
                    require(status.get("arm_status") == 0 and status.get("err_code") == 0,
                            f"Arm status fault: {channel} {status}")
                    require(len(faults) == 6 and not any(faults), f"Driver fault: {channel} {faults}")
                    require(qpos.shape == (7,) and np.isfinite(qpos).all(), f"Invalid qpos: {channel}")
                    can_info[channel] = {"component_age_s": ages, "qpos_rad_and_gripper_m": qpos.tolist(),
                                         "arm_status": status, "joint_driver_fault_masks": faults,
                                         "joint_driver_enabled": row.get("joint_driver_enabled"),
                                         "joint_feedback_hz": row.get("joint_feedback_hz")}
                state = np.concatenate([can_info[channel]["qpos_rad_and_gripper_m"]
                                        for channel in ("can_left_slave", "can_right_slave")])
                payload = {"state": state, "images": {name: resized[i].transpose(2, 0, 1)
                                                       for i, name in enumerate(ordered)},
                           "prompt": "fold the cloth"}
                entry = {"request_index": index + 1, "observation_timestamp_sec": wall,
                         "images": image_info, "can": can_info, "state_shape": list(state.shape),
                         "model_image_shapes": {key: list(value.shape) for key, value in payload["images"].items()}}
                report["model_requests"].append(entry)
                started = time.monotonic()
                response = policy.infer(payload)
                actions = np.asarray(response["actions"])
                entry.update(latency_ms=(time.monotonic() - started) * 1000,
                             action_shape=list(actions.shape), finite=bool(np.isfinite(actions).all()),
                             action_horizon=int(actions.shape[0]) if actions.ndim else 0)
                require(actions.shape == (50, 14) and entry["finite"], f"Invalid model actions: {entry['action_shape']}")
                entry.update(action_min=float(actions.min()), action_max=float(actions.max()))
                delta = actions[0] - state
                entry.update(first_action=actions[0].tolist(), first_action_delta=delta.tolist(),
                             max_first_joint_delta_rad=float(np.max(np.abs(delta[[0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]]))))
        report["passed"] = True
    except Exception as exc:
        report["error"] = repr(exc)
    finally:
        for subscriber in subscribers:
            subscriber.unregister()
        if policy is not None:
            try:
                policy._ws.close()
            except Exception as exc:
                report["connection_close_error"] = repr(exc)
        rospy.signal_shutdown("Read-only real observation probe complete")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2,
                                         default=lambda value: value.tolist() if hasattr(value, "tolist") else str(value)) + "\n")
        print(json.dumps({"passed": report["passed"], "report": str(args.output),
                          "model_requests": len(report["model_requests"]),
                          "hardware_control_writes": 0, "error": report.get("error")}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
