#!/usr/bin/env python3
"""Exercise the real ROS collector and real model with synthetic ROS inputs only.

Run in the ROS environment, with generated piper_msgs and openpi_client importable.
The dedicated loopback master must be unused. No hardware nodes are launched.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import errno
import json
import math
import os
from pathlib import Path
import pty
import re
import select
import signal
import socket
import subprocess
import sys
import threading
import time
import xmlrpc.client


MASTER_PORT = 11321
PROXY_PORT = 11322
HERE = Path(__file__).resolve().parent
CAMERAS = ("cam_high", "cam_left_wrist", "cam_right_wrist")
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def sentinel_rgb(camera_index, phase):
    # Widely separated red/blue values expose swaps for all three cameras.
    return (210, 30 + 35 * phase, 20 + 45 * camera_index)


def model_rgb_sentinels(images, phase):
    import numpy as np
    actual = {}
    correct = True
    for name, camera_index in (("top_head", 0), ("hand_left", 1), ("hand_right", 2)):
        image = np.asarray(images[name])
        if image.shape != (3, 224, 224):
            actual[name] = {"invalid_shape": list(image.shape)}
            correct = False
            continue
        # Center is outside both resize padding and the changing sequence patch.
        rgb = image[:, 112, 112].astype(float)
        actual[name] = rgb.tolist()
        correct &= bool(np.all(np.abs(rgb - sentinel_rgb(camera_index, phase)) <= 10))
    return actual, correct


def wait_until(predicate, description, timeout=30):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def require_unused_port(port):
    with socket.socket() as probe:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        probe.bind(("127.0.0.1", port))


def stop_process(proc):
    if proc is None or proc.poll() is not None:
        return
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGTERM)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=3)


class CollectorTerminal:
    def __init__(self, command, env, log_path):
        self.buffer = ""
        self.changed = threading.Condition()
        self.master_fd, slave_fd = pty.openpty()
        self.log = log_path.open("w", encoding="utf-8")
        self.proc = subprocess.Popen(command, stdin=slave_fd, stdout=slave_fd,
                                     stderr=slave_fd, env=env, start_new_session=True)
        os.close(slave_fd)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        import codecs
        decoder = codecs.getincrementaldecoder("utf-8")(errors="replace")
        while True:
            try:
                ready, _, _ = select.select([self.master_fd], [], [], 0.2)
                if not ready:
                    if self.proc.poll() is not None:
                        break
                    continue
                data = os.read(self.master_fd, 65536)
                if not data:
                    break
            except OSError as exc:
                if exc.errno in (errno.EIO, errno.EBADF):
                    break
                raise
            chunk = decoder.decode(data)
            self.log.write(chunk)
            self.log.flush()
            with self.changed:
                self.buffer += ANSI.sub("", chunk)
                self.changed.notify_all()

    def mark(self):
        with self.changed:
            return len(self.buffer)

    def expect(self, text, after=0, timeout=60):
        deadline = time.monotonic() + timeout
        with self.changed:
            while text not in self.buffer[after:]:
                if self.proc.poll() is not None:
                    raise RuntimeError(f"Collector exited ({self.proc.returncode}) before {text!r}")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"Collector did not print {text!r}; see collector.log")
                self.changed.wait(min(remaining, 0.2))
            return self.buffer.index(text, after) + len(text)

    def key(self, value):
        os.write(self.master_fd, value.encode())

    def close(self):
        stop_process(self.proc)
        os.close(self.master_fd)


class ModelRelay:
    """Transparent loopback relay; observe actual collector/model round trips."""
    def __init__(self, model_port, fixture):
        from websockets.sync.server import serve
        self.fixture = fixture
        self.model_port = model_port
        self.records = []
        self.errors = []
        self.server = serve(self._handle, "127.0.0.1", PROXY_PORT,
                            compression=None, max_size=None)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def _handle(self, downstream):
        import numpy as np
        from openpi_client import msgpack_numpy
        from websockets.exceptions import ConnectionClosed
        from websockets.sync.client import connect
        try:
            with connect(f"ws://127.0.0.1:{self.model_port}", compression=None,
                         max_size=None, open_timeout=20) as upstream:
                downstream.send(upstream.recv())  # Official server metadata.
                for request in downstream:
                    phase = self.fixture.phase
                    before = time.monotonic()
                    master_status = self.fixture.mode_snapshot()
                    request_ros_time = self.fixture.rospy.Time.now().to_sec()
                    upstream.send(request)
                    response = upstream.recv()
                    downstream.send(response)
                    unpacked = msgpack_numpy.unpackb(response)
                    actions = np.asarray(unpacked["actions"])
                    observation = msgpack_numpy.unpackb(request)
                    state = np.asarray(observation["state"])
                    centers, rgb_correct = model_rgb_sentinels(observation["images"], phase)
                    self.records.append({
                        "phase": phase, "action_shape": list(actions.shape),
                        "finite": bool(np.isfinite(actions).all()),
                        "state_shape": list(state.shape),
                        "rgb_center": centers,
                        "rgb_sentinel_correct": rgb_correct,
                        "request_time": before,
                        "request_ros_time": request_ros_time,
                        "master_status": master_status,
                        "latency_ms": (time.monotonic() - before) * 1000,
                    })
        except ConnectionClosed:
            pass
        except Exception as exc:
            self.errors.append(repr(exc))

    def completed_in_phase(self, phase):
        return sum(record["phase"] == phase for record in self.records)

    def correct_rgb_in_phase(self, phase):
        return any(record["phase"] == phase and record["rgb_sentinel_correct"]
                   for record in self.records)


class FakeRosInputs:
    def __init__(self):
        import rospy
        from geometry_msgs.msg import Twist
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Bool, Int32, String
        self.rospy = rospy
        self.Image = Image
        self.JointState = JointState
        self.Twist = Twist
        self.phase = 0
        self.sequence = 0
        self.commands = Counter()
        self.command_errors = []
        self.control_values = {}
        self.state_lock = threading.Lock()
        self.master_states = {side: {"config": 0xFC, "teach": False, "enabled": True,
                                     "status_published_at": 0.0}
                              for side in ("left", "right")}
        self.pending_controls = {}
        self.control_events = []
        self.command_sources = []
        rospy.init_node("dagger_validation_fake_inputs", anonymous=True, disable_signals=True)
        self.image_pubs = [rospy.Publisher(f"/camera_{side}/color/image_raw", Image, queue_size=2)
                           for side in ("f", "l", "r")]
        self.joint_pubs = [rospy.Publisher(topic, JointState, queue_size=4) for topic in (
            "/puppet/joint_left", "/puppet/joint_right",
            "/puppet_master/joint_left", "/puppet_master/joint_right")]
        self.master_command_pubs = [rospy.Publisher(f"/master/joint_{side}", JointState, queue_size=4)
                                    for side in ("left", "right")]
        self.status_pubs = {side: rospy.Publisher(f"/teach/master_control_status_{side}", Twist, queue_size=4)
                            for side in ("left", "right")}
        self.mode_pubs = {side: rospy.Publisher(f"/teach/master_mode_status_{side}", String, queue_size=4)
                          for side in ("left", "right")}
        self.String = String
        self.subscribers = []
        for topic in ("/master/joint_left", "/master/joint_right",
                      "/master_controled/joint_left", "/master_controled/joint_right"):
            self.subscribers.append(rospy.Subscriber(topic, JointState, self._command, topic))
        for kind, msg_type in (("master_enable", Bool), ("master_config", String), ("teach_mode", Int32)):
            for side in ("left", "right"):
                topic = f"/teach/{kind}_{side}"
                self.subscribers.append(rospy.Subscriber(topic, msg_type, self._control, topic))
        self.timer = rospy.Timer(rospy.Duration(1 / 30), self._publish)

    def _command(self, message, topic):
        self.commands[topic] += 1
        self.command_sources.append({"topic": topic,
                                     "source": getattr(message, "_connection_header", {}).get("callerid", ""),
                                     "phase": self.phase, "stamp": message.header.stamp.to_sec(),
                                     "received_time": time.monotonic()})
        if len(message.position) != 7 or not all(math.isfinite(v) for v in message.position):
            self.command_errors.append(topic)

    def _control(self, message, topic):
        self.commands[topic] += 1
        self.control_values[topic] = message.data
        side = topic.rsplit("_", 1)[1]
        kind = "config" if "/master_config_" in topic else "teach" if "/teach_mode_" in topic else "enabled"
        value = ({"master": 0xFA, "slave": 0xFC}[message.data]
                 if kind == "config" else bool(message.data))
        with self.state_lock:
            self.control_events.append({"topic": topic, "value": message.data, "time": time.monotonic()})
            # Apply asynchronously to expose a blind resume without status waits.
            key = (side, kind)
            if key not in self.pending_controls or self.pending_controls[key][1] != value:
                self.pending_controls[key] = (time.monotonic() + 0.4, value)

    def mode_snapshot(self):
        with self.state_lock:
            return {side: dict(state) for side, state in self.master_states.items()}

    def _publish_master_status(self, stamp):
        with self.state_lock:
            now = time.monotonic()
            for (side, kind), (ready, value) in list(self.pending_controls.items()):
                if now >= ready:
                    self.master_states[side][kind] = value
                    if kind == "config":
                        self.master_states[side]["teach"] = value == 0xFA
                    del self.pending_controls[(side, kind)]
            for side, state in self.master_states.items():
                msg = self.Twist()
                msg.linear.x, msg.linear.y, msg.linear.z = 1.0, float(state["enabled"]), float(state["teach"])
                msg.angular.x, msg.angular.y = stamp.to_sec(), float(state["config"])
                self.status_pubs[side].publish(msg)
                self.mode_pubs[side].publish(self.String(data=f"0x{state['config']:02X}:synthetic"))
                state["status_published_at"] = stamp.to_sec()
            return {side: dict(state) for side, state in self.master_states.items()}

    def _publish(self, _event):
        import numpy as np
        self.sequence += 1
        stamp = self.rospy.Time.now()
        master_states = self._publish_master_status(stamp)
        # Publish joints first, with the exact same stamp as all three images.
        for index, publisher in enumerate(self.joint_pubs):
            msg = self.JointState()
            msg.header.stamp = stamp
            msg.header.seq = self.sequence
            msg.name = [f"joint{i}" for i in range(7)]
            msg.position = [0.04 * self.phase + 0.002 * math.sin(self.sequence / 7),
                            0.32, -0.36, 0.0, 0.24, 0.0, 0.07]
            if index % 2:
                msg.position[0] *= -1
            msg.velocity = [0.0] * 7
            msg.effort = [float(index + 1)] * 7
            publisher.publish(msg)
            if index >= 2:
                side = ("left", "right")[index - 2]
                if master_states[side]["config"] == 0xFA:
                    # Distinct ROS callerid from the collector's policy publisher.
                    self.master_command_pubs[index - 2].publish(msg)
        for index, publisher in enumerate(self.image_pubs):
            pixels = np.empty((240, 320, 3), dtype=np.uint8)
            pixels[:] = sentinel_rgb(index, self.phase)
            pixels[:8, :8] = self.sequence % 256
            msg = self.Image()
            msg.header.stamp = stamp
            msg.header.seq = self.sequence
            msg.height, msg.width, msg.encoding = 240, 320, "rgb8"
            msg.step = 320 * 3
            msg.data = pixels.tobytes()
            publisher.publish(msg)


def inspect_outputs(run_dir, report):
    import av
    import h5py
    import numpy as np
    episodes = []
    for path in sorted((run_dir / "synthetic_datasets").rglob("episode_*.hdf5")):
        # Label every synthetic file, including any produced by a failing run.
        with h5py.File(path, "r+") as root:
            root.attrs["validation_only"] = True
            root.attrs["training_allowed"] = False
            root.attrs["fake_ros_inputs"] = True
            root.attrs["hardware_io"] = False
            root.attrs["validation_master_port"] = MASTER_PORT
            rows = root["action"].shape[0]
            expected = {"observations/qpos": (rows, 14), "observations/qvel": (rows, 14),
                        "observations/effort": (rows, 14), "action": (rows, 14),
                        "base_action": (rows, 2)}
            for key, shape in expected.items():
                assert root[key].shape == shape, (path, key, root[key].shape)
                assert np.isfinite(root[key][:]).all(), (path, key, "non-finite")
            assert rows >= 5, (path, "too few frames", rows)
            qpos_median = float(np.median(root["observations/qpos"][:, 0]))
        videos = []
        episode_index = int(path.stem.split("_")[-1])
        phase = episode_index * 2 + int("_dagger_hdf5" in str(path))
        for camera_index, camera in enumerate(CAMERAS):
            video = path.parent / "video" / camera / (path.stem + ".mp4")
            decoded_colors = []
            with av.open(str(video)) as container:
                for frame in container.decode(video=0):
                    rgb = frame.to_ndarray(format="rgb24")
                    decoded_colors.append(np.median(rgb[80:160, 100:220], axis=(0, 1)))
            frame_count = len(decoded_colors)
            assert frame_count == rows, (video, frame_count, rows)
            expected_rgb = np.asarray(sentinel_rgb(camera_index, phase))
            rgb_median = np.median(decoded_colors, axis=0)
            matched = np.all(np.abs(np.asarray(decoded_colors) - expected_rgb) <= 10, axis=1)
            assert np.all(np.abs(rgb_median - expected_rgb) <= 10), (video, rgb_median, expected_rgb)
            # A first cached frame may precede r; the episode must chiefly contain
            # its own phase, with correct camera identity and RGB channel order.
            assert int(matched.sum()) >= math.ceil(rows / 2), (video, "stale or swapped RGB", matched.sum(), rows)
            videos.append({"path": str(video.relative_to(run_dir)), "frames": frame_count,
                           "rgb_median": rgb_median.tolist(), "expected_rgb": expected_rgb.tolist(),
                           "rgb_matching_frames": int(matched.sum()), "rgb_tolerance": 10})
        episodes.append({"path": str(path.relative_to(run_dir)), "rows": rows,
                         "qpos_j0_median": qpos_median, "videos": videos})
    report["episodes"] = episodes
    assert len(episodes) == 4, f"Expected two inference and two correction files, got {len(episodes)}"
    for kind in ("inference", "dagger"):
        selected = [e for e in episodes if f"_{kind}_hdf5/" in e["path"]]
        assert len(selected) == 2, (kind, selected)
        assert [Path(e["path"]).name for e in selected] == ["episode_0.hdf5", "episode_1.hdf5"]
        assert selected[1]["qpos_j0_median"] - selected[0]["qpos_j0_median"] > 0.04, (
            kind, "resumed recording reused old inputs", selected)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collector", type=Path,
                        default=HERE.parent / "collector-source" / "agilex_openpi_dagger_collect.py")
    parser.add_argument("--model-port", type=int, default=8014)
    parser.add_argument("--blocking-inference", action="store_true",
                        help="Use the official blocking chunk path instead of deployed temporal smoothing")
    args = parser.parse_args()
    collector_path = args.collector.resolve(strict=True)
    for port in (MASTER_PORT, PROXY_PORT):
        require_unused_port(port)
    run_dir = HERE / "runs" / datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S.%fZ")
    run_dir.mkdir(parents=True)
    (run_dir / "DO_NOT_TRAIN.txt").write_text(
        "SYNTHETIC VALIDATION INPUTS ONLY. Never use these files for training or demonstrations.\n")
    env = os.environ.copy()
    env.update(ROS_MASTER_URI=f"http://127.0.0.1:{MASTER_PORT}", ROS_IP="127.0.0.1",
               ROS_HOSTNAME="127.0.0.1", ROS_HOME=str(run_dir / "ros-home"),
               ROS_LOG_DIR=str(run_dir / "ros-logs"), PYTHONUNBUFFERED="1")
    os.environ.update({k: env[k] for k in ("ROS_MASTER_URI", "ROS_IP", "ROS_HOSTNAME", "ROS_HOME", "ROS_LOG_DIR")})
    report = {"hardware_io": False, "fake_ros_inputs": True, "training_allowed": False,
              "validation_only": True, "ros_master_uri": env["ROS_MASTER_URI"],
              "model_uri": f"ws://127.0.0.1:{args.model_port}", "cycles": 2,
              "collector": str(collector_path), "passed": False}
    master = terminal = fixture = relay = None
    print(f"Synthetic validation output: {run_dir}", flush=True)
    master_log = (run_dir / "roscore.log").open("w")
    try:
        master = subprocess.Popen(["roscore", "-p", str(MASTER_PORT)], env=env,
                                  stdout=master_log, stderr=subprocess.STDOUT, start_new_session=True)
        def master_ready():
            if master.poll() is not None:
                raise RuntimeError("Dedicated roscore exited; see roscore.log")
            try:
                return xmlrpc.client.ServerProxy(env["ROS_MASTER_URI"]).getPid("validation")[0] == 1
            except OSError:
                return False
        wait_until(master_ready, "Dedicated roscore did not start", timeout=20)
        fixture = FakeRosInputs()
        relay = ModelRelay(args.model_port, fixture)
        command = [sys.executable, "-u", str(collector_path), "--host", "127.0.0.1",
                   "--port", str(PROXY_PORT), "--ctrl_type", "joint", "--chunk_size", "50",
                   "--publish_rate", "30", "--inference_rate", "3", "--frame_rate", "30",
                   "--dataset_dir", str(run_dir / "synthetic_datasets"),
                   "--dataset_name", "VALIDATION_ONLY_DO_NOT_TRAIN", "--max_timesteps", "100000"]
        if not args.blocking_inference:
            command.append("--use_temporal_smoothing")
        report["command"] = command
        terminal = CollectorTerminal(command, env, run_dir / "collector.log")
        terminal.expect("Keyboard monitor thread started")
        cursor = 0
        report["handoff_checks"] = []
        for cycle in range(2):
            phase = cycle * 2
            wait_until(lambda: relay.completed_in_phase(phase) >= 2 and relay.correct_rgb_in_phase(phase),
                       f"No real model replies with correct RGB in inference phase {phase}", timeout=40)
            terminal.expect("Frame data:", after=cursor, timeout=15)
            mark = terminal.mark()
            terminal.key("d")
            terminal.expect("DAGGER MODE ACTIVE", after=mark, timeout=60)
            fixture.phase = phase + 1
            time.sleep(0.3)
            terminal.key(" ")
            terminal.expect("Data collection started.", after=mark)
            time.sleep(1.5)
            terminal.key("s")
            terminal.expect("Waiting for input (10s timeout)...", after=mark, timeout=30)
            terminal.key("y")
            terminal.expect("Save done. Collection ended.", after=mark, timeout=15)
            assert any(event["phase"] == phase + 1 and event["source"] == fixture.rospy.get_name()
                       and event["topic"].startswith("/master/joint_") for event in fixture.command_sources), (
                           "Fake master did not publish during takeover", phase + 1)
            fixture.phase = phase + 2
            cursor = terminal.mark()
            resume_time = time.monotonic()
            resume_ros_time = fixture.rospy.Time.now().to_sec()
            terminal.key("r")
            terminal.expect("DAGGER MODE DEACTIVATED - RESUMING INFERENCE", after=cursor, timeout=15)
            terminal.expect("Inference collection started.", after=cursor, timeout=15)
            wait_until(lambda: relay.completed_in_phase(phase + 2) >= 2 and relay.correct_rgb_in_phase(phase + 2),
                       f"No new model replies with correct RGB after resume {cycle + 1}", timeout=40)
            replies = [record for record in relay.records
                       if record["phase"] == phase + 2 and record["request_time"] >= resume_time]
            assert replies, (phase + 2, "No new model request after r")
            first_reply = replies[0]
            for record in replies:
                for side, state in record["master_status"].items():
                    assert state["config"] == 0xFC and not state["teach"], (
                        side, "Model resumed before master relinquished control", record)
                    assert resume_ros_time < state["status_published_at"] <= record["request_ros_time"], (
                        side, "Missing fresh status acknowledgement before model request", record)
            source_events = [event for event in fixture.command_sources
                             if event["topic"].startswith("/master/joint_")
                             and event["stamp"] >= first_reply["request_ros_time"]]
            assert not any(event["source"] == fixture.rospy.get_name() for event in source_events), (
                "Master still publishing after inference resumed", source_events)
            assert any(event["source"] != fixture.rospy.get_name() for event in source_events), (
                "Policy command publisher did not resume", source_events)
            last_master_received = max(event["received_time"] for event in fixture.command_sources
                                       if event["source"] == fixture.rospy.get_name()
                                       and event["topic"].startswith("/master/joint_")
                                       and event["received_time"] < first_reply["request_time"])
            quiet_seconds = first_reply["request_time"] - last_master_received
            assert quiet_seconds >= 0.2, ("No quiet interval before policy resumed", quiet_seconds)
            for side in ("left", "right"):
                for kind, value in (("master_config", "slave"),):
                    assert any(event["topic"] == f"/teach/{kind}_{side}" and event["value"] == value
                               and resume_time <= event["time"] < first_reply["request_time"]
                               for event in fixture.control_events), (side, kind, "Missing handoff command before model")
            report["handoff_checks"].append({"phase": phase + 2, "master_status_before_first_model": first_reply["master_status"],
                                               "master_commands_after_resume": 0,
                                               "master_command_quiet_seconds": quiet_seconds,
                                               "policy_commands_after_resume": len(source_events)})
            print(f"Cycle {cycle + 1}: d / Space / s / confirm / r passed", flush=True)
        stop_process(terminal.proc)
        for record in relay.records:
            assert record["action_shape"] == [50, 14] and record["finite"], record
            assert record["state_shape"] == [14], record
        for phase in (0, 2, 4):
            assert relay.correct_rgb_in_phase(phase), (phase, "No correctly mapped RGB model request")
        assert not relay.errors, relay.errors
        assert not fixture.command_errors, fixture.command_errors
        for topic in ("/master/joint_left", "/master/joint_right",
                      "/master_controled/joint_left", "/master_controled/joint_right"):
            assert fixture.commands[topic] > 0, (topic, "no command observed")
        for side in ("left", "right"):
            for name, expected in (("master_enable", True), ("master_config", "slave"), ("teach_mode", 1)):
                topic = f"/teach/{name}_{side}"
                assert fixture.control_values.get(topic) == expected, (topic, fixture.control_values)
            state = fixture.mode_snapshot()[side]
            assert state["config"] == 0xFC and not state["teach"], (side, state)
        inspect_outputs(run_dir, report)
        report["passed"] = True
    except Exception as exc:
        report["error"] = repr(exc)
        raise
    finally:
        if terminal is not None:
            terminal.close()
        if fixture is not None:
            fixture.timer.shutdown()
            fixture.rospy.signal_shutdown("Synthetic validation complete")
            report["command_counts"] = dict(fixture.commands)
            report["control_events"] = fixture.control_events
            report["command_sources"] = fixture.command_sources
        if relay is not None:
            relay.server.shutdown()
            report["model_replies"] = relay.records
            report["model_errors"] = relay.errors
        stop_process(master)
        # Apply labels even if an earlier validation assertion failed.
        import h5py
        for path in (run_dir / "synthetic_datasets").rglob("*.hdf5"):
            with h5py.File(path, "r+") as root:
                root.attrs.update(validation_only=True, training_allowed=False,
                                  fake_ros_inputs=True, hardware_io=False)
        (run_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        master_log.close()
        print(f"Report: {run_dir / 'report.json'} (passed={report['passed']})", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
