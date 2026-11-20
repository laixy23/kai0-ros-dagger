#!/usr/bin/env python3
"""Exercise the actual Bash wrapper with private PTYs and inert fake commands.

No ROS, systemd service, SDK, CAN socket, or real collector is started. Signals
target only the Bash owner, reproducing interruption during synchronous startup
without also interrupting its preparation child.
"""
from __future__ import annotations

import errno
import os
from pathlib import Path
import pty
import select
import signal
import sys
import tempfile
import time
import unittest


WRAPPER = Path(__file__).resolve().parents[1] / "run_dagger.sh"

FAKE_PYTHON = '''#!/usr/bin/env python3
import json
import os
from pathlib import Path
import sys
import time

root = Path(os.environ["DAGGER_ROOT"])
log = root / "events.log"

def record(event):
    with log.open("a") as stream:
        stream.write(event + "\\n")
    print(event, flush=True)

arguments = sys.argv[1:]
if arguments[0] == "-":
    sys.stdin.read()  # Consume only the wrapper's supplied Python heredoc.
    record("ownership_checked")
    raise SystemExit(2 if os.environ.get("FAKE_OWNERSHIP_FAILURE") else 0)

script = Path(arguments[0]).name
if script == "prepare_keyboard_session.py":
    record("prepare_started")
    if os.environ.get("FAKE_PREPARATION_FAILURE"):
        record("preparation_rejected_existing_owner")
        raise SystemExit(2)
    deadline = time.monotonic() + 5
    while not (root / "release_prepare").exists():
        if time.monotonic() >= deadline:
            record("prepare_timed_out")
            raise SystemExit(2)
        time.sleep(0.01)
    record("driver_started")
    raise SystemExit(0)
if script == "stop_arm_drivers.py":
    record("driver_cleanup")
    print(json.dumps({"passed": True}))
    raise SystemExit(0)
if script == "run_keyboard_session.py":
    record("collector_started")
    raise SystemExit(0)
if script == "preflight_v3_feedback.py":
    record("preflight_started")
    rows = {name: {"ctrl_mode": 1, "driver_enabled": [True] * 6}
            for name in ("can_left_slave", "can_right_slave", "can_left_mas", "can_right_mas")}
    print(json.dumps({"passed": True, "channels": rows}))
    raise SystemExit(0)
if script == "probe_official_topics.py":
    record("camera_probe_started")
    raise SystemExit(0)
raise SystemExit("Unexpected fake Python invocation: " + repr(arguments))
'''


class FakeRound:
    def __init__(self, **extra_environment):
        self.temp = tempfile.TemporaryDirectory(prefix="dagger-wrapper-signal-")
        self.root = Path(self.temp.name)
        self.pid = None
        self.fd = None
        self.status = None
        self.output = bytearray()
        for directory in (".ros-env/bin", "ros-launch", "operator", "bin"):
            (self.root / directory).mkdir(parents=True)
        python = self.root / ".ros-env/bin/python"
        python.write_text(FAKE_PYTHON)
        python.chmod(0o755)
        (self.root / "ros-launch/run_ros.sh").write_text('#!/usr/bin/env bash\nexec "$@"\n')
        (self.root / "ros-launch/run_official_collector.sh").write_text("#!/usr/bin/env bash\nexit 99\n")
        for name in ("prepare_keyboard_session.py", "stop_arm_drivers.py", "run_keyboard_session.py"):
            (self.root / "operator" / name).write_text("# Inert placeholder; the fake interpreter owns this path.\n")
        for name in ("systemctl", "flock"):
            command = self.root / "bin" / name
            command.write_text("#!/usr/bin/env bash\nexit 0\n")
            command.chmod(0o755)
        environment = os.environ.copy()
        environment.update(DAGGER_ROOT=str(self.root),
                           PATH=str(self.root / "bin") + os.pathsep + environment["PATH"],
                           PYTHONUNBUFFERED="1")
        environment.update(extra_environment)
        # Use the same Python installation that is running the test even if a
        # system python3 differs. Only the fake interpreter is resolved here.
        target_python = self.root / "bin/python3"
        target_python.symlink_to(sys.executable)
        pid, fd = pty.fork()
        if pid == 0:
            os.execvpe("bash", ["bash", str(WRAPPER)], environment)
        self.pid, self.fd = pid, fd

    def events(self):
        path = self.root / "events.log"
        return path.read_text().splitlines() if path.exists() else []

    def drain(self, duration=0.02):
        if self.fd is None or not select.select([self.fd], [], [], duration)[0]:
            return
        try:
            chunk = os.read(self.fd, 65536)
            self.output.extend(chunk)
        except OSError as exc:
            if exc.errno != errno.EIO:
                raise

    def wait_for_event(self, event, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.drain()
            if event in self.events():
                return
            self.poll()
            if self.status is not None:
                break
        raise AssertionError(f"Missing {event!r}; events={self.events()!r}; output={self.output.decode(errors='replace')!r}")

    def poll(self):
        if self.status is None:
            waited, status = os.waitpid(self.pid, os.WNOHANG)
            if waited:
                self.status = status
        return self.status

    def wait(self, timeout=5):
        deadline = time.monotonic() + timeout
        while self.poll() is None and time.monotonic() < deadline:
            self.drain()
        self.drain(0)
        if self.status is None:
            raise AssertionError(f"Wrapper remained alive; events={self.events()!r}")
        return os.waitstatus_to_exitcode(self.status)

    def close(self):
        if self.pid is not None and self.poll() is None:
            # The test owns a private session/group, containing only inert fakes.
            try:
                os.killpg(self.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            os.waitpid(self.pid, 0)
            self.status = 0
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        self.temp.cleanup()


class WrapperStartupSignalTests(unittest.TestCase):
    def setUp(self):
        self.round = None

    def tearDown(self):
        if self.round is not None:
            self.round.close()

    def check_signal_during_preparation(self, signum):
        self.round = FakeRound()
        self.round.wait_for_event("prepare_started")
        os.kill(self.round.pid, signum)  # Only Bash; not its preparation child.
        time.sleep(0.03)
        (self.round.root / "release_prepare").touch()
        self.assertEqual(self.round.wait(), 128 + signum)
        events = self.round.events()
        self.assertIn("driver_started", events, events)
        self.assertIn("driver_cleanup", events, events)
        self.assertLess(events.index("driver_started"), events.index("driver_cleanup"))
        self.assertNotIn("collector_started", events, events)
        self.assertNotIn("preflight_started", events, events)

    def test_term_to_bash_during_preparation_cleans_up_new_driver(self):
        self.check_signal_during_preparation(signal.SIGTERM)

    def test_hup_to_bash_during_preparation_cleans_up_new_driver(self):
        self.check_signal_during_preparation(signal.SIGHUP)

    def test_int_to_bash_during_preparation_cleans_up_new_driver(self):
        self.check_signal_during_preparation(signal.SIGINT)

    def test_existing_owner_at_initial_check_does_not_stop_unowned_driver(self):
        self.round = FakeRound(FAKE_OWNERSHIP_FAILURE="1")
        self.assertEqual(self.round.wait(), 2)
        self.assertEqual(self.round.events(), ["ownership_checked"])

    def test_preparation_ownership_failure_does_not_claim_or_stop_driver(self):
        self.round = FakeRound(FAKE_PREPARATION_FAILURE="1")
        self.assertEqual(self.round.wait(), 2)
        events = self.round.events()
        self.assertIn("preparation_rejected_existing_owner", events)
        self.assertNotIn("driver_started", events)
        self.assertNotIn("driver_cleanup", events)
        self.assertNotIn("collector_started", events)


if __name__ == "__main__":
    unittest.main()
