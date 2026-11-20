#!/usr/bin/env python3
"""Lifecycle tests with private PTYs, fake collectors and fake systemd only."""
import errno
import fcntl
import io
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from contextlib import redirect_stdout
from unittest import mock

import stop_arm_drivers as stop_module


class Systemd:
    def __init__(self, *, loaded=True, active=True, refuse_kill=False, refuse_freeze=False, group=""):
        self.loaded, self.active = loaded, active
        self.refuse_kill, self.refuse_freeze = refuse_kill, refuse_freeze
        self.group = group
        self.calls = []

    def __call__(self, command, **kwargs):
        self.calls.append(command)
        action = command[2]
        output, code = "", 0
        if action == "show":
            output = "LoadState=" + ("loaded" if self.loaded else "not-found") + "\n"
            output += "ActiveState=" + ("active" if self.active else "inactive") + "\n"
            output += "SubState=" + ("running" if self.active else "dead") + "\n"
            output += "MainPID=" + ("42" if self.active else "0") + "\n"
            output += "ControlGroup=" + self.group + "\n"
        elif action == "kill" and "--signal=SIGKILL" in command:
            if self.refuse_kill:
                code = 1
            else:
                self.active = False
        elif action == "kill" and self.refuse_freeze:
            code = 1
        return subprocess.CompletedProcess(command, code, output, "rejected" if code else "")


class StopServiceTests(unittest.TestCase):
    def test_paused_or_live_group_is_frozen_then_killed_before_normal_stop(self):
        service = Systemd()
        report = stop_module.stop_arm_drivers(runner=service, timeout=0)
        self.assertTrue(report["passed"], report)
        commands = service.calls
        freezes = next(i for i, cmd in enumerate(commands) if "--signal=SIGSTOP" in cmd)
        kills = next(i for i, cmd in enumerate(commands) if "--signal=SIGKILL" in cmd)
        stops = next(i for i, cmd in enumerate(commands) if cmd[2] == "stop")
        self.assertLess(freezes, kills)
        self.assertLess(kills, stops)
        self.assertFalse(any("SIGCONT" in " ".join(cmd) for cmd in commands))
        self.assertTrue(all(cmd[-1] == "kai0-official-arms.service" for cmd in commands))
        self.assertEqual(report["hardware_api_calls"], 0)

    def test_inactive_or_missing_unit_is_already_stopped_without_signals(self):
        for service in (Systemd(active=False), Systemd(loaded=False)):
            with self.subTest(loaded=service.loaded):
                report = stop_module.stop_arm_drivers(runner=service)
                self.assertTrue(report["passed"])
                self.assertEqual(len(service.calls), 1)

    def test_failed_kill_does_not_call_stop_which_could_sigcont_live_nodes(self):
        service = Systemd(refuse_kill=True)
        report = stop_module.stop_arm_drivers(runner=service, timeout=0)
        self.assertFalse(report["passed"])
        self.assertFalse(any(cmd[2] == "stop" for cmd in service.calls))

    def test_failed_freeze_still_attempts_kill_and_verifies_final_state(self):
        service = Systemd(refuse_freeze=True)
        report = stop_module.stop_arm_drivers(runner=service, timeout=0)
        self.assertTrue(report["passed"])
        self.assertTrue(report["errors"])

    def test_remaining_cgroup_member_prevents_success_or_normal_stop(self):
        service = Systemd(group="/fake-test-group")
        with mock.patch.object(Path, "read_text", return_value="33\n"):
            report = stop_module.stop_arm_drivers(runner=service, timeout=0)
        self.assertFalse(report["passed"])
        self.assertFalse(any(cmd[2] == "stop" for cmd in service.calls))

    def test_unavailable_systemd_fails_closed_without_trying_other_processes(self):
        calls = []
        def unavailable(command, **kwargs):
            calls.append(command)
            raise OSError("no user bus")
        report = stop_module.stop_arm_drivers(runner=unavailable)
        self.assertFalse(report["passed"])
        self.assertEqual(len(calls), 1)

    def test_human_cli_reports_success_without_json_and_failure_with_details(self):
        for passed in (True, False):
            output = io.StringIO()
            with mock.patch.object(sys, "argv", ["stop_arm_drivers.py", "--human"]), \
                    mock.patch.object(stop_module, "stop_arm_drivers", return_value={"passed": passed}), \
                    redirect_stdout(output):
                code = stop_module.main()
            self.assertEqual(code, 0 if passed else 2)
            if passed:
                self.assertIn("四臂 ROS 驱动和转发已停止", output.getvalue())
                self.assertNotIn('"passed"', output.getvalue())
            else:
                self.assertIn('"passed": false', output.getvalue())


class KeyboardSessionTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.log = self.root / "events.log"
        self.lock = self.root / "round.lock"
        self.pid = self.master = None
        self.output = b""

    def tearDown(self):
        if self.pid is not None:
            try:
                os.kill(self.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                pid, _ = os.waitpid(self.pid, os.WNOHANG)
                if pid:
                    break
                time.sleep(.02)
            else:
                os.kill(self.pid, signal.SIGKILL)
                os.waitpid(self.pid, 0)
        if self.master is not None:
            os.close(self.master)
        # Defensive cleanup of only the fake child recorded by this test.
        for line in self.events():
            if line.startswith("CHILD_PID="):
                try:
                    os.killpg(int(line.split("=")[1]), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self.temp.cleanup()

    def events(self):
        return self.log.read_text().splitlines() if self.log.exists() else []

    def wait_for(self, value, timeout=4):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if value in self.events():
                return
            self.read_output(.03)
        self.fail(f"Missing {value!r}; events={self.events()!r}; output={self.output!r}")

    def read_output(self, timeout=.03):
        if self.master is not None and select.select([self.master], [], [], timeout)[0]:
            try:
                self.output += os.read(self.master, 65536)
            except OSError as exc:
                if exc.errno != errno.EIO:
                    raise

    def launch(self, *, ignore_signals=False, missing_child=False, cleanup_ok=True,
               save_delay=None, grace=.15):
        fake = self.root / "fake_collector.py"
        fake.write_text('''import os, signal, sys, time, tty
logpath=sys.argv[1]
def log(value):
    with open(logpath, 'a') as out: out.write(value+'\\n')
tty.setcbreak(0)
log('CHILD_PID='+str(os.getpid()))
log('CHILD_TTY='+str(os.isatty(0)))
try:
    os.fstat(9)
    log('CHILD_FD9=True')
except OSError: log('CHILD_FD9=False')
def finish(signum, frame):
    log('CHILD_SIGNAL='+str(signum))
    if len(sys.argv)>3 and sys.argv[3]!='None':
        log('SAVE_BEGIN')
        # More than a PTY's buffer: an owner waiting without draining deadlocks.
        for _ in range(128): os.write(1,b'EXPORT_PROGRESS '+b'.'*4096+b'\\n')
        time.sleep(float(sys.argv[3]))
        log('SAVE_DONE')
        print('FINAL_INFERENCE_SAVED',flush=True)
    raise SystemExit(0)
for s in (signal.SIGINT,signal.SIGTERM):
    signal.signal(s, signal.SIG_IGN if sys.argv[2]=='ignore' else finish)
log('READY')
while True:
    key=os.read(0,1)
    if not key: break
    log('KEY='+repr(key))
    if key==b'x': break
''')
        launcher = self.root / "launcher.py"
        module_dir = Path(__file__).resolve().parent
        command = [sys.executable, str(fake if not missing_child else self.root / "missing.py"),
                   str(self.log), "ignore" if ignore_signals else "normal", str(save_delay)]
        launcher.write_text(f'''import fcntl, os, sys, time, termios
sys.path.insert(0,{str(module_dir)!r})
from run_keyboard_session import run_session
fd=os.open({str(self.lock)!r},os.O_CREAT|os.O_RDWR,0o600)
fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
os.dup2(fd,9,inheritable=True)
if fd!=9: os.close(fd)
def stop():
    with open({str(self.log)!r},'a') as out: out.write('DRIVERS_STOP_BEGIN\\n')
    time.sleep(.2)
    with open({str(self.log)!r},'a') as out: out.write('DRIVERS_STOP_END\\n')
    return {{'passed':{cleanup_ok!r}}}
before=termios.tcgetattr(0)
result=run_session({command!r},stop_drivers=stop,grace={grace!r},terminate_grace=.1)
with open({str(self.log)!r},'a') as out:
    after=termios.tcgetattr(0)
    # macOS can set this transient kernel input-reprint flag when switching
    # back to canonical mode. All configured modes/control chars must match.
    before[3]&=~getattr(termios,'PENDIN',0)
    after[3]&=~getattr(termios,'PENDIN',0)
    out.write('TERMINAL_RESTORED='+str(before==after)+'\\n')
raise SystemExit(result)
''')
        pid, master = pty.fork()
        if pid == 0:
            os.execv(sys.executable, [sys.executable, "-u", str(launcher)])
        self.pid, self.master = pid, master
        if not missing_child:
            self.wait_for("READY")

    def finish(self, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.read_output(.03)
            pid, status = os.waitpid(self.pid, os.WNOHANG)
            if pid:
                self.pid = None
                self.read_output(0)
                return os.waitstatus_to_exitcode(status)
        self.fail(f"Session did not stop; events={self.events()}, output={self.output!r}")

    def test_keys_remain_single_key_and_ctrl_c_stops_drivers_before_child(self):
        self.launch()
        os.write(self.master, b"d sw")
        self.wait_for("KEY=b'w'")
        os.write(self.master, b"\x03")  # The real terminal generates SIGINT.
        self.assertEqual(self.finish(), 130)
        events = self.events()
        self.assertIn("CHILD_TTY=True", events)
        self.assertIn("CHILD_FD9=False", events)
        self.assertEqual([e for e in events if e.startswith("KEY=")],
                         ["KEY=b'd'", "KEY=b' '", "KEY=b's'", "KEY=b'w'"])
        self.assertLess(events.index("DRIVERS_STOP_END"), events.index("CHILD_SIGNAL=2"))
        self.assertNotIn("KEY=b'r'", events)
        self.assertIn("TERMINAL_RESTORED=True", events)

    def test_term_stops_round_without_resume(self):
        self.launch()
        os.kill(self.pid, signal.SIGTERM)
        self.assertEqual(self.finish(), 143)
        self.assertNotIn("KEY=b'r'", self.events())

    def test_hup_cleans_up_without_resume(self):
        self.launch()
        os.kill(self.pid, signal.SIGHUP)
        self.assertEqual(self.finish(), 129)
        self.assertIn("DRIVERS_STOP_END", self.events())

    def test_child_start_failure_still_stops_claimed_driver(self):
        self.launch(missing_child=True)
        self.assertNotEqual(self.finish(), 0)
        self.assertIn("DRIVERS_STOP_END", self.events())

    def test_refusing_child_is_killed_after_forwarding_was_stopped(self):
        self.launch(ignore_signals=True)
        os.write(self.master, b"\x03")
        self.assertEqual(self.finish(), 130)
        self.assertIn("DRIVERS_STOP_END", self.events())
        self.assertFalse(any(e.startswith("CHILD_SIGNAL=") for e in self.events()))

    def test_cleanup_failure_is_not_reported_as_success(self):
        self.launch(cleanup_ok=False)
        os.write(self.master, b"\x03")
        self.assertEqual(self.finish(), 2)
        self.assertNotIn("[本轮结束]".encode(), self.output)

    def test_inherited_round_lock_held_through_cleanup_then_released(self):
        self.launch()
        os.write(self.master, b"\x03")
        self.wait_for("DRIVERS_STOP_BEGIN")
        with self.lock.open("r+") as fd:
            with self.assertRaises(BlockingIOError):
                fcntl.flock(fd.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.assertEqual(self.finish(), 130)
            fcntl.flock(fd.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)

    def test_normal_collector_exit_also_stops_drivers(self):
        self.launch()
        os.write(self.master, b"x")
        self.assertEqual(self.finish(), 0)
        self.assertIn("DRIVERS_STOP_END", self.events())

    def test_exit_save_past_old_eight_second_limit_drains_pty_and_finishes(self):
        self.launch(save_delay=8.2, grace=None)
        os.write(self.master, b"\x03")
        self.assertEqual(self.finish(timeout=12), 130)
        events = self.events()
        self.assertLess(events.index("DRIVERS_STOP_END"), events.index("SAVE_BEGIN"))
        self.assertEqual(events.count("CHILD_SIGNAL=2"), 1)
        self.assertEqual(events.count("SAVE_DONE"), 1)
        self.assertIn(b"FINAL_INFERENCE_SAVED", self.output)
        self.assertGreater(self.output.count(b"EXPORT_PROGRESS"), 100)
        self.assertIn("TERMINAL_RESTORED=True", events)

    def test_second_ctrl_c_can_force_exit_during_a_slow_save(self):
        self.launch(save_delay=30, grace=None)
        os.write(self.master, b"\x03")
        self.wait_for("SAVE_BEGIN")
        os.write(self.master, b"\x03")
        self.assertEqual(self.finish(timeout=4), 130, self.output[-3000:])
        events = self.events()
        self.assertLess(events.index("DRIVERS_STOP_END"), events.index("SAVE_BEGIN"))
        self.assertNotIn("SAVE_DONE", events)
        self.assertIn("TERMINAL_RESTORED=True", events)


if __name__ == "__main__":
    unittest.main()
