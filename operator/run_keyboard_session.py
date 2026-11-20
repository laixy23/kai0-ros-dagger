#!/usr/bin/env python3
"""Own one keyboard DAgger round, its terminal and managed arm-driver lifetime.

The collector has a private PTY/session, so its normal single-key interface is
preserved. Ctrl+C stops the arm service first, then the collector; it never sends
the 'r' key or requests a control-mode change. The Bash wrapper's inherited
flock descriptor remains held in this supervisor until cleanup completes.
"""
from __future__ import annotations

import argparse
import errno
import os
import pty
import select
import selectors
import signal
import stat
import subprocess
import sys
import termios
import time
import tty

from stop_arm_drivers import stop_arm_drivers


def inherited_pedal_fds(environ=None):
    """Preserve only the optional pedal launcher's private pipe endpoints."""
    environ = os.environ if environ is None else environ
    names = ('DAGGER_PEDAL_COMMAND_FD', 'DAGGER_PEDAL_READY_FD')
    if not any(name in environ for name in names):
        return ()
    try:
        descriptors = tuple(int(environ[name]) for name in names)
        if len(set(descriptors)) != 2 or any(fd < 3 or not stat.S_ISFIFO(os.fstat(fd).st_mode)
                                           for fd in descriptors):
            raise ValueError('Expected two distinct private pipe endpoints')
    except (KeyError, ValueError, OSError) as exc:
        raise RuntimeError('Invalid pedal control pipe descriptors') from exc
    return descriptors


def drain_pty(master_fd, output_fd):
    """Forward pending child output without waiting for another line or key."""
    if master_fd is None:
        return
    # Bound each drain so a noisy child cannot prevent checking a second Ctrl+C.
    for _ in range(64):
        if not select.select([master_fd], [], [], 0)[0]:
            return
        try:
            data = os.read(master_fd, 65536)
        except OSError as exc:
            if exc.errno in (errno.EIO, errno.EBADF):
                return
            raise
        if not data:
            return
        try:
            while data:
                data = data[os.write(output_fd, data):]
        except OSError:
            # A closed terminal must not stop us draining the owned child.
            pass


def stop_collector(proc, *, force_requested=lambda: False, grace=None, terminate_grace=3.0,
                   drain_output=lambda: None):
    """Let the stopped round finish saving; a second interrupt forces exit.

    An explicit grace is retained for callers testing/refusing collectors. The
    normal entry has no export deadline after the arm drivers have stopped.
    """
    if proc is None:
        return
    for signum, duration in ((signal.SIGINT, grace), (signal.SIGTERM, terminate_grace),
                             (signal.SIGKILL, 3.0)):
        if proc.poll() is not None:
            drain_output()
            return
        actual_signal = signal.SIGKILL if force_requested() else signum
        try:
            os.killpg(proc.pid, actual_signal)
        except ProcessLookupError:
            pass
        if actual_signal == signal.SIGKILL:
            duration = 3.0
        deadline = None if duration is None else time.monotonic() + duration
        while proc.poll() is None and (deadline is None or time.monotonic() < deadline):
            drain_output()
            if force_requested() and actual_signal != signal.SIGKILL:
                break
            time.sleep(0.02)
        drain_output()
        if actual_signal == signal.SIGKILL:
            break
    if proc.poll() is None:
        raise RuntimeError("Owned collector did not exit after KILL")


def run_session(command, *, stop_drivers=stop_arm_drivers, input_fd=0, output_fd=1,
                grace=None, terminate_grace=3.0):
    process = None
    master_fd = slave_fd = None
    old_terminal = None
    old_handlers = {}
    requested_signal = None
    signal_count = 0
    driver_report = None
    exit_code = 1
    error = None
    pedal_fds = ()

    def emit(value):
        try:
            os.write(output_fd, value.encode())
        except OSError:
            pass

    def on_signal(signum, _frame):
        nonlocal requested_signal, signal_count
        signal_count += 1
        if requested_signal is None:
            requested_signal = signum

    try:
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            old_handlers[signum] = signal.signal(signum, on_signal)
        if not os.isatty(input_fd) or not os.isatty(output_fd):
            raise RuntimeError("Keyboard DAgger requires an interactive terminal")
        old_terminal = termios.tcgetattr(input_fd)
        tty.setcbreak(input_fd)  # Keep ISIG: real-terminal Ctrl+C reaches this owner.
        master_fd, slave_fd = pty.openpty()
        pedal_fds = inherited_pedal_fds()
        process = subprocess.Popen(command, stdin=slave_fd, stdout=slave_fd, stderr=slave_fd,
                                   start_new_session=True, close_fds=True, pass_fds=pedal_fds)
        for descriptor in pedal_fds:
            os.close(descriptor)
        pedal_fds = ()
        os.close(slave_fd)
        slave_fd = None
        with selectors.DefaultSelector() as selector:
            selector.register(input_fd, selectors.EVENT_READ, "keyboard")
            selector.register(master_fd, selectors.EVENT_READ, "collector")
            while requested_signal is None and process.poll() is None:
                for key, _ in selector.select(timeout=0.05):
                    if requested_signal is not None:
                        break
                    try:
                        data = os.read(key.fd, 65536)
                    except OSError as exc:
                        if key.data == "collector" and exc.errno == errno.EIO:
                            data = b""
                        else:
                            raise
                    if not data:
                        if key.data == "keyboard":
                            requested_signal = signal.SIGHUP
                        else:
                            selector.unregister(master_fd)
                        continue
                    if key.data == "keyboard" and requested_signal is None:
                        os.write(master_fd, data)
                    elif key.data == "collector":
                        os.write(output_fd, data)
        exit_code = 128 + requested_signal if requested_signal else (process.poll() or 0)
    except Exception as exc:
        error = repr(exc)
        exit_code = 1
    finally:
        emit("\n[停止] 停止本轮和四臂 ROS 转发，不恢复推理……\n")
        # Stop both policy-to-follower callbacks and independent master
        # forwarding BEFORE waiting for data export or collector termination.
        try:
            driver_report = stop_drivers()
        except Exception as exc:
            driver_report = {"passed": False, "error": repr(exc)}
        try:
            if process is not None and process.poll() is None:
                emit("[保存] 等待采集器保存剩余 inference；再次按 Ctrl+C 可强制退出。\n")
            stop_collector(process, force_requested=lambda: signal_count > 1,
                           grace=grace, terminate_grace=terminate_grace,
                           drain_output=lambda: drain_pty(master_fd, output_fd))
        except Exception as exc:
            error = repr(exc)
            exit_code = 2
        for fd in (master_fd, slave_fd, *pedal_fds):
            if fd is not None:
                os.close(fd)
        if old_terminal is not None:
            try:
                termios.tcsetattr(input_fd, termios.TCSADRAIN, old_terminal)
            except OSError:
                pass
        for signum, handler in old_handlers.items():
            signal.signal(signum, handler)
        if not driver_report or not driver_report.get("passed"):
            emit("[错误] 四臂 ROS 驱动退出未确认；请检查服务状态，异常运动时使用现场急停。\n")
            emit(str(driver_report) + "\n")
            exit_code = 2
        elif process is None or process.poll() is not None:
            emit("[本轮结束] 采集器和四臂 ROS 转发已停止；退出时未使能/失能、未归位、未恢复推理。\n")
        if error:
            emit("[错误] " + error + "\n")
    return exit_code


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--owns-arms", action="store_true", required=True,
                        help="Caller has claimed the managed arm service for this round")
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("A collector command after -- is required")
    return run_session(command)


if __name__ == "__main__":
    raise SystemExit(main())
