#!/usr/bin/env python3
"""Add calibrated USB pedal input around the unchanged keyboard Bash entry.

The existing Bash/supervisor still owns all robot startup and shutdown. This
outer process owns only its child process group, PTY and private input pipes.
It never imports ROS/SDK, sends robot commands, or stops unrelated services.
"""
from __future__ import annotations

import argparse
import errno
import fcntl
import os
import pty
import select
import selectors
import signal
import subprocess
import sys
import termios
import time
import tty

from foot_pedal_device import FootPedalDevice, PedalGestures, load_config
from run_keyboard_session import drain_pty


def run_pedal_session(command, config, *, device=None, input_fd=0, output_fd=1):
    config = load_config(config)
    pedals = FootPedalDevice(config) if device is None else device
    gestures = PedalGestures(config['bindings'])
    process = None
    master_fd = slave_fd = command_read = command_write = ready_read = ready_write = None
    saved_terminal = None
    saved_handlers = {}
    requested_signal = None
    ready = False
    ready_seen_monotonic = None
    ready_buffer = b''
    error = None

    def emit(message):
        try:
            os.write(output_fd, message.encode())
        except OSError:
            pass

    def forward_signal(signum, _frame=None):
        nonlocal requested_signal
        if requested_signal is None:
            requested_signal = signum
        if process is not None and process.poll() is None:
            try:
                # Exactly the foreground group created for this Bash invocation.
                os.killpg(process.pid, signum)
            except ProcessLookupError:
                pass

    def deliver(tokens):
        for token in tokens:
            if requested_signal is not None:
                break
            if token == 'stop':
                emit('\n[踏板] 右踏板双踩：停止本轮并保存剩余 inference，不恢复推理。\n')
                forward_signal(signal.SIGINT)
            elif ready and token in ('left', 'save', 'resume', 'delete'):
                try:
                    os.write(command_write, token.encode('ascii') + b'\n')
                except (BrokenPipeError, BlockingIOError):
                    emit('\n[踏板] 操作未送达；键盘仍可用，右双踩仍可退出。\n')

    try:
        if not os.isatty(input_fd) or not os.isatty(output_fd):
            raise RuntimeError('Pedal DAgger requires an interactive terminal')
        pedals.open()  # Identify/grab this pedal before the old Bash can start arms.
        for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
            saved_handlers[signum] = signal.signal(signum, forward_signal)
        saved_terminal = termios.tcgetattr(input_fd)
        tty.setcbreak(input_fd)
        master_fd, slave_fd = pty.openpty()
        command_read, command_write = os.pipe()
        ready_read, ready_write = os.pipe()
        # The unchanged Bash entry owns fd9 for its round lock. Keep inherited
        # pedal endpoints away from that fixed descriptor, regardless of how
        # many event interfaces this device exposes.
        inherited_command = fcntl.fcntl(command_read, fcntl.F_DUPFD_CLOEXEC, 20)
        os.close(command_read)
        command_read = inherited_command
        inherited_ready = fcntl.fcntl(ready_write, fcntl.F_DUPFD_CLOEXEC, 20)
        os.close(ready_write)
        ready_write = inherited_ready
        os.set_blocking(command_write, False)
        os.set_blocking(ready_read, False)
        environment = dict(os.environ, DAGGER_PEDAL_COMMAND_FD=str(command_read),
                           DAGGER_PEDAL_READY_FD=str(ready_write))
        if requested_signal is not None:
            return 128 + requested_signal
        process = subprocess.Popen(command, stdin=slave_fd, stdout=slave_fd, stderr=slave_fd,
                                   start_new_session=True, close_fds=True,
                                   pass_fds=(command_read, ready_write), env=environment)
        for descriptor in (slave_fd, command_read, ready_write):
            os.close(descriptor)
        slave_fd = command_read = ready_write = None
        if requested_signal is not None:
            forward_signal(requested_signal)
        emit('[踏板] 左单踩示教/新段，中单踩保存、中双踩删除确认，右单踩恢复、右双踩退出。\n')
        emit('[踏板] 中/右单踩等待0.5秒，双踩不会先发单踩；准备期间仅右双踩和键盘 Ctrl+C 可退出。\n')
        with selectors.DefaultSelector() as selector:
            selector.register(input_fd, selectors.EVENT_READ, ('keyboard', None))
            selector.register(master_fd, selectors.EVENT_READ, ('child', None))
            selector.register(ready_read, selectors.EVENT_READ, ('ready', None))
            for descriptor in pedals.filenos:
                selector.register(descriptor, selectors.EVENT_READ, ('pedal', None))

            def read_pedal_events(fd):
                nonlocal gestures
                try:
                    return pedals.read_events(fd)
                except (OSError, RuntimeError) as exc:
                    for descriptor in pedals.filenos:
                        try:
                            selector.unregister(descriptor)
                        except KeyError:
                            pass
                    pedals.close()
                    gestures = PedalGestures(config['bindings'])
                    emit('\n[踏板] 输入已断开：' + str(exc) + '；键盘继续可用。\n')
                    return None

            while process.poll() is None:
                pedal_events = []
                for key, _ in selector.select(timeout=0.02):
                    kind = key.data[0]
                    if kind == 'pedal':
                        events = read_pedal_events(key.fd)
                        if events is None:
                            pedal_events.clear()
                            break
                        pedal_events.extend(events)
                        continue
                    try:
                        data = os.read(key.fd, 65536)
                    except OSError as exc:
                        if kind == 'child' and exc.errno == errno.EIO:
                            data = b''
                        else:
                            raise
                    if kind == 'ready':
                        if data:
                            ready_buffer += data
                            if b'ready\n' in ready_buffer:
                                ready = True
                                ready_seen_monotonic = time.monotonic()
                                emit('\n[踏板] 控制已就绪；保存确认期间单踩只确认保留。\n')
                                selector.unregister(key.fd)
                        else:
                            selector.unregister(key.fd)
                        continue
                    if not data:
                        if kind == 'keyboard':
                            selector.unregister(key.fd)
                            if requested_signal is None:
                                forward_signal(signal.SIGHUP)
                        else:
                            selector.unregister(key.fd)
                        continue
                    if kind == 'keyboard':
                        if requested_signal is None:
                            os.write(master_fd, data)
                    else:
                        os.write(output_fd, data)
                # Keyboard/child output may have taken time since selector's
                # snapshot. Read pedal input queued meanwhile before deciding
                # that the single-click window expired.
                pedal_fds = list(pedals.filenos)
                if pedal_fds:
                    for descriptor in select.select(pedal_fds, [], [], 0)[0]:
                        events = read_pedal_events(descriptor)
                        if events is None:
                            pedal_events.clear()
                            break
                        pedal_events.extend(events)
                # Process all device queues in kernel-time order before running
                # click timers. A delayed read must not split a real double
                # into two single actions, or turn a pre-READY press into a new command.
                processing_now = time.monotonic()
                pedal_events.sort(key=lambda event: event[4] if len(event) == 5 else processing_now)
                for event in pedal_events:
                    eligible = ready and (len(event) != 5 or event[4] >= ready_seen_monotonic)
                    deliver(gestures.feed(event, processing_now, allow_short=eligible))
                if requested_signal is None:
                    deliver(gestures.tick(time.monotonic()))
        code = process.wait()
        if code == 0 and requested_signal is not None:
            return 128 + requested_signal
        return 128 - code if code < 0 else code
    except (OSError, RuntimeError, ValueError) as exc:
        error = str(exc)
        emit('\n[踏板错误] ' + error + '\n')
        return 2
    finally:
        if process is not None and process.poll() is None:
            if requested_signal is None:
                forward_signal(signal.SIGINT)
            # Let the original owner perform its normal driver-first cleanup.
            # Keep exporting even if an outer input error led here. In
            # particular, do not fill its PTY or terminate a slow video export.
            # Another real Ctrl+C is still forwarded to the original owner.
            while process.poll() is None:
                drain_pty(master_fd, output_fd)
                time.sleep(0.02)
        drain_pty(master_fd, output_fd)
        pedals.close()
        for descriptor in (master_fd, slave_fd, command_read, command_write, ready_read, ready_write):
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except OSError:
                    pass
        if saved_terminal is not None:
            termios.tcsetattr(input_fd, termios.TCSADRAIN, saved_terminal)
        for signum, handler in saved_handlers.items():
            signal.signal(signum, handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command:
        parser.error('A Bash command after -- is required')
    try:
        return run_pedal_session(command, args.config)
    except (OSError, RuntimeError, ValueError) as exc:
        print('[踏板错误] ' + str(exc) + '\n请先运行 run_dagger_pedal.sh --calibrate。', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
