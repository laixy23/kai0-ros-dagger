#!/usr/bin/env python3
"""Read and exclusively grab one identified USB pedal; never inject OS keys.

No ROS, SDK, uinput, or third-party evdev dependency is used. Gesture output is
plain tokens for the caller to interpret. Calibration and --test only print.
"""
from __future__ import annotations

import argparse
import datetime
import errno
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import select
import struct
import sys
import tempfile
import time


VENDOR = '16c0'
PRODUCT = '05e2'
SERIAL = '864032454065'
EV_KEY = 1
EV_SYN = 0
SYN_DROPPED = 3
EVIOCGRAB = 0x40044590  # _IOW('E', 0x90, int), Linux evdev.
EVIOCSCLOCKID = 0x400445A0  # _IOW('E', 0xa0, int).
INPUT_EVENT = struct.Struct('@llHHi')  # timeval, type, code, signed value.
PEDALS = ('left', 'middle', 'right')
SHORT = {'left': 'left', 'middle': 'save', 'right': 'resume'}
DOUBLE = {'middle': 'delete', 'right': 'stop'}
CALIBRATION_HOLD_S = 2.2
CALIBRATION_PRESSES = 3


def _hex_id(value):
    if type(value) is not str or len(value) != 4:
        raise ValueError('USB IDs must be four hexadecimal characters')
    int(value, 16)
    return value.lower()


def _bindings(value):
    if not isinstance(value, dict) or set(value) != set(PEDALS):
        raise ValueError('Bindings must contain exactly left, middle and right')
    result = {}
    for name in PEDALS:
        row = value[name]
        if (not isinstance(row, dict) or type(row.get('interface')) is not str
                or not row['interface'] or type(row.get('code')) is not int
                or not 1 <= row['code'] <= 0xFFFF):
            raise ValueError('Each binding requires a stable interface and EV_KEY code')
        result[name] = {'interface': row['interface'], 'code': row['code']}
    if len({(row['interface'], row['code']) for row in result.values()}) != 3:
        raise ValueError('The three pedal bindings must be unique')
    return result


def load_config(config):
    """Load/validate config without opening any input device."""
    if isinstance(config, (str, os.PathLike)):
        config = json.loads(Path(config).read_text())
    if not isinstance(config, dict) or config.get('version') != 1:
        raise ValueError('Unsupported pedal config version')
    device = config.get('device', {})
    if (not isinstance(device, dict) or _hex_id(device.get('vendor')) != VENDOR
            or _hex_id(device.get('product')) != PRODUCT or device.get('serial') != SERIAL):
        raise ValueError('Config must identify USB 16c0:05e2 serial 864032454065')
    return dict(config, device=dict(vendor=VENDOR, product=PRODUCT, serial=SERIAL),
                bindings=_bindings(config.get('bindings')))


def _read_text(path):
    try:
        return path.read_text().strip()
    except (OSError, UnicodeError):
        return ''


def discover_interfaces(*, sysfs_root='/sys/class/input', dev_root='/dev/input',
                        vendor=VENDOR, product=PRODUCT, serial=SERIAL):
    """Find every evdev interface belonging to this exact physical USB device.

    Interface IDs exclude changing event/input numbers and USB bus-port paths.
    Capabilities distinguish keyboard/mouse/consumer collections exposed by the
    same USB interface. Duplicate identities are rejected rather than guessed.
    """
    vendor, product = _hex_id(vendor), _hex_id(product)
    rows = []
    for entry in sorted(Path(sysfs_root).glob('event*')):
        if not entry.name[5:].isdigit():
            continue
        event = entry.resolve()
        ancestors = list(event.parents)
        usb = next((parent for parent in ancestors
                    if (parent / 'idVendor').is_file() and (parent / 'idProduct').is_file()), None)
        if usb is None or (_read_text(usb / 'idVendor').lower(),
                           _read_text(usb / 'idProduct').lower(),
                           _read_text(usb / 'serial')) != (vendor, product, serial):
            continue
        interface_dir = next((parent for parent in ancestors if (parent / 'bInterfaceNumber').is_file()), None)
        if interface_dir is None:
            raise RuntimeError('Identified pedal input has no USB interface number: ' + str(entry))
        number = _read_text(interface_dir / 'bInterfaceNumber')
        try:
            number = f'{int(number, 16):02x}'
        except ValueError as exc:
            raise RuntimeError('Invalid USB interface number for ' + str(entry)) from exc
        input_dir = event.parent
        phys, name = _read_text(input_dir / 'phys'), _read_text(input_dir / 'name')
        # Linux phys ends in /inputN; retain only that collection suffix.
        collection = phys.rsplit('/', 1)[-1] if '/input' in phys else 'collection'
        capabilities = '\n'.join(_read_text(input_dir / 'capabilities' / key)
                                 for key in ('ev', 'key', 'rel', 'abs'))
        fingerprint = hashlib.sha256((name + '\n' + capabilities).encode()).hexdigest()[:12]
        identity = f'usb-if{number}/{collection}/{fingerprint}'
        rows.append(dict(interface=identity, path=str(Path(dev_root) / entry.name), name=name, phys=phys))
    if len({row['interface'] for row in rows}) != len(rows):
        raise RuntimeError('Ambiguous pedal input collections; refusing unstable bindings')
    return rows


class FootPedalDevice:
    """Explicit open/context manager, fd→interface mapping, and raw event reads."""
    def __init__(self, config=None, *, vendor=VENDOR, product=PRODUCT, serial=SERIAL,
                 sysfs_root='/sys/class/input', dev_root='/dev/input'):
        if (_hex_id(vendor), _hex_id(product), serial) != (VENDOR, PRODUCT, SERIAL):
            raise ValueError('Only the explicitly identified pedal is supported')
        self.config = load_config(config) if config is not None else None
        self._sysfs_root, self._dev_root = sysfs_root, dev_root
        self._fds = {}
        self._buffers = {}
        self._grabbed = set()
        self.interfaces = []

    @property
    def filenos(self):
        return dict(self._fds)

    def open(self):
        if self._fds:
            return self
        self.interfaces = discover_interfaces(sysfs_root=self._sysfs_root, dev_root=self._dev_root)
        if not self.interfaces:
            raise FileNotFoundError('No USB pedal 16c0:05e2 serial 864032454065 input interfaces found')
        if self.config is not None:
            available = {row['interface'] for row in self.interfaces}
            if not all(row['interface'] in available for row in self.config['bindings'].values()):
                raise RuntimeError('Calibrated pedal interface is absent; recalibration is required')
        try:
            for row in self.interfaces:
                fd = os.open(row['path'], os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
                self._fds[fd] = row['interface']
                self._buffers[fd] = b''
                # All native timestamps must share the gesture timer's clock.
                # Failing this ioctl must never silently select wall time.
                fcntl.ioctl(fd, EVIOCSCLOCKID, struct.pack('@i', time.CLOCK_MONOTONIC))
            for fd in self._fds:
                fcntl.ioctl(fd, EVIOCGRAB, 1)
                self._grabbed.add(fd)
        except BaseException:
            self.close()
            raise
        return self

    def read_events(self, fd):
        if fd not in self._fds:
            raise ValueError('Unknown pedal input descriptor')
        buffer = self._buffers[fd]
        while True:
            try:
                chunk = os.read(fd, INPUT_EVENT.size * 128)
            except OSError as exc:
                if exc.errno == errno.EINTR:
                    continue
                if isinstance(exc, BlockingIOError) or exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                    break
                raise
            if not chunk:
                raise OSError(errno.ENODEV, 'Pedal input disconnected')
            buffer += chunk
        whole = len(buffer) // INPUT_EVENT.size * INPUT_EVENT.size
        self._buffers[fd] = buffer[whole:]
        result = []
        for offset in range(0, whole, INPUT_EVENT.size):
            sec, usec, kind, code, value = INPUT_EVENT.unpack_from(buffer, offset)
            if sec < 0 or not 0 <= usec < 1_000_000:
                raise RuntimeError('Invalid monotonic kernel input-event timestamp')
            result.append((self._fds[fd], kind, code, value, sec + usec / 1_000_000))
        return result

    def close(self):
        for fd in tuple(self._fds):
            if fd in self._grabbed:
                try:
                    fcntl.ioctl(fd, EVIOCGRAB, 0)
                except OSError:
                    pass
            try:
                os.close(fd)
            except OSError:
                pass
        self._grabbed.clear()
        self._fds.clear()
        self._buffers.clear()

    def __enter__(self):
        return self.open()

    def __exit__(self, *_exc):
        self.close()


class PedalGestures:
    """Count complete clicks; physical hold duration never selects an action.

    Left emits on release. Middle/right singles wait for the double-click
    window; a second press inside it cancels the single until its release.
    Overlapping pedals cancel ordinary actions, but a complete right double
    click still stops, including before the collector's readiness handshake.
    """
    def __init__(self, bindings, double_click_window_s=0.5):
        self.bindings = _bindings(bindings)
        if (type(double_click_window_s) not in (int, float)
                or not math.isfinite(double_click_window_s) or double_click_window_s <= 0):
            raise ValueError('Invalid double-click window')
        self.double_click_window_s = double_click_window_s
        self._lookup = {(row['interface'], row['code']): name for name, row in self.bindings.items()}
        self._pressed = {}
        self._pending = {}
        self._chord = False
        self._last_now = None
        self._last_source = {}
        self._syn_dropped = set()

    def _clock(self, now):
        if (type(now) not in (int, float) or not math.isfinite(now)
                or self._last_now is not None and now < self._last_now):
            raise ValueError('Gesture clock must be finite and monotonic')
        self._last_now = now

    def _due_singles(self, now):
        result = []
        for name, pending in sorted(list(self._pending.items()),
                                    key=lambda item: item[1]['released_at']):
            # Equality belongs to the second press. The live reader drains
            # queued native events before running this timer.
            if now > pending['released_at'] + self.double_click_window_s:
                self._pending.pop(name)
                if pending['eligible']:
                    result.append(SHORT[name])
        return result

    def feed(self, event, now, *, allow_short=True):
        if len(event) == 5:
            interface, kind, code, value, source = event
            if (type(source) not in (int, float) or not math.isfinite(source) or source < 0
                    or source < self._last_source.get(interface, source)):
                raise ValueError('Input source time must be finite and monotonic per interface')
            self._last_source[interface] = source
            now = source  # Processing latency must not change click grouping.
        elif len(event) == 4:
            interface, kind, code, value = event
            self._clock(now)  # Legacy fake/manual sources use the caller's clock.
        else:
            raise ValueError('Expected four legacy fields or five source-timestamped fields')
        if type(allow_short) is not bool:
            raise ValueError('allow_short must be boolean')
        if kind == EV_SYN and code == SYN_DROPPED:
            self._pressed.clear()
            self._pending.clear()
            self._chord = False
            self._syn_dropped.add(interface)
            return []
        if interface in self._syn_dropped:
            if kind == EV_SYN and code == 0:  # SYN_REPORT ends the lost-event interval.
                self._syn_dropped.remove(interface)
            return []
        result = self._due_singles(now)
        name = self._lookup.get((interface, code)) if kind == EV_KEY else None
        if name is None or value == 2:
            return result
        if value == 1:
            if name not in self._pressed:
                pending = self._pending.pop(name, None)
                double = pending is not None
                eligible = allow_short and (pending is None or pending['eligible'])
                if self._pressed:
                    self._chord = True
                    for held in self._pressed.values():
                        held['eligible'] = False
                    # Ordinary queued singles must not escape a chord later.
                    for queued_name in list(self._pending):
                        if queued_name == 'right':
                            self._pending[queued_name]['eligible'] = False
                        else:
                            self._pending.pop(queued_name)
                    eligible = False
                self._pressed[name] = {'double': double, 'eligible': eligible}
            return result
        if value != 0 or name not in self._pressed:
            return result
        pressed = self._pressed.pop(name)
        eligible = pressed['eligible'] and not self._chord
        if pressed['double']:
            if name == 'right':
                self._pending.clear()
                result = ['stop']
            elif name == 'middle' and eligible:
                result.append(DOUBLE[name])
        elif name == 'left':
            if eligible:
                result.append(SHORT[name])
        else:
            # Retain an ineligible first click so a pair straddling READY cannot
            # become an ordinary action. Right can still form its stop double.
            self._pending[name] = {'released_at': now, 'eligible': eligible}
        if not self._pressed:
            self._chord = False
        return result

    def tick(self, now):
        self._clock(now)
        return self._due_singles(now)


def calibrate(device, timeout=90.0, *, clock=time.monotonic, select_fn=select.select, emit=print):
    """Identify each pedal from three complete, matching press/release pairs."""
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('Calibration timeout must be positive')
    deadline = clock() + timeout
    bindings, counts, durations = {}, {}, {}
    pressed, released, dropped_interfaces = {}, [], set()
    last_source = {}
    candidate, count, measured = None, 0, []
    chord = False

    def prompt():
        pedal = PEDALS[len(bindings)]
        label = {'left': '左', 'middle': '中', 'right': '右'}[pedal]
        emit(f'请只踩{label}踏板 {CALIBRATION_PRESSES} 次，每次松开；不要求长按，其余踏板不要按。')

    prompt()
    while len(bindings) < len(PEDALS) and clock() < deadline:
        ready, _, _ = select_fn(list(device.filenos), [], [], min(0.05, max(0, deadline - clock())))
        batch = [event for fd in ready for event in device.read_events(fd)]
        processing_now = clock()
        batch.sort(key=lambda row: row[4] if len(row) == 5 else processing_now)
        for event in batch:
            if len(event) == 5:
                interface, kind, code, value, now = event
                if (type(now) not in (int, float) or not math.isfinite(now) or now < 0
                        or now < last_source.get(interface, now)):
                    raise ValueError('Calibration requires valid monotonic source timestamps')
                last_source[interface] = now
            elif len(event) == 4:  # Compatibility for explicit fake devices.
                interface, kind, code, value = event
                now = clock()
            else:
                raise ValueError('Invalid calibration input event')
            if kind == EV_SYN and code == SYN_DROPPED:
                pressed.clear()
                released.clear()
                candidate, count, measured, chord = None, 0, [], False
                dropped_interfaces.add(interface)
                emit('输入事件丢失，当前踏板重新计数，请重新按。')
                continue
            if interface in dropped_interfaces:
                if kind == EV_SYN and code == 0:
                    dropped_interfaces.remove(interface)
                continue
            if kind != EV_KEY or value == 2:
                continue
            key = (interface, code)
            if value == 1 and key not in pressed:
                chord = chord or bool(pressed)
                pressed[key] = now
                emit(f'keydown interface={interface} code={code}')
            elif value == 0 and key in pressed:
                duration = now - pressed.pop(key)
                released.append((key, duration))
                emit(f'keyup interface={interface} code={code} hold={duration:.3f}s')
                if pressed:
                    continue
                if chord or len(released) != 1:
                    candidate, count, measured = None, 0, []
                    emit('多键同时按压，当前踏板重新计数。')
                elif key in {(row['interface'], row['code']) for row in bindings.values()}:
                    candidate, count, measured = None, 0, []
                    emit('与已校准踏板重复，请踩当前提示的另一块，重新计数。')
                else:
                    if key != candidate:
                        if candidate is not None:
                            emit('键码改变，当前踏板重新计数。')
                        candidate, count, measured = key, 0, []
                    count += 1
                    measured.append(duration)
                    pedal = PEDALS[len(bindings)]
                    emit(f'{pedal}: {count}/{CALIBRATION_PRESSES}')
                    if count == CALIBRATION_PRESSES:
                        bindings[pedal] = {'interface': interface, 'code': code}
                        counts[pedal] = count
                        durations[pedal] = measured
                        candidate, count, measured = None, 0, []
                        if len(bindings) < len(PEDALS):
                            prompt()
                released.clear()
                chord = False
                if len(bindings) == len(PEDALS):
                    break
    if len(bindings) < len(PEDALS):
        raise TimeoutError('Calibration timed out before three presses per pedal were verified')
    return load_config({'version': 1, 'device': dict(vendor=VENDOR, product=PRODUCT, serial=SERIAL),
                        'bindings': bindings,
                        'calibration': {'method': 'press_count',
                                        'required_presses': CALIBRATION_PRESSES,
                                        'measured_press_counts': counts,
                                        'measured_press_durations_s': durations,
                                        'created_at_utc': datetime.datetime.now(datetime.timezone.utc).isoformat()}})


def write_config(path, config):
    """Only persist a completed calibration, atomically and with private access."""
    config = load_config(config)
    calibration = config.get('calibration', {})
    if calibration.get('method') == 'press_count':
        counts = calibration.get('measured_press_counts', {})
        if (calibration.get('required_presses') != CALIBRATION_PRESSES
                or not all(type(counts.get(name)) is int and counts[name] >= CALIBRATION_PRESSES
                           for name in PEDALS)):
            raise ValueError('All three pedals must have three verified press/release pairs')
    else:  # Existing verified bindings remain usable; only new calibration changes.
        holds = calibration.get('measured_hold_seconds', {})
        if not all(type(holds.get(name)) in (int, float) and math.isfinite(holds[name])
                   and holds[name] >= CALIBRATION_HOLD_S for name in PEDALS):
            raise ValueError('A completed press-count or legacy hold calibration is required')
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=path.name + '.', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as output:
            json.dump(config, output, indent=2)
            output.write('\n')
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--calibrate', metavar='PATH')
    mode.add_argument('--test', metavar='PATH')
    parser.add_argument('--timeout', type=float, default=90.0)
    args = parser.parse_args(argv)
    try:
        if args.calibrate:
            with FootPedalDevice() as device:
                for row in device.interfaces:
                    print('grabbed', row['interface'], row['path'], row['name'], flush=True)
                config = calibrate(device, args.timeout, emit=lambda line: print(line, flush=True))
            write_config(args.calibrate, config)
            print('已保存真实校准配置：', args.calibrate)
        else:
            config = load_config(args.test)
            gestures = PedalGestures(config['bindings'])
            with FootPedalDevice(config) as device:
                print('仅打印踏板动作，不发送机器人命令；Ctrl+C退出。', flush=True)
                while True:
                    ready, _, _ = select.select(list(device.filenos), [], [], 0.02)
                    events = [event for fd in ready for event in device.read_events(fd)]
                    processing_now = time.monotonic()
                    events.sort(key=lambda event: event[4] if len(event) == 5 else processing_now)
                    for event in events:
                        for token in gestures.feed(event, processing_now):
                            print(token, flush=True)
                    for token in gestures.tick(time.monotonic()):
                        print(token, flush=True)
    except KeyboardInterrupt:
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        print(f'Pedal error: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
