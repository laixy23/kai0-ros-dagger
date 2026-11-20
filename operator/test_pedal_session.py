"""Real nested PTY/private-FD integration, using only fake input and collectors.

The inner owner receives an explicit stop stub; its default stop implementation
is replaced by a function which raises before it could access any service.
No ROS, CAN, real input device or original robot Bash entry is executed.
"""
import errno
import json
import os
from pathlib import Path
import pty
import select
import signal
import sys
import tempfile
import textwrap
import time
import unittest


MODULE_DIR = Path(__file__).resolve().parent
BINDINGS = {'left': {'interface': 'keyboard', 'code': 30},
            'middle': {'interface': 'mouse', 'code': 272},
            'right': {'interface': 'consumer', 'code': 164}}


COLLECTOR = r'''
import os, selectors, signal, stat, sys, time, tty
from pathlib import Path
folder=Path(sys.argv[1])
def log(value):
    with (folder/'events.log').open('a') as output: output.write(value+'\n')
def finish(signum, _frame):
    log('COLLECTOR_SIGNAL='+str(signum))
    if (folder/'save-delay').exists():
        log('SAVE_BEGIN')
        for _ in range(128): os.write(1,b'EXPORT_PROGRESS '+b'.'*4096+b'\n')
        time.sleep(float((folder/'save-delay').read_text()))
        log('SAVE_DONE')
        print('FINAL_INFERENCE_SAVED',flush=True)
    raise SystemExit(0)
for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    signal.signal(signum, finish)
command_fd=int(os.environ['DAGGER_PEDAL_COMMAND_FD'])
ready_fd=int(os.environ['DAGGER_PEDAL_READY_FD'])
log('COLLECTOR_PID='+str(os.getpid()))
log('COLLECTOR_TTY='+str(os.isatty(0)))
log('PRIVATE_FDS_VALID='+str(command_fd>=20 and ready_fd>=20 and
    stat.S_ISFIFO(os.fstat(command_fd).st_mode) and stat.S_ISFIFO(os.fstat(ready_fd).st_mode)))
log('FD9_IS_FIFO='+str(stat.S_ISFIFO(os.fstat(9).st_mode)) if os.path.exists('/proc/self/fd/9') else 'FD9_NOT_INHERITED')
tty.setcbreak(0)
log('COLLECTOR_STARTED')
sent_ready=False
pending=b''
with selectors.DefaultSelector() as selector:
    selector.register(0,selectors.EVENT_READ,'keyboard')
    selector.register(command_fd,selectors.EVENT_READ,'command')
    while True:
        if not sent_ready and (folder/'allow-ready').exists():
            os.write(ready_fd,b'ready\n')
            os.close(ready_fd)
            sent_ready=True
            log('COLLECTOR_READY')
        for key,_ in selector.select(timeout=.01):
            data=os.read(key.fd,4096)
            if not data:
                selector.unregister(key.fd)
                continue
            if key.data=='command':
                pending+=data
                while b'\n' in pending:
                    token,pending=pending.split(b'\n',1)
                    log('COMMAND='+token.decode('ascii'))
            else:
                for value in data:
                    byte=bytes([value])
                    log('STDIN='+repr(byte))
                    if byte==b'x':
                        log('COLLECTOR_NORMAL_EXIT')
                        raise SystemExit(0)
'''


OWNER = r'''
import os, sys, time, types
from pathlib import Path
folder=Path(sys.argv[1])
sys.path.insert(0,sys.argv[2])
def forbidden_default(*args,**kwargs):
    raise AssertionError('Real stop_drivers must never run in this test')
sys.modules['stop_arm_drivers']=types.SimpleNamespace(stop_arm_drivers=forbidden_default)
from run_keyboard_session import run_session
def log(value):
    with (folder/'events.log').open('a') as output: output.write(value+'\n')
def stop_stub():
    log('DRIVERS_STOP_BEGIN')
    time.sleep(.08)
    log('DRIVERS_STOP_END')
    return {'passed':True}
log('OWNER_PID='+str(os.getpid()))
raise SystemExit(run_session([sys.executable,'-u',str(folder/'collector.py'),str(folder)],
    stop_drivers=stop_stub,grace=None,terminate_grace=.1))
'''


OUTER = r'''
import json, os, sys, termios, time
from pathlib import Path
folder=Path(sys.argv[1])
sys.path.insert(0,sys.argv[2])
from run_pedal_session import run_pedal_session
# Linux evdev and its supervisor share CLOCK_MONOTONIC. macOS Python 3.9's
# time.monotonic epoch differs by process; use the explicit shared POSIX clock
# in this fake launcher to model the Linux source-timestamp contract.
time.monotonic=lambda: time.clock_gettime(time.CLOCK_MONOTONIC)
def log(value):
    with (folder/'events.log').open('a') as output: output.write(value+'\n')
import run_pedal_session as session_module
BaseSelector=session_module.selectors.DefaultSelector
class SnapshotSelector:
    def __init__(self): self.inner=BaseSelector()
    def __getattr__(self,name): return getattr(self.inner,name)
    def __enter__(self):
        self.inner.__enter__()
        return self
    def __exit__(self,*args): return self.inner.__exit__(*args)
    def select(self,timeout=None):
        result=self.inner.select(timeout)
        marker=folder/'after-snapshot-input.json'
        if marker.exists():
            events=json.loads(marker.read_text())
            marker.unlink()
            fd=os.open(folder/'events.fifo',os.O_WRONLY|os.O_NONBLOCK)
            os.write(fd,('\n'.join(json.dumps(row) for row in events)+'\n').encode())
            os.close(fd)
            log('AFTER_SNAPSHOT_INPUT_QUEUED')
            time.sleep(.6)
        return result
session_module.selectors.DefaultSelector=SnapshotSelector
class FakePedal:
    def __init__(self):
        self.fd=None
        self.buffer=b''
        self.closed_fd=None
    @property
    def filenos(self): return {} if self.fd is None else {self.fd:'fake-json-input'}
    def open(self):
        self.fd=os.open(folder/'events.fifo',os.O_RDONLY|os.O_NONBLOCK)
        log('GRABS_OPEN')
        return self
    def read_events(self,fd):
        data=os.read(fd,65536)
        if not data: raise OSError('fake pedal disconnected')
        self.buffer+=data
        rows=[]
        while b'\n' in self.buffer:
            line,self.buffer=self.buffer.split(b'\n',1)
            value=json.loads(line)
            rows.append(tuple(value))
            log('EVENT_READ='+line.decode())
        return rows
    def close(self):
        if self.fd is not None:
            self.closed_fd=self.fd
            os.close(self.fd)
            self.fd=None
            log('GRABS_RELEASED')
pedal=FakePedal()
before=termios.tcgetattr(0)
code=run_pedal_session(['/bin/bash',str(folder/'old-bash.sh'),str(folder),sys.executable,sys.argv[2]],
    str(folder/'config.json'),device=pedal)
after=termios.tcgetattr(0)
before[3]&=~getattr(termios,'PENDIN',0)
after[3]&=~getattr(termios,'PENDIN',0)
log('TERMINAL_RESTORED='+str(before==after))
try:
    os.fstat(pedal.closed_fd)
except OSError: log('DEVICE_FD_CLOSED=True')
else: log('DEVICE_FD_CLOSED=False')
log('OUTER_EXIT='+str(code))
raise SystemExit(code)
'''


OLD_BASH = r'''
set -eu
folder=$1
python=$2
module_dir=$3
exec 9>"$folder/round.lock"
printf 'PREP_PID=%s\nPREP_READY\n' "$$" >>"$folder/events.log"
prep_stop() {
    printf 'PREP_SIGNAL=2\nDRIVERS_STOP_BEGIN\nDRIVERS_STOP_END\n' >>"$folder/events.log"
    exit 130
}
trap prep_stop INT
while [[ ! -e "$folder/allow-owner" ]]; do sleep .02; done
trap - INT
exec "$python" -u "$folder/owner.py" "$folder" "$module_dir"
'''


@unittest.skipUnless(hasattr(pty, 'fork'), 'Requires POSIX PTYs')
class PedalPTYIntegrationTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.log = self.root / 'events.log'
        self.pid = self.master = self.event_writer = None
        self.output = b''
        for name, source in (('collector.py', COLLECTOR), ('owner.py', OWNER),
                             ('outer.py', OUTER), ('old-bash.sh', OLD_BASH)):
            (self.root / name).write_text(textwrap.dedent(source))
        (self.root / 'config.json').write_text(json.dumps({
            'version': 1, 'device': {'vendor': '16c0', 'product': '05e2', 'serial': '864032454065'},
            'bindings': BINDINGS}))
        os.mkfifo(self.root / 'events.fifo', 0o600)
        self.event_writer = os.open(self.root / 'events.fifo', os.O_RDWR | os.O_NONBLOCK)

    def tearDown(self):
        if self.pid is not None:
            try:
                os.kill(self.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                found, _ = os.waitpid(self.pid, os.WNOHANG)
                if found:
                    break
                time.sleep(.02)
            else:
                os.kill(self.pid, signal.SIGKILL)
                os.waitpid(self.pid, 0)
        # Only process groups recorded by these known fake scripts are cleaned.
        for line in self.events():
            if line.startswith(('PREP_PID=', 'OWNER_PID=', 'COLLECTOR_PID=')):
                try:
                    os.killpg(int(line.split('=')[1]), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        for descriptor in (self.master, self.event_writer):
            if descriptor is not None:
                os.close(descriptor)
        self.temp.cleanup()

    def events(self):
        return self.log.read_text().splitlines() if self.log.exists() else []

    def read_output(self, timeout=.02):
        if self.master is not None and select.select([self.master], [], [], timeout)[0]:
            try:
                self.output += os.read(self.master, 65536)
            except OSError as exc:
                if exc.errno != errno.EIO:
                    raise

    def wait_for(self, value, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if value in self.events():
                return
            self.read_output(.02)
        self.fail(f'Missing {value!r}; events={self.events()}; output={self.output!r}')

    def wait_output(self, value, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if value.encode() in self.output:
                return
            self.read_output(.02)
        self.fail(f'Missing output {value!r}; events={self.events()}; output={self.output!r}')

    def settle(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.read_output(min(.02, deadline - time.monotonic()))

    def launch(self, *, owner=True, ready=False):
        if owner:
            (self.root / 'allow-owner').touch()
        if ready:
            (self.root / 'allow-ready').touch()
        pid, master = pty.fork()
        if pid == 0:
            os.close(self.event_writer)
            os.execv(sys.executable, [sys.executable, '-u', str(self.root / 'outer.py'),
                                     str(self.root), str(MODULE_DIR)])
        self.pid, self.master = pid, master
        self.wait_for('PREP_READY')
        if owner:
            self.wait_for('COLLECTOR_STARTED')
            self.assertIn('PRIVATE_FDS_VALID=True', self.events())
            self.assertIn('COLLECTOR_TTY=True', self.events())
        if ready:
            self.wait_output('[踏板] 控制已就绪')

    def pedal(self, name, value):
        binding = BINDINGS[name]
        line = json.dumps([binding['interface'], 1, binding['code'], value])
        os.write(self.event_writer, (line + '\n').encode())
        self.wait_for('EVENT_READ=' + line)

    def short(self, name):
        self.pedal(name, 1)
        time.sleep(.035)
        self.pedal(name, 0)

    def double(self, name):
        self.short(name)
        time.sleep(.035)
        self.short(name)

    def source_batch(self, name, down_at, up_at):
        binding = BINDINGS[name]
        lines = [json.dumps([binding['interface'], 1, binding['code'], value, stamp])
                 for value, stamp in ((1, down_at), (0, up_at))]
        os.write(self.event_writer, ('\n'.join(lines) + '\n').encode())
        for line in lines:
            self.wait_for('EVENT_READ=' + line)

    def source_double_batch(self, name, first_down, gap=.1, duration=.1):
        binding = BINDINGS[name]
        values = ((1, first_down), (0, first_down + duration),
                  (1, first_down + duration + gap),
                  (0, first_down + 2 * duration + gap))
        lines = [json.dumps([binding['interface'], 1, binding['code'], value, stamp])
                 for value, stamp in values]
        os.write(self.event_writer, ('\n'.join(lines) + '\n').encode())
        for line in lines:
            self.wait_for('EVENT_READ=' + line)

    def finish(self, timeout=6):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.read_output(.02)
            found, status = os.waitpid(self.pid, os.WNOHANG)
            if found:
                self.pid = None
                self.read_output(0)
                return os.waitstatus_to_exitcode(status)
        self.fail(f'Session did not stop; events={self.events()}, output={self.output!r}')

    def assert_clean(self):
        self.assertEqual(self.events().count('GRABS_RELEASED'), 1)
        self.assertIn('DEVICE_FD_CLOSED=True', self.events())
        self.assertIn('TERMINAL_RESTORED=True', self.events())

    def assert_stop_order(self):
        events = self.events()
        self.assertLess(events.index('DRIVERS_STOP_END'), events.index('COLLECTOR_SIGNAL=2'))
        self.assertNotIn('COMMAND=resume', events)
        self.assertNotIn("STDIN=b'r'", events)

    def test_pre_ready_and_cross_ready_presses_are_not_queued_and_keyboard_ctrl_c_cleans_up(self):
        self.launch()
        self.short('middle')
        self.pedal('left', 1)
        (self.root / 'allow-ready').touch()
        self.wait_output('[踏板] 控制已就绪')
        self.pedal('left', 0)
        self.read_output(.05)
        self.assertFalse(any(line.startswith('COMMAND=') for line in self.events()))
        os.write(self.master, b'\x03')
        self.assertEqual(self.finish(), 130)
        self.assert_stop_order()
        self.assert_clean()

    def test_ready_tokens_cross_bash_fd9_and_inner_owner_only_via_private_pipe(self):
        self.launch(ready=True)
        for name, token in (('left', 'left'), ('middle', 'save'), ('right', 'resume')):
            self.short(name)
            self.wait_for('COMMAND=' + token)
        self.double('middle')
        self.wait_for('COMMAND=delete')
        self.read_output(.05)
        self.assertEqual([line for line in self.events() if line.startswith('COMMAND=')],
                         ['COMMAND=left', 'COMMAND=save', 'COMMAND=resume', 'COMMAND=delete'])
        self.assertFalse(any(line.startswith('STDIN=') for line in self.events()))
        os.write(self.master, b'kx')
        self.assertEqual(self.finish(), 0)
        self.assertEqual([line for line in self.events() if line.startswith('STDIN=')],
                         ["STDIN=b'k'", "STDIN=b'x'"])
        self.assertIn('DRIVERS_STOP_END', self.events())
        self.assert_clean()

    def test_right_double_before_collector_ready_stops_owner_without_resume(self):
        self.launch()
        self.double('right')
        self.assertEqual(self.finish(timeout=4), 130)
        self.assert_stop_order()
        self.assertFalse(any(line.startswith('COMMAND=') for line in self.events()))
        self.assert_clean()

    def test_right_double_during_bash_preparation_stops_even_before_inner_owner_exists(self):
        self.launch(owner=False)
        self.double('right')
        self.assertEqual(self.finish(timeout=4), 130)
        self.assertIn('PREP_SIGNAL=2', self.events())
        self.assertIn('DRIVERS_STOP_END', self.events())
        self.assertNotIn('COLLECTOR_STARTED', self.events())
        self.assert_clean()

    def test_ready_overlapping_pedals_do_not_send_single_delete_or_resume(self):
        self.launch(ready=True)
        self.pedal('middle', 1)
        self.pedal('right', 1)
        self.pedal('right', 0)
        self.pedal('middle', 0)
        self.settle(.7)
        self.assertFalse(any(line.startswith('COMMAND=') for line in self.events()))
        os.write(self.master, b'\x03')
        self.assertEqual(self.finish(), 130)
        self.assert_stop_order()
        self.assertFalse(any(line.startswith('COMMAND=') for line in self.events()))
        self.assert_clean()

    def test_delayed_native_double_batch_stops_immediately_without_a_resume_token(self):
        self.launch(ready=True)
        processing = time.clock_gettime(time.CLOCK_MONOTONIC)
        self.source_double_batch('right', processing - .4)
        self.assertEqual(self.finish(timeout=2), 130)
        self.assert_stop_order()
        self.assertFalse(any(line.startswith('COMMAND=') for line in self.events()))
        self.assert_clean()

    def test_second_click_queued_after_selector_snapshot_is_read_before_single_timer(self):
        self.launch(ready=True)
        self.short('right')
        source = time.clock_gettime(time.CLOCK_MONOTONIC)
        binding = BINDINGS['right']
        events = [[binding['interface'], 1, binding['code'], value, stamp]
                  for value, stamp in ((1, source), (0, source + .01))]
        (self.root / 'after-snapshot-input.json').write_text(json.dumps(events))
        self.wait_for('AFTER_SNAPSHOT_INPUT_QUEUED')
        self.assertEqual(self.finish(timeout=3), 130)
        self.assert_stop_order()
        self.assertFalse(any(line.startswith('COMMAND=') for line in self.events()))
        self.assert_clean()

    def test_native_short_queued_before_ready_is_not_reclassified_when_processed_after_ready(self):
        self.launch()
        queued_before_ready = time.clock_gettime(time.CLOCK_MONOTONIC)
        (self.root / 'allow-ready').touch()
        self.wait_output('[踏板] 控制已就绪')
        self.source_batch('left', queued_before_ready - 0.2, queued_before_ready - 0.1)
        self.read_output(.08)
        self.assertFalse(any(line.startswith('COMMAND=') for line in self.events()))
        # A genuine post-READY source pair still works through the same pipe.
        pressed = time.clock_gettime(time.CLOCK_MONOTONIC)
        time.sleep(.04)
        self.source_batch('left', pressed, time.clock_gettime(time.CLOCK_MONOTONIC))
        self.wait_for('COMMAND=left')
        self.assertEqual([line for line in self.events() if line.startswith('COMMAND=')], ['COMMAND=left'])
        os.write(self.master, b'x')
        self.assertEqual(self.finish(), 0)
        self.assert_clean()

    def test_middle_double_crossing_ready_does_not_delete_or_confirm(self):
        self.launch()
        self.short('middle')
        (self.root / 'allow-ready').touch()
        self.wait_output('[踏板] 控制已就绪')
        self.short('middle')
        self.settle(.65)
        self.assertFalse(any(line.startswith('COMMAND=') for line in self.events()))
        os.write(self.master, b'x')
        self.assertEqual(self.finish(), 0)
        self.assert_clean()

    def test_single_right_waits_for_double_window_and_does_not_stop(self):
        self.launch(ready=True)
        self.short('right')
        self.settle(.15)
        self.assertNotIn('COMMAND=resume', self.events())
        self.assertNotIn('DRIVERS_STOP_BEGIN', self.events())
        self.wait_for('COMMAND=resume')
        self.assertNotIn('DRIVERS_STOP_BEGIN', self.events())
        os.write(self.master, b'x')
        self.assertEqual(self.finish(), 0)
        self.assert_clean()

    def test_right_double_with_held_middle_only_stops_without_other_commands(self):
        self.launch(ready=True)
        self.pedal('middle', 1)
        self.double('right')
        self.assertEqual(self.finish(), 130)
        self.assert_stop_order()
        self.assertFalse(any(line.startswith('COMMAND=') for line in self.events()))
        self.assert_clean()

    def test_pedal_stop_waits_for_nested_inference_export_and_forwards_progress(self):
        (self.root / 'save-delay').write_text('.5')
        self.launch(ready=True)
        processing = time.clock_gettime(time.CLOCK_MONOTONIC)
        self.source_double_batch('right', processing - .4)
        self.assertEqual(self.finish(), 130)
        self.assert_stop_order()
        self.assertEqual(self.events().count('SAVE_DONE'), 1)
        self.assertIn(b'FINAL_INFERENCE_SAVED', self.output)
        self.assertGreater(self.output.count(b'EXPORT_PROGRESS'), 100)
        self.assert_clean()

    def test_second_keyboard_ctrl_c_reaches_inner_owner_during_pedal_exit_save(self):
        (self.root / 'save-delay').write_text('30')
        self.launch(ready=True)
        processing = time.clock_gettime(time.CLOCK_MONOTONIC)
        self.source_double_batch('right', processing - .4)
        self.wait_for('SAVE_BEGIN')
        os.write(self.master, b'\x03')
        self.assertEqual(self.finish(timeout=4), 130, self.output[-3000:])
        self.assert_stop_order()
        self.assertNotIn('SAVE_DONE', self.events())
        self.assert_clean()


class InheritedFDTests(unittest.TestCase):
    def test_optional_pipe_endpoints_are_real_distinct_fifos_and_absent_env_is_unchanged(self):
        from run_keyboard_session import inherited_pedal_fds
        command_read, command_write = os.pipe()
        ready_read, ready_write = os.pipe()
        try:
            environ = {'DAGGER_PEDAL_COMMAND_FD': str(command_read), 'DAGGER_PEDAL_READY_FD': str(ready_write)}
            self.assertEqual(inherited_pedal_fds(environ), (command_read, ready_write))
            self.assertEqual(inherited_pedal_fds({}), ())
            for invalid in ({'DAGGER_PEDAL_COMMAND_FD': str(command_read)},
                            dict(environ, DAGGER_PEDAL_READY_FD=str(command_read)),
                            dict(environ, DAGGER_PEDAL_READY_FD='0'),
                            dict(environ, DAGGER_PEDAL_READY_FD='not-an-fd')):
                with self.subTest(invalid=invalid), self.assertRaises(RuntimeError):
                    inherited_pedal_fds(invalid)
            with tempfile.TemporaryFile() as regular:
                with self.assertRaises(RuntimeError):
                    inherited_pedal_fds(dict(environ, DAGGER_PEDAL_READY_FD=str(regular.fileno())))
        finally:
            for descriptor in (command_read, command_write, ready_read, ready_write):
                os.close(descriptor)


if __name__ == '__main__':
    unittest.main()
