"""Run the opt-in Bash launcher only against inert temporary entry scripts."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


LAUNCHER = Path(__file__).resolve().parents[1] / 'run_dagger_gripper_handoff_test.sh'


class GripperTestLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix='dagger gripper launcher ')
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.trace = self.root / 'entry.json'
        for filename, kind in (('run_dagger.sh', 'keyboard'), ('run_dagger_pedal.sh', 'pedal')):
            script = self.root / filename
            script.write_text('''#!/usr/bin/env bash
exec "${TEST_LAUNCHER_PYTHON}" - ''' + kind + ''' "$@" <<'PY'
import json, os, pathlib, sys
pathlib.Path(os.environ['TEST_LAUNCHER_TRACE']).write_text(json.dumps({
    'kind': sys.argv[1], 'argv': sys.argv[2:],
    'opt_in': os.environ.get('DAGGER_TEST_MASTER_GRIPPER_RELEASE'),
    'root': os.environ.get('DAGGER_ROOT')}))
PY
''')
            script.chmod(0o755)

    def run_launcher(self, arguments=(), **extra_environment):
        environment = dict(os.environ, DAGGER_ROOT=str(self.root),
                           TEST_LAUNCHER_PYTHON=sys.executable,
                           TEST_LAUNCHER_TRACE=str(self.trace))
        environment.update(extra_environment)
        result = subprocess.run(['bash', str(LAUNCHER), *arguments],
                                env=environment, cwd=self.root,
                                capture_output=True, text=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def read_trace(self):
        return json.loads(self.trace.read_text())

    def assert_entry(self, kind, arguments):
        self.assertEqual(self.read_trace(), dict(kind=kind, argv=arguments,
                         opt_in='both', root=str(self.root)))

    def test_default_uses_original_keyboard_entry_with_explicit_both_opt_in(self):
        self.run_launcher()
        self.assert_entry('keyboard', [])

    def test_pedal_selection_uses_original_pedal_entry_without_forwarding_selector(self):
        self.run_launcher(['--pedal', '--check'])
        self.assert_entry('pedal', ['--check'])

    def test_arguments_and_spaces_pass_through_without_shell_reinterpretation(self):
        arguments = ['--dataset_name', 'trial with spaces', '--task_name',
                     'literal $(not-a-command); no expansion']
        self.run_launcher(arguments)
        self.assert_entry('keyboard', arguments)

    def test_stop_argument_is_forwarded_to_the_selected_original_entry(self):
        self.run_launcher(['--stop'])
        self.assert_entry('keyboard', ['--stop'])

    def test_named_test_entry_always_selects_both_even_with_an_ambient_override(self):
        for selection in ('left', 'right'):
            with self.subTest(selection=selection):
                self.run_launcher(DAGGER_TEST_MASTER_GRIPPER_RELEASE=selection)
                self.assert_entry('keyboard', [])

    def test_help_never_launches_either_original_entry(self):
        for flag in ('--help', '-h'):
            with self.subTest(flag=flag):
                result = self.run_launcher([flag])
                self.assertTrue(result.stdout.strip() or result.stderr.strip())
                self.assertFalse(self.trace.exists())


if __name__ == '__main__':
    unittest.main()
