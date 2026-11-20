"""Check that pedal access setup never grants broad keyboard/device access."""
import contextlib
import io
import unittest
from unittest.mock import patch

import setup_foot_pedal_access as access


class FootPedalAccessTests(unittest.TestCase):
    def properties(self):
        return {'ID_BUS': 'usb', 'ID_VENDOR_ID': '16c0', 'ID_MODEL_ID': '05e2',
                'ID_SERIAL_SHORT': '864032454065'}

    def test_only_the_designated_usb_identity_matches(self):
        self.assertTrue(access.check_identity(self.properties()))
        for key, value in [('ID_BUS', 'bluetooth'), ('ID_VENDOR_ID', '09da'),
                           ('ID_MODEL_ID', '6402'), ('ID_SERIAL_SHORT', 'another-pedal')]:
            with self.subTest(key=key):
                self.assertFalse(access.check_identity(dict(self.properties(), **{key: value})))

    def test_plan_does_not_write_rules_or_change_device_permissions(self):
        with patch.object(access, 'discover', return_value=['/dev/input/event23']), \
                patch.object(access.os, 'fchown') as owner, \
                patch.object(access.os, 'fchmod') as mode, \
                patch.object(access.os, 'replace') as replace, \
                patch.object(access.subprocess, 'run') as run, \
                patch('sys.argv', ['setup_foot_pedal_access.py']), \
                contextlib.redirect_stdout(io.StringIO()):
            access.main()
        owner.assert_not_called()
        mode.assert_not_called()
        replace.assert_not_called()
        run.assert_not_called()

    def test_apply_refuses_without_administrator_before_any_change(self):
        with patch.object(access, 'discover', return_value=['/dev/input/event23']), \
                patch.object(access.os, 'geteuid', return_value=1000), \
                patch.object(access.os, 'replace') as replace, \
                patch.object(access.os, 'fchmod') as mode, \
                patch('sys.argv', ['setup_foot_pedal_access.py', '--apply']):
            with self.assertRaisesRegex(SystemExit, 'administrator'):
                access.main()
        replace.assert_not_called()
        mode.assert_not_called()


if __name__ == '__main__':
    unittest.main()
