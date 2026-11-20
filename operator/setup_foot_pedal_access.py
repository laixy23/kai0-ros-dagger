#!/usr/bin/env python3
"""Grant rie read access only to the identified USB pedal's event interfaces.

Default is a read-only plan. --apply needs sudo, installs one serial-specific
udev rule and applies the same read-only group permissions to current nodes.
This script never opens CAN, starts DAgger, or writes USB/HID configuration.
"""
import argparse
import grp
import json
import os
from pathlib import Path
import subprocess
import tempfile

SERIAL = '864032454065'
VENDOR = '16c0'
PRODUCT = '05e2'
RULE_PATH = Path('/etc/udev/rules.d/99-kai0-foot-pedal.rules')
RULE = ('# Only the designated three-switch USB pedal; no general input access.\n'
        'SUBSYSTEM=="input", KERNEL=="event*", ATTRS{idVendor}=="16c0", '
        'ATTRS{idProduct}=="05e2", ATTRS{serial}=="864032454065", '
        'GROUP="rie", MODE="0640"\n')


def check_identity(properties):
    return (properties.get('ID_BUS') == 'usb'
            and properties.get('ID_VENDOR_ID', '').lower() == VENDOR
            and properties.get('ID_MODEL_ID', '').lower() == PRODUCT
            and properties.get('ID_SERIAL_SHORT') == SERIAL)


def properties(node):
    output = subprocess.run(['udevadm', 'info', '--query=property', '--name', str(node)],
                            check=True, capture_output=True, text=True).stdout
    return dict(line.split('=', 1) for line in output.splitlines() if '=' in line)


def discover():
    nodes = set()
    for alias in Path('/dev/input/by-id').glob('usb-USBYY.COM_USBKEY_' + SERIAL + '*event*'):
        node = alias.resolve(strict=True)
        if node.parent != Path('/dev/input') or not node.name.startswith('event'):
            raise RuntimeError('Unexpected device path: ' + str(node))
        if not check_identity(properties(node)):
            raise RuntimeError('USB pedal identity mismatch: ' + str(node))
        nodes.add(node)
    if not nodes:
        raise RuntimeError('Designated USB pedal is not connected')
    return sorted(nodes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    nodes = discover()
    if args.apply:
        if os.geteuid() != 0:
            raise SystemExit('Needs administrator access: sudo python3 ' + __file__ + ' --apply')
        group = grp.getgrnam('rie').gr_gid
        # Verify all targets before changing any permission or rule.
        for node in nodes:
            if not check_identity(properties(node)):
                raise RuntimeError('Pedal changed during setup')
        with tempfile.NamedTemporaryFile('w', dir=RULE_PATH.parent, delete=False) as stream:
            temporary = Path(stream.name)
            stream.write(RULE)
            stream.flush()
            os.fchmod(stream.fileno(), 0o644)
        os.replace(temporary, RULE_PATH)
        subprocess.run(['udevadm', 'control', '--reload-rules'], check=True)
        for node in nodes:
            # Apply to only this device; no global udev trigger or USB reset.
            if not check_identity(properties(node)):
                raise RuntimeError('Pedal disconnected during setup')
            descriptor = os.open(node, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
            try:
                os.fchown(descriptor, 0, group)
                os.fchmod(descriptor, 0o640)
            finally:
                os.close(descriptor)
    print(json.dumps({'applied': args.apply, 'device_serial': SERIAL,
                      'read_only_user': 'rie', 'event_nodes': list(map(str, nodes)),
                      'persistent_rule': str(RULE_PATH), 'rule': RULE,
                      'hardware_commands_sent': 0}, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
