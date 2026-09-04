#!/usr/bin/env python3
"""Push a Pybricks program to the SPIKE Prime hub over Bluetooth LE.

Usage:
    python3 push_ble.py [script.py]

Defaults to update_robot_code_pybricks.py. Streams the hub's print() output
back over BLE; press Ctrl-C to stop the program on the hub.

Why this wrapper exists: this machine's python3 is 3.9, which pins pybricksdev
to an old release (1.0.0a49). Two things in it are stale versus current
Pybricks firmware, so we monkeypatch them before handing off to the real CLI:

  1. find_device() refuses a hub whose BLE advert has no name; macOS often
     never surfaces one, so we match purely on the Pybricks service UUID.
  2. unpack_hub_capabilities() hard-codes a 10-byte layout; newer firmware
     appends a byte, so we parse the first 10 and ignore the rest.

To retire this wrapper: install Python 3.11+ and `pip install pybricksdev`,
then just use `pybricksdev run ble <script.py>`.

Note: press the hub's center button to stop any running program before each
push, otherwise the hub stays connected and won't advertise for discovery.
"""
import asyncio
import struct
import sys

import pybricksdev.ble as ble
import pybricksdev.ble.pybricks as pb
from bleak import BleakScanner


async def patched_find(name=None, service=ble.PYBRICKS_SERVICE_UUID, timeout=15):
    def match(device, adv):
        return service in adv.service_uuids

    dev = await BleakScanner.find_device_by_filter(
        match, timeout, service_uuids=[service]
    )
    if dev is None:
        raise asyncio.TimeoutError
    print("found hub:", dev.address)
    return dev


ble.find_device = patched_find
import pybricksdev.cli as cli  # noqa: E402

cli.find_device = patched_find

_HubCapabilityFlag = pb.HubCapabilityFlag


def patched_caps(data):
    max_char_size, flags, max_user_prog_size = struct.unpack("<HII", data[:10])
    return max_char_size, _HubCapabilityFlag(flags), max_user_prog_size


pb.unpack_hub_capabilities = patched_caps
import pybricksdev.connections.pybricks as cpb  # noqa: E402

cpb.unpack_hub_capabilities = patched_caps

script = sys.argv[1] if len(sys.argv) > 1 else "update_robot_code_pybricks.py"

from pybricksdev.cli import main  # noqa: E402

sys.argv = ["pybricksdev", "run", "ble", script]
main()
