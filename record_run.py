#!/usr/bin/env python3
"""Controller: keeps one persistent BLE connection to the hub and swaps
which Pybricks program is running on it, without ever disconnecting.

Usage:
    python3 record_run.py

On the hub, running update_robot_code_pybricks.py:
  - RIGHT (at home, "H") starts a recording, CENTER stops it. Each one is
    saved here as runs/run_<timestamp>.csv.
  - LEFT counts up (shown on the display), RIGHT counts back down to home.
  - CENTER, with a number selected, replays runs/<N>_run_*.csv -- rename a
    run file yourself to give it that prefix. This script builds a one-off
    Pybricks program reproducing the recorded motor deltas and runs it in
    place of the main program, then restarts the main program when done.
  - During a replay, pressing CENTER three times stops it early and
    returns home.

Press Ctrl-C to stop.

Why one persistent connection: disconnecting and reconnecting BLE between
program swaps was unreliable on this Mac/firmware combination -- killing
the connection left the hub's BLE stack in a stuck state for anywhere from
15 seconds to over a minute before it would advertise again. Pybricks
supports stopping and starting a new program over an already-open
connection, so we do that instead and never disconnect until Ctrl-C.

This also needs the same two monkeypatches as push_ble.py, for the same
reason (system python3 is 3.9, pinning pybricksdev to an old release that
predates some of this firmware's protocol details):
  1. find_device() refuses a hub whose BLE advert has no name; macOS often
     never surfaces one, so we match purely on the Pybricks service UUID.
  2. unpack_hub_capabilities() hard-codes a 10-byte layout; newer firmware
     appends a byte, so we parse the first 10 and ignore the rest.
"""
import asyncio
import csv
import re
import struct
import sys
import tempfile
from datetime import datetime
from pathlib import Path

import pybricksdev.ble as ble
import pybricksdev.ble.pybricks as pb
import pybricksdev.connections.pybricks as cpb
from bleak import BleakScanner
from pybricksdev.connections.pybricks import PybricksHub


async def patched_find(name=None, service=ble.PYBRICKS_SERVICE_UUID, timeout=15):
    def match(device, adv):
        return service in adv.service_uuids

    dev = await BleakScanner.find_device_by_filter(
        match, timeout, service_uuids=[service]
    )
    if dev is None:
        raise asyncio.TimeoutError
    return dev


ble.find_device = patched_find

_HubCapabilityFlag = pb.HubCapabilityFlag


def patched_caps(data):
    max_char_size, flags, max_user_prog_size = struct.unpack("<HII", data[:10])
    return max_char_size, _HubCapabilityFlag(flags), max_user_prog_size


pb.unpack_hub_capabilities = patched_caps
cpb.unpack_hub_capabilities = patched_caps


ROW_RE = re.compile(
    r"ROW: Left:\s*(-?\d+), Right:\s*(-?\d+), DLeft:\s*(-?\d+), DRight:\s*(-?\d+)"
)
REPLAY_RE = re.compile(r"REPLAY_REQUEST:\s*(\d+)")

REPO_ROOT = Path(__file__).resolve().parent
RUNS_DIR = REPO_ROOT / "runs"
MAIN_SCRIPT = REPO_ROOT / "update_robot_code_pybricks.py"

REPLAY_TEMPLATE = """from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Button
from pybricks.tools import wait

hub = PrimeHub()
hub.system.set_stop_button(None)

left_motor = Motor(Port.C)
right_motor = Motor(Port.E)

deltas = {deltas!r}
interval_ms = {interval_ms}

press_count = 0
was_pressed = False
aborted = False


def tick(ms):
    global press_count, was_pressed, aborted
    remaining = ms
    step = 10
    while remaining > 0 and not aborted:
        pressed = Button.CENTER in hub.buttons.pressed()
        if pressed and not was_pressed:
            press_count += 1
            if press_count >= 3:
                aborted = True
        was_pressed = pressed
        wait(step)
        remaining -= step


for dleft, dright in deltas:
    if aborted:
        break
    if dleft:
        left_motor.run_angle(abs(dleft) / (interval_ms / 1000), dleft, wait=False)
    if dright:
        right_motor.run_angle(abs(dright) / (interval_ms / 1000), dright, wait=False)
    tick(interval_ms)

left_motor.stop()
right_motor.stop()
print("REPLAY_ABORTED" if aborted else "REPLAY_DONE")
"""


def find_run_csv(n):
    matches = sorted(RUNS_DIR.glob(f"{n}_run_*.csv"))
    return matches[-1] if matches else None


def build_replay_script(csv_path):
    deltas = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            deltas.append((int(row["dleft"]), int(row["dright"])))
    code = REPLAY_TEMPLATE.format(deltas=deltas, interval_ms=333)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".py", prefix="_replay_", delete=False, dir=REPO_ROOT
    )
    tmp.write(code)
    tmp.close()
    return Path(tmp.name)


async def main():
    RUNS_DIR.mkdir(exist_ok=True)

    hub = PybricksHub()
    while True:
        try:
            print("Searching for any hub with Pybricks service...")
            device = await ble.find_device(timeout=15)
            await hub.connect(device)
            break
        except Exception as e:
            print(f"Connect failed ({e}), retrying...")
    print(f"Connected to {device.address}")

    run_file = run_writer = run_path = None

    def close_run():
        nonlocal run_file, run_writer, run_path
        if run_file:
            run_file.close()
            print(f"Saved {run_path}")
        run_file = run_writer = run_path = None

    try:
        current_script = MAIN_SCRIPT
        while True:
            replay_requested = None

            async def reader():
                nonlocal replay_requested, run_file, run_writer, run_path
                while True:
                    line = await hub.read_line()
                    print(line)

                    if line == "RUN_START":
                        close_run()
                        run_path = (
                            RUNS_DIR / f"run_{datetime.now():%Y%m%d_%H%M%S}.csv"
                        )
                        run_file = open(run_path, "w", newline="")
                        run_writer = csv.writer(run_file)
                        run_writer.writerow(
                            ["timestamp", "left", "right", "dleft", "dright"]
                        )
                        run_file.flush()
                        print(f"Recording to {run_path}")
                        continue

                    if line == "RUN_STOP":
                        close_run()
                        continue

                    match = ROW_RE.search(line)
                    if match and run_writer:
                        run_writer.writerow(
                            [datetime.now().isoformat(), *match.groups()]
                        )
                        run_file.flush()
                        continue

                    match = REPLAY_RE.search(line)
                    if match:
                        replay_requested = int(match.group(1))
                        return

            # hub.run() resets the stdout line queue as its first action.
            # Start it and let that happen before reader() starts reading,
            # or reader() can grab a reference to the queue it's about to
            # replace and then listen on it forever.
            run_task = asyncio.ensure_future(
                hub.run(str(current_script), wait=True, print_output=False)
            )
            await asyncio.sleep(0)
            reader_task = asyncio.ensure_future(reader())

            done, _ = await asyncio.wait(
                {reader_task, run_task}, return_when=asyncio.FIRST_COMPLETED
            )

            if reader_task in done:
                await hub.stop_user_program()
                await run_task
            else:
                reader_task.cancel()

            if replay_requested is None:
                current_script = MAIN_SCRIPT
                continue

            csv_path = find_run_csv(replay_requested)
            if csv_path is None:
                print(f"No run found starting with '{replay_requested}_run_'")
                current_script = MAIN_SCRIPT
                continue

            print(f"Replaying {csv_path}")
            replay_script = build_replay_script(csv_path)
            try:
                await hub.run(str(replay_script), wait=True, print_output=True)
            finally:
                replay_script.unlink(missing_ok=True)

            current_script = MAIN_SCRIPT
    except KeyboardInterrupt:
        close_run()
    finally:
        await hub.disconnect()


if __name__ == "__main__":
    asyncio.run(main())
