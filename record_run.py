#!/usr/bin/env python3
"""Controller: keeps one persistent BLE connection to the hub and swaps
which Pybricks program is running on it, without ever disconnecting.

Usage:
    python3 record_run.py

On the hub, running update_robot_code_pybricks.py:
  - RIGHT (at home, "H") starts a recording, CENTER stops it. Each one is
    saved here as runs/run_<timestamp>.csv.
  - LEFT counts up (shown on the display), RIGHT counts back down to home.
  - CENTER, with a number selected, replays runs/<N>_run_*.csv -- rename
    run files yourself to give them that prefix. Multiple files can share
    a prefix (e.g. 3_run_a.csv, 3_run_b.csv); they're averaged sample by
    sample (truncated to the shortest one) before replay. This script
    builds a one-off Pybricks program that, for each recorded step, moves
    the wheels by roughly the recorded angle and then corrects with the
    hub's IMU until the recorded heading for that step is actually
    achieved, since wheel angle alone drifts from reality (friction,
    slip). Timing isn't preserved -- position (heading) is the priority.
    Runs in place of the main program, then restarts the main program
    when done.
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
from pybricksdev.connections import ConnectionState
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
    r"ROW: Left:\s*(-?\d+), Right:\s*(-?\d+), DLeft:\s*(-?\d+), DRight:\s*(-?\d+), "
    r"Heading:\s*(-?\d+(?:\.\d+)?)"
)
REPLAY_RE = re.compile(r"REPLAY_REQUEST:\s*(\d+)")

REPO_ROOT = Path(__file__).resolve().parent
RUNS_DIR = REPO_ROOT / "runs"
MAIN_SCRIPT = REPO_ROOT / "update_robot_code_pybricks.py"

REPLAY_TEMPLATE = """from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
hub.system.set_stop_button(None)
hub.imu.reset_heading(0)  # match the 0-reference the recording started from

left_motor = Motor(Port.C)
right_motor = Motor(Port.E)

# Each step is (dleft, dright, heading) -- heading is the IMU heading the
# hub recorded at that point in the original run. Timing doesn't need to
# match the recording; hitting the recorded heading at each step does, so
# wheel angle is only a starting estimate and the heading correction below
# is the authority on when a step is actually done.
steps = {steps!r}

watchdog = StopWatch()
MAX_TOTAL_MS = 60000  # hard stop so a stuck correction can't run forever

TURN_SPEED_MIN = 60  # deg/s, for heading correction
TURN_SPEED_MAX = 300
HEADING_TOLERANCE = 3  # degrees
CORRECT_TIMEOUT_MS = 2000
MOVE_TIMEOUT_MS = 2000

press_count = 0
was_pressed = False
aborted = False


def check_abort():
    global press_count, was_pressed, aborted
    pressed = Button.CENTER in hub.buttons.pressed()
    if pressed and not was_pressed:
        press_count += 1
        if press_count >= 3:
            aborted = True
    was_pressed = pressed
    return aborted


def correct_heading(target):
    \"\"\"Returns True if it converged, False if it timed out.\"\"\"
    elapsed = 0
    while elapsed < CORRECT_TIMEOUT_MS:
        if check_abort():
            return False
        error = target - hub.imu.heading()
        if abs(error) <= HEADING_TOLERANCE:
            left_motor.stop()
            right_motor.stop()
            return True
        # Speed scales with how far off we are, so small errors get precise
        # slow corrections and large ones get corrected quickly.
        speed = min(TURN_SPEED_MAX, max(TURN_SPEED_MIN, abs(error) * 8))
        # Recorded convention: heading increases when dleft is negative and
        # dright is positive (see update_robot_code_pybricks.py's ROW log).
        if error > 0:
            left_motor.run(-speed)
            right_motor.run(speed)
        else:
            left_motor.run(speed)
            right_motor.run(-speed)
        wait(20)
        elapsed += 20
    left_motor.stop()
    right_motor.stop()
    return False


step_num = 0
for dleft, dright, heading in steps:
    step_num += 1
    if aborted:
        break
    if watchdog.time() > MAX_TOTAL_MS:
        print("REPLAY watchdog timeout")
        aborted = True
        break

    if dleft:
        left_motor.run_angle(max(abs(dleft), 1) / 0.333, dleft, wait=False)
    if dright:
        right_motor.run_angle(max(abs(dright), 1) / 0.333, dright, wait=False)

    elapsed = 0
    while elapsed < MOVE_TIMEOUT_MS:
        if check_abort():
            break
        if left_motor.done() and right_motor.done():
            break
        wait(20)
        elapsed += 20

    if aborted:
        break

    print("STEP {{}}/{{}} after-move={{:.1f}}".format(
        step_num, len(steps), hub.imu.heading()))

    converged = correct_heading(heading)
    print("STEP {{}}/{{}} target={{:.1f}} achieved={{:.1f}} {{}}".format(
        step_num, len(steps), heading, hub.imu.heading(),
        "ok" if converged else "TIMEOUT"))

left_motor.stop()
right_motor.stop()
print("REPLAY_ABORTED" if aborted else "REPLAY_DONE")
"""


def find_run_csvs(n):
    return sorted(RUNS_DIR.glob(f"{n}_run_*.csv"))


def read_steps(csv_path):
    steps = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            steps.append(
                (int(row["dleft"]), int(row["dright"]), float(row.get("heading", 0)))
            )
    return steps


def average_steps(csv_paths):
    """Averages multiple recordings sample-by-sample, truncated to the
    length of the shortest one."""
    runs = [read_steps(p) for p in csv_paths]
    length = min(len(run) for run in runs)
    return [
        (
            round(sum(run[i][0] for run in runs) / len(runs)),
            round(sum(run[i][1] for run in runs) / len(runs)),
            sum(run[i][2] for run in runs) / len(runs),
        )
        for i in range(length)
    ]


def build_replay_script(steps):
    code = REPLAY_TEMPLATE.format(steps=steps)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".py", prefix="_replay_", delete=False, dir=REPO_ROOT
    )
    tmp.write(code)
    tmp.close()
    return Path(tmp.name)


def _is_connected(hub):
    return hub.connection_state_observable.value == ConnectionState.CONNECTED


async def _reap(task):
    """Retrieve a task's result/exception so it can't be silently dropped
    (an unretrieved exception on a disconnect is what caused a runaway
    zero-delay reconnect loop before this was added)."""
    if task.done() and not task.cancelled():
        exc = task.exception()
        if exc is not None:
            print(f"(background task ended with: {exc!r})")


async def main():
    RUNS_DIR.mkdir(exist_ok=True)

    run_file = run_writer = run_path = None

    def close_run():
        nonlocal run_file, run_writer, run_path
        if run_file:
            run_file.close()
            print(f"Saved {run_path}")
        run_file = run_writer = run_path = None

    try:
        while True:  # reconnect loop
            hub = PybricksHub()
            while True:
                try:
                    print("Searching for any hub with Pybricks service...")
                    device = await ble.find_device(timeout=15)
                    await hub.connect(device)
                    break
                except Exception as e:
                    print(f"Connect failed ({e}), retrying...")
                    await asyncio.sleep(1)
            print(f"Connected to {device.address}")

            current_script = MAIN_SCRIPT
            while _is_connected(hub):
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
                                ["timestamp", "left", "right", "dleft", "dright", "heading"]
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
                # Start it and let that happen before reader() starts
                # reading, or reader() can grab a reference to the queue
                # it's about to replace and then listen on it forever.
                run_task = asyncio.ensure_future(
                    hub.run(str(current_script), wait=True, print_output=False)
                )
                await asyncio.sleep(0)
                reader_task = asyncio.ensure_future(reader())

                await asyncio.wait(
                    {reader_task, run_task}, return_when=asyncio.FIRST_COMPLETED
                )

                if not _is_connected(hub):
                    print("Hub disconnected, will reconnect...")
                    reader_task.cancel()
                    await _reap(run_task)
                    await _reap(reader_task)
                    break

                if reader_task.done() and not reader_task.cancelled():
                    await hub.stop_user_program()
                    await run_task
                else:
                    reader_task.cancel()
                await _reap(run_task)
                await _reap(reader_task)

                if replay_requested is None:
                    current_script = MAIN_SCRIPT
                    continue

                csv_paths = find_run_csvs(replay_requested)
                if not csv_paths:
                    print(f"No runs found starting with '{replay_requested}_run_'")
                    current_script = MAIN_SCRIPT
                    continue

                names = ", ".join(p.name for p in csv_paths)
                print(f"Replaying average of {len(csv_paths)} run(s): {names}")
                steps = average_steps(csv_paths)
                replay_script = build_replay_script(steps)
                try:
                    await hub.run(str(replay_script), wait=True, print_output=True)
                except Exception as e:
                    print(f"Replay error: {e}")
                finally:
                    replay_script.unlink(missing_ok=True)

                current_script = MAIN_SCRIPT

            try:
                await hub.disconnect()
            except Exception:
                pass
    except KeyboardInterrupt:
        close_run()


if __name__ == "__main__":
    asyncio.run(main())
