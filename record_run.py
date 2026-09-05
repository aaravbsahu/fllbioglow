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

# Replay model (from GummyBears Robotics' "trendline navigation" approach,
# https://www.youtube.com/watch?v=YvdNfw3_fhA): distance is the master
# variable and heading is a function of distance. The robot drives its
# recorded path at a steady cruise speed while a P-controller steers so
# the gyro heading matches the recorded heading for the current point.
#
# The path is parameterized by `s` = cumulative (|dist change| + K_ROT *
# |heading change|), so it advances even during an in-place rotation and
# stays single-valued when the run reverses. `dist` is the encoder proxy
# (right_angle - left_angle) / 2. Because this robot's wheels are
# mirrored, a spin adds the SAME term to both wheel commands.
REPLAY_TEMPLATE = """from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
hub.system.set_stop_button(None)
hub.imu.reset_heading(0)  # match the 0-reference the recording started from

left_motor = Motor(Port.C)
right_motor = Motor(Port.E)

# (s, dist, heading) rows.
table = {table!r}
S_FINAL = table[-1][0]
K_ROT = {k_rot}

DIST_SCALE = 1.0       # calibration: >1 makes replay travel farther
CRUISE = 600           # deg/s wheel speed on the straights
KP_SYNC = 1.5          # keeps actual distance synced to the target
KP_HEAD = 9.0          # steering gain
MAX_CMD = 950          # deg/s hard cap per wheel
LOOKAHEAD = 40         # +/- path units, window for reading local travel
HEAD_DEADBAND = 0.15   # deg/tick of heading noise to keep out of `s`
FINISH_DIST_TOL = 15   # motor-deg: close enough to the recorded end
FINISH_HEAD_TOL = 4    # deg
FINISH_TIMEOUT_MS = 8000
WATCHDOG_MS = 180000
REPORT_EVERY = 50      # ticks (~1s); printing over BLE stalls the loop

END_DIST = table[-1][1] * DIST_SCALE
END_HEAD = table[-1][2]
print("target end dist {{:.0f}}".format(END_DIST))

watchdog = StopWatch()
press_count = 0
was_pressed = False
aborted = False


def dist_now():
    return (right_motor.angle() - left_motor.angle()) / 2


def interp(q):
    if q <= table[0][0]:
        return table[0][1] * DIST_SCALE, table[0][2]
    if q >= table[-1][0]:
        return table[-1][1] * DIST_SCALE, table[-1][2]
    lo, hi = 0, len(table) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if table[mid][0] <= q:
            lo = mid
        else:
            hi = mid
    s0, d0, h0 = table[lo]
    s1, d1, h1 = table[hi]
    f = (q - s0) / (s1 - s0) if s1 > s0 else 0.0
    return (d0 + f * (d1 - d0)) * DIST_SCALE, h0 + f * (h1 - h0)


def clamp(v, lim):
    return lim if v > lim else (-lim if v < -lim else v)


zero = dist_now()
d_prev = 0.0
h_prev = hub.imu.heading()
s = 0.0
ticks = 0
finishing = False
finish_start = 0
path_dir = 1.0
WINDOW = 2.0 * LOOKAHEAD

while True:
    if watchdog.time() > WATCHDOG_MS:
        print("REPLAY watchdog timeout")
        break

    pressed = Button.CENTER in hub.buttons.pressed()
    if pressed and not was_pressed:
        press_count += 1
    was_pressed = pressed
    if press_count >= 3:
        aborted = True
        break

    d = dist_now() - zero
    h = hub.imu.heading()
    dh = abs(h - h_prev)
    if dh < HEAD_DEADBAND:
        dh = 0.0
    s += abs(d - d_prev) + K_ROT * dh
    d_prev = d
    h_prev = h

    if not finishing and s >= S_FINAL:
        finishing = True
        finish_start = watchdog.time()

    if finishing:
        # drive straight to the recorded end distance/heading
        dist_t, head_t = END_DIST, END_HEAD
        if (abs(END_DIST - d) <= FINISH_DIST_TOL
                and abs(END_HEAD - h) <= FINISH_HEAD_TOL):
            break
        if watchdog.time() - finish_start > FINISH_TIMEOUT_MS:
            print("finish timeout d={{:.0f}}/{{:.0f}}".format(d, END_DIST))
            break
        drive = clamp(3.0 * (END_DIST - d), CRUISE)
        turn = KP_HEAD * (h - END_HEAD)
    else:
        dist_t, head_t = interp(s)
        dist_behind, _ = interp(s - LOOKAHEAD)
        dist_ahead, _ = interp(s + LOOKAHEAD)
        travel = dist_ahead - dist_behind
        if travel > 5:
            path_dir = 1.0
        elif travel < -5:
            path_dir = -1.0
        # else: keep last path_dir (pure rotation -- don't flip direction)

        # scale cruise down where the path is rotation-heavy, so turns are
        # clean pivots instead of slow arcs
        trans_frac = abs(travel) / WINDOW
        if trans_frac > 1.0:
            trans_frac = 1.0

        drive = path_dir * CRUISE * trans_frac + KP_SYNC * (dist_t - d)
        # spin term: same sign on both wheels (mirrored wheels)
        turn = KP_HEAD * (h - head_t)

    left_cmd = clamp(-drive + turn, MAX_CMD)
    right_cmd = clamp(drive + turn, MAX_CMD)
    left_motor.run(left_cmd)
    right_motor.run(right_cmd)

    ticks += 1
    if ticks % REPORT_EVERY == 0:
        print("s={{:.0f}}/{{:.0f}} d {{:.0f}}>{{:.0f}} h {{:.0f}}>{{:.0f}}".format(
            s, S_FINAL, d, dist_t, h, head_t))

    wait(20)

left_motor.stop()
right_motor.stop()
print("end dist {{:.0f}}/{{:.0f}}  end head {{:.1f}}/{{:.1f}}".format(
    dist_now() - zero, END_DIST, hub.imu.heading(), END_HEAD))
print("REPLAY_ABORTED" if aborted else "REPLAY_DONE")
"""


def find_run_csvs(n):
    return sorted(RUNS_DIR.glob(f"{n}_run_*.csv"))


def read_trajectory(csv_path):
    """Returns [(dist, heading)] for a recording, dist zeroed to start at 0.
    dist is the encoder proxy (right - left) / 2."""
    pts = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            dist = (int(row["right"]) - int(row["left"])) / 2
            pts.append((dist, float(row.get("heading", 0) or 0)))
    if not pts:
        return pts
    d0 = pts[0][0]
    return [(d - d0, h) for d, h in pts]


def average_trajectories(csv_paths):
    trajs = [read_trajectory(p) for p in csv_paths]
    n = min(len(t) for t in trajs)
    return [
        (
            sum(t[i][0] for t in trajs) / len(trajs),
            sum(t[i][1] for t in trajs) / len(trajs),
        )
        for i in range(n)
    ]


K_ROT = 4.0  # weight of 1 deg heading change vs 1 unit distance, in path length


def build_path_table(traj):
    """Turns [(dist, heading)] into [(s, dist, heading)] where
    s = cumulative (|dist change| + K_ROT * |heading change|)."""
    table = []
    s = 0.0
    prev_d, prev_h = traj[0]
    for d, h in traj:
        s += abs(d - prev_d) + K_ROT * abs(h - prev_h)
        prev_d, prev_h = d, h
        table.append((round(s, 1), round(d, 1), round(h, 2)))
    return table


def build_replay_script(table):
    code = REPLAY_TEMPLATE.format(table=table, k_rot=K_ROT)
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
                table = build_path_table(average_trajectories(csv_paths))
                print(f"  path length {table[-1][0]:.0f}, {len(table)} points")
                replay_script = build_replay_script(table)
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
