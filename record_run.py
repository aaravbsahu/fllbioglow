#!/usr/bin/env python3
"""Controller: keeps one persistent BLE connection per connected robot and
swaps which Pybricks program is running on each, without ever
disconnecting. Runs NUM_ROBOTS independent "connect to whichever Pybricks
hub is currently advertising" loops concurrently -- all connected robots
record/replay independently and concurrently, saving into the same shared
runs/ folder (there's no identification between robots; whichever one
starts a recording or requests a replay, that's what happens on it).

This works without needing to tell robots apart because a BLE peripheral
stops advertising once it has a central connected to it: once loop A
connects to whichever hub it found first, that hub stops advertising, so
loop B's scan then only sees the other one. (We tried identifying hubs by
BLE address and by Hub Name; address doesn't work because macOS doesn't
give Pybricks hubs a stable address -- it can even change *within* a
single scan -- and renaming hubs via Pybricks firmware reinstall turned
out to be more hassle than wanted. This "whichever's free" approach
doesn't need either.)

Usage:
    python3 record_run.py

On each hub, running update_robot_code_pybricks.py:
  - RIGHT (at home, "H") starts a recording, CENTER stops it. Each one is
    saved to runs/run_<timestamp>.csv.
  - LEFT counts up (shown on the display), RIGHT counts back down to home.
  - CENTER, with a number selected, replays runs/<N>_run_*.csv -- rename
    run files yourself to give them that prefix. Multiple files can share
    a prefix (e.g. 3_run_a.csv, 3_run_b.csv); they're averaged sample by
    sample (truncated to the shortest one) before replay. This script
    builds a one-off Pybricks program that, for each recorded step, moves
    the wheels by roughly the recorded angle and then corrects with the
    hub's IMU until the recorded heading for that step is actually
    achieved, since wheel angle alone drifts from reality (friction,
    slip). Timing isn't preserved for driving -- position (heading) is
    the priority. Runs in place of the main program, then restarts the
    main program when done.
  - During a replay, pressing CENTER three times stops it early and
    returns home.

Press Ctrl-C to stop.

Why one persistent connection per robot: disconnecting and reconnecting
BLE between program swaps was unreliable on this Mac/firmware combination
-- killing the connection left the hub's BLE stack in a stuck state for
anywhere from 15 seconds to over a minute before it would advertise
again. Pybricks supports stopping and starting a new program over an
already-open connection, so we do that instead and never disconnect until
Ctrl-C.

This also needs the same monkeypatch as push_ble.py, for the same reason
(system python3 is 3.9, pinning pybricksdev to an old release that
predates some of this firmware's protocol details): unpack_hub_capabilities()
hard-codes a 10-byte layout; newer firmware appends a byte, so we parse
the first 10 and ignore the rest.
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

# How many robots to connect to simultaneously.
NUM_ROBOTS = 1


async def find_any_hub(service=ble.PYBRICKS_SERVICE_UUID, timeout=15):
    """Finds any Pybricks hub currently advertising (i.e. not already
    connected to something). Several of these can run concurrently --
    once one succeeds, that hub stops advertising, so a concurrent call
    naturally won't find the same device again."""

    def match(device, adv):
        return service in adv.service_uuids

    dev = await BleakScanner.find_device_by_filter(
        match, timeout, service_uuids=[service]
    )
    if dev is None:
        raise asyncio.TimeoutError
    return dev


_HubCapabilityFlag = pb.HubCapabilityFlag


def patched_caps(data):
    max_char_size, flags, max_user_prog_size = struct.unpack("<HII", data[:10])
    return max_char_size, _HubCapabilityFlag(flags), max_user_prog_size


pb.unpack_hub_capabilities = patched_caps
cpb.unpack_hub_capabilities = patched_caps


ROW_RE = re.compile(
    r"ROW: Left:\s*(-?\d+), Right:\s*(-?\d+), DLeft:\s*(-?\d+), DRight:\s*(-?\d+), "
    r"Heading:\s*(-?\d+(?:\.\d+)?), "
    r"RightArm:\s*(-?\d+), LeftArm:\s*(-?\d+), DRightArm:\s*(-?\d+), DLeftArm:\s*(-?\d+)"
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
#
# That model only fits the drive motors: `s` is driven by wheel distance,
# so during a recorded arm move where the robot was stationary, `s` barely
# advances and the arm's recorded back-and-forth (e.g. clockwise then
# counter-clockwise) would collapse into just the net delta instead of
# replaying the actual sequence. So the recording is split into alternating
# segments:
#   ("drive", [(s, dist, heading), ...], (right_arm_hold, left_arm_hold))
#     -- normal trendline driving; arms held at a constant angle.
#   ("arm", [(right_arm, left_arm), ...], (dist_hold, heading_hold))
#     -- robot holds position/heading; arms step through the exact
#        recorded waypoint sequence, in order, one at a time.
# This preserves arm movement order/direction and makes the robot pause
# while the arms are actively moving, matching how the run was recorded.
REPLAY_TEMPLATE = """from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
hub.system.set_stop_button(None)
hub.imu.reset_heading(0)  # match the 0-reference the recording started from

left_motor = Motor(Port.F)
right_motor = Motor(Port.E)
right_arm_motor = Motor(Port.C)
left_arm_motor = Motor(Port.D)

# Alternating ("drive", table, arm_hold) / ("arm", waypoints, drive_hold)
# segments -- see comment above.
segments = {segments!r}
K_ROT = {k_rot}

DIST_SCALE = 1.0       # calibration: >1 makes replay travel farther
CRUISE = 600           # deg/s wheel speed on the straights
KP_SYNC = 1.5          # keeps actual distance synced to the target
KP_HEAD = 9.0          # steering gain
HOLD_KP_DIST = 2.0     # gentler: just resists drift while arms move
MAX_CMD = 950          # deg/s hard cap per wheel
LOOKAHEAD = 40         # +/- path units, window for reading local travel
FINISH_DIST_TOL = 15   # motor-deg: close enough to a segment's end
FINISH_HEAD_TOL = 4    # deg
FINISH_TIMEOUT_MS = 8000
ARM_WAYPOINT_TOL = 8        # motor-deg: close enough to an arm waypoint
ARM_WAYPOINT_TIMEOUT_MS = 1500
WATCHDOG_MS = 180000
REPORT_EVERY = 50      # ticks (~1s); printing over BLE stalls the loop

watchdog = StopWatch()
press_count = 0
was_pressed = False
aborted = False
ticks = 0


def dist_now():
    return (right_motor.angle() - left_motor.angle()) / 2


def clamp(v, lim):
    return lim if v > lim else (-lim if v < -lim else v)


def check_abort():
    global press_count, was_pressed, aborted
    pressed = Button.CENTER in hub.buttons.pressed()
    if pressed and not was_pressed:
        press_count += 1
    was_pressed = pressed
    if press_count >= 3:
        aborted = True
    return aborted


zero = dist_now()
right_arm_zero = right_arm_motor.angle()
left_arm_zero = left_arm_motor.angle()

for seg in segments:
    if aborted or watchdog.time() > WATCHDOG_MS:
        break
    kind, payload, extra = seg

    if kind == "drive":
        table = payload
        right_arm_t, left_arm_t = extra
        S_FINAL = table[-1][0]
        if S_FINAL <= 0:
            right_arm_motor.track_target(right_arm_zero + right_arm_t)
            left_arm_motor.track_target(left_arm_zero + left_arm_t)
            continue

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

        END_DIST = table[-1][1] * DIST_SCALE
        END_HEAD = table[-1][2]

        d_prev = dist_now() - zero
        h_prev = hub.imu.heading()
        s = 0.0
        finishing = False
        finish_start = 0
        path_dir = 1.0
        WINDOW = 2.0 * LOOKAHEAD

        while True:
            if check_abort() or watchdog.time() > WATCHDOG_MS:
                break

            d = dist_now() - zero
            h = hub.imu.heading()
            s += abs(d - d_prev) + K_ROT * abs(h - h_prev)
            d_prev = d
            h_prev = h

            if not finishing and s >= S_FINAL:
                finishing = True
                finish_start = watchdog.time()

            if finishing:
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
                # else: keep last path_dir (pure rotation -- don't flip)

                trans_frac = abs(travel) / WINDOW
                if trans_frac > 1.0:
                    trans_frac = 1.0

                drive = path_dir * CRUISE * trans_frac + KP_SYNC * (dist_t - d)
                turn = KP_HEAD * (h - head_t)

            left_motor.run(clamp(-drive + turn, MAX_CMD))
            right_motor.run(clamp(drive + turn, MAX_CMD))
            right_arm_motor.track_target(right_arm_zero + right_arm_t)
            left_arm_motor.track_target(left_arm_zero + left_arm_t)

            ticks += 1
            if ticks % REPORT_EVERY == 0:
                print("drive s={{:.0f}}/{{:.0f}} d {{:.0f}}>{{:.0f}} h {{:.0f}}>{{:.0f}}".format(
                    s, S_FINAL, d, dist_t, h, head_t))

            wait(20)

        left_motor.stop()
        right_motor.stop()

    else:  # kind == "arm"
        waypoints = payload
        dist_hold, head_hold = extra
        dist_target = dist_hold * DIST_SCALE

        for ra, la in waypoints:
            if check_abort() or watchdog.time() > WATCHDOG_MS:
                break
            wp_timer = StopWatch()
            while True:
                if check_abort() or watchdog.time() > WATCHDOG_MS:
                    break
                d = dist_now() - zero
                h = hub.imu.heading()
                drive = clamp(HOLD_KP_DIST * (dist_target - d), CRUISE)
                turn = KP_HEAD * (h - head_hold)
                left_motor.run(clamp(-drive + turn, MAX_CMD))
                right_motor.run(clamp(drive + turn, MAX_CMD))
                right_arm_motor.track_target(right_arm_zero + ra)
                left_arm_motor.track_target(left_arm_zero + la)

                ra_now = right_arm_motor.angle() - right_arm_zero
                la_now = left_arm_motor.angle() - left_arm_zero
                reached = (abs(ra_now - ra) <= ARM_WAYPOINT_TOL
                           and abs(la_now - la) <= ARM_WAYPOINT_TOL)

                ticks += 1
                if ticks % REPORT_EVERY == 0:
                    print("arm ra {{:.0f}}>{{:.0f}} la {{:.0f}}>{{:.0f}}".format(
                        ra_now, ra, la_now, la))

                if reached or wp_timer.time() > ARM_WAYPOINT_TIMEOUT_MS:
                    break
                wait(20)

        left_motor.stop()
        right_motor.stop()

print("REPLAY_ABORTED" if aborted else "REPLAY_DONE")
"""


def find_run_csvs(n):
    return sorted(RUNS_DIR.glob(f"{n}_run_*.csv"))


def read_trajectory(csv_path):
    """Returns [(dist, heading, right_arm, left_arm)] for a recording, dist
    and arm angles zeroed to start at 0. dist is the encoder proxy
    (right - left) / 2."""
    pts = []
    with open(csv_path, newline="") as f:
        for row in csv.DictReader(f):
            dist = (int(row["right"]) - int(row["left"])) / 2
            pts.append((
                dist,
                float(row.get("heading", 0) or 0),
                int(row.get("right_arm", 0) or 0),
                int(row.get("left_arm", 0) or 0),
            ))
    if not pts:
        return pts
    d0, _, ra0, la0 = pts[0]
    return [(d - d0, h, ra - ra0, la - la0) for d, h, ra, la in pts]


def average_trajectories(csv_paths):
    trajs = [read_trajectory(p) for p in csv_paths]
    n = min(len(t) for t in trajs)
    return [
        tuple(sum(t[i][col] for t in trajs) / len(trajs) for col in range(4))
        for i in range(n)
    ]


K_ROT = 4.0  # weight of 1 deg heading change vs 1 unit distance, in path length

# A row counts as "arm moving" if either arm changed by more than this
# many degrees since the previous row -- used to split a recording into
# alternating drive/arm segments (see REPLAY_TEMPLATE comment above).
ARM_ACTIVE_THRESH = 3


def _drive_table(chunk):
    """[(dist, heading, ra, la)] -> [(s, dist, heading)] for one drive
    segment, s = cumulative (|dist change| + K_ROT * |heading change|)."""
    table = []
    s = 0.0
    prev_d, prev_h = chunk[0][0], chunk[0][1]
    for d, h, _, _ in chunk:
        s += abs(d - prev_d) + K_ROT * abs(h - prev_h)
        prev_d, prev_h = d, h
        table.append((round(s, 1), round(d, 1), round(h, 2)))
    return table


def build_segments(traj):
    """Splits [(dist, heading, right_arm, left_arm)] into alternating
    ("drive", table, (right_arm_hold, left_arm_hold)) and
    ("arm", waypoints, (dist_hold, heading_hold)) segments."""
    n = len(traj)
    active = [False] * n
    for i in range(1, n):
        _, _, ra0, la0 = traj[i - 1]
        _, _, ra1, la1 = traj[i]
        active[i] = abs(ra1 - ra0) + abs(la1 - la0) > ARM_ACTIVE_THRESH

    segments = []
    arm_hold = (round(traj[0][2], 1), round(traj[0][3], 1))
    i = 0
    while i < n:
        is_arm = active[i]
        j = i
        while j < n and active[j] == is_arm:
            j += 1
        chunk = traj[i:j]
        if is_arm:
            dist_hold, head_hold = traj[i - 1][0], traj[i - 1][1]
            waypoints = [(round(ra, 1), round(la, 1)) for _, _, ra, la in chunk]
            segments.append(("arm", waypoints, (round(dist_hold, 1), round(head_hold, 2))))
            arm_hold = waypoints[-1]
        else:
            segments.append(("drive", _drive_table(chunk), arm_hold))
        i = j
    return segments


def build_replay_script(segments, slot):
    code = REPLAY_TEMPLATE.format(segments=segments, k_rot=K_ROT)
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=".py", prefix=f"_replay_{slot}_", delete=False, dir=REPO_ROOT
    )
    tmp.write(code)
    tmp.close()
    return Path(tmp.name)


# --- Deterministic script generation ---------------------------------
#
# The REPLAY_TEMPLATE approach above re-interprets the recorded CSV every
# time it replays, driven by a big embedded data table. This is a
# different output: a flat, ordinary-looking Pybricks program with a
# hardcoded sequence of drive(...)/turn(...)/arm_to(...) calls -- meant to
# be generated once from a recording and then hand-edited (tune a
# distance, nudge a turn angle, delete a step) rather than regenerated.
# Each call still uses the gyro to correct drift during its own leg, the
# same way the recording-replay does, just one discrete leg at a time
# instead of a continuous path-following loop.

TURN_HEADING_THRESH = 3  # deg/sample: a row counts as "turning" past this
MIN_DRIVE_DIST = 10      # encoder-deg: skip legs smaller than this (noise)
MIN_TURN_ANGLE = 3       # deg: skip turns smaller than this (noise)


def _classify_rows(traj):
    """Labels each row 'arm', 'turn', or 'drive' based on the change from
    the previous row. Row 0 copies row 1's label (nothing to compare it
    to)."""
    n = len(traj)
    labels = ["drive"] * n
    for i in range(1, n):
        d0, h0, ra0, la0 = traj[i - 1]
        d1, h1, ra1, la1 = traj[i]
        if abs(ra1 - ra0) + abs(la1 - la0) > ARM_ACTIVE_THRESH:
            labels[i] = "arm"
        elif abs(h1 - h0) > TURN_HEADING_THRESH and abs(h1 - h0) > abs(d1 - d0):
            labels[i] = "turn"
        else:
            labels[i] = "drive"
    if n > 1:
        labels[0] = labels[1]
    return labels


def build_program(traj):
    """Turns [(dist, heading, right_arm, left_arm)] into a flat list of
    steps: ("drive", delta_dist), ("turn", delta_heading), or
    ("arm", right_arm, left_arm) -- one per recorded arm waypoint."""
    n = len(traj)
    if n == 0:
        return []
    labels = _classify_rows(traj)

    steps = []
    i = 0
    while i < n:
        label = labels[i]
        j = i
        while j < n and labels[j] == label:
            j += 1
        chunk = traj[i:j]

        if label == "arm":
            for _, _, ra, la in chunk:
                steps.append(("arm", round(ra, 1), round(la, 1)))
        elif label == "turn":
            delta = chunk[-1][1] - chunk[0][1]
            if abs(delta) >= MIN_TURN_ANGLE:
                steps.append(("turn", round(delta, 1)))
        else:
            delta = chunk[-1][0] - chunk[0][0]
            if abs(delta) >= MIN_DRIVE_DIST:
                steps.append(("drive", round(delta, 1)))
        i = j
    return steps


DETERMINISTIC_TEMPLATE = '''"""Deterministic replay, generated from {source}.

Hand-edit the numbers in PROGRAM below to fine-tune -- each tuple is one
step, run in order:
  ("drive", encoder_degrees)   -- straight, holding heading via gyro
  ("turn", degrees)            -- relative pivot turn, gyro-corrected
  ("arm", right_angle, left_angle)  -- move arms to these angles
    (relative to wherever the arms were at program start)
Delete a step, change a number, add a new one -- it's just a list.
"""
from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch

hub = PrimeHub()
hub.system.set_stop_button(None)
hub.imu.reset_heading(0)

left_motor = Motor(Port.F)
right_motor = Motor(Port.E)
right_arm_motor = Motor(Port.C)
left_arm_motor = Motor(Port.D)

CRUISE = 600        # deg/s wheel speed while driving straight
KP_HEAD = 9.0        # steering gain, both drive() and turn()
MAX_CMD = 950        # deg/s hard cap per wheel
DRIVE_TOL = 15       # encoder-deg: close enough to end a drive() leg
TURN_TOL = 4         # deg: close enough to end a turn() leg
ARM_TOL = 8          # motor-deg: close enough to end an arm_to() step
STEP_TIMEOUT_MS = 6000
WATCHDOG_MS = 180000

watchdog = StopWatch()
press_count = 0
was_pressed = False
aborted = False


def dist_now():
    return (right_motor.angle() - left_motor.angle()) / 2


def clamp(v, lim):
    return lim if v > lim else (-lim if v < -lim else v)


def check_abort():
    global press_count, was_pressed, aborted
    pressed = Button.CENTER in hub.buttons.pressed()
    if pressed and not was_pressed:
        press_count += 1
    was_pressed = pressed
    if press_count >= 3:
        aborted = True
    return aborted


def drive(distance):
    """Drive `distance` encoder-degrees straight, holding the heading we
    had when this step started."""
    target_heading = hub.imu.heading()
    start = dist_now()
    target = start + distance
    timer = StopWatch()
    while not (aborted or watchdog.time() > WATCHDOG_MS):
        if check_abort():
            break
        err = target - dist_now()
        if abs(err) <= DRIVE_TOL or timer.time() > STEP_TIMEOUT_MS:
            break
        drive_cmd = clamp(3.0 * err, CRUISE)
        turn_cmd = KP_HEAD * (hub.imu.heading() - target_heading)
        left_motor.run(clamp(-drive_cmd + turn_cmd, MAX_CMD))
        right_motor.run(clamp(drive_cmd + turn_cmd, MAX_CMD))
        wait(20)
    left_motor.stop()
    right_motor.stop()


def turn(angle):
    """Pivot turn by `angle` degrees relative to the current heading,
    using the gyro to stop exactly there."""
    target_heading = hub.imu.heading() + angle
    timer = StopWatch()
    while not (aborted or watchdog.time() > WATCHDOG_MS):
        if check_abort():
            break
        err = hub.imu.heading() - target_heading
        if abs(err) <= TURN_TOL or timer.time() > STEP_TIMEOUT_MS:
            break
        turn_cmd = clamp(KP_HEAD * err, MAX_CMD)
        left_motor.run(turn_cmd)
        right_motor.run(turn_cmd)
        wait(20)
    left_motor.stop()
    right_motor.stop()


right_arm_zero = right_arm_motor.angle()
left_arm_zero = left_arm_motor.angle()


def arm_to(right_angle, left_angle):
    """Move both arms to the given angles, relative to their position
    when this program started."""
    timer = StopWatch()
    while not (aborted or watchdog.time() > WATCHDOG_MS):
        if check_abort():
            break
        right_arm_motor.track_target(right_arm_zero + right_angle)
        left_arm_motor.track_target(left_arm_zero + left_angle)
        ra_now = right_arm_motor.angle() - right_arm_zero
        la_now = left_arm_motor.angle() - left_arm_zero
        reached = abs(ra_now - right_angle) <= ARM_TOL and abs(la_now - left_angle) <= ARM_TOL
        if reached or timer.time() > STEP_TIMEOUT_MS:
            break
        wait(20)


# PROGRAM: edit freely. Run in order, top to bottom.
PROGRAM = [
{program_lines}
]

for step in PROGRAM:
    if aborted or watchdog.time() > WATCHDOG_MS:
        break
    kind = step[0]
    if kind == "drive":
        drive(step[1])
    elif kind == "turn":
        turn(step[1])
    else:
        arm_to(step[1], step[2])

print("DETERMINISTIC_ABORTED" if aborted else "DETERMINISTIC_DONE")
'''


def build_deterministic_script(csv_paths, out_path):
    traj = average_trajectories(csv_paths)
    program = build_program(traj)
    source = ", ".join(p.name for p in csv_paths)
    program_lines = "\n".join(f"    {step!r}," for step in program)
    code = DETERMINISTIC_TEMPLATE.format(program_lines=program_lines, source=source)
    out_path.write_text(code)
    return program


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


async def run_robot(slot):
    """Connects to whichever Pybricks hub is currently advertising and
    runs the record/replay loop for it, forever (until cancelled).
    `slot` is just an internal label (1, 2, ...) for log lines and temp
    file names -- it has no relationship to which physical robot ends up
    here, and two robots are otherwise treated identically, sharing the
    same runs/ folder."""
    run_file = run_writer = run_path = None

    def close_run():
        nonlocal run_file, run_writer, run_path
        if run_file:
            run_file.close()
            print(f"[{slot}] Saved {run_path}")
        run_file = run_writer = run_path = None

    try:
        while True:  # reconnect loop
            hub = PybricksHub()
            while True:
                try:
                    print(f"[{slot}] Searching for any hub with Pybricks service...")
                    device = await find_any_hub(timeout=15)
                    await hub.connect(device)
                    break
                except Exception as e:
                    print(f"[{slot}] Connect failed ({e}), retrying...")
                    await asyncio.sleep(1)
            print(f"[{slot}] Connected to {device.address}")

            current_script = MAIN_SCRIPT
            while _is_connected(hub):
                replay_requested = None

                async def reader():
                    nonlocal replay_requested, run_file, run_writer, run_path
                    while True:
                        line = await hub.read_line()
                        print(f"[{slot}] {line}")

                        if line == "RUN_START":
                            close_run()
                            run_path = (
                                RUNS_DIR / f"run_{datetime.now():%Y%m%d_%H%M%S}.csv"
                            )
                            run_file = open(run_path, "w", newline="")
                            run_writer = csv.writer(run_file)
                            run_writer.writerow(
                                ["timestamp", "left", "right", "dleft", "dright", "heading",
                                 "right_arm", "left_arm", "dright_arm", "dleft_arm"]
                            )
                            run_file.flush()
                            print(f"[{slot}] Recording to {run_path}")
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
                    print(f"[{slot}] Hub disconnected, will reconnect...")
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
                    print(f"[{slot}] No runs found starting with '{replay_requested}_run_'")
                    current_script = MAIN_SCRIPT
                    continue

                csv_names = ", ".join(p.name for p in csv_paths)
                print(f"[{slot}] Replaying average of {len(csv_paths)} run(s): {csv_names}")
                segments = build_segments(average_trajectories(csv_paths))
                kinds = ", ".join(seg[0] for seg in segments)
                print(f"[{slot}]  {len(segments)} segment(s): {kinds}")
                replay_script = build_replay_script(segments, slot)
                try:
                    await hub.run(str(replay_script), wait=True, print_output=True)
                except Exception as e:
                    print(f"[{slot}] Replay error: {e}")
                finally:
                    replay_script.unlink(missing_ok=True)

                current_script = MAIN_SCRIPT

            try:
                await hub.disconnect()
            except Exception:
                pass
    except asyncio.CancelledError:
        close_run()
        raise


async def main():
    RUNS_DIR.mkdir(exist_ok=True)
    tasks = [asyncio.ensure_future(run_robot(slot)) for slot in range(1, NUM_ROBOTS + 1)]
    try:
        await asyncio.gather(*tasks)
    finally:
        for task in tasks:
            task.cancel()
        for task in tasks:
            await _reap(task)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
