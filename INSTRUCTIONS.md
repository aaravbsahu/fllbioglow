# How this repo works

The hub is flashed with **Pybricks firmware** (not stock LEGO firmware). Code
is deployed over Bluetooth from this laptop -- the SPIKE app is not used at
all. `update_robot_code.py` is the old LEGO-firmware version and is dead code
now; `computer_helper.py` / `app_script.gs` are the old Google Sheets logging
setup, superseded by the CSV logging described below.

## Files

- `update_robot_code_pybricks.py` -- the program that runs on the hub. Reads
  the LEFT/RIGHT/CENTER buttons, drives the recording state machine, and
  prints lines the laptop parses (`RUN_START`, `ROW: ...`, `RUN_STOP`,
  `SELECTED: n`, `REPLAY_REQUEST: n`).
- `record_run.py` -- runs on the laptop. Holds one persistent Bluetooth
  connection *per robot*, up to `NUM_ROBOTS` (set at the top of the file)
  connected at once, keeps `update_robot_code_pybricks.py` running on
  each, logs recordings to `runs/*.csv`, and handles replay requests per
  robot by building and running a one-off replay program on that hub.
  This is the thing you actually run: `python3 record_run.py`. Robots
  aren't individually identified -- each connects to whichever Pybricks
  hub is currently advertising (a hub stops advertising once connected,
  so concurrent connection attempts naturally spread across different
  physical hubs), and all of them share the same `runs/` folder.
- `push_ble.py` -- a simpler one-off tool: `python3 push_ble.py [script.py]`
  pushes and runs a single script over Bluetooth, streaming its output, then
  exits. Useful for testing a script by hand. `record_run.py` shells out to
  this internally for some cases, but mostly talks to the hub directly.
- `generate_deterministic.py` -- `python3 generate_deterministic.py <n>`
  converts `runs/<n>_run_*.csv` into `runs/<n>_deterministic.py`: a flat,
  hand-editable Pybricks program (`drive()`/`turn()`/`arm_to()` calls with
  gyro-based heading correction) instead of the data-table approach
  `record_run.py`'s own replay uses. `record_run.py` also runs this
  automatically right after every recording is saved (see below), writing
  `runs/<original CSV name>_deterministic.py` next to it -- so only run
  this script by hand to regenerate one on demand (e.g. after editing the
  generator itself). Either way, hand-tune the `PROGRAM` list in the
  output file afterward: each step is `("drive", dist, speed)`,
  `("turn", angle, speed)`, or `("arm", right, left, speed)` -- `speed` is
  deg/s, editable per step. Push it to the hub with
  `python3 push_ble.py runs/<n>_..._deterministic.py`.
- `runs/` -- recorded telemetry shared across all connected robots, one
  CSV per recording. See `runs/README.md`.

## Hub controls (running update_robot_code_pybricks.py)

The display shows `H` (home) when idle.

- **RIGHT** at home: start recording. Display shows `R`, hub light turns red.
- **CENTER** while recording: stop and save it.
- **LEFT**: count up (display shows the number) -- used to pick a run to
  replay.
- **RIGHT** with a number selected: count back down towards home.
- **CENTER** with a number selected (not `H`): request a replay of
  `runs/<n>_run_*.csv` (shared across all connected robots) -- or, if a
  matching `runs/<n>_run_*_deterministic.py` exists, that's run directly
  instead (see below).
- **CENTER pressed 3 times** during a replay: abort it and return home.

Each connected robot runs this independently and concurrently -- recording
or replaying on one doesn't block or interact with any other.

## Recording -> replay workflow

1. Run `python3 record_run.py` and leave it running in a terminal -- it
   connects to up to `NUM_ROBOTS` Pybricks hubs, whichever are currently
   on and in range.
2. On a hub, RIGHT to record, move the robot, CENTER to stop. This saves
   `runs/run_<timestamp>.csv` (not numbered automatically), and
   immediately also writes `runs/run_<timestamp>_deterministic.py` next
   to it (see `generate_deterministic.py` above) -- generated fresh every
   time, so don't hand-edit it until after the number-renaming step below.
3. Rename the run(s) you want replayable to `<n>_run_...csv` yourself, e.g.
   `3_run_20260904_193417.csv`, **and** rename its matching
   `..._deterministic.py` file to the same `<n>_run_..._deterministic.py`
   pattern if you want that one preferred. Multiple CSVs can share a
   prefix (e.g. `3_run_a.csv`, `3_run_b.csv`) -- replaying `3` averages
   all of them sample-by-sample (truncated to the shortest one) when no
   deterministic script is found for that prefix.
4. On any connected hub, LEFT to count up to `n`, then CENTER to replay.
   If `runs/<n>_run_*_deterministic.py` exists (pick the most recently
   modified one if several match), it's pushed and run as-is, numbered
   CSVs and all; otherwise falls back to the table-driven CSV replay.
   This means hand-edits you make to a deterministic script are what
   actually runs -- it's never silently regenerated or overwritten by
   `record_run.py` once you've renamed/numbered it.

Each recorded row has cumulative drive motor angles (`left`, `right`), the
change since the last sample (`dleft`, `dright`), the hub's IMU `heading`
(reset to 0 at the start of the recording), and cumulative/delta angles for
the two attachment motors (`right_arm`/`left_arm`, `dright_arm`/`dleft_arm`
on ports C and D).

### Replay model

Based on GummyBears Robotics' "trendline navigation" approach
(https://www.youtube.com/watch?v=YvdNfw3_fhA). **Distance is the master
variable; heading is a function of distance.** The robot drives its recorded
distance profile while a P-controller steers so the gyro heading matches the
recorded heading for wherever it currently is along the path.

- Distance is the wheel-encoder proxy `(right_angle - left_angle) / 2`.
- The path is parameterized by cumulative *absolute* path length `s`, so a
  run that reverses is still single-valued.
- Each control tick runs two P-controllers:
  - `drive = KP_DIST * (target_dist - actual_dist)` -- goes negative and
    reverses automatically when the recorded path doubles back.
  - `steer = KP_HEAD * (target_heading - gyro_heading)`.
- Stops when `s` reaches the recorded total. Timing is not preserved.

The gains (`KP_DIST`, `KP_HEAD`, `MAX_SPEED`) live in `REPLAY_TEMPLATE` in
`record_run.py` and still need tuning for this robot. Replay prints a
progress line every ~15 ticks: `s=.../... dist X->Y head A->B` (actual ->
target).

## Known issues / things to watch for

- **BLE reconnect is slow and flaky on this Mac.** Disconnecting and
  reconnecting Bluetooth between program pushes was unreliable -- it could
  take 15 seconds to over a minute for the hub to become discoverable again
  after a connection drops. `record_run.py` works around this by holding one
  persistent connection for the whole session and using Pybricks' own
  stop/start-program commands to swap scripts, instead of disconnecting.
  If the hub *does* disconnect (power cycle, out of range, crash), the
  script detects it and loops back to reconnecting -- this used to spin in
  a runaway zero-delay loop before it was fixed; if you ever see it hammering
  the terminal with the same error over and over, that regression is back.
- **Old `pybricksdev` version.** This Mac's system `python3` is 3.9, which
  pins `pybricksdev` to an old release (1.0.0a49) that predates some details
  of the current firmware's protocol. Both `push_ble.py` and `record_run.py`
  monkeypatch two things to compensate: `find_device()` matching by BLE
  service UUID only (macOS often doesn't surface an advertised name), and
  `unpack_hub_capabilities()` parsing only the first 10 bytes (newer firmware
  appends one more). To retire these patches: install Python 3.11+ and a
  current `pybricksdev`.
- **Replay gains still need tuning.** `CRUISE`, `KP_HEAD`, `KP_SYNC`,
  `MAX_CMD` in `REPLAY_TEMPLATE` were guessed. If steering goes the wrong
  way, flip the sign on the `turn` term. There's a watchdog so a stuck
  replay can't run forever, but it may still need a 3x-CENTER abort or a hub
  power-cycle if the robot does something unwanted.
- **`DIST_SCALE` calibration.** Replay prints `target end dist X` at the
  start and `end dist Y/X` at the end. If replay consistently runs short or
  long by a roughly fixed ratio, set `DIST_SCALE` in `REPLAY_TEMPLATE`
  (e.g. 1.1 if it runs ~10% short). After the path completes there's a
  finish phase that drives straight to the recorded end distance/heading.
- **Distance from the encoders, not the gyro.** The IMU can't give a usable
  distance (accelerometer integration drifts badly at robot speeds), so the
  wheel encoders are the distance authority. Wheel slip still means encoder
  distance can differ from real ground distance; there's no on-robot sensor
  to close that gap.
