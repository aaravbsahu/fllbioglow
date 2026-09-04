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
  connection to the hub, keeps `update_robot_code_pybricks.py` running on it,
  logs recordings to `runs/*.csv`, and handles replay requests by building
  and running a one-off replay program on the hub. This is the thing you
  actually run: `python3 record_run.py`.
- `push_ble.py` -- a simpler one-off tool: `python3 push_ble.py [script.py]`
  pushes and runs a single script over Bluetooth, streaming its output, then
  exits. Useful for testing a script by hand. `record_run.py` shells out to
  this internally for some cases, but mostly talks to the hub directly.
- `runs/` -- recorded telemetry, one CSV per recording. See `runs/README.md`.

## Hub controls (running update_robot_code_pybricks.py)

The display shows `H` (home) when idle.

- **RIGHT** at home: start recording. Display shows `R`, hub light turns red.
- **CENTER** while recording: stop and save it.
- **LEFT**: count up (display shows the number) -- used to pick a run to
  replay.
- **RIGHT** with a number selected: count back down towards home.
- **CENTER** with a number selected (not `H`): request a replay of
  `runs/<n>_run_*.csv`.
- **CENTER pressed 3 times** during a replay: abort it and return home.

## Recording -> replay workflow

1. Run `python3 record_run.py` and leave it running in a terminal.
2. On the hub, RIGHT to record, move the robot, CENTER to stop. This saves
   `runs/run_<timestamp>.csv` (not numbered automatically).
3. Rename the run(s) you want replayable to `<n>_run_...csv` yourself, e.g.
   `3_run_20260904_193417.csv`. Multiple files can share a prefix (e.g.
   `3_run_a.csv`, `3_run_b.csv`) -- replaying `3` averages all of them
   sample-by-sample (truncated to the shortest one).
4. On the hub, LEFT to count up to `n`, then CENTER to replay.

Each recorded row is `(dleft, dright, heading)` -- the motor angle change
since the last sample, plus the hub's IMU heading (reset to 0 at the start of
the recording). Replay does NOT just blindly re-run the recorded wheel
angles: for each step it moves the wheels by roughly the recorded amount,
then uses the IMU to correct the heading until it actually matches the
recorded value for that step, since wheel angle alone drifts from reality
(friction, wheel slip). Timing is not preserved on replay -- getting the
heading/position right matters more than matching the original speed.

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
- **The heading-correction replay is still being tuned.** The turn speed and
  timeouts in `record_run.py`'s `REPLAY_TEMPLATE` were guessed without
  knowing the robot's actual wheel diameter/track width, so convergence can
  be slow or fail to converge within the per-step timeout on sharp turns.
  Replay prints `STEP n/N after-move=X` and `STEP n/N target=X achieved=Y
  ok/TIMEOUT` for every step so you can see exactly where it's diverging.
  There's a 60-second hard watchdog cutoff so a stuck replay can't run
  forever, but it may still need to be aborted by hand (3x CENTER) or the
  hub power-cycled if the robot is doing something unwanted.
- **Not yet implemented:** using the recorded heading to actively verify the
  robot reached the right position rather than just matching heading -- the
  data is recorded and available (`runs/*.csv`'s `heading` column) but only
  the replay's per-step turn correction uses it so far.
