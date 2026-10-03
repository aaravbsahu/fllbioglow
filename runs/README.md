# Runs

Motor telemetry logged from the robot, written by `record_run.py`.

On the hub: press the **right** button to start a recording, press **center** to
stop it. Each start/stop pair produces one CSV file here named
`run_YYYYMMDD_HHMMSS.csv`. Columns:

```
timestamp,left,right,dleft,dright,heading,right_arm,left_arm,dright_arm,dleft_arm
```

- `timestamp` — time on the laptop when the row was received (ISO 8601)
- `left` / `right` — cumulative drive motor angle on ports F and E
- `dleft` / `dright` — change since the previous sample
- `heading` — hub IMU heading in degrees, reset to 0 at the start of the
  recording. This is the gold-standard reference for where the robot
  actually pointed/turned during the run, independent of wheel slip --
  motor angle alone can drift from reality due to wheel friction. Not used
  by replay yet; the plan is to later drive the robot to match these
  headings and verify it got there, rather than blindly replaying deltas.
- `right_arm` / `left_arm` — cumulative attachment motor angle on ports C
  and D (not drive motors -- these are the arm/attachment motors)
- `dright_arm` / `dleft_arm` — change since the previous sample. Not used
  by replay yet.

## Replaying a run

Recordings aren't numbered automatically -- rename the one(s) you want to
replay to `<N>_run_...csv` yourself. Then on the hub, press **left** to
count up to that number (the display shows it), and press **center** to
replay it.

Multiple files can share the same `<N>` prefix (e.g. `3_run_a.csv`,
`3_run_b.csv`, `3_run_c.csv`). `record_run.py` averages their
`dleft`/`dright` columns sample by sample -- truncated to the length of the
shortest one -- and drives the motors through the resulting averaged
sequence of moves. Pressing **center** three times during a replay stops it
early and returns home.

This replaced the earlier Google Sheets logging (`computer_helper.py` / `app_script.gs`).
