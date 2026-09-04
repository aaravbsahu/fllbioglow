# Runs

Motor telemetry logged from the robot, written by `record_run.py`.

On the hub: press the **right** button to start a recording, press **center** to
stop it. Each start/stop pair produces one CSV file here named
`run_YYYYMMDD_HHMMSS.csv`. Columns:

```
timestamp,left,right,dleft,dright
```

- `timestamp` — time on the laptop when the row was received (ISO 8601)
- `left` / `right` — cumulative motor angle on ports C and E
- `dleft` / `dright` — change since the previous sample

## Replaying a run

Recordings aren't numbered automatically -- rename the one you want to replay
to `<N>_run_...csv` yourself. Then on the hub, press **left** to count up to
that number (the display shows it), and press **center** to replay it.
`record_run.py` reads that file's `dleft`/`dright` columns and drives the
motors through the same sequence of moves.

This replaced the earlier Google Sheets logging (`computer_helper.py` / `app_script.gs`).
