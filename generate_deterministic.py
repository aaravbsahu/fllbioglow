#!/usr/bin/env python3
"""Generates a hardcoded, hand-editable Pybricks program from a recording.

Usage:
    python3 generate_deterministic.py <n>

Reads runs/<n>_run_*.csv (averaging multiple files sharing that prefix,
same as replay does), and writes runs/<n>_deterministic.py: a flat
sequence of drive()/turn()/arm_to() calls with gyro-based heading
correction, instead of the data-table-driven replay record_run.py builds
on the fly. Meant to be generated once and then hand-tuned -- edit the
PROGRAM list in the output file directly.

To run it on the hub: python3 push_ble.py runs/<n>_deterministic.py
"""
import sys
from pathlib import Path

import record_run as rr


def main():
    if len(sys.argv) != 2:
        print(f"Usage: python3 {sys.argv[0]} <n>")
        sys.exit(1)

    n = sys.argv[1]
    csv_paths = rr.find_run_csvs(n)
    if not csv_paths:
        print(f"No runs found starting with '{n}_run_' in {rr.RUNS_DIR}")
        sys.exit(1)

    out_path = rr.RUNS_DIR / f"{n}_deterministic.py"
    program = rr.build_deterministic_script(csv_paths, out_path)

    names = ", ".join(p.name for p in csv_paths)
    print(f"Built from {len(csv_paths)} run(s): {names}")
    print(f"{len(program)} step(s):")
    for step in program:
        print(f"  {step}")
    print(f"\nWrote {out_path}")
    print(f"Push it to the hub with: python3 push_ble.py {out_path}")


if __name__ == "__main__":
    main()
