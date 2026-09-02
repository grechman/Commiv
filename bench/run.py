#!/usr/bin/env python3
"""Fixed-work PDPTW throughput harness used by bench/config.json.

Defaults reproduce the frozen ordinary-PDPTW command in bench/config.json.
--time-pen/--veh-pen switch the same fixed-work protocol to money mode.

The solver owns its iteration budget (no wall timeout), so timing comparisons do
not silently compare different amounts of search. Raw rows go to stderr and the
last stdout line is one JSON object for the bench extractor.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--binary", default="zig-out/bin/commiv-pdptwbench")
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--iters", type=int, default=20_000)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--build", action="store_true")
    ap.add_argument("--time-pen", type=int, default=0, help="PB_TIMEPEN; >0 selects money mode")
    ap.add_argument("--veh-pen", type=int, default=0, help="PB_VEH_PEN; money mode fleet price")
    ap.add_argument("--dir", default="vendor/pdptw/1000")
    ap.add_argument("--instance", default="lr2_10_1")
    args = ap.parse_args()
    if args.runs < 1 or args.iters < 1:
        ap.error("--runs and --iters must be positive")

    root = Path(__file__).resolve().parents[1]
    if args.build:
        subprocess.run(
            ["zig", "build", "pdptwbench", "-Doptimize=ReleaseFast"],
            cwd=root,
            check=True,
        )
    binary = (root / args.binary).resolve()
    if not binary.is_file():
        raise SystemExit(f"missing {binary}; pass --build")

    env = os.environ.copy()
    env.update(
        PB_DIR=args.dir,
        PB_FILES=args.instance,
        PB_FLEET="0",
        PB_TIME_MS="0",
        PB_ITERS=str(args.iters),
        PB_SEED=str(args.seed),
        PB_TIMEPEN=str(args.time_pen),
        PB_VEH_PEN=str(args.veh_pen),
    )
    rows: list[list[str]] = []
    for run in range(args.runs):
        cp = subprocess.run(
            [str(binary)], cwd=root, env=env, text=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True,
        )
        text = cp.stdout + cp.stderr
        row = next((line for line in text.splitlines() if line.startswith(args.instance + ",")), None)
        if row is None:
            raise SystemExit(f"run {run + 1}: benchmark row not found\n{text}")
        fields = row.split(",")
        if len(fields) != 10:
            raise SystemExit(f"run {run + 1}: unexpected row: {row}")
        print(f"run {run + 1}/{args.runs}: {row}", file=sys.stderr)
        rows.append(fields)

    # Everything but elapsed milliseconds must be deterministic at fixed work.
    signatures = {tuple(row[:7] + row[8:]) for row in rows}
    if len(signatures) != 1:
        raise SystemExit(f"fixed-seed result collision: {sorted(signatures)!r}")
    times = [int(row[7]) for row in rows]
    first = rows[0]
    print(json.dumps({
        "median_ms": statistics.median(times),
        "runs_ms": times,
        "instance": first[0],
        "pairs": int(first[1]),
        "vehicles": int(first[3]),
        "distance": float(first[5]),
        "duration": float(first[8]),
        "wait": float(first[9]),
        "seed": args.seed,
        "iters": args.iters,
        "time_pen": args.time_pen,
        "veh_pen": args.veh_pen,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
