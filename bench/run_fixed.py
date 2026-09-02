#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys

SOLVERS = {
    "vrptw": {
        "target": "vrptwbench",
        "binary": "zig-out/bin/commiv-vrptwbench",
        "instance": "c1_10_1",
        "fields": 10,
        "iters": 300_000,
        "env": lambda a: {"VT_DIR": "vendor/vrptw/gh", "VT_FILES": "c1_10_1", "VT_SEED": str(a.seed), "VT_ITERS": str(a.iters)},
    },
    "cvrp": {
        "target": "cvrpbench",
        "binary": "zig-out/bin/commiv-cvrpbench",
        "instance": "X-n1001-k43",
        "fields": 9,
        "iters": 600_000,
        "env": lambda a: {"CB_DIR": "vendor/cvrp_x", "CB_FILES": "X-n1001-k43", "CB_SISR": "1", "CB_THREADS": "1", "CB_SEED": str(a.seed), "CB_ITERS": str(a.iters)},
    },
}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("solver", choices=sorted(SOLVERS))
    ap.add_argument("--binary")
    ap.add_argument("--runs", type=int, default=5)
    ap.add_argument("--iters", type=int)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--build", action="store_true")
    args = ap.parse_args()
    cfg = SOLVERS[args.solver]
    if args.iters is None:
        args.iters = cfg["iters"]
    if args.runs < 1 or args.iters < 1:
        ap.error("--runs and --iters must be positive")

    root = Path(__file__).resolve().parents[1]
    if args.build:
        subprocess.run(["zig", "build", cfg["target"], "-Doptimize=ReleaseFast"], cwd=root, check=True)
    binary = (root / (args.binary or cfg["binary"])).resolve()
    if not binary.is_file():
        raise SystemExit(f"missing {binary}; pass --build")

    env = os.environ.copy()
    env.update(cfg["env"](args))
    prefix = cfg["instance"] + ","
    rows: list[list[str]] = []
    for run in range(args.runs):
        cp = subprocess.run([str(binary)], cwd=root, env=env, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=True)
        text = cp.stdout + cp.stderr
        row = next((line for line in text.splitlines() if line.startswith(prefix)), None)
        if row is None:
            raise SystemExit(f"run {run + 1}: benchmark row not found\n{text}")
        fields = row.split(",")
        if len(fields) != cfg["fields"]:
            raise SystemExit(f"run {run + 1}: unexpected row: {row}")
        print(f"run {run + 1}/{args.runs}: {row}", file=sys.stderr)
        rows.append(fields)

    signatures = {tuple(row[:7] + row[8:]) for row in rows}
    if len(signatures) != 1:
        raise SystemExit(f"fixed-seed result collision: {sorted(signatures)!r}")
    times = [int(float(row[7])) for row in rows]
    first = rows[0]
    print(json.dumps({
        "median_ms": statistics.median(times),
        "runs_ms": times,
        "solver": args.solver,
        "instance": first[0],
        "signature": ",".join(first[:7] + first[8:]),
        "seed": args.seed,
        "iters": args.iters,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
