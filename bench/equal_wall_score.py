#!/usr/bin/env python3
"""Score paired equal-wall rows from bench/equal_wall_runner.py.

usage: equal_wall_score.py money|pdptw RESULTS_JSONL

money: $ = 140 * veh + 0.5 * dist + 0.5 * dur; lower wins.
pdptw: fewer vehicles wins, then lower distance; rows without a parsed
result count as incomplete and lose to any complete row.
"""

from __future__ import annotations

import collections
import json
import sys


def parse(row: str):
    f = row.split(",")
    if len(f) != 10:
        return None
    return {
        "veh": int(f[3]),
        "dist": float(f[5]),
        "ms": int(f[7]),
        "dur": float(f[8]),
        "wait": float(f[9]),
    }


def money(r):
    return 140.0 * r["veh"] + 0.5 * r["dist"] + 0.5 * r["dur"]


def compare(mode: str, old, new) -> str:
    if old is None and new is None:
        return "T"
    if old is None:
        return "W"
    if new is None:
        return "L"
    if mode == "money":
        a, b = money(old), money(new)
        if abs(a - b) < 1e-6:
            return "T"
        return "W" if b < a else "L"
    if new["veh"] != old["veh"]:
        return "W" if new["veh"] < old["veh"] else "L"
    if abs(new["dist"] - old["dist"]) < 1e-6:
        return "T"
    return "W" if new["dist"] < old["dist"] else "L"


def main() -> int:
    mode, path = sys.argv[1], sys.argv[2]
    cells: dict[str, dict] = collections.defaultdict(dict)
    meta = None
    for line in open(path):
        rec = json.loads(line)
        if rec.get("family") == "meta":
            meta = rec
            continue
        cells[rec["_cell"]][rec["bin"]] = rec
    per_size = collections.defaultdict(
        lambda: {
            "n": 0,
            "W": 0,
            "T": 0,
            "L": 0,
            "old": 0.0,
            "new": 0.0,
            "old_veh": 0,
            "new_veh": 0,
            "old_dist": 0.0,
            "new_dist": 0.0,
            "incomplete": 0,
        }
    )
    for cid in sorted(cells):
        pair = cells[cid]
        if "old" not in pair or "new" not in pair:
            continue
        old = parse(pair["old"]["row"]) if "row" in pair["old"] else None
        new = parse(pair["new"]["row"]) if "row" in pair["new"] else None
        s = per_size[pair["old"]["size"]]
        s["n"] += 1
        s[compare(mode, old, new)] += 1
        if old is None or new is None:
            s["incomplete"] += 1
            continue
        if mode == "money":
            s["old"] += money(old)
            s["new"] += money(new)
        s["old_veh"] += old["veh"]
        s["new_veh"] += new["veh"]
        s["old_dist"] += old["dist"]
        s["new_dist"] += new["dist"]
    if meta:
        print(f"old {meta['sha256_old'][:12]} new {meta['sha256_new'][:12]}")
    tot = {
        "n": 0,
        "W": 0,
        "T": 0,
        "L": 0,
        "old": 0.0,
        "new": 0.0,
        "old_veh": 0,
        "new_veh": 0,
        "old_dist": 0.0,
        "new_dist": 0.0,
        "incomplete": 0,
    }
    if mode == "money":
        print("| size | cells | new W/T/L | old $ | new $ | delta | incomplete |")
        print("|---:|---:|---|---:|---:|---:|---:|")
    else:
        print(
            "| size | cells | new W/T/L | old veh | new veh | old dist | new dist | incomplete |"
        )
        print("|---:|---:|---|---:|---:|---:|---:|---:|")
    for size in sorted(per_size, key=int):
        s = per_size[size]
        for k in tot:
            tot[k] += s[k]
        if mode == "money":
            d = (s["new"] - s["old"]) / s["old"] * 100 if s["old"] else 0.0
            print(
                f"| {size} | {s['n']} | {s['W']}/{s['T']}/{s['L']} | {s['old']:,.2f} | {s['new']:,.2f} | {d:+.3f}% | {s['incomplete']} |"
            )
        else:
            print(
                f"| {size} | {s['n']} | {s['W']}/{s['T']}/{s['L']} | {s['old_veh']} | {s['new_veh']} | {s['old_dist']:,.1f} | {s['new_dist']:,.1f} | {s['incomplete']} |"
            )
    if mode == "money":
        d = (tot["new"] - tot["old"]) / tot["old"] * 100 if tot["old"] else 0.0
        print(
            f"| all | {tot['n']} | {tot['W']}/{tot['T']}/{tot['L']} | {tot['old']:,.2f} | {tot['new']:,.2f} | {d:+.3f}% | {tot['incomplete']} |"
        )
    else:
        print(
            f"| all | {tot['n']} | {tot['W']}/{tot['T']}/{tot['L']} | {tot['old_veh']} | {tot['new_veh']} | {tot['old_dist']:,.1f} | {tot['new_dist']:,.1f} | {tot['incomplete']} |"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
