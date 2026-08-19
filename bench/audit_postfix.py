#!/usr/bin/env python3
"""Validate the immutable post-fix opposition artifacts and print a compact audit.

The default mode accepts the recorded GH competitor validity failures but reports
them. Pass --require-valid-gh to turn those known failures into a nonzero exit.
"""
from __future__ import annotations

import argparse
import collections
import hashlib
import json
from pathlib import Path

EXPECTED_FAMILIES = {
    "meta": 1,
    "money_real": 24,
    "road": 18,
    "roadtw": 18,
    "vrptw_gh": 30,
    "money": 704,
    "pdptw": 816,
}
EXPECTED_JOURNAL_PREFIXES = {
    "build": 1,
    "money-real": 24,
    "road": 12,
    "roadtw": 12,
    "gh": 20,
    "money": 704,
    "pdptw": 816,
}
SIZE_COUNTS = {"100": 56, "200": 60, "400": 58, "600": 60, "800": 60, "1000": 58}
COMMIT = "26c8a1cca3f97e6d38498a968c6a4e786c8815ca"


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def reject_constant(value: str):
    raise ValueError(f"non-finite JSON constant: {value}")


def logical_key(row: dict):
    return tuple(row.get(k) for k in ("family", "size", "instance", "solver", "seed", "driver"))


def parse_vroom(row: dict):
    fields = row["row"].split(",")
    assert len(fields) == 7 and fields[0] == row["instance"]
    tagged = dict(field.split("=", 1) for field in fields[4:])
    return {
        "dist": int(fields[1]),
        "unassigned": int(fields[2]),
        "vehicles": int(tagged["veh"]),
        "duration": int(tagged["dur"]),
        "wait": int(tagged["wait"]),
    }


def parse_commiv(row: dict):
    fields = row["row"].split(",")
    assert len(fields) == 10 and fields[0] == row["instance"]
    return {
        "vehicles": int(fields[3]),
        "dist": round(float(fields[5]) * 1000),
        "duration": round(float(fields[8]) * 1000),
        "wait": round(float(fields[9]) * 1000),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dir", type=Path, default=Path(__file__).resolve().parent)
    ap.add_argument("--require-valid-gh", action="store_true")
    ns = ap.parse_args()
    results = ns.dir / "postfix-remaining.jsonl"
    journal = ns.dir / "postfix-remaining.cells"

    raw = results.read_bytes()
    assert raw.endswith(b"\n")
    lines = raw.decode().splitlines()
    rows = [json.loads(line, parse_constant=reject_constant) for line in lines]
    assert len(rows) == 1611 == len(set(lines))
    assert len({logical_key(row) for row in rows}) == 1611
    assert collections.Counter(row["family"] for row in rows) == collections.Counter(EXPECTED_FAMILIES)
    meta = [row for row in rows if row["family"] == "meta"]
    assert meta == [{"commit": COMMIT, "family": "meta", "solver": "commiv"}]

    jraw = journal.read_bytes()
    assert jraw.endswith(b"\n")
    cells = jraw.decode().splitlines()
    assert len(cells) == len(set(cells)) == 1589
    prefixes = collections.Counter(cell.split("/", 1)[0] for cell in cells)
    assert prefixes == collections.Counter(EXPECTED_JOURNAL_PREFIXES)

    missing_tags = [logical_key(row) for row in rows if "_cell" not in row]
    assert len(missing_tags) == 4
    assert {key[2:] for key in missing_tags} == {
        (None, "commiv", None, None),
        ("moscow-100", "instance_dump", None, None),
        ("moscow-100", "commiv", None, "fleetmin"),
        ("moscow-100", "commiv", None, "plain"),
    }
    tagged = collections.Counter(row["_cell"] for row in rows if "_cell" in row)
    assert set(tagged) <= set(cells)
    assert set(cells) - set(tagged) == {
        "build",
        "money-real/moscow-100/dump",
        "money-real/moscow-100/commiv/fleetmin",
        "money-real/moscow-100/commiv/plain",
    }
    assert collections.Counter(tagged.values()) == {1: 1574, 3: 11}

    # Exact three-seed coverage for the grouped opposition families.
    expected_instances = {"road": 3, "roadtw": 3, "vrptw_gh": 5}
    for family, count in expected_instances.items():
        fam = [row for row in rows if row["family"] == family]
        names = {row["instance"] for row in fam}
        assert len(names) == count
        for instance in names:
            current = [row for row in fam if row["instance"] == instance]
            assert {row["solver"] for row in current} == {"commiv", "pyvrp"}
            for solver in ("commiv", "pyvrp"):
                assert {row["seed"] for row in current if row["solver"] == solver} == {1, 2, 3}

    # Li & Lim coverage and payload schemas.
    for family in ("money", "pdptw"):
        fam = [row for row in rows if row["family"] == family]
        for size, count in SIZE_COUNTS.items():
            names = {row["instance"] for row in fam if row["size"] == size}
            assert len(names) == count
        for instance in {row["instance"] for row in fam}:
            current = [row for row in fam if row["instance"] == instance]
            crows = [row for row in current if row["solver"] == "commiv"]
            vrows = [row for row in current if row["solver"] == "vroom"]
            assert len(vrows) == 1
            expected_seeds = {1, 2, 3} if family == "pdptw" and current[0]["size"] == "100" else {1}
            assert {row["seed"] for row in crows} == expected_seeds
            for row in crows:
                parse_commiv(row)
            parse_vroom(vrows[0])

    money = [row for row in rows if row["family"] == "money"]
    money_index = {(row["instance"], row["solver"]): row for row in money}
    commiv_total = vroom_total = 0
    wtl = collections.Counter()
    for instance in {row["instance"] for row in money}:
        c = parse_commiv(money_index[instance, "commiv"])
        v = parse_vroom(money_index[instance, "vroom"])
        assert v["unassigned"] == 0
        cs = 280000 * c["vehicles"] + c["dist"] + c["duration"]
        vs = 280000 * v["vehicles"] + v["dist"] + v["duration"]
        commiv_total += cs
        vroom_total += vs
        wtl["W" if cs < vs else "L" if cs > vs else "T"] += 1
    assert (commiv_total / 2000, vroom_total / 2000) == (10970675.9225, 12236718.412)
    assert (wtl["W"], wtl["T"], wtl["L"]) == (304, 26, 22)

    pd = [row for row in rows if row["family"] == "pdptw"]
    vroom_pd = [parse_vroom(row) for row in pd if row["solver"] == "vroom"]
    assert sum(v["unassigned"] > 0 for v in vroom_pd) == 238
    assert sum(v["unassigned"] for v in vroom_pd) == 5752

    # Family-specific completion/validity fields.
    real = [row for row in rows if row["family"] == "money_real"]
    assert len({row["instance"] for row in real}) == 6
    for instance in {row["instance"] for row in real}:
        current = [row for row in real if row["instance"] == instance]
        assert collections.Counter(row["solver"] for row in current) == {
            "instance_dump": 1, "commiv": 2, "vroom": 1
        }
        assert {row.get("driver") for row in current if row["solver"] == "commiv"} == {"fleetmin", "plain"}
        vfields = next(row for row in current if row["solver"] == "vroom")["row"].split(",")
        assert len(vfields) == 9 and int(vfields[7]) == 0

    for row in (row for row in rows if row["family"] == "road"):
        fields = row["row"].split(",")
        if row["solver"] == "commiv":
            assert len(fields) == 10 and int(fields[6]) == 0
        else:
            assert len(fields) == 8 and fields[7].lower() == "true"
    for row in (row for row in rows if row["family"] == "roadtw"):
        fields = row["row"].split(",")
        if row["solver"] == "commiv":
            assert len(fields) == 4
        else:
            assert len(fields) == 8 and fields[7].lower() == "true"
    for row in (row for row in rows if row["family"] == "vrptw_gh" and row["solver"] == "commiv"):
        fields = row["row"].split(",")
        assert len(fields) == 10 and fields[8].lower() == "true"

    gh_py = [row for row in rows if row["family"] == "vrptw_gh" and row["solver"] == "pyvrp"]
    gh_invalid = []
    for row in gh_py:
        fields = row["row"].split(",")
        assert len(fields) == 8 and fields[0] == "pyvrp" and fields[1] == row["instance"]
        if fields[6].lower() != "true":
            gh_invalid.append(f"{row['instance']}/s{row['seed']}")
    assert gh_invalid == [
        "c2_10_1/s2", "c2_10_1/s3",
        "r1_10_1/s1", "r1_10_1/s2", "r1_10_1/s3",
        "rc1_10_1/s1", "rc1_10_1/s2",
        "rc2_10_1/s1", "rc2_10_1/s3",
    ]

    report = {
        "commit": COMMIT,
        "results": {"rows": len(rows), "sha256": sha256(results)},
        "journal": {"cells": len(cells), "sha256": sha256(journal)},
        "money": {
            "commiv_total_usd": commiv_total / 2000,
            "vroom_total_usd": vroom_total / 2000,
            "wtl": [wtl["W"], wtl["T"], wtl["L"]],
        },
        "pdptw_vroom": {"incomplete_cells": 238, "unassigned_tasks": 5752},
        "gh_invalid_pyvrp": gh_invalid,
    }
    print(json.dumps(report, indent=2, sort_keys=True))
    return 2 if ns.require_valid_gh and gh_invalid else 0


if __name__ == "__main__":
    raise SystemExit(main())
