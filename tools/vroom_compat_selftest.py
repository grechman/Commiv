#!/usr/bin/env python3
# Acceptance self-test for POST /compat/vroom (the vroom-compat adapter).
#
# It proves a REAL VROOM client can point at commiv unchanged:
#   1. PDPTW path: take a VROOM request JSON built by tools/vroom_road_pdptw.py
#      from a moneyroadbench dump (VROOM_PRINT_REQUEST=1), POST it UNMODIFIED to
#      /compat/vroom, and assert code:0, every shipment served, steps parse, and
#      the summary/route/step numbers reconcile with an INDEPENDENT schedule walk
#      over the returned routes (leave depot at t=0, travel, wait until ready,
#      serve, return) using the request's own durations + windows.
#   2. VRPTW path: a hand-built jobs request -> code:0, every job served, walk
#      reconciles.
#   3. Negative: a request without matrices -> HTTP 422 + {"code":1,...}.
#
# Usage:
#   COMPAT_URL=http://127.0.0.1:8080/compat/vroom \
#     python3 tools/vroom_compat_selftest.py /path/to/vroom_request.json
#
# The argv/REQ_JSON file is the PDPTW request; if omitted, only the VRPTW +
# negative cases run.
import json, os, sys, urllib.request, urllib.error

URL = os.environ.get("COMPAT_URL", "http://127.0.0.1:8080/compat/vroom")


def post(body):
    data = json.dumps(body).encode()
    req = urllib.request.Request(URL, data=data,
                                 headers={"content-type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=120) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        raw = e.read()
        try:
            return e.code, json.loads(raw)
        except Exception:
            return e.code, {"_raw": raw.decode("utf-8", "replace")}


def walk(routes, dur, ready, service):
    """Independent schedule walk. Returns per-route dicts + totals, mirroring the
    VROOM recurrence: depart depot at t=0; arrival = t + travel; wait until ready;
    serve; return to depot at the end."""
    tot = {"duration": 0, "service": 0, "waiting_time": 0}
    per = []
    for rt in routes:
        steps = rt["steps"]
        assert steps[0]["type"] == "start" and steps[0]["location_index"] == 0
        assert steps[-1]["type"] == "end" and steps[-1]["location_index"] == 0
        prev, t = 0, 0
        r_travel = r_service = r_wait = 0
        arrivals = []  # (step_index, expected_arrival, expected_cum_duration)
        for si in range(1, len(steps) - 1):
            node = steps[si]["location_index"]
            travel = dur[prev][node]
            r_travel += travel
            arrival = t + travel
            wait = max(0, ready.get(node, 0) - arrival)
            svc = service.get(node, 0)
            r_wait += wait
            r_service += svc
            arrivals.append((si, arrival, r_travel, wait, svc))
            t = arrival + wait + svc
            prev = node
        back = dur[prev][0]
        r_travel += back
        depot_arrival = t + back
        per.append({"route": rt, "duration": r_travel, "service": r_service,
                    "waiting_time": r_wait, "depot_arrival": depot_arrival,
                    "arrivals": arrivals})
        tot["duration"] += r_travel
        tot["service"] += r_service
        tot["waiting_time"] += r_wait
    return tot, per


def check_reconcile(label, req, out, expect_types):
    assert out.get("code") == 0, f"{label}: code != 0: {out}"
    s = out["summary"]
    assert s.get("unassigned", -1) == 0, f"{label}: summary.unassigned != 0"
    assert out.get("unassigned") == [], f"{label}: unassigned list not empty"

    dur = req["matrices"]["car"]["durations"]
    ready, service = {0: 0}, {0: 0}
    served = set()
    expected = set()
    if "shipments" in req and req["shipments"]:
        for sh in req["shipments"]:
            for side in ("pickup", "delivery"):
                t = sh[side]
                loc = t["location_index"]
                tw = t.get("time_windows", [])
                ready[loc] = tw[0][0] if tw else 0
                service[loc] = t.get("service", 0)
                expected.add(loc)
    else:
        for jb in req["jobs"]:
            loc = jb["location_index"]
            tw = jb.get("time_windows", [])
            ready[loc] = tw[0][0] if tw else 0
            service[loc] = jb.get("service", 0)
            expected.add(loc)

    types_seen = {}
    for rt in out["routes"]:
        for st in rt["steps"]:
            ty = st["type"]
            types_seen[ty] = types_seen.get(ty, 0) + 1
            if ty in ("pickup", "delivery", "job"):
                assert "id" in st, f"{label}: task step missing id: {st}"
                served.add(st["location_index"])
    assert served == expected, (
        f"{label}: served {len(served)} nodes but expected {len(expected)} "
        f"(missing {sorted(expected - served)}, extra {sorted(served - expected)})")

    tot, per = walk(out["routes"], dur, ready, service)
    for k in ("duration", "service", "waiting_time"):
        assert s[k] == tot[k], f"{label}: summary.{k} {s[k]} != walk {tot[k]}"
    for p in per:
        rt = p["route"]
        for k in ("duration", "service", "waiting_time"):
            assert rt[k] == p[k], f"{label}: route.{k} {rt[k]} != walk {p[k]}"
        # step arrivals + cumulative travel reconcile
        for (si, arr, cum, wait, svc) in p["arrivals"]:
            st = rt["steps"][si]
            assert st["arrival"] == arr, f"{label}: step arrival {st['arrival']} != {arr}"
            assert st["duration"] == cum, f"{label}: step cum duration {st['duration']} != {cum}"
            assert st.get("waiting_time", 0) == wait, f"{label}: step wait mismatch"
            assert st.get("service", 0) == svc, f"{label}: step service mismatch"
        assert rt["steps"][-1]["arrival"] == p["depot_arrival"], f"{label}: end arrival mismatch"
    for t in expect_types:
        assert types_seen.get(t, 0) > 0, f"{label}: expected step type {t} not present"
    print(f"OK  {label}: code=0 routes={s['routes']} served={len(served)} "
          f"dur={s['duration']} service={s['service']} wait={s['waiting_time']} "
          f"cost={s['cost']}")


def test_pdptw(path):
    req = json.load(open(path))
    n_ship = len(req["shipments"])
    st, out = post(req)
    assert st == 200, f"PDPTW: HTTP {st}: {out}"
    check_reconcile(f"PDPTW({n_ship} shipments)", req, out, ["start", "pickup", "delivery", "end"])


def test_vrptw():
    # Hand-built 4-customer VRPTW. depot=0, jobs at 1..4. Directed matrix.
    dur = [
        [0, 10, 12, 14, 16],
        [10, 0, 6, 8, 10],
        [12, 6, 0, 7, 9],
        [14, 8, 7, 0, 6],
        [16, 10, 9, 6, 0],
    ]
    req = {
        "matrices": {"car": {"durations": dur}},
        "vehicles": [{"id": i + 1, "start_index": 0, "end_index": 0,
                      "capacity": [10], "time_window": [0, 10000],
                      "costs": {"fixed": 1000}} for i in range(4)],
        "jobs": [
            {"id": 101, "location_index": 1, "delivery": [3], "service": 5,
             "time_windows": [[0, 500]]},
            {"id": 102, "location_index": 2, "delivery": [4], "service": 5,
             "time_windows": [[30, 500]]},
            {"id": 103, "location_index": 3, "delivery": [2], "service": 5,
             "time_windows": [[0, 500]]},
            {"id": 104, "location_index": 4, "delivery": [5], "service": 5,
             "time_windows": [[50, 500]]},
        ],
    }
    st, out = post(req)
    assert st == 200, f"VRPTW: HTTP {st}: {out}"
    check_reconcile("VRPTW(4 jobs)", req, out, ["start", "job", "end"])


def test_negative():
    st, out = post({"vehicles": [{"start_index": 0, "capacity": [10]}],
                    "jobs": [{"location_index": 1}]})
    assert st == 422, f"negative(no matrix): expected 422, got {st}: {out}"
    assert out.get("code") == 1 and "error" in out, f"negative: bad shape {out}"
    print(f"OK  negative(no matrix): HTTP 422 {out['error']!r}")


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("REQ_JSON")
    if path:
        test_pdptw(path)
    else:
        print("SKIP PDPTW (no request json given)", file=sys.stderr)
    test_vrptw()
    test_negative()
    print("ALL GREEN")
