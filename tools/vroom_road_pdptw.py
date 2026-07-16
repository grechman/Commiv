#!/usr/bin/env python3
# VROOM head-to-head on the REAL directed-road PDPTW money instance produced by
# examples/moneyroadbench.zig (MR_DUMP=...). Reads that dump JSON, builds the
# closest VROOM can express (custom directed duration matrix + shipments +
# per-vehicle fixed cost = the only fleet/money lever VROOM has; it cannot price
# route DURATION or WAITING, which is exactly the commiv edge), runs the vroom
# binary at equal wall, then scores VROOM's plan under the MONEY_REAL Prices model.
#
# Prices model (MONEY_REAL.md §2): dollars = (drive_sec + 7*dur_sec + 30000*veh) * 0.0015
#   drive_sec = travel only  (VROOM summary "duration" = travel time only, verified
#               against VROOM API docs and mirrored from tools/vroom_pdptw.py)
#   dur_sec   = drive + service + waiting  (total route duration commiv prices)
#
# Usage:
#   # 1. dump the instance (no solve):
#   MR_FILE=nyc-100 MR_DUMP=/tmp/nyc100.json MR_SOLVE=0 ./zig-out/bin/commiv-moneyroadbench
#   # 2. score VROOM on it:
#   VROOM_BIN=~/cbench/vroom/bin/vroom VROOM_TIME=30 python3 tools/vroom_road_pdptw.py /tmp/nyc100.json
#
# Env:
#   VROOM_BIN     path to vroom binary (default ~/cbench/vroom/bin/vroom)
#   VROOM_TIME    equal-wall exploration budget seconds (default 10) -> vroom -l
#   VROOM_FIXED   per-vehicle fixed cost fed to VROOM to steer fleet-min
#                 (default 30000 = MR_VEH_PEN; only steers VROOM, not our scoring)
#   MR_DUMP       dump path (alternative to argv[1])
#   VROOM_PRINT_REQUEST=1  print the constructed request JSON to stdout and exit
#                          (no vroom run; for local unit verification)
import json, os, subprocess, sys, time

BIN = os.environ.get("VROOM_BIN", os.path.expanduser("~/cbench/vroom/bin/vroom"))
T = int(os.environ.get("VROOM_TIME", "10"))
# Prices model constants (MONEY_REAL.md §2). Overridable but default to the real model.
TIMEPEN = float(os.environ.get("MR_TIMEPEN", "7"))       # driver/operating rate ratio
VEHPEN = float(os.environ.get("MR_VEH_PEN", "30000"))    # fixed-cost in money units
UNIT = float(os.environ.get("MR_MONEY_UNIT", "0.0015"))  # dollars per money unit
FIXED = int(os.environ.get("VROOM_FIXED", "30000"))      # VROOM steering fixed cost


def build_request(d):
    """Construct the VROOM request from the moneyroadbench dump dict."""
    matrix = d["matrix"]
    cap = d["capacity"]
    ready = d["ready"]
    due = d["due"]
    service = d["service"]
    n_pairs = d["n_pairs"]

    shipments = []
    for pr in d["pairs"]:
        p, q, amt = pr["pickup"], pr["delivery"], pr["amount"]
        shipments.append({
            "amount": [amt],
            "pickup": {"id": p, "location_index": p,
                       "time_windows": [[ready[p], due[p]]], "service": service[p]},
            "delivery": {"id": q, "location_index": q,
                         "time_windows": [[ready[q], due[q]]], "service": service[q]},
        })
    # Free fleet: one vehicle per shipment (always enough), each with a fixed
    # cost so VROOM minimizes the fleet it actually uses (its best money proxy).
    horizon = due[0]
    vehicles = [{"id": i + 1, "start_index": 0, "end_index": 0,
                 "capacity": [cap], "time_window": [0, horizon],
                 "costs": {"fixed": FIXED}} for i in range(n_pairs)]
    return {"shipments": shipments, "vehicles": vehicles,
            "matrices": {"car": {"durations": matrix, "costs": matrix}}}


def count_unassigned_shipments(out):
    un = out.get("unassigned", [])
    if isinstance(un, list) and un:
        pk = sum(1 for u in un if u.get("type", "pickup") == "pickup")
        return pk if pk else len(un)
    return out.get("summary", {}).get("unassigned", 0)


def run(path):
    d = json.load(open(path))
    name = d.get("name", os.path.basename(path))
    n_pairs = d["n_pairs"]
    req = build_request(d)

    if os.environ.get("VROOM_PRINT_REQUEST") == "1":
        print(json.dumps(req, indent=2))
        return None

    t0 = time.time()
    try:
        r = subprocess.run(
            [BIN, "-t", os.environ.get("VROOM_THREADS", "1"), "-x", "5", "-l", str(T)],
            input=json.dumps(req), capture_output=True, text=True, timeout=T * 3 + 60)
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError) as e:
        wall = time.time() - t0
        return f"{name},{n_pairs},ERROR,0,0,0,0,0,{wall:.1f}  # {type(e).__name__}: {str(e)[:120]}", 1
    wall = time.time() - t0

    if r.returncode != 0 and not r.stdout.strip():
        return f"{name},{n_pairs},ERROR,0,0,0,0,0,{wall:.1f}  # exit={r.returncode} {r.stderr[:120]}", 1
    try:
        out = json.loads(r.stdout)
    except Exception:
        return f"{name},{n_pairs},ERROR,0,0,0,0,0,{wall:.1f}  # unparseable stdout {r.stderr[:120]}", 1
    if out.get("code", 1) != 0:
        return f"{name},{n_pairs},ERROR,0,0,0,0,0,{wall:.1f}  # vroom code={out.get('code')} {str(out.get('error'))[:120]}", 1

    s = out["summary"]
    drive_sec = s.get("duration", 0)                       # travel only
    wait_sec = s.get("waiting_time", 0)
    svc = s.get("service", 0)
    dur_sec = drive_sec + wait_sec + svc                   # total route duration
    used = sum(1 for rt in out.get("routes", []) if rt.get("steps"))
    unassigned = count_unassigned_shipments(out)
    usd = (drive_sec + TIMEPEN * dur_sec + VEHPEN * used) * UNIT
    # An incomplete plan (unassigned>0) served fewer stops with fewer vehicles, so
    # its dollar figure is artificially LOW and must never be scored as a win. Emit
    # a non-numeric sentinel in the usd column so no downstream overspend%.
    usd_field = f"{usd:.2f}" if unassigned == 0 else "INCOMPLETE"

    line = f"{name},{n_pairs},{used},{drive_sec},{dur_sec},{wait_sec},{usd_field},{unassigned},{wall:.1f}"
    if unassigned > 0:
        line += "  # INCOMPLETE plan not scored (unassigned>0)"
    return line, 0


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("MR_DUMP")
    if not path:
        sys.stderr.write("usage: vroom_road_pdptw.py <dump.json>  (or MR_DUMP=...)\n")
        sys.exit(2)
    res = run(path)
    if res is None:  # VROOM_PRINT_REQUEST path
        sys.exit(0)
    line, rc = res
    print(line, flush=True)
    sys.exit(rc)
