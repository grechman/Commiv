#!/usr/bin/env python3
# VROOM head-to-head on Li & Lim PDPTW: same integer matrix as commiv's bench
# (round(euclid*1000), times scaled x1000), shipments = pickup/delivery pairs,
# fleet = the BKS vehicle count (same convention as the LKH harness).
# Usage: VROOM_BIN=... VROOM_TIME=10 python3 tools/vroom_pdptw.py "lc101,lr101"
import json, math, os, subprocess, sys, time

VENDOR = os.environ.get("VROOM_DIR", "vendor/pdptw")
BIN = os.environ.get("VROOM_BIN", os.path.expanduser("~/cbench/vroom/bin/vroom"))
T = int(os.environ.get("VROOM_TIME", "10"))
SCALE = 1000
# VROOM_FLEETMIN=1: give 2x the BKS fleet but a large fixed cost per used
# vehicle (mirrors commiv's veh_penalty) so VROOM minimizes fleet itself.
FLEETMIN = os.environ.get("VROOM_FLEETMIN", "0") == "1"
FIXED = 10_000_000

def parse(name):
    lines = open(f"{VENDOR}/{name}.txt").read().split("\n")
    k, cap, _ = map(int, lines[0].split())
    nodes = []
    for ln in lines[1:]:
        f = ln.split()
        if len(f) < 9: continue
        nodes.append(tuple(map(int, f)))
    return k, cap, nodes

def bks_veh(name):
    return sum(1 for ln in open(f"{VENDOR}/{name}.sol") if ln.strip().startswith("Route"))

def run(name):
    _, cap, nodes = parse(name)
    dim = len(nodes)
    xs = [n[1] for n in nodes]; ys = [n[2] for n in nodes]
    mat = [[round(math.dist((xs[a], ys[a]), (xs[b], ys[b])) * SCALE) for b in range(dim)] for a in range(dim)]
    K = bks_veh(name)
    horizon = nodes[0][5] * SCALE
    shipments = []
    for n in nodes[1:]:
        nid, x, y, dem, rdy, due, srv, pick, deliv = n
        if dem <= 0: continue
        d = next(m for m in nodes if m[0] == deliv)
        shipments.append({
            "amount": [dem],
            "pickup": {"id": nid, "location_index": nid,
                       "time_windows": [[rdy*SCALE, due*SCALE]], "service": srv*SCALE},
            "delivery": {"id": d[0], "location_index": d[0],
                         "time_windows": [[d[4]*SCALE, d[5]*SCALE]], "service": d[6]*SCALE},
        })
    nveh = 2 * K if FLEETMIN else K
    vehicles = [{"id": i+1, "start_index": 0, "end_index": 0, "capacity": [cap],
                 "time_window": [0, horizon]} for i in range(nveh)]
    if FLEETMIN:
        for v in vehicles:
            v["costs"] = {"fixed": FIXED}
    query = {"shipments": shipments, "vehicles": vehicles,
             "matrices": {"car": {"durations": mat, "costs": mat}}}
    t0 = time.time()
    r = subprocess.run([BIN, "-t", os.environ.get("VROOM_THREADS", "1"), "-x", "5", "-l", str(T)],
                       input=json.dumps(query), capture_output=True, text=True, timeout=T*3+60)
    wall = time.time() - t0
    try:
        out = json.loads(r.stdout)
    except Exception:
        return name, "ERROR", 0, wall, r.stderr[:200]
    if out.get("code", 1) != 0:
        return name, "ERROR", 0, wall, str(out.get("error"))[:200]
    unass = len(out["summary"].get("unassigned", [])) if isinstance(out["summary"].get("unassigned"), list) else out["summary"].get("unassigned", 0)
    cost = out["summary"]["cost"]
    used = sum(1 for rt in out.get("routes", []) if rt.get("steps"))
    if FLEETMIN:
        cost -= FIXED * used  # report pure distance; fleet shows in veh=
    return name, cost, unass, wall, f"veh={used}"

if __name__ == "__main__":
    for name in sys.argv[1].split(","):
        n, cost, unass, wall, extra = run(name)
        print(f"{n},{cost},{unass},{wall:.1f},{extra}", flush=True)
