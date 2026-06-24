"""LKH-3 on directed CVRP with a FEASIBLE WARMSTART. The plain run starts from a WALK
tour that collapses to ~10 overloaded routes and never recovers at n>=1000. Here we
build a capacity-feasible sweep solution from the lng/lat coords and hand it to LKH as
INITIAL_TOUR_FILE (depot=node 1, salesmen copies = nodes N+1..N+k-1 separate routes),
so LKH starts feasible and only has to optimize distance.

Usage: lkh_warm.py <road> <time_s> <lkh_binary> [move_type]
"""
import sys
import os
import time
import math
import subprocess
import roadlib
from lkh_cvrp import write_vrp, parse_tour

SCRATCH = os.path.dirname(os.path.abspath(__file__))


def sweep(coords, demand, cap, dim):
    """Classic sweep: sort customers by polar angle from depot, fill routes to capacity."""
    lng0, lat0 = coords[0]
    custs = sorted(range(1, dim), key=lambda i: math.atan2(coords[i][1] - lat0, coords[i][0] - lng0))
    routes, cur, load = [], [], 0
    for c in custs:
        d = demand[c]
        if cur and load + d > cap:
            routes.append(cur)
            cur, load = [], 0
        cur.append(c)
        load += d
    if cur:
        routes.append(cur)
    return routes


def write_init_tour(path, dim, routes, vehicles):
    """LKH internal tour over 1..(dim + vehicles - 1): depot=1, copies=dim+1.. separate
    the `vehicles` routes (first len(routes) hold customers, the rest are empty for slack).
    Our customer i (1-indexed in 1..dim-1) -> LKH node i+1."""
    seq = []
    for k in range(vehicles):
        seq.append(1 if k == 0 else dim + k)     # depot for route 0, copy node for the rest
        if k < len(routes):
            seq.extend(c + 1 for c in routes[k])
    with open(path, 'w') as f:
        f.write(f"NAME : init\nTYPE : TOUR\nDIMENSION : {dim + vehicles - 1}\nTOUR_SECTION\n")
        f.write('\n'.join(map(str, seq)))
        f.write('\n-1\nEOF\n')


def main():
    path, tlimit, lkh = sys.argv[1], int(sys.argv[2]), sys.argv[3]
    move_type = sys.argv[4] if len(sys.argv) > 4 else "5"
    dim, cap, demand, M, coords = roadlib.parse_road(path)
    base = os.path.splitext(os.path.basename(path))[0]
    vrp = os.path.join(SCRATCH, base + '.vrp')
    par = os.path.join(SCRATCH, base + '_warm.par')
    init = os.path.join(SCRATCH, base + '.init')
    tour = os.path.join(SCRATCH, base + '_warm.tour')

    write_vrp(vrp, dim, cap, demand, M)
    routes0 = sweep(coords, demand, cap, dim)
    r = len(routes0)
    vehicles = r + max(2, r // 10)        # slack vehicles so LKH can stay feasible while optimizing
    write_init_tour(init, dim, routes0, vehicles)
    sweep_cost = roadlib.score(M, dim, routes0)
    sweep_err = roadlib.validate(dim, cap, demand, routes0)

    with open(par, 'w') as f:
        f.write(f"PROBLEM_FILE = {vrp}\nTIME_LIMIT = {tlimit}\nRUNS = 1\n")
        f.write(f"VEHICLES = {vehicles}\nMTSP_MIN_SIZE = 0\nINITIAL_TOUR_FILE = {init}\n")
        f.write(f"MOVE_TYPE = {move_type}\nMAX_CANDIDATES = 6\nINITIAL_PERIOD = 100\n")
        f.write(f"TOUR_FILE = {tour}\nSEED = 1\nTRACE_LEVEL = 1\n")

    print(f"# sweep: {r} routes ({vehicles} vehicles), cost {sweep_cost}, valid={sweep_err or 'OK'}")
    # Clear any stale tour from a prior run on this instance: LKH writes TOUR_FILE
    # only on success, so a leftover file would be mis-parsed as this run's result.
    if os.path.exists(tour):
        os.remove(tour)
    t0 = time.time()
    try:
        res = subprocess.run([lkh, par], capture_output=True, text=True, timeout=tlimit + 600)
    except subprocess.TimeoutExpired:
        el = time.time() - t0
        print(f"lkh-warm,{path},{dim},NA,NA,{el:.1f},TIMEOUT")
        return
    el = time.time() - t0
    with open(os.path.join(SCRATCH, base + '_warm.log'), 'w') as lf:
        lf.write(res.stdout or "")
    if not os.path.exists(tour):
        print(f"lkh-warm,{path},{dim},NA,NA,{el:.1f},NO_TOUR: {(res.stdout or res.stderr)[-300:]}")
        return
    routes = parse_tour(tour, dim)
    cost = roadlib.score(M, dim, routes)
    err = roadlib.validate(dim, cap, demand, routes)
    print(f"lkh-warm,{path},{dim},{cost},{len(routes)},{el:.1f},{'OK' if not err else err}")


if __name__ == '__main__':
    main()
