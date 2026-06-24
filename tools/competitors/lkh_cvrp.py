"""LKH-3 on the TRUE directed matrix (TYPE: ACVRP, explicit full matrix).
Usage: lkh_cvrp.py <road> <time_s> <lkh_binary>"""
import sys
import os
import time
import subprocess
import roadlib

SCRATCH = os.path.dirname(os.path.abspath(__file__))


def write_vrp(path, dim, cap, demand, M):
    with open(path, 'w') as f:
        f.write(f"NAME : prob\nTYPE : ACVRP\nDIMENSION : {dim}\nCAPACITY : {cap}\n")
        f.write("EDGE_WEIGHT_TYPE : EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\n")
        f.write("EDGE_WEIGHT_SECTION\n")
        for i in range(dim):
            b = i * dim
            f.write(' '.join(map(str, M[b:b + dim])))
            f.write('\n')
        f.write("DEMAND_SECTION\n")
        for i in range(dim):
            f.write(f"{i + 1} {demand[i]}\n")     # LKH 1-indexed; node 1 = depot
        f.write("DEPOT_SECTION\n1\n-1\nEOF\n")


def parse_tour(path, dim):
    seq = []
    in_sec = False
    with open(path) as f:
        for line in f:
            s = line.strip()
            if s == 'TOUR_SECTION':
                in_sec = True
                continue
            if not in_sec:
                continue
            if s in ('-1', 'EOF', ''):
                break
            seq.append(int(s))
    # rotate so a depot/depot-copy boundary is first, then linear split
    bnds = [k for k, v in enumerate(seq) if v == 1 or v > dim]
    if bnds:
        s = bnds[0]
        seq = seq[s:] + seq[:s]
    routes = []
    cur = []
    for v in seq:
        if v == 1 or v > dim:
            if cur:
                routes.append(cur)
                cur = []
        else:
            cur.append(v - 1)                       # LKH node -> our 0-indexed customer
    if cur:
        routes.append(cur)
    return routes


def main():
    path = sys.argv[1]
    tlimit = int(sys.argv[2])
    lkh = sys.argv[3]
    dim, cap, demand, M, coords = roadlib.parse_road(path)
    base = os.path.splitext(os.path.basename(path))[0]
    vrp = os.path.join(SCRATCH, base + '.vrp')
    par = os.path.join(SCRATCH, base + '.par')
    tour = os.path.join(SCRATCH, base + '.tour')

    write_vrp(vrp, dim, cap, demand, M)
    lb = -(-sum(demand) // cap)            # ceil(total_demand / capacity) = min fleet
    vehicles = lb + 2                      # +2 slack so the bin-packing is feasible
    with open(par, 'w') as f:
        f.write(f"PROBLEM_FILE = {vrp}\nTIME_LIMIT = {tlimit}\nRUNS = 1\n")
        f.write(f"VEHICLES = {vehicles}\nMTSP_MIN_SIZE = 0\n")   # allow empty routes (fleet <= vehicles)
        f.write("INITIAL_PERIOD = 100\n")                        # cut the alpha-nearness ascent (was ~n) so trials actually run
        f.write("MOVE_TYPE = 2\nMAX_CANDIDATES = 5\n")            # cheap 2-opt moves -> more trials (best config found at n=1000)
        f.write(f"TOUR_FILE = {tour}\nSEED = 1\nTRACE_LEVEL = 1\n")

    # LKH writes TOUR_FILE only on a successful run; a stale tour left by a prior
    # run on the same instance would otherwise be parsed as this run's result.
    if os.path.exists(tour):
        os.remove(tour)

    t0 = time.time()
    try:
        res = subprocess.run([lkh, par], capture_output=True, text=True, timeout=tlimit + 120)
    except subprocess.TimeoutExpired:
        el = time.time() - t0
        print(f"lkh,{path},{dim},NA,NA,{el:.1f},TIMEOUT")
        return
    el = time.time() - t0
    if not os.path.exists(tour):
        tail = (res.stdout or res.stderr)[-400:]
        print(f"lkh,{path},{dim},NA,NA,{el:.1f},NO_TOUR: {tail}")
        return

    routes = parse_tour(tour, dim)
    cost = roadlib.score(M, dim, routes)
    err = roadlib.validate(dim, cap, demand, routes)
    print(f"lkh,{path},{dim},{cost},{len(routes)},{el:.1f},{'OK' if not err else err}")


if __name__ == '__main__':
    main()
