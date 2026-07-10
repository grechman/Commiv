#!/usr/bin/env python3
# OR-Tools routing head-to-head on Li & Lim PDPTW: same integer matrix
# (round(euclid*1000)), pickup-delivery pairs + time windows, fleet = BKS
# vehicle count, 1 search thread, guided local search with a time limit.
# Usage: OT_TIME=10 ~/cbench/venv/bin/python tools/ortools_pdptw.py "lc101,..."
import math, os, sys, time
from ortools.constraint_solver import pywrapcp, routing_enums_pb2

VENDOR = "vendor/pdptw"
T = int(os.environ.get("OT_TIME", "10"))
SCALE = 1000

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

    man = pywrapcp.RoutingIndexManager(dim, K, 0)
    r = pywrapcp.RoutingModel(man)

    def dist_cb(i, j):
        return mat[man.IndexToNode(i)][man.IndexToNode(j)]
    dcb = r.RegisterTransitCallback(dist_cb)
    r.SetArcCostEvaluatorOfAllVehicles(dcb)

    def time_cb(i, j):
        a = man.IndexToNode(i)
        return mat[a][man.IndexToNode(j)] + nodes[a][6] * SCALE
    tcb = r.RegisterTransitCallback(time_cb)
    r.AddDimension(tcb, horizon, horizon, False, "Time")
    tdim = r.GetDimensionOrDie("Time")
    for c in range(1, dim):
        idx = man.NodeToIndex(c)
        tdim.CumulVar(idx).SetRange(nodes[c][4] * SCALE, nodes[c][5] * SCALE)
    for v in range(K):
        tdim.CumulVar(r.Start(v)).SetRange(0, horizon)
        tdim.CumulVar(r.End(v)).SetRange(0, horizon)

    def dem_cb(i):
        return nodes[man.IndexToNode(i)][3]
    demcb = r.RegisterUnaryTransitCallback(dem_cb)
    r.AddDimensionWithVehicleCapacity(demcb, 0, [cap] * K, True, "Load")

    for n in nodes[1:]:
        if n[3] <= 0: continue
        p = man.NodeToIndex(n[0]); d = man.NodeToIndex(n[8])
        r.AddPickupAndDelivery(p, d)
        r.solver().Add(r.VehicleVar(p) == r.VehicleVar(d))
        r.solver().Add(tdim.CumulVar(p) <= tdim.CumulVar(d))

    # droppable visits so a first solution always exists; dropped pairs are
    # reported as incomplete (same reading as VROOM's unassigned)
    PEN = 100_000_000
    for n in nodes[1:]:
        if n[3] <= 0: continue
        r.AddDisjunction([man.NodeToIndex(n[0]), man.NodeToIndex(n[8])], PEN, 2)

    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PARALLEL_CHEAPEST_INSERTION
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.FromSeconds(T)
    params.number_of_solutions_to_collect = 1

    t0 = time.time()
    sol = r.SolveWithParameters(params)
    wall = time.time() - t0
    if sol is None:
        return name, "FAIL", 0, 0, wall
    dropped = 0
    idx = 0
    for c in range(1, dim):
        i = man.NodeToIndex(c)
        if sol.Value(r.NextVar(i)) == i and not r.IsStart(i):
            pass
    dropped = sum(1 for c in range(1, dim)
                  if nodes[c][3] > 0 and sol.Value(r.ActiveVar(man.NodeToIndex(c))) == 0)
    cost = sol.ObjectiveValue() - 100_000_000 * dropped
    used = sum(1 for v in range(K) if r.IsVehicleUsed(sol, v))
    return name, cost, dropped, used, wall

if __name__ == "__main__":
    for name in sys.argv[1].split(","):
        n, cost, dropped, veh, wall = run(name)
        print(f"{n},{cost},dropped={dropped},veh={veh},{wall:.1f}", flush=True)
