"""Directed VRPTW benchmark on the Moscow OSRM matrices — the actual last-mile
game: real directed travel times + delivery time slots.

Generates a deterministic time-window overlay on a moscow-*.road instance
(courier slots), solves it with commiv (via commiv-serve) and OR-Tools (time
dimension + GLS), and scores BOTH through the same independent validator below,
so neither solver's own accounting is trusted.

Window model (seed-deterministic):
  - service 300 s at every customer;
  - 60% of customers get one 2 h delivery slot out of four covering an 8 h
    shift; 40% are flexible (whole shift);
  - windows bound the START of service; waiting is free; every vehicle leaves
    the depot at t=0 and must be back by the 9 h horizon.

Usage:
  # commiv-serve must be listening (COMMIV_PORT=18090 zig-out/bin/commiv-serve)
  python3 vrptw_moscow.py ../../vendor/road/moscow-100.road commiv 18090 [rounds,restarts]
  python3 vrptw_moscow.py ../../vendor/road/moscow-100.road ortools 60      # arg = seconds
  python3 vrptw_moscow.py ../../vendor/road/moscow-100.road vroom 5         # arg = exploration
Prints: solver,instance,dim,cost,vehicles,seconds,valid
"""
import json
import math
import random
import sys
import time
import urllib.request

import roadlib

SERVICE = 300
SLOT = 7200            # 2 h slots
SHIFT = 4 * SLOT       # 8 h of slots
HORIZON = SHIFT + 3600 # + 1 h to get back to the depot
TW_SEED = 42
SLOTTED_FRACTION = 0.6


def make_windows(dim, M):
    rng = random.Random(TW_SEED)
    ready = [0] * dim
    due = [HORIZON] + [SHIFT] * (dim - 1)
    service = [0] + [SERVICE] * (dim - 1)
    for c in range(1, dim):
        if rng.random() < SLOTTED_FRACTION:
            slot = rng.randrange(4)
            ready[c] = slot * SLOT
            due[c] = (slot + 1) * SLOT
        # singleton feasibility: depot->c inside the window, then home in time
        start = max(M[c], ready[c])  # M[0*dim+c]
        assert start <= due[c], f"customer {c} unreachable inside its window"
        assert start + service[c] + M[c * dim] <= HORIZON, f"customer {c} cannot return"
    return ready, due, service


def validate_tw(dim, cap, demand, M, ready, due, service, routes):
    """Independent schedule check, same semantics both solvers were given:
    leave depot at t=0, wait for free, start service within [ready, due],
    return by HORIZON, capacity respected, every customer exactly once.
    Returns (error | None, total_travel_cost)."""
    seen = [False] * dim
    total = 0
    for ri, r in enumerate(routes):
        if not r:
            continue
        t = 0
        load = 0
        prev = 0
        for c in r:
            if not (1 <= c < dim):
                return f"route {ri}: bad node {c}", 0
            if seen[c]:
                return f"customer {c} visited twice", 0
            seen[c] = True
            load += demand[c]
            if load > cap:
                return f"route {ri}: load {load} > cap {cap}", 0
            tr = M[prev * dim + c]
            total += tr
            start = max(t + tr, ready[c])
            if start > due[c]:
                return f"route {ri}: customer {c} starts {start} > due {due[c]}", 0
            t = start + service[c]
            prev = c
        back = M[prev * dim + 0]
        total += back
        if t + back > HORIZON:
            return f"route {ri}: returns {t + back} > horizon {HORIZON}", 0
    missing = [c for c in range(1, dim) if not seen[c]]
    if missing:
        return f"{len(missing)} customers unserved (first: {missing[:5]})", 0
    return None, total


def run_commiv(dim, cap, demand, M, ready, due, service, port, rounds=0, restarts=0):
    body = json.dumps({
        "matrix": [[M[i * dim + j] for j in range(dim)] for i in range(dim)],
        "demand": list(demand),
        "capacity": cap,
        "ready": ready,
        "due": due,
        "service": service,
        "seed": 12345,
        "rounds": rounds,
        "restarts": restarts,
    }).encode()
    t0 = time.time()
    req = urllib.request.Request(f"http://127.0.0.1:{port}/solve/vrptw", data=body,
                                 headers={"Content-Type": "application/json"})
    resp = json.loads(urllib.request.urlopen(req, timeout=3600).read())
    return resp["routes"], time.time() - t0


def run_vroom(dim, cap, demand, M, ready, due, service, expl):
    import numpy as np
    import vroom

    mat = np.array(M, dtype=np.int64).reshape(dim, dim).astype(np.uint32)
    inp = vroom.Input()
    inp.set_durations_matrix("car", mat)
    lb = math.ceil(sum(demand) / cap)
    for v in range(lb * 2 + 10):
        inp.add_vehicle(vroom.Vehicle(
            v, start=0, end=0, capacity=[cap], profile="car",
            time_window=vroom.TimeWindow(0, HORIZON)))
    for c in range(1, dim):
        # VROOM's TW bounds the ARRIVAL/start like ours; default_service is the
        # per-stop service duration.
        inp.add_job(vroom.Job(
            c, location=c, delivery=[int(demand[c])], default_service=service[c],
            time_windows=[vroom.TimeWindow(ready[c], due[c])]))
    t0 = time.time()
    sol = inp.solve(exploration_level=expl, nb_threads=3)
    el = time.time() - t0
    routes = []
    for _, grp in sol.routes.groupby("vehicle_id"):
        r = [int(jid) for typ, jid in zip(grp["type"], grp["id"]) if typ == "job"]
        if r:
            routes.append(r)
    return routes, el


def run_ortools(dim, cap, demand, M, ready, due, service, tlimit):
    from ortools.constraint_solver import pywrapcp, routing_enums_pb2

    lb = math.ceil(sum(demand) / cap)
    vehicles = lb * 2 + 10  # generous: TW fragmentation needs more than the packing bound

    mgr = pywrapcp.RoutingIndexManager(dim, vehicles, 0)
    routing = pywrapcp.RoutingModel(mgr)

    def travel_cb(fi, ti):
        return M[mgr.IndexToNode(fi) * dim + mgr.IndexToNode(ti)]
    tidx = routing.RegisterTransitCallback(travel_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(tidx)  # objective: pure travel, same as commiv

    def time_cb(fi, ti):
        f = mgr.IndexToNode(fi)
        return service[f] + M[f * dim + mgr.IndexToNode(ti)]
    routing.AddDimension(routing.RegisterTransitCallback(time_cb), HORIZON, HORIZON, True, "Time")
    tdim = routing.GetDimensionOrDie("Time")
    for c in range(1, dim):
        tdim.CumulVar(mgr.NodeToIndex(c)).SetRange(ready[c], due[c])
    for v in range(vehicles):
        tdim.CumulVar(routing.End(v)).SetRange(0, HORIZON)

    def demand_cb(fi):
        return demand[mgr.IndexToNode(fi)]
    routing.AddDimensionWithVehicleCapacity(
        routing.RegisterUnaryTransitCallback(demand_cb), 0, [cap] * vehicles, True, "Cap")

    p = pywrapcp.DefaultRoutingSearchParameters()
    p.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    p.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    p.time_limit.FromSeconds(int(tlimit))

    t0 = time.time()
    sol = routing.SolveWithParameters(p)
    el = time.time() - t0
    if not sol:
        return None, el
    routes = []
    for v in range(vehicles):
        idx = routing.Start(v)
        r = []
        while not routing.IsEnd(idx):
            node = mgr.IndexToNode(idx)
            if node != 0:
                r.append(node)
            idx = sol.Value(routing.NextVar(idx))
        if r:
            routes.append(r)
    return routes, el


def main():
    path, solver, arg = sys.argv[1], sys.argv[2], sys.argv[3]
    dim, cap, demand, M, _ = roadlib.parse_road(path)
    ready, due, service = make_windows(dim, M)

    if solver == "commiv":
        # optional argv[4] = "rounds,restarts" budget override (0,0 = defaults)
        rounds = restarts = 0
        if len(sys.argv) > 4:
            rounds, restarts = (int(x) for x in sys.argv[4].split(","))
        routes, el = run_commiv(dim, cap, demand, M, ready, due, service, int(arg), rounds, restarts)
    elif solver == "ortools":
        routes, el = run_ortools(dim, cap, demand, M, ready, due, service, int(arg))
    elif solver == "vroom":
        routes, el = run_vroom(dim, cap, demand, M, ready, due, service, int(arg))
    else:
        sys.exit(f"unknown solver {solver}")

    if routes is None:
        print(f"{solver},{path},{dim},NA,NA,{el:.1f},NO_SOLUTION")
        return
    err, cost = validate_tw(dim, cap, demand, M, ready, due, service, routes)
    print(f"{solver},{path},{dim},{cost},{len(routes)},{el:.1f},{'OK' if not err else err}")


if __name__ == "__main__":
    main()
