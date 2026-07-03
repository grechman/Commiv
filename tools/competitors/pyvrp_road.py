"""PyVRP adapter for the Moscow .road benchmarks — the one open-source HGS
descendant that natively accepts arbitrary DIRECTED duration matrices with time
windows, i.e. the only serious competitor on commiv's home turf. Same window
overlay and the same independent validator as every other adapter here, so no
solver's own accounting is trusted.

Usage (inside the cbench venv):
  ~/cbench/venv/bin/python pyvrp_road.py <file.road> cvrp  <seconds> [seed]
  ~/cbench/venv/bin/python pyvrp_road.py <file.road> vrptw <seconds> [seed]
Prints: pyvrp,<instance>,<mode>,<dim>,<cost>,<vehicles>,<seconds>,<valid>
"""
import math
import sys
import time

import numpy as np
from pyvrp import Client, Depot, ProblemData, VehicleType, solve
from pyvrp.stop import MaxRuntime

import roadlib
from vrptw_moscow import HORIZON, make_windows, validate_tw


def main():
    path, mode, secs = sys.argv[1], sys.argv[2], float(sys.argv[3])
    seed = int(sys.argv[4]) if len(sys.argv) > 4 else 12345
    dim, cap, demand, M, coords = roadlib.parse_road(path)

    mat = np.asarray(M, dtype=np.int64).reshape(dim, dim)
    if mode == "vrptw":
        ready, due, service = make_windows(dim, M)
        clients = [
            Client(x=coords[c][0], y=coords[c][1], delivery=[demand[c]],
                   service_duration=service[c], tw_early=ready[c], tw_late=due[c])
            for c in range(1, dim)
        ]
        depot = Depot(x=coords[0][0], y=coords[0][1])
        shift_late = HORIZON
    else:
        clients = [
            Client(x=coords[c][0], y=coords[c][1], delivery=[demand[c]])
            for c in range(1, dim)
        ]
        depot = Depot(x=coords[0][0], y=coords[0][1])
        shift_late = np.iinfo(np.int64).max

    # Uncapped-fleet convention (same as the commiv/VROOM runs): enough
    # vehicles that the count is never binding, zero fixed cost, objective =
    # pure travel time. PyVRP is free to use however many it wants.
    lb = math.ceil(sum(demand) / cap)
    veh = VehicleType(num_available=lb * 2 + 10, capacity=[cap],
                      tw_early=0, tw_late=shift_late)

    data = ProblemData(clients=clients, depots=[depot], vehicle_types=[veh],
                       distance_matrices=[mat], duration_matrices=[mat])

    t0 = time.time()
    res = solve(data, stop=MaxRuntime(secs), seed=seed, display=False)
    wall = time.time() - t0

    # location index 0 is the depot, clients are 1..dim-1 in node order, so
    # PyVRP's visit indices ARE our node indices.
    routes = [list(r.visits()) for r in res.best.routes()]

    if mode == "vrptw":
        err, cost = validate_tw(dim, cap, demand, M, ready, due, service, routes)
        valid = err is None
        if err:
            print(f"# INVALID: {err}", file=sys.stderr)
    else:
        seen = sorted(c for r in routes for c in r)
        valid = seen == list(range(1, dim)) and all(
            sum(demand[c] for c in r) <= cap for r in routes)
        cost = roadlib.score(M, dim, routes)

    name = path.rsplit("/", 1)[-1].removesuffix(".road")
    print(f"pyvrp,{name},{mode},{dim},{cost},{len(routes)},{wall:.1f},{valid}")


if __name__ == "__main__":
    main()
