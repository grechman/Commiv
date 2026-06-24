"""OR-Tools CVRP on the TRUE directed matrix. Usage: ortools_cvrp.py <road> <time_s>"""
import sys
import time
import math
import roadlib
from ortools.constraint_solver import pywrapcp, routing_enums_pb2


def main():
    path = sys.argv[1]
    tlimit = int(sys.argv[2])
    dim, cap, demand, M, coords = roadlib.parse_road(path)
    lb = math.ceil(sum(demand) / cap)
    vehicles = int(lb * 1.3) + 5            # generous; unused vehicles cost nothing

    mgr = pywrapcp.RoutingIndexManager(dim, vehicles, 0)
    routing = pywrapcp.RoutingModel(mgr)

    def tcb(fi, ti):
        return M[mgr.IndexToNode(fi) * dim + mgr.IndexToNode(ti)]
    tidx = routing.RegisterTransitCallback(tcb)
    routing.SetArcCostEvaluatorOfAllVehicles(tidx)

    def dcb(fi):
        return demand[mgr.IndexToNode(fi)]
    didx = routing.RegisterUnaryTransitCallback(dcb)
    routing.AddDimensionWithVehicleCapacity(didx, 0, [cap] * vehicles, True, 'Cap')

    p = pywrapcp.DefaultRoutingSearchParameters()
    p.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    p.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    p.time_limit.FromSeconds(tlimit)

    t0 = time.time()
    sol = routing.SolveWithParameters(p)
    el = time.time() - t0
    if not sol:
        print(f"ortools,{path},{dim},NA,NA,{el:.1f},NO_SOLUTION")
        return

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

    cost = roadlib.score(M, dim, routes)
    err = roadlib.validate(dim, cap, demand, routes)
    print(f"ortools,{path},{dim},{cost},{len(routes)},{el:.1f},{'OK' if not err else err}")


if __name__ == '__main__':
    main()
