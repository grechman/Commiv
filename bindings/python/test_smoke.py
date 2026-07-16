"""Smoke test for the commiv Python binding. Needs libcommiv built:
    zig build lib -Doptimize=ReleaseFast
Run:  python bindings/python/test_smoke.py
"""

import sys

import commiv

MATRIX = [
    [0, 10, 14, 12],
    [11, 0, 9, 20],
    [15, 8, 0, 7],
    [13, 18, 6, 0],
]
DEMAND = [0, 4, 6, 5]
CAPACITY = 10


def check_cvrp():
    sol = commiv.solve_cvrp(MATRIX, DEMAND, CAPACITY, seed=1)
    assert sol.total_cost > 0
    assert sol.vehicles >= 2, "demands 4+6+5 over capacity 10 need >= 2 vehicles"
    # Every customer exactly once, capacity respected, cost re-adds correctly.
    seen = set()
    recomputed = 0
    for route in sol.routes:
        load = 0
        prev = 0
        for c in route:
            assert 1 <= c <= 3 and c not in seen
            seen.add(c)
            load += DEMAND[c]
            recomputed += MATRIX[prev][c]
            prev = c
        recomputed += MATRIX[prev][0]
        assert load <= CAPACITY
    assert seen == {1, 2, 3}
    assert recomputed == sol.total_cost, (recomputed, sol.total_cost)
    # Determinism: same seed, same answer.
    again = commiv.solve_cvrp(MATRIX, DEMAND, CAPACITY, seed=1)
    assert again == sol


def check_vrptw():
    sol = commiv.solve_vrptw(
        MATRIX, DEMAND, CAPACITY,
        ready=[0, 0, 0, 0], due=[1000, 500, 500, 500], service=[0, 5, 5, 5],
        seed=7,
    )
    assert sol.total_cost > 0
    assert {c for r in sol.routes for c in r} == {1, 2, 3}


# Tiny 2-pair PDPTW: dim=5, depot 0, requests (1->3) and (2->4).
PDP_MATRIX = [
    [0, 10, 12, 14, 16],
    [10, 0, 6, 8, 10],
    [12, 6, 0, 7, 9],
    [14, 8, 7, 0, 6],
    [16, 10, 9, 6, 0],
]
PDP_PICKUPS = [1, 2]
PDP_DELIVERIES = [3, 4]
PDP_DEMAND = [4, 5]
PDP_READY = [0, 0, 0, 0, 0]
PDP_DUE = [10000, 10000, 10000, 10000, 10000]
PDP_SERVICE = [0, 3, 3, 3, 3]


def _assert_complete_pdptw_plan(sol):
    """Every node 1..4 served exactly once, and each pickup precedes its
    delivery on the same route."""
    pair = {1: 3, 2: 4}  # pickup -> delivery
    seen = set()
    for route in sol.routes:
        for c in route:
            assert 1 <= c <= 4 and c not in seen, (c, sol.routes)
            seen.add(c)
        for p, d in pair.items():
            if p in route or d in route:
                assert p in route and d in route, (p, d, route)
                assert route.index(p) < route.index(d), ("precedence", p, d, route)
    assert seen == {1, 2, 3, 4}, seen


def check_pdptw():
    sol = commiv.solve_pdptw(
        PDP_MATRIX, PDP_PICKUPS, PDP_DELIVERIES, PDP_DEMAND, 10,
        PDP_READY, PDP_DUE, PDP_SERVICE, seed=5, iters=2000,
    )
    assert sol.total_cost > 0
    _assert_complete_pdptw_plan(sol)


def check_pdptw_money():
    # The money objective (time_penalty>0) still returns a complete, feasible plan.
    sol = commiv.solve_pdptw(
        PDP_MATRIX, PDP_PICKUPS, PDP_DELIVERIES, PDP_DEMAND, 10,
        PDP_READY, PDP_DUE, PDP_SERVICE, seed=5, iters=2000, time_penalty=3,
    )
    assert sol.total_cost > 0
    _assert_complete_pdptw_plan(sol)


def check_vrptw_fleet_min():
    # fleet_min with a small wall budget still serves every customer.
    sol = commiv.solve_vrptw(
        MATRIX, DEMAND, CAPACITY,
        ready=[0, 0, 0, 0], due=[1000, 500, 500, 500], service=[0, 5, 5, 5],
        seed=7, fleet_min=True, wall_ms=500,
    )
    assert sol.total_cost > 0
    assert {c for r in sol.routes for c in r} == {1, 2, 3}


def check_atsp():
    cost, tour = commiv.solve_atsp(
        [[0, 1, 9, 9], [9, 0, 1, 9], [9, 9, 0, 1], [1, 9, 9, 0]], seed=42
    )
    assert cost == 4, cost  # the directed ring 0->1->2->3->0
    assert sorted(tour) == [0, 1, 2, 3]


def check_errors():
    try:
        commiv.solve_cvrp(MATRIX, [0, 4, 99, 5], CAPACITY)
    except commiv.InfeasibleError:
        pass
    else:
        raise AssertionError("demand 99 > capacity 10 must raise InfeasibleError")
    try:
        commiv.solve_cvrp([[0, 1], [1, 0], [1, 1]], [0, 1], 1)
    except ValueError:
        pass
    else:
        raise AssertionError("non-square matrix must raise ValueError")


def check_numpy_if_available():
    try:
        import numpy as np
    except ImportError:
        return "skipped (no numpy)"
    sol = commiv.solve_cvrp(np.array(MATRIX), np.array(DEMAND), CAPACITY, seed=1)
    ref = commiv.solve_cvrp(MATRIX, DEMAND, CAPACITY, seed=1)
    assert sol == ref, "numpy path must match the list path"
    return "ok"


if __name__ == "__main__":
    print(f"libcommiv {commiv.version()}")
    check_cvrp()
    print("cvrp ok")
    check_vrptw()
    print("vrptw ok")
    check_pdptw()
    print("pdptw ok")
    check_pdptw_money()
    print("pdptw money ok")
    check_vrptw_fleet_min()
    print("vrptw fleet_min ok")
    check_atsp()
    print("atsp ok")
    check_errors()
    print("errors ok")
    print(f"numpy {check_numpy_if_available()}")
    print("SMOKE OK")
    sys.exit(0)
