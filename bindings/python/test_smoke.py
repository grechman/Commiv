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


def check_pdptw_dispatch():
    sol = commiv.solve_pdptw(
        PDP_MATRIX, PDP_PICKUPS, PDP_DELIVERIES, PDP_DEMAND, 10,
        PDP_READY, PDP_DUE, PDP_SERVICE, seed=5, iters=2000,
    )
    # Lock the ENTIRE first nonempty route: a complete route is always
    # pair-closed (a pair never spans two routes), so this is always valid.
    current_routes = sol.routes
    locked = [0] * len(current_routes)
    lock_idx = next(i for i, r in enumerate(current_routes) if r)
    locked[lock_idx] = len(current_routes[lock_idx])
    locked_prefix = list(current_routes[lock_idx])

    sol2 = commiv.solve_pdptw_dispatch(
        PDP_MATRIX, PDP_PICKUPS, PDP_DELIVERIES, PDP_DEMAND, 10,
        PDP_READY, PDP_DUE, PDP_SERVICE, current_routes, locked,
        seed=9, iters=3000,
    )
    n = len(locked_prefix)
    assert any(list(r[:n]) == locked_prefix for r in sol2.routes if len(r) >= n)
    _assert_complete_pdptw_plan(sol2)


def check_dispatch_session():
    # 3-pair instance, generous windows so any order works timing-wise.
    n_pairs = 3
    dim = 2 * n_pairs + 1
    matrix = [[abs(i - j) * 7 + (3 if i != j else 0) for j in range(dim)] for i in range(dim)]
    pickups = [1, 2, 3]
    deliveries = [4, 5, 6]
    demand = [2, 3, 4]
    ready = [0] * dim
    due = [100_000] * dim
    service = [0] * dim

    session = commiv.DispatchSession(
        matrix, pickups, deliveries, demand, 10, ready, due, service,
        seed=3, iters=2000,
    )
    sol1 = session.solve()
    assert sol1.total_cost > 0
    pre_plan = [list(r) for r in session.plan]

    session.advance_to(100_000)  # generous windows -> locks the entire plan

    old_dim = len(session._matrix)
    new_p, new_q = session.add_order(
        row_p=[10] * old_dim, col_p=[10] * old_dim,
        row_q=[12] * old_dim, col_q=[12] * old_dim,
        p_to_q=5, q_to_p=5, amount=2,
        p_ready=0, p_due=100_000, q_ready=0, q_due=100_000,
        p_service=0, q_service=0,
    )
    assert new_p == old_dim and new_q == old_dim + 1

    sol2 = session.solve(iters=3000)
    # Locked stops unmoved: every pre-existing (fully-locked) route survives
    # verbatim as a leading prefix of some route in the new plan.
    for r in pre_plan:
        if not r:
            continue
        assert any(list(r2[: len(r)]) == r for r2 in sol2.routes if len(r2) >= len(r)), (r, sol2.routes)
    # The new order was served.
    served = {c for r in sol2.routes for c in r}
    assert new_p in served and new_q in served, (new_p, new_q, sol2.routes)


def _route_tws_duration(matrix, ready, due, service, route):
    """Ground-truth route duration via the SAME Tws schedule recurrence the
    engine uses (travel + service + unavoidable waiting at the
    duration-minimizing departure time). Ports src Tws.merge/routeDuration
    exactly so the smoke can verify the cap independently of the solver."""
    def merge(left, edge, right):
        ldur, ltw, lear, llate = left
        rdur, rtw, rear, rlate = right
        delta = ldur - ltw + edge
        d_wait = max(rear - delta - llate, 0)
        d_tw = max(lear + delta - rlate, 0)
        return (
            ldur + rdur + edge + d_wait,
            ltw + rtw + d_tw,
            max(rear - delta, lear) - d_wait,
            min(rlate - delta, llate) + d_tw,
        )
    depot = (0, 0, 0, due[0])
    acc = depot
    prev = 0
    for c in route:
        acc = merge(acc, matrix[prev][c], (service[c], 0, ready[c], due[c]))
        prev = c
    acc = merge(acc, matrix[prev][0], depot)
    return max(acc[0], 0)


def check_pdptw_max_route_duration():
    # Both pairs on ONE route (0,1,2,3,4,0) has duration 45 travel + 12 service
    # = 57; the min-distance uncapped answer. A cap of 50 makes that infeasible
    # and forces a split into two single-pair routes (durations 38 and 43).
    cap = 50
    one_route = _route_tws_duration(
        PDP_MATRIX, PDP_READY, PDP_DUE, PDP_SERVICE, [1, 2, 3, 4])
    assert one_route > cap, one_route  # the cap really bites
    sol = commiv.solve_pdptw(
        PDP_MATRIX, PDP_PICKUPS, PDP_DELIVERIES, PDP_DEMAND, 10,
        PDP_READY, PDP_DUE, PDP_SERVICE, seed=5, iters=6000,
        max_route_duration=cap,
    )
    _assert_complete_pdptw_plan(sol)
    assert len(sol.routes) >= 2, ("cap must force a split", sol.routes)
    for r in sol.routes:
        d = _route_tws_duration(PDP_MATRIX, PDP_READY, PDP_DUE, PDP_SERVICE, r)
        assert d <= cap, ("route over shift cap", r, d, cap)


# Duration-only VRPTW instance: capacity is slack (100 >> demand) so the ONLY
# binding split pressure is the shift cap. Wide windows -> zero waiting, so a
# route's duration is just travel + service.
DUR_MATRIX = [
    [0, 10, 10, 10],
    [10, 0, 10, 10],
    [10, 10, 0, 10],
    [10, 10, 10, 0],
]
DUR_DEMAND = [0, 1, 1, 1]
DUR_READY = [0, 0, 0, 0]
DUR_DUE = [100_000, 100_000, 100_000, 100_000]
DUR_SERVICE = [0, 5, 5, 5]


def check_vrptw_max_route_duration():
    # One route 0,1,2,3,0 = 40 travel + 15 service = 55; cap 45 forces a split.
    cap = 45
    one_route = _route_tws_duration(
        DUR_MATRIX, DUR_READY, DUR_DUE, DUR_SERVICE, [1, 2, 3])
    assert one_route > cap, one_route
    sol = commiv.solve_vrptw(
        DUR_MATRIX, DUR_DEMAND, 100,
        ready=DUR_READY, due=DUR_DUE, service=DUR_SERVICE,
        seed=7, iters=50_000, max_route_duration=cap,
    )
    assert {c for r in sol.routes for c in r} == {1, 2, 3}
    assert len(sol.routes) >= 2, ("cap must force a split", sol.routes)
    for r in sol.routes:
        d = _route_tws_duration(DUR_MATRIX, DUR_READY, DUR_DUE, DUR_SERVICE, r)
        assert d <= cap, ("route over shift cap", r, d, cap)


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
    check_pdptw_dispatch()
    print("pdptw dispatch ok")
    check_pdptw_max_route_duration()
    print("pdptw max_route_duration ok")
    check_vrptw_max_route_duration()
    print("vrptw max_route_duration ok")
    check_dispatch_session()
    print("dispatch session ok")
    check_vrptw_fleet_min()
    print("vrptw fleet_min ok")
    check_atsp()
    print("atsp ok")
    check_errors()
    print("errors ok")
    print(f"numpy {check_numpy_if_available()}")
    print("SMOKE OK")
    sys.exit(0)
