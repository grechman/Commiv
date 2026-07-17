"""Rolling-horizon dispatch demo: a synthetic express-delivery shift.

Simulates the q-commerce loop DispatchSession is built for: an initial fleet
plan, then repeated {clock advances, new orders arrive, re-solve around the
committed prefixes}. All data is synthetic (a fixed-seed random coordinate
field, plain Python, no numpy) so this runs in seconds with no external
instance files. Needs libcommiv built:
    zig build lib -Doptimize=ReleaseFast
Run:  python examples/dispatch_shift_demo.py
"""

from __future__ import annotations

import os
import random
import sys

# Runnable straight from the repo: find bindings/python next to this file.
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "bindings", "python"))

import commiv

N_INITIAL_PAIRS = 40
N_WAVES = 3
ORDERS_PER_WAVE = 5
CAPACITY = 15
WAVE_SECONDS = 1800  # 30 minutes between waves
COORD_SEED = 20260718


def travel_time(coords: list[tuple[int, int]], i: int, j: int) -> int:
    """Deterministic directed travel time between two node ids: Euclidean
    distance over the seeded coordinate field, plus a small asymmetric jitter
    (a function of the ids, not of call order) so a->b != b->a — symmetric-ish,
    like a real road network with mild one-way effects."""
    if i == j:
        return 0
    ax, ay = coords[i]
    bx, by = coords[j]
    base = int(((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5)
    jitter = (i * 31 + j * 17) % 5
    return max(1, base + jitter)


def build_order_vectors(coords, new_p: int, new_q: int, old_dim: int):
    """row_p/col_p/row_q/col_q for DispatchSession.add_order: travel times
    between the new pickup/delivery and every one of the old_dim existing
    nodes (depot included)."""
    row_p = [travel_time(coords, new_p, j) for j in range(old_dim)]
    col_p = [travel_time(coords, j, new_p) for j in range(old_dim)]
    row_q = [travel_time(coords, new_q, j) for j in range(old_dim)]
    col_q = [travel_time(coords, j, new_q) for j in range(old_dim)]
    p_to_q = travel_time(coords, new_p, new_q)
    q_to_p = travel_time(coords, new_q, new_p)
    return row_p, col_p, row_q, col_q, p_to_q, q_to_p


def check_locked_unmoved(pre_plan, locked_lens, new_routes, wave: int) -> None:
    """Hard-fail if any committed (locked) prefix from before this wave's
    solve does not survive verbatim at the head of some route in the result."""
    for route, lk in zip(pre_plan, locked_lens):
        if lk == 0:
            continue
        prefix = route[:lk]
        if not any(list(r[:lk]) == prefix for r in new_routes if len(r) >= lk):
            print(
                f"LOCKED-UNMOVED VIOLATION at wave {wave}: prefix {prefix} "
                f"(len {lk}) is not the head of any route in the new plan",
                file=sys.stderr,
            )
            sys.exit(1)


def main() -> None:
    rng = random.Random(COORD_SEED)
    # Pre-generate coordinates for every node this shift will ever create
    # (depot + initial pairs + every wave's new pairs), so travel_time stays
    # consistent regardless of when a node's rows/cols are actually built.
    max_nodes = 1 + 2 * (N_INITIAL_PAIRS + N_WAVES * ORDERS_PER_WAVE)
    coords = [(0, 0)] + [(rng.randint(0, 500), rng.randint(0, 500)) for _ in range(max_nodes - 1)]

    dim0 = 2 * N_INITIAL_PAIRS + 1
    matrix = [[travel_time(coords, i, j) for j in range(dim0)] for i in range(dim0)]
    pickups = [2 * k + 1 for k in range(N_INITIAL_PAIRS)]
    deliveries = [2 * k + 2 for k in range(N_INITIAL_PAIRS)]
    demand = [rng.randint(1, 4) for _ in range(N_INITIAL_PAIRS)]
    ready = [0] * dim0
    due = [1_000_000] * dim0  # generous horizon: this demo is about dispatch, not tightness
    service = [0] + [120] * (dim0 - 1)  # 2 minutes at each stop

    session = commiv.DispatchSession(
        matrix, pickups, deliveries, demand, CAPACITY, ready, due, service,
        seed=1, wall_ms=3000,
    )
    sol = session.solve()
    print(f"wave 0 (initial): orders={len(session.pickups)} vehicles={sol.vehicles} "
          f"total_cost={sol.total_cost} locked=0")

    t = 0
    for wave in range(1, N_WAVES + 1):
        t += WAVE_SECONDS
        session.advance_to(t)
        pre_plan = [list(r) for r in session.plan]
        locked_lens = list(session._locked)

        for _ in range(ORDERS_PER_WAVE):
            old_dim = len(session._matrix)
            new_p, new_q = old_dim, old_dim + 1
            row_p, col_p, row_q, col_q, p_to_q, q_to_p = build_order_vectors(coords, new_p, new_q, old_dim)
            amount = rng.randint(1, 4)
            session.add_order(
                row_p, col_p, row_q, col_q, p_to_q, q_to_p, amount,
                p_ready=0, p_due=1_000_000, q_ready=0, q_due=1_000_000,
                p_service=120, q_service=120,
            )

        sol = session.solve(wall_ms=2000)
        check_locked_unmoved(pre_plan, locked_lens, sol.routes, wave)

        print(f"wave {wave}: t={t}s orders={len(session.pickups)} vehicles={sol.vehicles} "
              f"total_cost={sol.total_cost} locked={sum(locked_lens)}")

    print("DISPATCH DEMO OK")
    sys.exit(0)


if __name__ == "__main__":
    main()
