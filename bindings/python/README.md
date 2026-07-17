# commiv Python binding

ctypes wrapper over `libcommiv` (the C ABI in `include/commiv.h`). No required
dependencies; numpy input takes a fast path when numpy is present. Four solver
families ship: `solve_cvrp`, `solve_vrptw`, `solve_pdptw` (pickup-and-delivery
with a money objective), and `solve_atsp` — plus `solve_pdptw_dispatch` and the
`DispatchSession` convenience class for rolling-horizon re-solve around a
committed (locked) plan.

## Setup

```sh
# from the repo root: build the native library once
zig build lib -Doptimize=ReleaseFast

# make the package importable (editable install or just add to PYTHONPATH)
pip install -e bindings/python
# if libcommiv.so is not in the default search paths:
export COMMIV_LIBRARY=$PWD/zig-out/lib/libcommiv.so
```

## Use

```python
import commiv

matrix = [  # directed costs, row-major, depot = node 0
    [0, 10, 14, 12],
    [11, 0, 9, 20],
    [15, 8, 0, 7],
    [13, 18, 6, 0],
]
sol = commiv.solve_cvrp(matrix, demand=[0, 4, 6, 5], capacity=10, seed=1)
print(sol.total_cost, sol.routes)          # 58 [[2, 1], [3]]

cost, tour = commiv.solve_atsp([[0, 1, 9], [9, 0, 1], [1, 9, 0]])

sol = commiv.solve_vrptw(
    matrix, demand=[0, 4, 6, 5], capacity=10,
    ready=[0, 0, 0, 0], due=[1000, 500, 500, 500], service=[0, 5, 5, 5],
)

# Pickup-and-delivery with the MONEY objective. dim = 2*n_pairs+1 nodes:
# depot 0, then pickups/deliveries. Requests (1 -> 3) and (2 -> 4). Passing
# time_penalty > 0 charges each route's duration (drive + service + waiting),
# not just distance, so the engine trades fuel for driver hours. This knob is
# PDPTW-only. Every node 1..dim-1 must appear once across pickups+deliveries.
pdp_matrix = [
    [0, 10, 12, 14, 16],
    [10, 0, 6, 8, 10],
    [12, 6, 0, 7, 9],
    [14, 8, 7, 0, 6],
    [16, 10, 9, 6, 0],
]
sol = commiv.solve_pdptw(
    pdp_matrix, pickups=[1, 2], deliveries=[3, 4], demand=[4, 5], capacity=10,
    ready=[0, 0, 0, 0, 0], due=[10000] * 5, service=[0, 3, 3, 3, 3],
    time_penalty=3,
)
print(sol.total_cost, sol.routes)          # 45 [[1, 2, 4, 3]]

# Rolling-horizon dispatch: re-solve around a committed plan as orders arrive
# mid-shift. current/locked describe what must not move; DispatchSession
# wraps the whole loop (advance the clock, add_order, solve).
session = commiv.DispatchSession(
    pdp_matrix, pickups=[1, 2], deliveries=[3, 4], demand=[4, 5], capacity=10,
    ready=[0, 0, 0, 0, 0], due=[10000] * 5, service=[0, 3, 3, 3, 3],
)
session.solve()
session.advance_to(60)          # lock stops already reached by t=60
session.add_order(                       # dim was 5, so the new pair is nodes 5,6
    row_p=[9, 8, 5, 6, 7], col_p=[9, 8, 5, 6, 7],   # new pickup <-> existing nodes
    row_q=[11, 9, 7, 8, 9], col_q=[11, 9, 7, 8, 9], # new delivery <-> existing nodes
    p_to_q=5, q_to_p=5, amount=3,
    p_ready=0, p_due=10000, q_ready=0, q_due=10000,
)
session.solve()                 # re-optimizes around the locked prefixes
```

`solve_cvrp(..., threads=4)` runs parallel SISR islands (faster; the result
then depends on the thread count). `threads=1` (default) is deterministic for
a fixed seed. `solve_vrptw` and `solve_pdptw` also take `fleet_min=True` +
`wall_ms=...` for hierarchical vehicle minimization; on PDPTW `max_vehicles=k`
pins the fleet to exactly `k` vehicles (`threads` is ignored for PDPTW). An
unsatisfiable instance raises `commiv.InfeasibleError` instead of returning a
quietly wrong answer.

Run the smoke test: `python bindings/python/test_smoke.py`
