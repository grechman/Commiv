# commiv Python binding

ctypes wrapper over `libcommiv` (the C ABI in `include/commiv.h`). No required
dependencies; numpy input takes a fast path when numpy is present.

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
```

`solve_cvrp(..., threads=4)` runs parallel SISR islands (faster; the result
then depends on the thread count). `threads=1` (default) is deterministic for
a fixed seed. An unsatisfiable instance raises `commiv.InfeasibleError` instead
of returning a quietly wrong answer.

Run the smoke test: `python bindings/python/test_smoke.py`
