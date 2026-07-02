"""commiv — near-optimal directed (asymmetric) TSP / CVRP / VRPTW routes.

ctypes binding over libcommiv (the C ABI in include/commiv.h). No required
dependencies; numpy arrays are accepted and take a fast path when numpy is
installed.

Build the library once, from the repo root:

    zig build lib -Doptimize=ReleaseFast

then point the binding at it (or rely on the defaults below):

    export COMMIV_LIBRARY=/path/to/libcommiv.so

Example:

    import commiv

    matrix = [  # directed costs, row-major, depot = node 0
        [0, 10, 14, 12],
        [11, 0, 9, 20],
        [15, 8, 0, 7],
        [13, 18, 6, 0],
    ]
    sol = commiv.solve_cvrp(matrix, demand=[0, 4, 6, 5], capacity=10, seed=1)
    print(sol.total_cost, sol.routes)
"""

from __future__ import annotations

import ctypes
import ctypes.util
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

__all__ = [
    "CvrpSolution",
    "InfeasibleError",
    "CommivError",
    "solve_cvrp",
    "solve_vrptw",
    "solve_atsp",
    "version",
]

_OK = 0
_ERR_INVALID_ARGUMENT = -1
_ERR_OUT_OF_MEMORY = -2
_ERR_INFEASIBLE = -3


class CommivError(RuntimeError):
    """libcommiv returned an error code."""


class InfeasibleError(CommivError):
    """The instance has no feasible solution (capacity or time windows)."""


class _Options(ctypes.Structure):
    _fields_ = [
        ("seed", ctypes.c_uint64),
        ("sisr_iters", ctypes.c_uint64),
        ("trials", ctypes.c_uint64),
        ("vrptw_rounds", ctypes.c_uint64),
        ("vrptw_restarts", ctypes.c_uint64),
        ("veh_penalty", ctypes.c_uint64),
        ("threads", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32),
    ]


def _candidate_paths() -> list[str]:
    paths = []
    env = os.environ.get("COMMIV_LIBRARY")
    if env:
        paths.append(env)
    names = ["libcommiv.so", "libcommiv.dylib", "commiv.dll"]
    here = Path(__file__).resolve()
    for base in [here.parent, *here.parents[:4]]:
        for name in names:
            paths.append(str(base / name))
            paths.append(str(base / "zig-out" / "lib" / name))
    found = ctypes.util.find_library("commiv")
    if found:
        paths.append(found)
    paths.extend(names)  # last resort: default loader search path
    return paths


def _load() -> ctypes.CDLL:
    tried = []
    for path in _candidate_paths():
        if os.path.sep in path and not os.path.exists(path):
            tried.append(path)
            continue
        try:
            return ctypes.CDLL(path)
        except OSError:
            tried.append(path)
    raise OSError(
        "could not load libcommiv. Build it with `zig build lib "
        "-Doptimize=ReleaseFast` and/or set COMMIV_LIBRARY to its path. "
        f"Tried: {tried}"
    )


_lib = _load()

_lib.commiv_version.restype = ctypes.c_char_p
_lib.commiv_version.argtypes = []
_lib.commiv_solve_cvrp.restype = ctypes.c_int
_lib.commiv_solve_cvrp.argtypes = [
    ctypes.POINTER(ctypes.c_uint32), ctypes.c_size_t,
    ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32,
    ctypes.POINTER(_Options), ctypes.POINTER(ctypes.c_void_p),
]
_lib.commiv_solve_vrptw.restype = ctypes.c_int
_lib.commiv_solve_vrptw.argtypes = [
    ctypes.POINTER(ctypes.c_uint32), ctypes.c_size_t,
    ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32,
    ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32),
    ctypes.POINTER(ctypes.c_uint32),
    ctypes.POINTER(_Options), ctypes.POINTER(ctypes.c_void_p),
]
_lib.commiv_solve_atsp.restype = ctypes.c_int
_lib.commiv_solve_atsp.argtypes = [
    ctypes.POINTER(ctypes.c_uint32), ctypes.c_size_t,
    ctypes.POINTER(_Options),
    ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint64),
]
_lib.commiv_routes_cost.restype = ctypes.c_uint64
_lib.commiv_routes_cost.argtypes = [ctypes.c_void_p]
_lib.commiv_routes_count.restype = ctypes.c_size_t
_lib.commiv_routes_count.argtypes = [ctypes.c_void_p]
_lib.commiv_routes_len.restype = ctypes.c_size_t
_lib.commiv_routes_len.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
_lib.commiv_routes_get.restype = ctypes.POINTER(ctypes.c_uint32)
_lib.commiv_routes_get.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
_lib.commiv_routes_free.restype = None
_lib.commiv_routes_free.argtypes = [ctypes.c_void_p]


def version() -> str:
    """libcommiv version string, e.g. '0.2.0'."""
    return _lib.commiv_version().decode()


def _as_u32_array(values, expected_len: int, what: str):
    """Copy any u32 source (numpy array, list, nested rows) into a ctypes array."""
    if hasattr(values, "astype") and hasattr(values, "ravel"):  # numpy fast path
        import numpy as np  # local import: numpy is optional

        arr = np.ascontiguousarray(values, dtype=np.uint32).ravel()
        if arr.size != expected_len:
            raise ValueError(f"{what}: expected {expected_len} entries, got {arr.size}")
        return (ctypes.c_uint32 * expected_len).from_buffer_copy(arr.tobytes())
    flat: list[int] = []
    if values and isinstance(values[0], (list, tuple)):
        for row in values:
            flat.extend(row)
    else:
        flat = list(values)
    if len(flat) != expected_len:
        raise ValueError(f"{what}: expected {expected_len} entries, got {len(flat)}")
    return (ctypes.c_uint32 * expected_len)(*flat)


def _matrix_dim(matrix) -> int:
    if hasattr(matrix, "shape"):  # numpy
        rows, cols = matrix.shape
        if rows != cols:
            raise ValueError(f"matrix must be square, got {rows}x{cols}")
        return rows
    dim = len(matrix)
    if isinstance(matrix[0], (list, tuple)):
        for row in matrix:
            if len(row) != dim:
                raise ValueError("matrix must be square (every row of length len(matrix))")
        return dim
    # flat matrix
    root = int(dim ** 0.5)
    if root * root != dim:
        raise ValueError("flat matrix length must be a perfect square")
    return root


def _raise_for(rc: int, context: str) -> None:
    if rc == _ERR_INFEASIBLE:
        raise InfeasibleError(f"{context}: instance is infeasible")
    if rc == _ERR_INVALID_ARGUMENT:
        raise ValueError(f"{context}: invalid arguments (shape, capacity, or demand[0] != 0)")
    if rc == _ERR_OUT_OF_MEMORY:
        raise MemoryError(context)
    raise CommivError(f"{context}: internal solver error ({rc})")


@dataclass(frozen=True)
class CvrpSolution:
    """One route per vehicle; each route lists customer indices (1..n) in visit
    order with the depot implied at both ends. total_cost is summed on the
    directed matrix you passed in."""

    total_cost: int
    routes: list[list[int]]

    @property
    def vehicles(self) -> int:
        return len(self.routes)


def _extract_routes(handle: ctypes.c_void_p) -> CvrpSolution:
    try:
        cost = _lib.commiv_routes_cost(handle)
        routes = []
        for i in range(_lib.commiv_routes_count(handle)):
            n = _lib.commiv_routes_len(handle, i)
            ptr = _lib.commiv_routes_get(handle, i)
            routes.append([ptr[j] for j in range(n)])
        return CvrpSolution(total_cost=cost, routes=routes)
    finally:
        _lib.commiv_routes_free(handle)


def solve_cvrp(
    matrix,
    demand: Sequence[int],
    capacity: int,
    *,
    seed: int = 1,
    iters: int = 0,
    threads: int = 1,
) -> CvrpSolution:
    """Solve a directed CVRP (uncapped fleet, SISR solver).

    matrix: (n+1)x(n+1) directed costs (nested lists, flat list, or numpy),
    row-major, matrix[a][b] = cost a -> b, depot = node 0.
    demand: n+1 entries, demand[0] = 0. capacity: per-vehicle capacity.
    iters=0 means the default budget (300k SISR iterations). threads>1 runs
    parallel islands (faster, result depends on the count); threads<=1 is
    deterministic for a fixed seed.
    """
    dim = _matrix_dim(matrix)
    n = dim - 1
    m = _as_u32_array(matrix, dim * dim, "matrix")
    d = _as_u32_array(demand, dim, "demand")
    opts = _Options(seed=seed, sisr_iters=iters, threads=threads)
    out = ctypes.c_void_p()
    rc = _lib.commiv_solve_cvrp(m, n, d, capacity, ctypes.byref(opts), ctypes.byref(out))
    if rc != _OK:
        _raise_for(rc, "solve_cvrp")
    return _extract_routes(out)


def solve_vrptw(
    matrix,
    demand: Sequence[int],
    capacity: int,
    ready: Sequence[int],
    due: Sequence[int],
    service: Sequence[int],
    *,
    seed: int = 1,
    iters: int = 0,
    threads: int = 1,
    rounds: int = 0,
    restarts: int = 0,
    veh_penalty: int = 0,
) -> CvrpSolution:
    """Solve a directed VRPTW with SISR (the flagship engine). ready/due bound
    the START of service per node (due[0] is the depot horizon); service is the
    per-node service duration. Waiting before a window opens is free.

    iters=0 means the default budget (300k); threads>1 runs best-of-K parallel
    chains. veh_penalty > 0 biases toward fewer vehicles at some distance cost.
    Setting rounds/restarts selects the legacy ILS engine instead of SISR."""
    dim = _matrix_dim(matrix)
    n = dim - 1
    m = _as_u32_array(matrix, dim * dim, "matrix")
    d = _as_u32_array(demand, dim, "demand")
    rd = _as_u32_array(ready, dim, "ready")
    du = _as_u32_array(due, dim, "due")
    sv = _as_u32_array(service, dim, "service")
    opts = _Options(seed=seed, sisr_iters=iters, threads=threads,
                    vrptw_rounds=rounds, vrptw_restarts=restarts, veh_penalty=veh_penalty)
    out = ctypes.c_void_p()
    rc = _lib.commiv_solve_vrptw(m, n, d, capacity, rd, du, sv, ctypes.byref(opts), ctypes.byref(out))
    if rc != _OK:
        _raise_for(rc, "solve_vrptw")
    return _extract_routes(out)


def solve_atsp(matrix, *, seed: int = 1, trials: int = 0) -> tuple[int, list[int]]:
    """Solve a directed TSP over an n x n matrix. Returns (cost, tour) where
    tour is the visit order, a permutation of 0..n-1."""
    n = _matrix_dim(matrix)
    m = _as_u32_array(matrix, n * n, "matrix")
    opts = _Options(seed=seed, trials=trials)
    tour = (ctypes.c_uint32 * n)()
    cost = ctypes.c_uint64()
    rc = _lib.commiv_solve_atsp(m, n, ctypes.byref(opts), tour, ctypes.byref(cost))
    if rc != _OK:
        _raise_for(rc, "solve_atsp")
    return cost.value, list(tour)
