"""commiv — near-optimal directed (asymmetric) TSP / ATSP / CVRP / VRPTW / PDPTW routes.

ctypes binding over libcommiv (the C ABI in include/commiv.h). No required
dependencies; numpy arrays are accepted and take a fast path when numpy is
installed. Four solver families ship: solve_cvrp, solve_vrptw, solve_pdptw
(pickup-and-delivery with a money objective), and solve_atsp — plus
solve_pdptw_dispatch and the DispatchSession convenience class for
rolling-horizon re-solves around a committed (locked) plan.

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

Pickup-and-delivery with the money objective (charge route duration, not just
distance, via time_penalty > 0 — this is the PDPTW-only knob that trades fuel
for driver hours wherever waiting exists):

    # dim = 2*n_pairs+1 nodes: depot 0, then pickups/deliveries. Requests
    # (1 -> 3) and (2 -> 4); demand is one load per request.
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
        time_penalty=3,  # money objective: price each route's duration
    )
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
    "solve_pdptw",
    "solve_pdptw_dispatch",
    "DispatchSession",
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
    # Must mirror commiv_options in include/commiv.h byte-for-byte (88 bytes).
    _fields_ = [
        ("seed", ctypes.c_uint64),
        ("sisr_iters", ctypes.c_uint64),
        ("trials", ctypes.c_uint64),
        ("veh_penalty", ctypes.c_uint64),
        ("threads", ctypes.c_uint32),
        ("fleet_min", ctypes.c_uint32),
        ("wall_ms", ctypes.c_uint64),
        ("max_vehicles", ctypes.c_uint64),
        ("time_penalty", ctypes.c_uint64),
        ("max_route_duration", ctypes.c_uint64),
        ("break_duration", ctypes.c_uint32),
        ("break_earliest", ctypes.c_uint32),
        ("break_latest", ctypes.c_uint32),
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
_lib.commiv_solve_pdptw.restype = ctypes.c_int
_lib.commiv_solve_pdptw.argtypes = [
    ctypes.POINTER(ctypes.c_uint32), ctypes.c_size_t,
    ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32),
    ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32,
    ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32),
    ctypes.POINTER(ctypes.c_uint32),
    ctypes.POINTER(_Options), ctypes.POINTER(ctypes.c_void_p),
]
_lib.commiv_solve_pdptw_dispatch.restype = ctypes.c_int
_lib.commiv_solve_pdptw_dispatch.argtypes = [
    ctypes.POINTER(ctypes.c_uint32), ctypes.c_size_t,
    ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32),
    ctypes.POINTER(ctypes.c_uint32), ctypes.c_uint32,
    ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32),
    ctypes.POINTER(ctypes.c_uint32),
    ctypes.POINTER(ctypes.c_size_t), ctypes.c_size_t,
    ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_size_t),
    ctypes.POINTER(_Options), ctypes.POINTER(ctypes.c_void_p),
]
_lib.commiv_solve_pdptw_typed.restype = ctypes.c_int
_lib.commiv_solve_pdptw_typed.argtypes = [
    ctypes.POINTER(ctypes.c_uint32), ctypes.c_size_t,
    ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32),
    ctypes.POINTER(ctypes.c_uint32),
    ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint32),
    ctypes.POINTER(ctypes.c_uint32),
    ctypes.POINTER(ctypes.c_uint32), ctypes.POINTER(ctypes.c_uint64),
    ctypes.POINTER(ctypes.c_uint32), ctypes.c_size_t,
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
_lib.commiv_routes_type.restype = ctypes.c_uint32
_lib.commiv_routes_type.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
_lib.commiv_routes_free.restype = None
_lib.commiv_routes_free.argtypes = [ctypes.c_void_p]

_NO_TYPE = 0xFFFFFFFF


def version() -> str:
    """libcommiv version string, e.g. '0.4.0'."""
    return _lib.commiv_version().decode()


def _as_u32_array(values, expected_len: int, what: str):
    """Expose contiguous numpy u32 storage, or copy generic input to ctypes."""
    if hasattr(values, "astype") and hasattr(values, "ravel"):  # numpy fast path
        import numpy as np  # local import: numpy is optional

        arr = np.ascontiguousarray(values, dtype=np.uint32).ravel()
        if arr.size != expected_len:
            raise ValueError(f"{what}: expected {expected_len} entries, got {arr.size}")
        # as_ctypes is a zero-copy view and retains `arr` for its own lifetime.
        # The old arr.tobytes() + from_buffer_copy path materialized two extra
        # full copies — 200 MB of avoidable peak RSS for a 5000² matrix.
        return np.ctypeslib.as_ctypes(arr)
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
    directed matrix you passed in. types is None for uniform-fleet solves;
    for solve_pdptw(vehicle_types=...) it holds route r's index into that
    vehicle_types list."""

    total_cost: int
    routes: list[list[int]]
    types: list[int] | None = None

    @property
    def vehicles(self) -> int:
        return len(self.routes)


def _extract_routes(handle: ctypes.c_void_p) -> CvrpSolution:
    try:
        cost = _lib.commiv_routes_cost(handle)
        routes = []
        types = []
        for i in range(_lib.commiv_routes_count(handle)):
            n = _lib.commiv_routes_len(handle, i)
            ptr = _lib.commiv_routes_get(handle, i)
            routes.append([ptr[j] for j in range(n)])
            types.append(_lib.commiv_routes_type(handle, i))
        if routes and all(t != _NO_TYPE for t in types):
            return CvrpSolution(total_cost=cost, routes=routes, types=types)
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
    veh_penalty: int = 0,
    fleet_min: bool = False,
    max_vehicles: int = 0,
    max_route_duration: int = 0,
    wall_ms: int = 0,
) -> CvrpSolution:
    """Solve a directed VRPTW with SISR (the flagship engine). ready/due bound
    the START of service per node (due[0] is the depot horizon); service is the
    per-node service duration. Waiting before a window opens is free.

    iters=0 means the default budget (300k); threads>1 runs best-of-K parallel
    chains. veh_penalty > 0 biases toward fewer vehicles at some distance cost.

    fleet_min=True runs the hierarchical fleet-minimization driver (minimize
    vehicles first, then distance); it needs a wall budget, so pass wall_ms > 0
    (defaults to 10s otherwise) and it takes precedence over threads.
    max_vehicles > 0 caps the number of routes. wall_ms is a wall-clock budget
    in milliseconds for the SISR / fleet-min search (0 = bounded by iters only).
    There is NO time_penalty on VRPTW — the money objective is PDPTW-only.
    max_route_duration > 0 caps every route's duration (travel + service +
    unavoidable waiting) at that shift length; 0 = uncapped."""
    dim = _matrix_dim(matrix)
    n = dim - 1
    m = _as_u32_array(matrix, dim * dim, "matrix")
    d = _as_u32_array(demand, dim, "demand")
    rd = _as_u32_array(ready, dim, "ready")
    du = _as_u32_array(due, dim, "due")
    sv = _as_u32_array(service, dim, "service")
    opts = _Options(seed=seed, sisr_iters=iters, threads=threads,
                    veh_penalty=veh_penalty,
                    fleet_min=1 if fleet_min else 0, max_vehicles=max_vehicles, wall_ms=wall_ms,
                    max_route_duration=max_route_duration)
    out = ctypes.c_void_p()
    rc = _lib.commiv_solve_vrptw(m, n, d, capacity, rd, du, sv, ctypes.byref(opts), ctypes.byref(out))
    if rc != _OK:
        _raise_for(rc, "solve_vrptw")
    return _extract_routes(out)


def solve_pdptw(
    matrix,
    pickups: Sequence[int],
    deliveries: Sequence[int],
    demand: Sequence[int],
    capacity: int,
    ready: Sequence[int],
    due: Sequence[int],
    service: Sequence[int],
    *,
    seed: int = 1,
    iters: int = 0,
    veh_penalty: int = 0,
    time_penalty: int = 0,
    fleet_min: bool = False,
    max_vehicles: int = 0,
    max_route_duration: int = 0,
    wall_ms: int = 0,
    vehicle_types: Sequence[tuple[int, int, int]] | None = None,
    break_: tuple[int, int, int] | None = None,
) -> CvrpSolution:
    """Solve a directed PDPTW (pickup-and-delivery with time windows), SISR.

    dim = 2*n_pairs + 1 nodes: node 0 is the depot, and n_pairs = len(pickups).
    matrix is dim x dim directed costs (nested lists, flat list, or numpy),
    row-major, depot = node 0. pickups[i] and deliveries[i] are the node ids of
    request i (each carried on the SAME route, pickup before delivery); every
    node 1..dim-1 must appear exactly once across the two arrays. demand has
    n_pairs entries (the load of each request). ready/due/service have dim
    entries each and bound the START of service per node (ready[0]=service[0]=0,
    due[0] = depot horizon); waiting before a window opens is free.

    time_penalty > 0 is the MONEY objective (PDPTW-only): it charges each route's
    DURATION (travel + service + unavoidable waiting) on top of distance and
    veh_penalty, so the engine trades fuel for driver hours wherever waiting
    exists. 0 = pure distance, bit-identical to the historic objective.

    fleet_min=True runs the hierarchical vehicle-minimization driver. When
    max_vehicles > 0 the PDPTW solver runs the PINNED driver: it targets EXACTLY
    that many vehicles (the enterprise fixed-fleet case), not merely an upper
    cap. Both wall-driven drivers use wall_ms as their budget (0 defaults to
    10s). iters=0 means the SISR default. Note: threads is IGNORED for PDPTW.

    max_route_duration > 0 caps every route's duration (travel + service +
    unavoidable waiting) at that shift length; 0 = uncapped. This is a hard
    feasibility bound, distinct from the soft time_penalty cost term.

    vehicle_types = [(capacity, fixed_cost, count), ...] (1..8 entries) runs
    the HETEROGENEOUS-FLEET solver: each route is served by one type — its
    capacity bounds the route load, its fixed_cost replaces veh_penalty for
    that route, and at most `count` routes of the type run simultaneously
    (count 0 = unlimited). The result's .types lists route r's index into
    this list. The uniform `capacity` argument is ignored then, and
    fleet_min / max_vehicles are unsupported (bound the fleet with counts).

    break_ = (duration, earliest, latest) requires every route whose
    depart-at-0 schedule finishes after `earliest` to contain ONE driver break
    of `duration` time units starting within [earliest, latest]. The break
    absorbs waiting first and counts into route duration (so time_penalty
    prices it). Unsupported with fleet_min / max_vehicles."""
    dim = _matrix_dim(matrix)
    n_pairs = len(pickups)
    if len(deliveries) != n_pairs:
        raise ValueError(f"pickups and deliveries must be the same length ({n_pairs} vs {len(deliveries)})")
    if dim != 2 * n_pairs + 1:
        raise ValueError(f"matrix dim must be 2*n_pairs+1 = {2 * n_pairs + 1}, got {dim}")
    m = _as_u32_array(matrix, dim * dim, "matrix")
    pk = _as_u32_array(pickups, n_pairs, "pickups")
    dl = _as_u32_array(deliveries, n_pairs, "deliveries")
    dm = _as_u32_array(demand, n_pairs, "demand")
    rd = _as_u32_array(ready, dim, "ready")
    du = _as_u32_array(due, dim, "due")
    sv = _as_u32_array(service, dim, "service")
    opts = _Options(seed=seed, sisr_iters=iters, veh_penalty=veh_penalty,
                    time_penalty=time_penalty, fleet_min=1 if fleet_min else 0,
                    max_vehicles=max_vehicles, wall_ms=wall_ms,
                    max_route_duration=max_route_duration)
    if break_ is not None:
        if fleet_min or max_vehicles:
            raise ValueError("break_ is incompatible with fleet_min/max_vehicles (v1)")
        opts.break_duration, opts.break_earliest, opts.break_latest = break_
    out = ctypes.c_void_p()
    if vehicle_types is not None:
        vts = list(vehicle_types)
        if not 1 <= len(vts) <= 8:
            raise ValueError(f"vehicle_types needs 1..8 entries, got {len(vts)}")
        if fleet_min or max_vehicles:
            raise ValueError("vehicle_types is incompatible with fleet_min/max_vehicles; "
                             "bound the fleet with per-type counts instead")
        tcap = _as_u32_array([t[0] for t in vts], len(vts), "vehicle_types capacity")
        tfix = (ctypes.c_uint64 * len(vts))(*[int(t[1]) for t in vts])
        tcnt = _as_u32_array([t[2] for t in vts], len(vts), "vehicle_types count")
        rc = _lib.commiv_solve_pdptw_typed(m, n_pairs, pk, dl, dm, rd, du, sv,
                                           tcap, tfix, tcnt, len(vts),
                                           ctypes.byref(opts), ctypes.byref(out))
    else:
        rc = _lib.commiv_solve_pdptw(m, n_pairs, pk, dl, dm, capacity, rd, du, sv,
                                     ctypes.byref(opts), ctypes.byref(out))
    if rc != _OK:
        _raise_for(rc, "solve_pdptw")
    return _extract_routes(out)


def solve_pdptw_dispatch(
    matrix,
    pickups: Sequence[int],
    deliveries: Sequence[int],
    demand: Sequence[int],
    capacity: int,
    ready: Sequence[int],
    due: Sequence[int],
    service: Sequence[int],
    current_routes: Sequence[Sequence[int]],
    locked: Sequence[int],
    *,
    seed: int = 1,
    iters: int = 0,
    veh_penalty: int = 0,
    time_penalty: int = 0,
    max_route_duration: int = 0,
    wall_ms: int = 0,
    break_: tuple[int, int, int] | None = None,
) -> CvrpSolution:
    """Rolling-horizon PDPTW re-solve: same instance contract as solve_pdptw,
    plus the CURRENT plan and its LOCKED prefixes.

    current_routes: one list of node ids per vehicle (may be empty: a cold
    start). locked: one entry per route, len(locked) == len(current_routes);
    locked[i] is how many of current_routes[i]'s leading stops are committed
    (in progress or already served) and must not move in the result. A locked
    delivery's pickup must also be locked, in the SAME route, or this raises
    ValueError.

    fleet_min/max_vehicles are not accepted here: dispatch keeps the current
    fleet shape. Node-id stability: when new orders arrive, rebuild a LARGER
    instance keeping every existing node's index and appending the new pairs
    at the end; old routes stay valid warm input for the next dispatch call.
    See DispatchSession for a stateful wrapper around this loop."""
    if len(locked) != len(current_routes):
        raise ValueError(f"locked must have one entry per route ({len(current_routes)}), got {len(locked)}")
    dim = _matrix_dim(matrix)
    n_pairs = len(pickups)
    if len(deliveries) != n_pairs:
        raise ValueError(f"pickups and deliveries must be the same length ({n_pairs} vs {len(deliveries)})")
    if dim != 2 * n_pairs + 1:
        raise ValueError(f"matrix dim must be 2*n_pairs+1 = {2 * n_pairs + 1}, got {dim}")
    m = _as_u32_array(matrix, dim * dim, "matrix")
    pk = _as_u32_array(pickups, n_pairs, "pickups")
    dl = _as_u32_array(deliveries, n_pairs, "deliveries")
    dm = _as_u32_array(demand, n_pairs, "demand")
    rd = _as_u32_array(ready, dim, "ready")
    du = _as_u32_array(due, dim, "due")
    sv = _as_u32_array(service, dim, "service")

    n_routes = len(current_routes)
    offsets = [0]
    flat_nodes: list[int] = []
    for route in current_routes:
        flat_nodes.extend(route)
        offsets.append(len(flat_nodes))
    cur_offsets = (ctypes.c_size_t * (n_routes + 1))(*offsets)
    cur_nodes = (ctypes.c_uint32 * len(flat_nodes))(*flat_nodes)
    locked_arr = (ctypes.c_size_t * n_routes)(*locked)

    opts = _Options(seed=seed, sisr_iters=iters, veh_penalty=veh_penalty,
                    time_penalty=time_penalty, wall_ms=wall_ms,
                    max_route_duration=max_route_duration)
    if break_ is not None:
        opts.break_duration, opts.break_earliest, opts.break_latest = break_
    out = ctypes.c_void_p()
    rc = _lib.commiv_solve_pdptw_dispatch(
        m, n_pairs, pk, dl, dm, capacity, rd, du, sv,
        cur_offsets, n_routes, cur_nodes, locked_arr,
        ctypes.byref(opts), ctypes.byref(out),
    )
    if rc != _OK:
        _raise_for(rc, "solve_pdptw_dispatch")
    return _extract_routes(out)


class DispatchSession:
    """Stateful convenience wrapper over solve_pdptw / solve_pdptw_dispatch for
    a rolling-horizon shift: owns the instance arrays and the current plan,
    grows the instance as new orders arrive, and re-solves around committed
    (locked) prefixes as the clock advances. The C ABI and REST doors stay
    stateless on purpose; this class is the session, in Python only.

    matrix must be a plain mutable nested list (row per node) — add_order
    grows it in place, so numpy/flat inputs are not accepted here.

    Q-commerce loop example::

        session = DispatchSession(matrix, pickups, deliveries, demand,
                                   capacity, ready, due, service, wall_ms=2000)
        session.solve()                     # initial plan
        while shift_running:
            session.advance_to(now)         # lock stops already reached
            for order in new_orders():
                session.add_order(...)      # grow the instance, unlocked
            session.solve()                  # re-optimize around the locks
    """

    def __init__(
        self,
        matrix,
        pickups: Sequence[int],
        deliveries: Sequence[int],
        demand: Sequence[int],
        capacity: int,
        ready: Sequence[int],
        due: Sequence[int],
        service: Sequence[int],
        *,
        seed: int = 1,
        iters: int = 0,
        veh_penalty: int = 0,
        time_penalty: int = 0,
        max_route_duration: int = 0,
        wall_ms: int = 0,
        break_: tuple[int, int, int] | None = None,
    ) -> None:
        dim = _matrix_dim(matrix)
        n_pairs = len(pickups)
        if len(deliveries) != n_pairs:
            raise ValueError(f"pickups and deliveries must be the same length ({n_pairs} vs {len(deliveries)})")
        if dim != 2 * n_pairs + 1:
            raise ValueError(f"matrix dim must be 2*n_pairs+1 = {2 * n_pairs + 1}, got {dim}")
        self._matrix: list[list[int]] = [list(row) for row in matrix]
        self.pickups = list(pickups)
        self.deliveries = list(deliveries)
        self.demand = list(demand)
        self.capacity = capacity
        self.ready = list(ready)
        self.due = list(due)
        self.service = list(service)
        self.seed = seed
        self.iters = iters
        self.veh_penalty = veh_penalty
        self.time_penalty = time_penalty
        self.max_route_duration = max_route_duration
        self.wall_ms = wall_ms
        self.break_ = break_
        self.t = 0
        self.plan: list[list[int]] = []  # current routes, node ids
        self._locked: list[int] | None = None

    def solve(self, **kw) -> CvrpSolution:
        """Solve the current instance. The first call is a cold solve_pdptw;
        every later call warm-starts from self.plan using the locks computed
        by the last advance_to() (all-zero locks if advance_to was never
        called since the last solve). Stores and returns the new plan."""
        opts = dict(seed=self.seed, iters=self.iters, veh_penalty=self.veh_penalty,
                    time_penalty=self.time_penalty, max_route_duration=self.max_route_duration,
                    wall_ms=self.wall_ms, break_=self.break_)
        opts.update(kw)
        if not self.plan:
            sol = solve_pdptw(
                self._matrix, self.pickups, self.deliveries, self.demand,
                self.capacity, self.ready, self.due, self.service, **opts,
            )
        else:
            locked = self._locked if self._locked is not None else [0] * len(self.plan)
            sol = solve_pdptw_dispatch(
                self._matrix, self.pickups, self.deliveries, self.demand,
                self.capacity, self.ready, self.due, self.service,
                self.plan, locked, **opts,
            )
        self.plan = [list(r) for r in sol.routes]
        self._locked = None  # consumed; advance_to must recompute it for the next solve
        return sol

    def add_order(
        self,
        row_p: Sequence[int],
        col_p: Sequence[int],
        row_q: Sequence[int],
        col_q: Sequence[int],
        p_to_q: int,
        q_to_p: int,
        amount: int,
        p_ready: int,
        p_due: int,
        q_ready: int,
        q_due: int,
        p_service: int = 300,
        q_service: int = 300,
    ) -> tuple[int, int]:
        """Grow the instance with one new pickup/delivery pair, keeping every
        existing node's index unchanged (the current plan and locks stay
        valid warm input).

        row_p/col_p: travel times pickup -> existing node / existing node ->
        pickup, one entry per current node (len == current dim, depot
        included). row_q/col_q: the same for the delivery. p_to_q/q_to_p:
        direct pickup<->delivery times. amount: request load. p_ready/p_due/
        q_ready/q_due: time windows for the pickup/delivery. p_service/
        q_service: service durations (default 300s = 5 min).

        Returns (pickup_node_id, delivery_node_id) — the new node ids, which
        are always dim and dim+1 of the instance BEFORE this call."""
        dim = len(self._matrix)
        if len(row_p) != dim or len(col_p) != dim or len(row_q) != dim or len(col_q) != dim:
            raise ValueError(f"row/col vectors must have length {dim} (current dim)")
        new_p, new_q = dim, dim + 1

        for i, row in enumerate(self._matrix):
            row.append(col_p[i])
            row.append(col_q[i])
        self._matrix.append(list(row_p) + [0, p_to_q])
        self._matrix.append(list(row_q) + [q_to_p, 0])

        self.pickups.append(new_p)
        self.deliveries.append(new_q)
        self.demand.append(amount)
        self.ready.extend([p_ready, q_ready])
        self.due.extend([p_due, q_due])
        self.service.extend([p_service, q_service])
        return new_p, new_q

    def advance_to(self, t: int) -> None:
        """Compute locked prefixes for the CURRENT plan as of clock time t,
        for the next solve(). Schedule recurrence per route: depart the depot
        at time 0; for each stop, arrival = time + travel(prev, node), start
        = max(arrival, ready[node]), departure = start + service[node]. A
        stop is LOCKED iff its start <= t; locked[route] = count of leading
        locked stops.

        A locked delivery's pickup is always locked too: the pickup precedes
        its delivery on the same route and start times are non-decreasing
        along a route, so the pickup was reached (and locked) no later."""
        self.t = t
        locked: list[int] = []
        for route in self.plan:
            time = 0
            prev = 0
            lk = 0
            for pos, node in enumerate(route):
                arrival = time + self._matrix[prev][node]
                start = max(arrival, self.ready[node])
                if start <= t:
                    lk = pos + 1
                time = start + self.service[node]
                prev = node
            locked.append(lk)
        self._locked = locked


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
