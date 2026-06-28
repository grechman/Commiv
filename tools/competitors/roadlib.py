"""Shared .road parser + scorer. Cost accounting MATCHES examples/roadbench.zig
exactly: each route starts at depot 0, visits customers, returns to depot 0."""
from array import array


def parse_road(path):
    dim = cap = 0
    demand = None
    M = None
    coords = None      # [(lng, lat)] per node, for spatial heuristics (sweep warmstart)
    mode = None
    row = 0
    with open(path) as f:
        for line in f:
            t = line.split()
            if not t:
                continue
            h = t[0]
            if h == 'NAME':
                continue
            elif h == 'DIM':
                dim = int(t[1])
                demand = array('i', bytes(4 * dim))        # dim zeros, no transient list
                M = array('q', bytes(8 * dim * dim))        # dim*dim zeros
                coords = [(0.0, 0.0)] * dim
            elif h == 'CAPACITY':
                cap = int(t[1])
            elif h == 'COORDS':
                mode = 'coords'
            elif h == 'MATRIX':
                mode = 'matrix'
            elif mode == 'coords':
                idx = int(t[0])
                coords[idx] = (float(t[1]), float(t[2]))
                demand[idx] = int(t[3])
            elif mode == 'matrix':
                b = row * dim
                assert len(t) == dim, f"bad matrix row {row}: {len(t)} != {dim}"
                for j, v in enumerate(t):
                    M[b + j] = int(v)
                row += 1
    assert dim and row == dim, f"bad parse: dim={dim} rows={row}"
    return dim, cap, demand, M, coords


def score(M, dim, routes):
    """Total directed cost: sum over routes of depot->...->depot."""
    tot = 0
    for r in routes:
        prev = 0
        for c in r:
            tot += M[prev * dim + c]
            prev = c
        tot += M[prev * dim + 0]
    return tot


def validate(dim, cap, demand, routes):
    """Return None if valid, else an error string. Every customer 1..dim-1 once,
    each route within capacity."""
    seen = set()
    for r in routes:
        load = 0
        for c in r:
            if c <= 0 or c >= dim:
                return f"bad customer index {c}"
            if c in seen:
                return f"duplicate customer {c}"
            seen.add(c)
            load += demand[c]
        if load > cap:
            return f"route load {load} > capacity {cap}"
    need = set(range(1, dim))
    if seen != need:
        return f"coverage off: missing {len(need - seen)}, extra {len(seen - need)}"
    return None
