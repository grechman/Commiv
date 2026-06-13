#!/usr/bin/env python3
"""Cheap static plateau-degeneracy proxy: among each node's k nearest
neighbours (TSPLIB EUC_2D rounded distances), what fraction sit on a tie?
Rattled grids on small integer coords produce many tied distances -> broad
cost-equal plateaus where the residual optimality gap hides."""
import sys, math


def read_coords(path):
    coords = []
    in_sec = False
    for line in open(path):
        s = line.strip()
        if s.startswith("NODE_COORD_SECTION"):
            in_sec = True
            continue
        if not in_sec:
            continue
        if s in ("EOF", ""):
            continue
        parts = s.split()
        if len(parts) >= 3:
            coords.append((float(parts[1]), float(parts[2])))
    return coords


def euc(a, b):
    return int(round(math.hypot(a[0] - b[0], a[1] - b[1])))


def plateau_density(coords, k=8):
    n = len(coords)
    tied_nodes = 0
    total_tie_pairs = 0
    distinct_global = set()
    for i in range(n):
        ds = sorted(euc(coords[i], coords[j]) for j in range(n) if j != i)[:k]
        distinct_global.update(ds)
        # count duplicate distance values among the k nearest
        seen = {}
        for d in ds:
            seen[d] = seen.get(d, 0) + 1
        ties = sum(c - 1 for c in seen.values() if c > 1)
        if ties:
            tied_nodes += 1
        total_tie_pairs += ties
    return tied_nodes / n, total_tie_pairs / n, len(distinct_global)


if __name__ == "__main__":
    for path in sys.argv[1:]:
        coords = read_coords(path)
        if len(coords) > 2500:
            coords = coords[:2500]  # cap cost for big instances; proxy only
        frac, per_node, distinct = plateau_density(coords)
        name = path.split("/")[-1]
        print(f"{name:16s} n={len(coords):5d} tied_node_frac={frac:.3f} "
              f"tie_pairs/node={per_node:.3f} distinct_knn_dists={distinct}")
