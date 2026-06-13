#!/usr/bin/env python3
"""Edge-set analysis for item-3 voting-freeze revival.

Parses LKH .tour files and PROF_TOUR_OUT dumps (both 1-indexed node
sequences) into undirected edge sets, and computes backbone/consensus
purity vs a reference optimal tour.
"""
import sys, glob


def read_lkh_tour(path):
    nodes = []
    in_sec = False
    for line in open(path):
        s = line.strip()
        if s == "TOUR_SECTION":
            in_sec = True
            continue
        if not in_sec:
            continue
        if s in ("-1", "EOF", ""):
            if s == "-1" or s == "EOF":
                break
            continue
        nodes.append(int(s))
    return nodes


def read_plain_tour(path):
    nodes = []
    for line in open(path):
        s = line.strip()
        if s and s not in ("-1", "EOF"):
            nodes.append(int(s))
    return nodes


def read_edge_pairs(path):
    """Frozen-edge dump: one 'u v' (1-indexed) undirected edge per line."""
    es = set()
    for line in open(path):
        s = line.split()
        if len(s) == 2:
            a, b = int(s[0]), int(s[1])
            es.add((a, b) if a < b else (b, a))
    return es


def edges(nodes):
    n = len(nodes)
    es = set()
    for i in range(n):
        a, b = nodes[i], nodes[(i + 1) % n]
        es.add((a, b) if a < b else (b, a))
    return es


def consensus(edge_sets, k):
    """Edges present in at least k of the given edge sets."""
    from collections import Counter
    c = Counter()
    for es in edge_sets:
        for e in es:
            c[e] += 1
    return {e for e, cnt in c.items() if cnt >= k}


def report(name, frozen, optimal, total_n):
    inter = frozen & optimal
    purity = len(inter) / len(frozen) if frozen else 0.0
    recall = len(inter) / len(optimal) if optimal else 0.0
    print(f"  {name}: |frozen|={len(frozen)} purity={purity:.3f} "
          f"recall_of_opt={recall:.3f} traps={len(frozen)-len(inter)} "
          f"(opt has {len(optimal)} edges, n={total_n})")
    return purity, recall


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "consensus":
        # consensus <optimal.tour> <k> <plain_tour_glob...>
        opt = edges(read_lkh_tour(sys.argv[2]))
        k = int(sys.argv[3])
        paths = []
        for g in sys.argv[4:]:
            paths += sorted(glob.glob(g))
        ess = [edges(read_plain_tour(p)) for p in paths]
        n = len(read_plain_tour(paths[0]))
        print(f"  votes from {len(ess)} tours, consensus k>={k}")
        cons = consensus(ess, k)
        report(f"consensus(k>={k})", cons, opt, n)
        # also report the single-tour purity (correlated proxy)
        report("single tour[0]", ess[0], opt, n)
        # pairwise diversity: avg symmetric-difference size
        import itertools
        diffs = [len(a ^ b) for a, b in itertools.combinations(ess, 2)]
        if diffs:
            print(f"  pairwise symdiff: min={min(diffs)} avg={sum(diffs)/len(diffs):.1f} max={max(diffs)}")
    elif cmd == "purity":
        # purity <optimal.tour> <frozen_edges.txt> [final_tour_plain]
        opt = edges(read_lkh_tour(sys.argv[2]))
        frozen = read_edge_pairs(sys.argv[3])
        n = len(read_lkh_tour(sys.argv[2]))
        report("frozen-set", frozen, opt, n)
        if len(sys.argv) > 4:
            final = edges(read_plain_tour(sys.argv[4]))
            # of the frozen edges, how many are NOT in the final tour (i.e. the
            # freeze fixed an edge the search later wanted gone — a real trap)?
            not_in_final = frozen - final
            print(f"  frozen edges absent from final tour: {len(not_in_final)}")
    elif cmd == "emit_consensus":
        # emit_consensus <out.txt> <k> <plain_tour_glob...>
        out = sys.argv[2]
        k = int(sys.argv[3])
        paths = []
        for g in sys.argv[4:]:
            paths += sorted(glob.glob(g))
        ess = [edges(read_plain_tour(p)) for p in paths]
        cons = consensus(ess, k)
        with open(out, "w") as f:
            for a, b in sorted(cons):
                f.write(f"{a} {b}\n")
        print(f"  wrote {len(cons)} consensus edges (k>={k} of {len(ess)}) to {out}")
    elif cmd == "intersect":
        # intersect <out.txt> <frozen.txt> <optimal.tour>
        #   emit frozen ∩ optimal — the voted set with its traps removed.
        out = sys.argv[2]
        frozen = read_edge_pairs(sys.argv[3])
        opt = edges(read_lkh_tour(sys.argv[4]))
        keep = frozen & opt
        with open(out, "w") as f:
            for a, b in sorted(keep):
                f.write(f"{a} {b}\n")
        print(f"  de-trapped: {len(frozen)} frozen -> {len(keep)} pure "
              f"(dropped {len(frozen)-len(keep)} traps) to {out}")
    elif cmd == "emit_optimal_subset":
        # emit_optimal_subset <out.txt> <recall> <seed> <optimal.tour>
        #   a deterministic 100%-pure backbone: keep `recall` of the optimal edges.
        import hashlib
        out = sys.argv[2]
        recall = float(sys.argv[3])
        seed = sys.argv[4]
        opt = sorted(edges(read_lkh_tour(sys.argv[5])))
        keep = []
        for a, b in opt:
            h = hashlib.md5(f"{seed}-{a}-{b}".encode()).hexdigest()
            if (int(h, 16) % 1000) / 1000.0 < recall:
                keep.append((a, b))
        with open(out, "w") as f:
            for a, b in keep:
                f.write(f"{a} {b}\n")
        print(f"  wrote {len(keep)}/{len(opt)} pure optimal edges (recall~{recall}) to {out}")
    elif cmd == "emit_optimal":
        # emit_optimal <out.txt> <optimal.tour>  (the perfect backbone)
        out = sys.argv[2]
        opt = edges(read_lkh_tour(sys.argv[3]))
        with open(out, "w") as f:
            for a, b in sorted(opt):
                f.write(f"{a} {b}\n")
        print(f"  wrote {len(opt)} optimal edges to {out}")
    elif cmd == "sweep_k":
        # sweep_k <optimal.tour> <plain_tour_glob...>
        opt = edges(read_lkh_tour(sys.argv[2]))
        paths = []
        for g in sys.argv[3:]:
            paths += sorted(glob.glob(g))
        ess = [edges(read_plain_tour(p)) for p in paths]
        n = len(read_plain_tour(paths[0]))
        K = len(ess)
        print(f"  {K} tours, n={n}, opt={len(opt)} edges")
        for k in range(1, K + 1):
            cons = consensus(ess, k)
            report(f"k>={k}/{K}", cons, opt, n)
