const std = @import("std");

// Shared HGS giant-tour genetic operators (fix G8). These three operators were
// byte-identical across vrp.zig (cvrpBuildEdges / cvrpEdgeOverlap / cvrpOxCrossover)
// and vrptw.zig (buildEdges / edgeOverlap / oxCrossover); they are extracted here
// once and called from both. Behaviour is intentionally unchanged.
//
// TODO(G8): the larger giant-tour scaffold (Solution / Split DP / educate /
// HgsParams) is still duplicated-but-diverged across the two families and is NOT
// unified here (out of scope for this commit, behaviour-sensitive). Unify in a
// follow-up.

/// Undirected giant-cycle edge set (sorted) used as a cheap structural fingerprint
/// for the broken-pairs diversity distance between two individuals.
pub fn buildEdges(allocator: std.mem.Allocator, giant: []const usize, n: usize) ![]u64 {
    const e = try allocator.alloc(u64, n);
    const base: u64 = @intCast(n + 1);
    for (0..n) |k| {
        const a = giant[k];
        const b = giant[(k + 1) % n];
        const lo: u64 = @intCast(@min(a, b));
        const hi: u64 = @intCast(@max(a, b));
        e[k] = lo * base + hi;
    }
    std.mem.sort(u64, e, {}, std.sort.asc(u64));
    return e;
}

/// Count of shared undirected edges between two sorted edge fingerprints
/// (merge-join); the broken-pairs diversity distance is `n - edgeOverlap(a, b)`.
pub fn edgeOverlap(a: []const u64, b: []const u64) usize {
    var i: usize = 0;
    var j: usize = 0;
    var c: usize = 0;
    while (i < a.len and j < b.len) {
        if (a[i] == b[j]) {
            c += 1;
            i += 1;
            j += 1;
        } else if (a[i] < b[j]) {
            i += 1;
        } else {
            j += 1;
        }
    }
    return c;
}

/// Order crossover (OX): copy a random slice of p1, fill the rest with p2's
/// customers in cyclic order (skipping those already taken). Yields a valid
/// permutation.
pub fn oxCrossover(allocator: std.mem.Allocator, p1: []const usize, p2: []const usize, n: usize, rng: std.Random) ![]usize {
    const child = try allocator.alloc(usize, n);
    errdefer allocator.free(child);
    const used = try allocator.alloc(bool, n + 1);
    defer allocator.free(used);
    @memset(used, false);
    var i = rng.uintLessThan(usize, n);
    var j = rng.uintLessThan(usize, n);
    if (i > j) {
        const t = i;
        i = j;
        j = t;
    }
    var k = i;
    while (k <= j) : (k += 1) {
        child[k] = p1[k];
        used[p1[k]] = true;
    }
    var pos = (j + 1) % n;
    var idx = (j + 1) % n;
    var remaining = n - (j - i + 1);
    while (remaining > 0) {
        const city = p2[idx];
        if (!used[city]) {
            child[pos] = city;
            used[city] = true;
            pos = (pos + 1) % n;
            remaining -= 1;
        }
        idx = (idx + 1) % n;
    }
    return child;
}
