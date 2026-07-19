const std = @import("std");

/// Granular neighbor-list proximity key. `.sum` (d(c,j)+d(j,c)) is the
/// historical default; `.min` is the measured lever for strongly one-way
/// street grids; `.out` ranks by outbound arc only.
pub const NbrKey = enum { sum, min, out };

/// k-nearest-neighbor list construction shared by the VRPTW and PDPTW SISR
/// engines: for each node c in 1..n, full-sort all other nodes by the chosen
/// key (std.sort.pdq — deterministic for a given input order) and keep the
/// first k. Returns an n*k row-major table; rows are 1-based nodes at
/// (c-1)*k, zero-padded when n-1 < k. `Inst` must provide d(a, b); `n` is
/// passed explicitly because the engines derive it differently (inst.n vs
/// 2*n_pairs). CVRP keeps its own bounded top-k variant in cvrp_solution.zig
/// (deliberate divergence: measured-equivalent optimization with an explicit
/// (key, index) tiebreak).
pub fn buildNeighborsKeyed(comptime Inst: type, allocator: std.mem.Allocator, inst: Inst, n: usize, k: usize, key_mode: NbrKey) ![]usize {
    const gran = try allocator.alloc(usize, n * k);
    @memset(gran, 0);
    const kk = @min(k, if (n > 1) n - 1 else 0);
    const idx = try allocator.alloc(usize, n);
    defer allocator.free(idx);
    const keyc = try allocator.alloc(u64, n + 1);
    defer allocator.free(keyc);
    for (1..n + 1) |c| {
        var m: usize = 0;
        for (1..n + 1) |j| {
            if (j == c) continue;
            keyc[j] = switch (key_mode) {
                .sum => inst.d(c, j) + inst.d(j, c),
                .min => @min(inst.d(c, j), inst.d(j, c)),
                .out => inst.d(c, j),
            };
            idx[m] = j;
            m += 1;
        }
        std.sort.pdq(usize, idx[0..m], keyc, struct {
            fn lt(key: []const u64, a: usize, b: usize) bool {
                return key[a] < key[b];
            }
        }.lt);
        for (0..kk) |i| gran[(c - 1) * k + i] = idx[i];
    }
    return gran;
}
