const std = @import("std");
const problem = @import("problem.zig");
const solver = @import("solver.zig");

// Asymmetric TSP (ATSP) support via the Jonker-Volgenant 2n-node transform: an
// n-city directed instance becomes a 2n-city SYMMETRIC instance solved by the
// existing (validated) symmetric core, then the directed tour is recovered. This
// is the FOUNDATION for real routing (road networks are asymmetric) and the
// correctness oracle for any future native-asymmetric search.
//
// Transform: city i -> tail node i and head node n+i.
//   pair edge   (i, n+i)      = 0            forces i's two halves adjacent
//   arc i->j    (n+i, j)      = a(i,j) + BIG   the directed arc, reachable from j's tail
//   everything else            = INF           forbidden
// BIG makes every arc cost strictly dominate the free pair edges, so the minimal
// symmetric tour uses all n pair edges -> it alternates tail,head,tail,head and
// the tails in tour order are the directed cycle. Directed length = symmetric
// length - n*BIG.

// Above this transformed-matrix size, solveAtsp routes to the native directed search
// instead of building the (2n)^2 transform matrix. 32M cells * 4B = 128MB; the doubled
// Problem copy would be another 128MB. Triggers at n > ~2828 (e.g. the n=5000 CVRP seed).
const ATSP_TRANSFORM_MAX_CELLS: u64 = 32 * 1024 * 1024;

// A read-only window into a (possibly larger) row-major matrix: cell (a,b) is
// base[(a+off)*stride + (b+off)]. Lets the native ATSP run on the customer block of a
// CVRP (n+1)x(n+1) matrix (stride=n+1, off=1) WITHOUT copying an n*n sub out — the
// giant-tour seed points straight at inst.matrix, so RAM stays at the base footprint.
const MatView = struct {
    base: []const u32,
    stride: usize,
    off: usize,
    inline fn at(self: MatView, a: usize, b: usize) u32 {
        return self.base[(a + self.off) * self.stride + (b + self.off)];
    }
};

pub const AtspResult = struct {
    allocator: std.mem.Allocator,
    tour: []usize, // directed city order, length n
    length: u64, // true directed tour length
    pub fn deinit(self: *AtspResult) void {
        self.allocator.free(self.tour);
        self.* = undefined;
    }
};

/// Solve an n-city ATSP given a row-major n*n directed cost matrix `asym`
/// (asym[i*n+j] = cost of arc i->j; diagonal ignored).
pub fn solveAtsp(allocator: std.mem.Allocator, asym: []const u32, n: usize, options: solver.SolveOptions) !AtspResult {
    if (n < 2 or asym.len != n * n) return error.InvalidMatrix;

    // Degeneracy = arcs tying their row minimum, averaged per row (~1 for a real /
    // classic matrix, ~15-31 for the rbg stacker-crane set). On those degenerate
    // instances the 2n transform is a net loss: the native directed search reaches
    // the OPTIMUM faster than the transform reaches its ~0.1% (rbg323 0.000%@18s vs
    // transform 0.075%@28s; rbg443 0.000%@25s vs 0.110%@28s), because it works n
    // nodes not 2n and carries no BIG-offset degeneracy. Route them to native; native
    // kicks are ~100x cheaper than an LK trial, so scale the budget accordingly.
    {
        var tied: u64 = 0;
        for (0..n) |i| {
            var rmin: u32 = std.math.maxInt(u32);
            for (0..n) |j| {
                if (i != j and asym[i * n + j] < rmin) rmin = asym[i * n + j];
            }
            for (0..n) |j| {
                if (i != j and asym[i * n + j] == rmin) tied += 1;
            }
        }
        if (tied > 4 * n) {
            var nopts = options;
            nopts.budget.trials = options.budget.trials *| 100;
            return solveAtspNative(allocator, asym, n, nopts);
        }
    }

    // The 2n transform materializes a (2n)^2 matrix AND Problem.initFullMatrix dupes it
    // — 400MB + 400MB at n=5000, per call. That transient peak (× each worker thread)
    // is where the CVRP giant-tour seed's ~2GB comes from. For large n the doubled-graph
    // LK isn't worth that memory, especially for a throwaway seed SISR will rip apart:
    // route to the native directed search (n nodes, no doubling, no full-matrix copy).
    // Budget passed through as-is (no degeneracy ×100 — large-n native descents aren't cheap).
    if (@as(u64, 2 * n) * @as(u64, 2 * n) > ATSP_TRANSFORM_MAX_CELLS) {
        return solveAtspNative(allocator, asym, n, options);
    }

    var max_arc: u64 = 0;
    for (0..n) |i| {
        for (0..n) |j| {
            if (i != j) max_arc = @max(max_arc, asym[i * n + j]);
        }
    }
    // BIG forces all n pair edges into the tour; INF forbids the rest. Sized to
    // stay within u32 matrix entries (ok for small/medium n — the routing regime).
    const big: u64 = (n + 1) * max_arc + 1;
    const inf: u64 = big + (n + 2) * (max_arc + 1) + 1;
    if (inf > std.math.maxInt(u32)) return error.InstanceTooLargeForTransform;
    const inf32: u32 = @intCast(inf);

    const m = 2 * n;
    const sym = try allocator.alloc(u32, m * m);
    defer allocator.free(sym);
    @memset(sym, inf32);
    for (0..m) |i| sym[i * m + i] = 0;
    for (0..n) |i| {
        const head = n + i;
        sym[i * m + head] = 0;
        sym[head * m + i] = 0;
        for (0..n) |j| {
            if (i == j) continue;
            const w: u32 = @intCast(@as(u64, asym[i * n + j]) + big);
            // arc i->j connects head(i) to tail(j)
            sym[head * m + j] = w;
            sym[j * m + head] = w;
        }
    }

    var p = try problem.Problem.initFullMatrix(allocator, "atsp", m, sym);
    defer p.deinit();

    // Inject candidates ranked by the ORIGINAL asymmetric costs. On the transformed
    // matrix every arc is BIG + a(i,j), so the cost differences that matter (a) are
    // a vanishing fraction of the edge weight and the 1-tree / nearest ranking can't
    // see them. Ranking by the raw arc costs instead fixes that (decisive on the
    // heavily-structured rbg instances).
    // Non-degenerate (classic / geographic) instances: the transform's LK is strong
    // and width 16 solves them to optimum. Degenerate instances were routed to the
    // native directed search above, so the old degeneracy-gated width-32 workaround
    // (which only ever existed to fight the transform) is retired.
    const cand_w = @min(@max(options.candidates.candidate_count, 16), n);
    var atsp_cands = try buildAtspCandidates(allocator, asym, n, big, cand_w);
    defer atsp_cands.deinit();
    var opts = options;
    opts.candidates.candidate_count = cand_w;
    opts.injected_candidates = &atsp_cands;

    var res = try solver.solve(allocator, &p, opts);
    defer res.deinit();

    // Recover the directed cycle: walk the symmetric tour, emit tail nodes (< n)
    // in order. Orientation: a tail i is followed (across its head) by the tail it
    // arcs INTO. We read in the tour's traversal direction, then fix orientation
    // by comparing the directed length both ways.
    const dir = try allocator.alloc(usize, n);
    errdefer allocator.free(dir);
    var k: usize = 0;
    for (res.tour) |node| {
        if (node < n) {
            dir[k] = node;
            k += 1;
        }
    }
    std.debug.assert(k == n);

    const fwd = directedLen(asym, n, dir, false);
    const bwd = directedLen(asym, n, dir, true);
    if (bwd < fwd) std.mem.reverse(usize, dir);
    const length = @min(fwd, bwd);

    return .{ .allocator = allocator, .tour = dir, .length = length };
}

// --- Best-of-K parallel ATSP -------------------------------------------------
const AtspSlot = struct {
    asym: []const u32,
    n: usize,
    options: solver.SolveOptions,
    seed: u64,
    tour: []usize, // parent-owned, size n
    length: u64 = std.math.maxInt(u64),
    ok: bool = false,
};

fn atspWorker(slot: *AtspSlot) void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    var opts = slot.options;
    opts.seed = slot.seed;
    const res = solveAtsp(arena.allocator(), slot.asym, slot.n, opts) catch {
        slot.ok = false;
        return;
    };
    @memcpy(slot.tour, res.tour);
    slot.length = res.length;
    slot.ok = true;
}

/// Run `threads` independent ATSP solves (seed + i) and return the best directed
/// tour. threads<=1 is the plain serial path.
pub fn solveAtspParallel(allocator: std.mem.Allocator, asym: []const u32, n: usize, options: solver.SolveOptions, threads: usize) !AtspResult {
    const cpus = std.Thread.getCpuCount() catch 1;
    const k = if (threads == 0) @max(@as(usize, 1), cpus -| 1) else threads;
    if (k <= 1 or n < 2) return solveAtsp(allocator, asym, n, options);

    const slots = try allocator.alloc(AtspSlot, k);
    defer allocator.free(slots);
    var allocated: usize = 0;
    errdefer for (slots[0..allocated]) |s| allocator.free(s.tour);
    for (slots, 0..) |*s, i| {
        s.* = .{ .asym = asym, .n = n, .options = options, .seed = options.seed +% i, .tour = try allocator.alloc(usize, n) };
        allocated += 1;
    }
    const ths = try allocator.alloc(std.Thread, k);
    defer allocator.free(ths);
    var spawned: usize = 0;
    for (0..k) |i| {
        ths[i] = std.Thread.spawn(.{}, atspWorker, .{&slots[i]}) catch break;
        spawned += 1;
    }
    for (spawned..k) |i| atspWorker(&slots[i]);
    for (0..spawned) |i| ths[i].join();

    var best: ?usize = null;
    for (slots, 0..) |s, i| {
        if (!s.ok) continue;
        if (best == null or s.length < slots[best.?].length) best = i;
    }
    const winner = best orelse return error.AllChainsFailed;
    const tour = try allocator.dupe(usize, slots[winner].tour);
    const len = slots[winner].length;
    for (slots) |s| allocator.free(s.tour);
    return .{ .allocator = allocator, .tour = tour, .length = len };
}

/// Build candidate lists for the 2n-node transform, ranked by the ORIGINAL
/// asymmetric arc costs. Each tail node i gets its pair head plus the heads of the
/// cities with the cheapest INCOMING arcs to i; each head node gets its pair tail
/// plus the tails of the cheapest OUTGOING arcs. cand_dist holds the transformed
/// distance (0 for the pair edge, a+BIG for an arc) so the solver's machinery is
/// unchanged; only the SELECTION/ORDER comes from the raw costs. width must be <= n.
fn buildAtspCandidates(allocator: std.mem.Allocator, asym: []const u32, n: usize, big: u64, width: usize) !solver.Candidates {
    const m = 2 * n;
    const data = try allocator.alloc(usize, m * width);
    errdefer allocator.free(data);
    const alpha = try allocator.alloc(u64, m * width);
    errdefer allocator.free(alpha);
    const cand_dist = try allocator.alloc(u32, m * width);
    errdefer allocator.free(cand_dist);
    const idx = try allocator.alloc(usize, n);
    defer allocator.free(idx);
    const key = try allocator.alloc(u64, n);
    defer allocator.free(key);

    const lessThan = struct {
        fn lt(k: []const u64, a: usize, b: usize) bool {
            return k[a] < k[b];
        }
    }.lt;

    var node: usize = 0;
    while (node < m) : (node += 1) {
        const is_tail = node < n;
        const i = if (is_tail) node else node - n;
        const base = node * width;
        // candidate 0 = the mandatory pair edge (free, forces i's halves adjacent)
        data[base] = if (is_tail) n + i else i;
        cand_dist[base] = 0;
        alpha[base] = 0;
        // rank the other cities by the relevant directed arc cost
        var cnt: usize = 0;
        for (0..n) |j| {
            if (j == i) continue;
            idx[cnt] = j;
            key[j] = if (is_tail) asym[j * n + i] else asym[i * n + j]; // incoming / outgoing
            cnt += 1;
        }
        std.sort.pdq(usize, idx[0..cnt], key, lessThan);
        for (1..width) |c| {
            const j = idx[c - 1];
            data[base + c] = if (is_tail) n + j else j;
            const d = key[j] + big;
            cand_dist[base + c] = @intCast(d);
            alpha[base + c] = d;
        }
    }
    return .{
        .allocator = allocator,
        .width = width,
        .data = data,
        .alpha = alpha,
        .cand_dist = cand_dist,
        .dist_sorted = true,
    };
}

fn directedLen(asym: []const u32, n: usize, tour: []const usize, reversed: bool) u64 {
    var total: u64 = 0;
    for (0..n) |idx| {
        const a = tour[idx];
        const b = tour[(idx + 1) % n];
        const from = if (reversed) b else a;
        const to = if (reversed) a else b;
        total += asym[from * n + to];
    }
    return total;
}

// =============================================================================
// NATIVE asymmetric search (no 2n transform): operates directly on the directed
// n x n matrix. Premise check (docs/engineering-problem.html): does a native
// directed local search reach the transform's quality at lower cost? Moves are
// Or-opt (segment relocate, no reversal -> O(1) delta, direction-safe) and directed
// 2-opt (reverses a segment, re-pricing the internal arcs -> O(segment)). City 0 is
// pinned at position 0 (the cycle is rotation-invariant), so moves act on positions
// 1..n-1 with no array wraparound to special-case. ILS via double-bridge kicks.
// =============================================================================
const NativeWs = struct {
    mat: MatView,
    n: usize,
    tour: []usize, // position -> city, tour[0] == 0 always
    pos: []usize, // city -> position
    cand: []const usize,
    k: usize,
    scratch: []usize,

    inline fn d(self: *const NativeWs, a: usize, b: usize) i64 {
        return @intCast(self.mat.at(a, b));
    }
    inline fn succ(self: *const NativeWs, p: usize) usize {
        return if (p + 1 == self.n) 0 else p + 1;
    }
    fn syncPos(self: *NativeWs) void {
        for (self.tour, 0..) |c, p| self.pos[c] = p;
    }
};

fn nativeLen(mat: MatView, n: usize, tour: []const usize) u64 {
    var s: u64 = 0;
    for (0..n) |p| s += mat.at(tour[p], tour[(p + 1) % n]);
    return s;
}

fn buildNativeCands(allocator: std.mem.Allocator, mat: MatView, n: usize, k: usize) ![]usize {
    const cand = try allocator.alloc(usize, n * k);
    errdefer allocator.free(cand);
    const idx = try allocator.alloc(usize, n);
    defer allocator.free(idx);
    const key = try allocator.alloc(u64, n);
    defer allocator.free(key);
    for (0..n) |i| {
        var cnt: usize = 0;
        for (0..n) |j| {
            if (j == i) continue;
            idx[cnt] = j;
            key[j] = @min(mat.at(i, j), mat.at(j, i)); // directed proximity
            cnt += 1;
        }
        std.sort.pdq(usize, idx[0..cnt], key, struct {
            fn lt(kk: []const u64, a: usize, b: usize) bool {
                return kk[a] < kk[b];
            }
        }.lt);
        for (0..k) |c| cand[i * k + c] = idx[c];
    }
    return cand;
}

fn nativeNN(mat: MatView, n: usize, tour: []usize, visited: []bool) void {
    @memset(visited, false);
    tour[0] = 0;
    visited[0] = true;
    var p: usize = 1;
    while (p < n) : (p += 1) {
        const cur = tour[p - 1];
        var bd: u32 = std.math.maxInt(u32);
        var bj: usize = 0;
        for (0..n) |j| {
            if (!visited[j] and mat.at(cur, j) < bd) {
                bd = mat.at(cur, j);
                bj = j;
            }
        }
        tour[p] = bj;
        visited[bj] = true;
    }
}

// One improving Or-opt (segment length 1..3) rooted at position i (>=1). Applies and
// returns true on the first improving relocation among the segment ends' candidates.
fn nativeOrOpt(ws: *NativeWs, i: usize) bool {
    const n = ws.n;
    for ([_]usize{ 1, 2, 3 }) |L| {
        if (i + L > n) continue;
        const seg_first = ws.tour[i];
        const seg_last = ws.tour[i + L - 1];
        const p = ws.tour[i - 1];
        const q = ws.tour[if (i + L == n) 0 else i + L];
        const removal = ws.d(p, seg_first) + ws.d(seg_last, q) - ws.d(p, q);
        for ([_]usize{ seg_first, seg_last }) |anchor| {
            for (ws.cand[anchor * ws.k ..][0..ws.k]) |c| {
                const pj = ws.pos[c];
                if (pj + 1 >= i and pj <= i + L - 1) continue; // own gap / inside segment
                const cn = ws.tour[ws.succ(pj)];
                if (cn == seg_first) continue;
                const insert = ws.d(c, seg_first) + ws.d(seg_last, cn) - ws.d(c, cn);
                if (insert - removal < 0) {
                    nativeApplyOrOpt(ws, i, L, c);
                    return true;
                }
            }
        }
    }
    return false;
}

fn nativeApplyOrOpt(ws: *NativeWs, i: usize, L: usize, after_city: usize) void {
    const n = ws.n;
    const s = ws.scratch;
    var seg: [3]usize = undefined;
    for (0..L) |t| seg[t] = ws.tour[i + t];
    var w: usize = 0;
    var srcp: usize = 0;
    while (srcp < n) : (srcp += 1) {
        if (srcp >= i and srcp < i + L) continue; // drop the segment
        s[w] = ws.tour[srcp];
        w += 1;
        if (ws.tour[srcp] == after_city) {
            for (0..L) |t| {
                s[w] = seg[t];
                w += 1;
            }
        }
    }
    @memcpy(ws.tour, s[0..n]);
    ws.syncPos();
}

// One improving directed 2-opt rooted at position lo (new lead edge from tour[lo] to
// one of its candidates). Reverses the shorter interior; re-prices internal arcs.
fn nativeTwoOpt(ws: *NativeWs, lo: usize) bool {
    const a = ws.tour[lo];
    const b = ws.tour[lo + 1];
    for (ws.cand[a * ws.k ..][0..ws.k]) |c| {
        const hi = ws.pos[c];
        if (hi <= lo) continue;
        const thi = ws.tour[hi]; // == c
        const dlast = ws.tour[ws.succ(hi)];
        if (dlast == a) continue;
        var flip: i64 = 0;
        var kk: usize = lo + 1;
        while (kk < hi) : (kk += 1) {
            flip += ws.d(ws.tour[kk + 1], ws.tour[kk]) - ws.d(ws.tour[kk], ws.tour[kk + 1]);
        }
        const delta = ws.d(a, thi) + ws.d(b, dlast) - ws.d(a, b) - ws.d(thi, dlast) + flip;
        if (delta < 0) {
            var x = lo + 1;
            var y = hi;
            while (x < y) : ({
                x += 1;
                y -= 1;
            }) {
                const tmp = ws.tour[x];
                ws.tour[x] = ws.tour[y];
                ws.tour[y] = tmp;
            }
            ws.syncPos();
            return true;
        }
    }
    return false;
}

fn nativeDescent(ws: *NativeWs) void {
    const n = ws.n;
    var improved = true;
    while (improved) {
        improved = false;
        var i: usize = 1;
        while (i < n) : (i += 1) {
            while (nativeOrOpt(ws, i)) improved = true;
        }
        var lo: usize = 0;
        while (lo + 1 < n) : (lo += 1) {
            while (nativeTwoOpt(ws, lo)) improved = true;
        }
    }
}

fn nativeDoubleBridge(ws: *NativeWs, rng: std.Random) void {
    const n = ws.n;
    if (n < 8) return;
    const p1 = 1 + rng.uintLessThan(usize, n - 3);
    const p2 = p1 + 1 + rng.uintLessThan(usize, n - p1 - 2);
    const p3 = p2 + 1 + rng.uintLessThan(usize, n - p2 - 1);
    const s = ws.scratch;
    var w: usize = 0;
    for (0..p1) |t| {
        s[w] = ws.tour[t];
        w += 1;
    } // A
    for (p2..p3) |t| {
        s[w] = ws.tour[t];
        w += 1;
    } // C
    for (p1..p2) |t| {
        s[w] = ws.tour[t];
        w += 1;
    } // B
    for (p3..n) |t| {
        s[w] = ws.tour[t];
        w += 1;
    } // D
    @memcpy(ws.tour, s[0..n]);
    ws.syncPos();
}

/// Native directed ATSP solver — no 2n transform. NN seed -> Or-opt+2-opt descent ->
/// double-bridge ILS (accept-best). budget.trials = ILS kicks.
pub fn solveAtspNative(allocator: std.mem.Allocator, asym: []const u32, n: usize, options: solver.SolveOptions) !AtspResult {
    if (n < 2 or asym.len != n * n) return error.InvalidMatrix;
    return solveAtspNativeImpl(allocator, .{ .base = asym, .stride = n, .off = 0 }, n, options);
}

/// Native ATSP over the n-city block at (off,off) of a larger row-major matrix, read in
/// place — no n*n copy. The CVRP giant-tour seed uses this to point straight at
/// inst.matrix (stride=n+1, off=1) so the solve adds no matrix-sized allocation.
pub fn solveAtspNativeView(allocator: std.mem.Allocator, base: []const u32, n: usize, stride: usize, off: usize, options: solver.SolveOptions) !AtspResult {
    if (n < 2 or base.len < (n - 1 + off) * stride + (n - 1 + off) + 1) return error.InvalidMatrix;
    return solveAtspNativeImpl(allocator, .{ .base = base, .stride = stride, .off = off }, n, options);
}

fn solveAtspNativeImpl(allocator: std.mem.Allocator, mat: MatView, n: usize, options: solver.SolveOptions) !AtspResult {
    const k = @min(@as(usize, 16), n - 1);
    const cand = try buildNativeCands(allocator, mat, n, k);
    defer allocator.free(cand);
    const tour = try allocator.alloc(usize, n);
    errdefer allocator.free(tour);
    const pos = try allocator.alloc(usize, n);
    defer allocator.free(pos);
    const scratch = try allocator.alloc(usize, n);
    defer allocator.free(scratch);
    const visited = try allocator.alloc(bool, n);
    defer allocator.free(visited);
    const best = try allocator.alloc(usize, n);
    defer allocator.free(best);

    nativeNN(mat, n, tour, visited);
    var ws = NativeWs{ .mat = mat, .n = n, .tour = tour, .pos = pos, .cand = cand, .k = k, .scratch = scratch };
    ws.syncPos();
    nativeDescent(&ws);
    var best_len = nativeLen(mat, n, tour);
    @memcpy(best, tour);

    var prng = std.Random.DefaultPrng.init(options.seed);
    const rng = prng.random();
    const iters = @max(options.budget.trials, 1);
    var it: usize = 0;
    while (it < iters) : (it += 1) {
        nativeDoubleBridge(&ws, rng);
        nativeDescent(&ws);
        const len = nativeLen(mat, n, tour);
        if (len < best_len) {
            best_len = len;
            @memcpy(best, tour);
        } else {
            @memcpy(tour, best);
            ws.syncPos();
        }
    }
    @memcpy(tour, best);
    return .{ .allocator = allocator, .tour = tour, .length = best_len };
}

test "ATSP transform recovers a known directed optimum" {
    const allocator = std.testing.allocator;
    // 4 cities; the cheap directed cycle is 0->1->2->3->0 (cost 4). The reverse
    // 0->3->2->1->0 is expensive (cost 40), so symmetry would miss it.
    const n = 4;
    const big = 9; // forbidding-ish high cost for non-cycle arcs
    var a = [_]u32{
        0, 1,  big, big,
        big, 0,  1,  big,
        big, big, 0,  1,
        1, big, big, 0,
    };
    // reverse arcs expensive
    a[1 * n + 0] = 10;
    a[2 * n + 1] = 10;
    a[3 * n + 2] = 10;
    a[0 * n + 3] = 10;

    var res = try solveAtsp(allocator, &a, n, .{
        .seed = 1,
        .budget = .{ .trials = 20, .max_passes = 40 },
        .candidates = .{ .candidate_count = 3, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 4 },
    });
    defer res.deinit();
    try std.testing.expectEqual(@as(u64, 4), res.length);
}

test "ATSP native: recovers the directed optimum and stays a valid permutation" {
    const allocator = std.testing.allocator;
    const n = 4;
    const big = 9;
    var a = [_]u32{
        0,   1,   big, big,
        big, 0,   1,   big,
        big, big, 0,   1,
        1,   big, big, 0,
    };
    a[1 * n + 0] = 10;
    a[2 * n + 1] = 10;
    a[3 * n + 2] = 10;
    a[0 * n + 3] = 10;
    var res = try solveAtspNative(allocator, &a, n, .{ .seed = 1, .budget = .{ .trials = 50 } });
    defer res.deinit();
    try std.testing.expectEqual(@as(u64, 4), res.length);

    // medium random: valid permutation + reported length matches a fresh scan
    const m = 60;
    var prng = std.Random.DefaultPrng.init(0xC3);
    const rng = prng.random();
    const big_a = try allocator.alloc(u32, m * m);
    defer allocator.free(big_a);
    for (0..m) |i| {
        for (0..m) |j| big_a[i * m + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 1000);
    }
    var r2 = try solveAtspNative(allocator, big_a, m, .{ .seed = 7, .budget = .{ .trials = 200 } });
    defer r2.deinit();
    const seen = try allocator.alloc(bool, m);
    defer allocator.free(seen);
    @memset(seen, false);
    for (r2.tour) |c| {
        try std.testing.expect(c < m and !seen[c]);
        seen[c] = true;
    }
    try std.testing.expectEqual(directedLen(big_a, m, r2.tour, false), r2.length);
}

test "ATSP transform: medium random instance is valid and beats nearest-neighbour" {
    const allocator = std.testing.allocator;
    const n = 50;
    var prng = std.Random.DefaultPrng.init(0xA5);
    const rng = prng.random();
    const a = try allocator.alloc(u32, n * n);
    defer allocator.free(a);
    for (0..n) |i| {
        for (0..n) |j| {
            a[i * n + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 1000);
        }
    }
    var res = try solveAtsp(allocator, a, n, .{
        .seed = 7,
        .budget = .{ .trials = 60, .trial_extension_factor = 2, .max_passes = 60 },
        .candidates = .{ .candidate_count = 8, .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
        .search = .{ .enable_lk = true, .lk_max_depth = 5 },
    });
    defer res.deinit();

    // valid permutation
    const seen = try allocator.alloc(bool, n);
    defer allocator.free(seen);
    @memset(seen, false);
    for (res.tour) |c| {
        try std.testing.expect(c < n and !seen[c]);
        seen[c] = true;
    }
    // reported length matches a fresh directed scan
    try std.testing.expectEqual(directedLen(a, n, res.tour, false), res.length);
    // and it beats the directed nearest-neighbour tour from city 0
    try std.testing.expect(res.length < directedNearestNeighbour(a, n, allocator));
}

fn directedNearestNeighbour(asym: []const u32, n: usize, allocator: std.mem.Allocator) u64 {
    const visited = allocator.alloc(bool, n) catch return std.math.maxInt(u64);
    defer allocator.free(visited);
    @memset(visited, false);
    var cur: usize = 0;
    visited[0] = true;
    var total: u64 = 0;
    for (1..n) |_| {
        var best: usize = std.math.maxInt(usize);
        var bd: u32 = std.math.maxInt(u32);
        for (0..n) |j| {
            if (!visited[j] and asym[cur * n + j] < bd) {
                bd = asym[cur * n + j];
                best = j;
            }
        }
        visited[best] = true;
        total += bd;
        cur = best;
    }
    return total + asym[cur * n + 0];
}

// ---------------------------------------------------------------------------
// Conservativeness analyzer — "does this network even need directional routing?"
//
// Helmholtz-Hodge decomposition of a directed cost matrix's antisymmetric part.
// Write d(i,j) = S(i,j) + A(i,j) where S = (d(i,j)+d(j,i))/2 is the direction-free
// average cost and A(i,j) = d(i,j)-d(j,i) is antisymmetric (A(j,i) = -A(i,j)).
//
// A splits into a GRADIENT (conservative) part and a RESIDUAL (curl):
//   A(i,j) ~= phi(i) - phi(j)            <- gradient of a node potential phi
//   R(i,j)  = A(i,j) - (phi(i)-phi(j))   <- the non-conservative remainder
// The least-squares phi on the complete graph is phi(i) = mean_j A(i,j). The
// gradient part telescopes to ZERO around any closed route (sum of phi(i)-phi(j)
// over a cycle = 0), so a pure gradient asymmetry costs nothing to ignore — you
// can symmetrize, route, and pay the same. The curl part does NOT cancel over a
// cycle: that is the asymmetry that actually changes optimal routes (one-way
// streets, turn restrictions, divided roads).
//
//   curl_fraction = ||R|| / ||A||   (Frobenius, off-diagonal)
//     ~0  => asymmetry is a congestion gradient; direction is free to ignore.
//     ~1  => structural asymmetry; native directed handling is required.
//
// The headline of the project's asymmetry study: the asymmetry *magnitude* (ratio)
// is not the signal — the *structure* (curl_fraction) is. This function is that
// measurement, runnable on any cost matrix.
pub const Conservativeness = struct {
    dim: usize,
    /// ||A|| / ||S|| over off-diagonal entries: how directional the matrix is overall.
    asym_magnitude: f64,
    /// ||R|| / ||A||: the non-conservative (curl) share of the asymmetry. THE number.
    curl_fraction: f64,
    /// mean over reachable pairs of max(d_ij,d_ji)/min(d_ij,d_ji): human-readable ratio.
    mean_ratio: f64,
};

/// Decompose a row-major `dim*dim` directed cost matrix into its conservative
/// (gradient) and non-conservative (curl) asymmetry. See `Conservativeness`.
pub fn conservativeness(
    allocator: std.mem.Allocator,
    matrix: []const u32,
    dim: usize,
) !Conservativeness {
    std.debug.assert(matrix.len >= dim * dim);
    const phi = try allocator.alloc(f64, dim);
    defer allocator.free(phi);

    // phi(i) = mean_j A(i,j) = mean_j (d(i,j) - d(j,i)); the LS potential on K_dim.
    for (0..dim) |i| {
        var acc: i64 = 0;
        for (0..dim) |j| {
            acc += @as(i64, matrix[i * dim + j]) - @as(i64, matrix[j * dim + i]);
        }
        phi[i] = @as(f64, @floatFromInt(acc)) / @as(f64, @floatFromInt(dim));
    }

    var norm_a2: f64 = 0; // ||A||^2
    var norm_r2: f64 = 0; // ||R||^2
    var norm_s2: f64 = 0; // ||S||^2 (using full average cost, not halved)
    var ratio_sum: f64 = 0;
    var ratio_cnt: usize = 0;
    for (0..dim) |i| {
        for (0..dim) |j| {
            if (i == j) continue;
            const dij: i64 = matrix[i * dim + j];
            const dji: i64 = matrix[j * dim + i];
            const a: f64 = @floatFromInt(dij - dji);
            const r: f64 = a - (phi[i] - phi[j]);
            norm_a2 += a * a;
            norm_r2 += r * r;
            const s: f64 = @as(f64, @floatFromInt(dij + dji)) / 2.0;
            norm_s2 += s * s;
            if (dij > 0 and dji > 0) {
                const hi: f64 = @floatFromInt(@max(dij, dji));
                const lo: f64 = @floatFromInt(@min(dij, dji));
                ratio_sum += hi / lo;
                ratio_cnt += 1;
            }
        }
    }
    const norm_a = @sqrt(norm_a2);
    return .{
        .dim = dim,
        .asym_magnitude = if (norm_s2 > 0) norm_a / @sqrt(norm_s2) else 0,
        .curl_fraction = if (norm_a > 0) @sqrt(norm_r2) / norm_a else 0,
        .mean_ratio = if (ratio_cnt > 0) ratio_sum / @as(f64, @floatFromInt(ratio_cnt)) else 1,
    };
}

test "conservativeness: pure gradient asymmetry has ~zero curl" {
    // d(i,j) = C + psi(i) - psi(j): antisymmetric part is a pure gradient 2*(psi_i-psi_j).
    const dim = 6;
    const psi = [_]i64{ 0, 3, 7, 2, 9, 1 };
    var m: [dim * dim]u32 = undefined;
    for (0..dim) |i| for (0..dim) |j| {
        m[i * dim + j] = if (i == j) 0 else @intCast(100 + psi[i] - psi[j]);
    };
    const c = try conservativeness(std.testing.allocator, &m, dim);
    try std.testing.expect(c.curl_fraction < 1e-9); // gradient => no curl
    try std.testing.expect(c.asym_magnitude > 0); // but it IS asymmetric
}

test "conservativeness: pure circulation is all curl" {
    // A 3-cycle with a directional circulation: A(0,1)=A(1,2)=A(2,0)=+2. phi=0 => R=A.
    const dim = 3;
    const m = [_]u32{
        0, 3, 1,
        1, 0, 3,
        3, 1, 0,
    };
    const c = try conservativeness(std.testing.allocator, &m, dim);
    try std.testing.expect(c.curl_fraction > 0.999); // circulation => all curl
}
