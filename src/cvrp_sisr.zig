const std = @import("std");
const core_anneal = @import("core/anneal.zig");
const builtin = @import("builtin");
const asymmetric = @import("asymmetric.zig");
const solver = @import("solver.zig");
const cvrp_types = @import("cvrp_types.zig");
const cvrp_split = @import("cvrp_split.zig");
const cvrp_solution = @import("cvrp_solution.zig");
const CvrpInstance = cvrp_types.CvrpInstance;
const CvrpResult = cvrp_types.CvrpResult;
const Solution = cvrp_solution.Solution;
const SisrCtx = cvrp_solution.SisrCtx;
const educateGiant = cvrp_solution.educateGiant;
const buildCvrpNeighbors = cvrp_solution.buildCvrpNeighbors;
const buildCvrpNeighborsKeyed = cvrp_solution.buildCvrpNeighborsKeyed;
const solveCvrpImpl = cvrp_solution.solveCvrpImpl;
const validate = cvrp_solution.validate;
const POP_CROSSOVER_N = cvrp_split.POP_CROSSOVER_N;
const REGRET_MAX_N = cvrp_split.REGRET_MAX_N;
const UCB_C = cvrp_split.UCB_C;

// ---- SISR: single-solution ruin-and-recreate (Christiaens & Vanden Berghe 2020) -
// HGS plateaus on large CVRP (X-n1001 ~2.78%) because a population cannot afford
// enough generations at scale. SISR is the scale answer: one solution, millions of
// cheap O(removed) ruin+recreate moves under simulated-annealing acceptance. Ruin
// removes spatially-adjacent strings of customers; recreate greedily re-inserts
// them (cheapest position, granular candidates, small "blink" skip probability for
// diversity). Runs entirely on the linked route rep; capacity-feasible throughout
// (recreate only inserts where it fits, or opens a route), so the reported distance
// is the true objective.

pub const SisrParams = struct {
    iters: usize = 300_000, // ruin+recreate iterations
    l_max: usize = 10, // maximum string cardinality (L^max)
    cbar: f64 = 10.0, // average number of customers removed per ruin
    blink: f64 = 0.01, // recreate: probability of skipping a candidate position
    t0_factor: f64 = 1.0, // initial SA temperature = t0_factor * (dist0 / n)
    tf_factor: f64 = 0.01, // final SA temperature   = tf_factor * (dist0 / n)
    split_rate: f64 = -1.0, // prob a ruin uses split-string; <0 = auto (on for large n), 0 = off
    split_alpha: f64 = 0.5, // split mode: geometric growth of the preserved-block size
    bandit: bool = false, // UCB1 online choice of split vs plain (overrides split_rate)
    // Recreate strategy: probability a given recreate uses regret-2 (insert the
    // customer with the largest best/2nd-best insertion-cost gap) instead of
    // greedy+blink. <0 = auto = on only for 250 <= n <= 600 (see REGRET_MAX_N).
    // Regret reconstructs MID-size instances (X-n303/X-n502) better than greedy can
    // at any budget (greedy plateaus there), but its determinism + per-recreate cost
    // hurt small/tight instances (n<=200) AND large iteration-starved ones (X-n1001
    // loses on both quality and wall). 0 = always greedy. 1 = always regret.
    regret_rate: f64 = -1.0,
    // Marathon profile: for very long runs (iters >= 1,000,000) swap in a
    // colder-finish, larger-neighborhood constant set (cbar=13, l_max=13,
    // tf_factor=0.001, blink=0.02) tuned for the extra budget instead of the
    // short-run defaults above. Below the 1M-iter gate this is a no-op — the
    // default (false) path is bit-identical to before this flag existed.
    marathon: bool = false,
    // Granular neighbor-list proximity key (see cvrp_solution.NbrKey). The
    // historical .sum key symmetrizes and can hide one-way-close pairs on
    // directed matrices; .min matches the ATSP seed's metric. Default .sum =
    // bit-identical to before this knob existed.
    nbr_key: cvrp_solution.NbrKey = .sum,
    // Granular neighbor-list size. 0 = auto (the historical min(20, n-1)).
    gk: usize = 0,
    // Final education: after the ruin-recreate schedule ends, run the full
    // granular local search (educateGiant's engine: relocate/or-opt/swap/
    // swap*/2-opt*) on the BEST solution until no improving move remains.
    // Improvement-only, so the returned cost is never worse than without it;
    // costs one LS convergence (~ms at n=100, ~seconds at n=2000). Default
    // off = bit-identical output to before this flag existed.
    final_ls: bool = false,
    // Per-route ATSP re-solve on the returned best (see Solution.routeAtspRefine),
    // alternated with final-education drains until neither improves. Monotone:
    // every accepted step strictly reduces distance. Default off = bit-identical.
    route_atsp: bool = false,
    // Iterated kicks: after the monotone refiners, run this many deterministic
    // zero-temperature perturbation rounds on the refined best — one small
    // ruin+recreate, a full education drain, keep only strict improvements
    // (reverting to best otherwise). The one refiner that can move customers
    // BETWEEN routes, so it attacks the small-n inter-route lock-in the
    // intra-route refiners cannot. Cost ~= kicks * one LS convergence.
    // Default 0 = off = bit-identical.
    kicks: usize = 0,
    // Route-pair sub-CVRP re-solve: the refiner that moves LOAD between
    // routes. Up to subsolve_pairs disjoint route pairs (ranked by granular
    // cross-link count) are extracted as standalone sub-CVRPs (pair customers
    // + depot) and re-solved with a fresh seeded SISR of subsolve_iters
    // iterations; the result replaces the pair only on strict distance
    // improvement using at most 2 vehicles. Deterministic. 0 = off =
    // bit-identical.
    subsolve_iters: usize = 0,
    subsolve_pairs: usize = 8,
    // In-chain kicks (parallel driver only): each of the K chains runs this
    // many iterated kicks on its own best BEFORE winner selection. The kicks
    // run concurrently across chains, so K chains' worth of kicked
    // exploration costs one chain's kick wall, and selection then picks the
    // best refined chain (best-of-refined beats refined-best). Guaranteed
    // never worse: each chain's kicked result <= its raw result, so the min
    // over chains can only drop. The serial engine ignores this knob (use
    // `kicks`). Default 0 = off = bit-identical.
    chain_kicks: usize = 0,
    // Restart-to-best rounds: re-run the t0->tf geometric schedule this many
    // times, each round restarting the trajectory from the best-so-far. The
    // geometric schedule stretches with iters and can be non-monotone in
    // budget (nyc-1000: 5M scores worse than 4M); rounds spend extra budget
    // as more restarts of the proven schedule shape instead, monotone by
    // construction (best never regresses). Total work = rounds * iters.
    // Default 1 = bit-identical.
    rounds: usize = 1,
    // Restart temperature for rounds >= 2, as a fraction of the t0->tf LOG
    // range: round temp starts at tf * (t0/tf)^reheat. 1.0 = full re-melt
    // (measured useless at n=1000: the trajectory wanders off and never
    // re-finds the basin); ~0.2-0.5 = warm restart that keeps the basin.
    reheat: f64 = 1.0,
};

/// SISR solver for (symmetric or asymmetric) CVRP, uncapped fleet. Builds a feasible
/// start from the ATSP-seed giant tour, then runs `params.iters` ruin+recreate steps
/// under a geometric SA schedule, returning the best solution found.
pub fn solveCvrpSisr(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, params: SisrParams) !CvrpResult {
    const n = inst.n;
    if (inst.demand.len != n + 1 or inst.matrix.len != (std.math.mul(usize, n + 1, n + 1) catch return error.InvalidInstance)) return error.InvalidInstance;
    if (inst.demand[0] != 0) return error.InvalidInstance; // depot demand must be 0: a nonzero value is a caller data-mapping bug, not something to silently ignore
    if (n <= 2) return solveCvrpImpl(allocator, inst, options, 10, 1, 0);

    const gk: usize = @min(if (params.gk == 0) @as(usize, 20) else params.gk, n - 1);
    const gran = try buildCvrpNeighborsKeyed(allocator, inst, gk, params.nbr_key);
    defer allocator.free(gran);

    // initial solution: ATSP giant -> Split -> local optimum (strict-feasible).
    // Seed read straight off inst.matrix's customer block (stride n+1, off 1) — no n*n
    // sub copy, no 2n transform. SISR ruins/recreates the seed away, so a fast native
    // directed tour is all it needs; pointing at the existing matrix holds RAM at the
    // base footprint (was ~100MB/thread of sub at n=5000).
    var atsp = try asymmetric.solveAtspNativeView(allocator, inst.matrix, n, n + 1, 1, options);
    defer atsp.deinit();
    const seed_giant = try allocator.alloc(usize, n);
    defer allocator.free(seed_giant);
    for (atsp.tour, 0..) |c, idx| seed_giant[idx] = c + 1;

    var cur = try educateGiant(allocator, inst, seed_giant, 0, gran, gk, 0);
    defer cur.deinit();
    cur.buildLinks(); // guarantee links match order

    // `best` is the only full-state copy; refreshed only when cur improves (rare),
    // so the hot loop carries no O(n) per-iteration snapshot.
    var best = try cur.clone();
    defer best.deinit();
    best.copyLiveFrom(&cur);
    var best_dist = cur.distance;

    const present = try allocator.alloc(bool, n + 1);
    defer allocator.free(present);
    @memset(present, true);
    const removed = try allocator.alloc(usize, n);
    defer allocator.free(removed);
    const rprev = try allocator.alloc(usize, n);
    defer allocator.free(rprev);
    const rroute = try allocator.alloc(usize, n);
    defer allocator.free(rroute);
    const ins = try allocator.alloc(usize, n);
    defer allocator.free(ins);
    const touched = try allocator.alloc(usize, n);
    defer allocator.free(touched);
    const rmark = try allocator.alloc(bool, n);
    defer allocator.free(rmark);
    @memset(rmark, false);
    // Split-string ("slack induction") is a large-n lever: it helps where there is
    // room to redistribute (X-n303/502/1001 all improve; X-n1001 1.41% -> 0.94%) but
    // disrupts small, tight instances (X-n200 regressed). Auto (split_rate < 0) gates
    // it on for n >= 250 — the same regime split the HGS population uses.
    const eff_split: f64 = if (params.split_rate < 0) (if (n >= POP_CROSSOVER_N) @as(f64, 0.5) else 0.0) else params.split_rate;
    // Regret recreate is a mid/large-n lever (see SisrParams.regret_rate): auto-gate
    // it on for n >= 250 (the split-string boundary), off below.
    const eff_regret: f64 = if (params.regret_rate < 0) (if (n >= POP_CROSSOVER_N and n <= REGRET_MAX_N) @as(f64, 1.0) else 0.0) else params.regret_rate;
    // Marathon profile (see SisrParams.marathon): only takes effect at
    // iters >= 1,000,000, so this is a pure no-op below that gate.
    const marathon_on = params.marathon and params.iters >= 1_000_000;
    const eff_cbar: f64 = if (marathon_on) 13.0 else params.cbar;
    const eff_lmax: usize = if (marathon_on) 13 else params.l_max;
    const eff_blink: f64 = if (marathon_on) 0.02 else params.blink;
    const eff_tf_factor: f64 = if (marathon_on) 0.001 else params.tf_factor;
    var prng = std.Random.DefaultPrng.init(options.seed);
    var ctx = SisrCtx{ .present = present, .removed = removed, .rprev = rprev, .rroute = rroute, .ins = ins, .touched = touched, .rmark = rmark, .blink = eff_blink, .l_max = eff_lmax, .cbar = eff_cbar, .split_rate = eff_split, .split_alpha = params.split_alpha, .regret_rate = eff_regret, .prng = &prng };

    const rng = prng.random();

    const unit = @as(f64, @floatFromInt(cur.distance)) / @as(f64, @floatFromInt(n));
    const sched = core_anneal.geometric(unit, params.t0_factor, eff_tf_factor, params.iters);
    const t0 = sched.t0;
    const tf = sched.tf;
    const iters = sched.iters;
    var cf = sched.cf;
    var temp = t0;

    // UCB1 bandit over {plain, split} ruin: learns the best mix online instead of the
    // static n>=250 gate. Q = mean reward (move improved), N = pulls, exploration ~sqrt2.
    var bq = [2]f64{ 0, 0 };
    var bn = [2]f64{ 1, 1 };
    var bt: f64 = 2;

    // In-place ruin+recreate with O(removed) rollback on reject (no snapshot copy).
    // rounds > 1: restart-to-best (see SisrParams.rounds/reheat). The restart
    // temperature interpolates the log range; the geometric decay is re-derived
    // per round so each round still lands on tf at its end.
    const rounds = @max(@as(usize, 1), params.rounds);
    const round_t0 = @max(tf, tf * std.math.pow(f64, t0 / tf, std.math.clamp(params.reheat, 0.0, 1.0)));
    const round_cf = std.math.pow(f64, tf / round_t0, 1.0 / @as(f64, @floatFromInt(iters)));
    var round: usize = 0;
    while (round < rounds) : (round += 1) {
        if (round > 0) {
            cur.copyLiveFrom(&best);
            temp = round_t0;
            cf = round_cf;
        }
        var it: usize = 0;
        while (it < iters) : (it += 1) {
            const saved_dist = cur.distance;
            const saved_nroutes = cur.nroutes;
            var arm: usize = 0;
            if (params.bandit) {
                const ucb_plain = bq[0] + UCB_C * @sqrt(@log(bt) / bn[0]);
                const ucb_split = bq[1] + UCB_C * @sqrt(@log(bt) / bn[1]);
                arm = if (ucb_split > ucb_plain) 1 else 0;
                ctx.force_split = @intCast(arm);
            }
            cur.sisrRuin(&ctx, rng);
            const use_regret = ctx.regret_rate >= 1.0 or (ctx.regret_rate > 0 and rng.float(f64) < ctx.regret_rate);
            if (use_regret) cur.sisrRecreateRegret(&ctx) else cur.sisrRecreate(&ctx);
            const dt = @as(i64, @intCast(cur.distance)) - @as(i64, @intCast(saved_dist));
            if (params.bandit) {
                const reward: f64 = if (dt < 0) 1 else 0;
                bn[arm] += 1;
                bt += 1;
                bq[arm] += (reward - bq[arm]) / bn[arm];
            }
            // Threshold Accepting (Dueck & Scheuer): accept any move not worse than the
            // current threshold `temp`. Deterministic — no exp(), no acceptance RNG draw —
            // vs Metropolis exp(-dt/temp). Same geometric schedule drives the threshold.
            const accept = @as(f64, @floatFromInt(dt)) < temp;
            if (accept) {
                if (cur.distance < best_dist) {
                    best.copyLiveFrom(&cur);
                    best_dist = cur.distance;
                }
            } else {
                cur.sisrUndo(&ctx, saved_dist, saved_nroutes);
            }
            // Debug invariant: the live structure's true distance must match the value
            // maintained incrementally through ruin/recreate (and restored by undo). Run
            // every iteration in Debug so every reject+undo path is validated (tests are
            // tiny-n); release builds skip it entirely.
            if (builtin.mode == .Debug) {
                const inc = cur.distance;
                cur.flushLinks();
                std.debug.assert(cur.distance == inc);
            }
            temp *= cf;
        }
    }

    if (params.final_ls or params.route_atsp or params.kicks > 0 or params.subsolve_iters > 0) {
        best.flushLinks();
        try refineBest(allocator, &best, options, params, 1);
    }
    best.flushLinks(); // order/route_end/load/distance/cost from the linked rep
    var result = try best.toResult(allocator);
    errdefer result.deinit();
    if (validate(inst, result.routes) == null) return error.Infeasible;
    return result;
}

// --- Post-run refine pipeline -------------------------------------------------
// Monotone refiners applied to the best solution after the ruin-recreate
// schedule: final education (final_ls), per-route ATSP re-solve (route_atsp).
// Precondition: `best` has valid links AND arrays synced (flushLinks'd).
// Postcondition: the same (every stage ends on an LS flush or a no-op sweep).

/// One sweep over eligible routes: sub-solves run concurrently on up to
/// `threads` workers (read-only on `sol`), accepted reorders are applied
/// serially afterward. Routes are disjoint and each sub-solve is seeded
/// deterministically by (seed +% r), so the outcome is bit-identical to a
/// serial sweep regardless of thread count. Returns true if any route improved.
const RouteJob = struct {
    sol: *const Solution,
    r: usize,
    hash: u64,
    seed: u64,
    out: []usize,
    improved: bool = false,
    failed: bool = false,
};

fn routeJobWorker(jobs: []RouteJob) void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    for (jobs) |*j| {
        _ = arena.reset(.retain_capacity);
        j.improved = j.sol.routeAtspSolveOne(j.r, j.seed, arena.allocator(), j.out) catch blk: {
            j.failed = true;
            break :blk false;
        };
    }
}

fn routeAtspSweep(allocator: std.mem.Allocator, sol: *Solution, seed: u64, solved_hash: []u64, threads: usize) !bool {
    var jobs: std.ArrayList(RouteJob) = .empty;
    defer {
        for (jobs.items) |j| allocator.free(j.out);
        jobs.deinit(allocator);
    }
    for (0..sol.nroutes) |r| {
        const span = sol.routeSpan(r);
        const len = span.end - span.start;
        if (len < 5) continue; // short routes are already LS-optimal
        const hash = Solution.orderHash(sol.order[span.start..span.end]);
        if (solved_hash[r] == hash) continue; // unchanged since last no-improvement solve
        const out = try allocator.alloc(usize, len);
        errdefer allocator.free(out);
        try jobs.append(allocator, .{ .sol = sol, .r = r, .hash = hash, .seed = seed, .out = out });
    }
    if (jobs.items.len == 0) return false;

    const nw = @min(@max(threads, 1), jobs.items.len);
    if (nw <= 1) {
        routeJobWorker(jobs.items);
    } else {
        const ths = try allocator.alloc(std.Thread, nw);
        defer allocator.free(ths);
        var spawned: usize = 0;
        const per = (jobs.items.len + nw - 1) / nw;
        for (0..nw) |w| {
            const lo = w * per;
            if (lo >= jobs.items.len) break;
            const hi = @min(lo + per, jobs.items.len);
            ths[spawned] = std.Thread.spawn(.{}, routeJobWorker, .{jobs.items[lo..hi]}) catch {
                routeJobWorker(jobs.items[lo..hi]);
                continue;
            };
            spawned += 1;
        }
        for (0..spawned) |i| ths[i].join();
    }

    var any = false;
    for (jobs.items) |j| {
        if (j.failed) continue; // memo left open; retried next sweep
        if (!j.improved) {
            solved_hash[j.r] = j.hash;
            continue;
        }
        if (sol.applyRouteOrder(j.r, j.out)) {
            // Improved: leave the slot open so the next sweep re-solves the
            // new order (skipping is only sound for contents already solved
            // to a no-improvement verdict — the sub-solve is deterministic).
            solved_hash[j.r] = 0;
            any = true;
        } else {
            solved_hash[j.r] = j.hash;
        }
    }
    if (any) sol.recompute();
    return any;
}

fn refineBest(allocator: std.mem.Allocator, best: *Solution, options: solver.SolveOptions, params: SisrParams, threads: usize) anyerror!void {
    if (params.final_ls) {
        // Educate to a full local optimum. Each localSearch call is a fast
        // loose drain (see localSearchLinked); iterate until it stops
        // improving so the answer is a genuine local optimum of the whole
        // move vocabulary.
        var prev_dist = best.distance;
        while (true) {
            try best.localSearch();
            best.flushLinks();
            if (best.distance >= prev_dist) break;
            prev_dist = best.distance;
        }
    }
    if (params.route_atsp) {
        // A route reorder can open inter-route moves and vice versa, so
        // alternate refine and education until the refiner finds nothing.
        // The memo array makes re-sweeps skip routes untouched since their
        // last (non-improving) sub-solve.
        const solved_hash = try allocator.alloc(u64, best.nroutes);
        defer allocator.free(solved_hash);
        @memset(solved_hash, 0);
        while (true) {
            if (!(try routeAtspSweep(allocator, best, options.seed ^ 0xA75A, solved_hash, threads))) break;
            var prev_dist = best.distance;
            while (true) {
                try best.localSearch();
                best.flushLinks();
                if (best.distance >= prev_dist) break;
                prev_dist = best.distance;
            }
        }
    }
    if (params.subsolve_iters > 0) {
        // Route-pair sub-CVRP re-solve (see SisrParams.subsolve_iters). Pair
        // selection: rank route pairs by how many granular-neighbor links
        // cross between them — the cheap proxy for spatial adjacency on a
        // matrix-only instance.
        const n = best.inst.n;
        const nr = best.nroutes;
        if (nr >= 4 and nr * nr <= 4_000_000) {
            const counts = try allocator.alloc(u32, nr * nr);
            defer allocator.free(counts);
            @memset(counts, 0);
            for (1..n + 1) |c| {
                const rc = best.rof[c];
                for (0..best.gk) |i| {
                    const j = best.gran[(c - 1) * best.gk + i];
                    const rj = best.rof[j];
                    if (rj != rc) counts[rc * nr + rj] += 1;
                }
            }
            const used = try allocator.alloc(bool, nr);
            defer allocator.free(used);
            @memset(used, false);
            var any_spliced = false;
            var pairs_done: usize = 0;
            while (pairs_done < params.subsolve_pairs) : (pairs_done += 1) {
                // highest cross-link score among unused pairs (first hit wins ties)
                var best_score: u64 = 0;
                var pa: usize = 0;
                var pb: usize = 0;
                for (0..nr) |x| {
                    if (used[x]) continue;
                    for (x + 1..nr) |y| {
                        if (used[y]) continue;
                        const sc = @as(u64, counts[x * nr + y]) + @as(u64, counts[y * nr + x]);
                        if (sc > best_score) {
                            best_score = sc;
                            pa = x;
                            pb = y;
                        }
                    }
                }
                if (best_score == 0) break;
                used[pa] = true;
                used[pb] = true;
                const spa = best.routeSpan(pa);
                const spb = best.routeSpan(pb);
                const ka = spa.end - spa.start;
                const kb = spb.end - spb.start;
                const kk = ka + kb;
                if (kk < 10) continue; // tiny pairs are already LS-territory
                const globals = try allocator.alloc(usize, kk);
                defer allocator.free(globals);
                @memcpy(globals[0..ka], best.order[spa.start..spa.end]);
                @memcpy(globals[ka..], best.order[spb.start..spb.end]);
                const sdim = kk + 1;
                const smat = try allocator.alloc(u32, sdim * sdim);
                defer allocator.free(smat);
                const sdem = try allocator.alloc(u32, sdim);
                defer allocator.free(sdem);
                sdem[0] = 0;
                for (0..sdim) |i| {
                    const gi = if (i == 0) 0 else globals[i - 1];
                    if (i > 0) sdem[i] = best.inst.demand[gi];
                    for (0..sdim) |j| {
                        const gj = if (j == 0) 0 else globals[j - 1];
                        smat[i * sdim + j] = if (i == j) 0 else @as(u32, @intCast(best.inst.d(gi, gj)));
                    }
                }
                const sinst = CvrpInstance{ .n = kk, .matrix = smat, .demand = sdem, .capacity = best.inst.capacity };
                var sres = solveCvrpSisr(allocator, sinst, .{ .seed = (options.seed ^ 0x5B5B) +% pairs_done }, .{ .iters = params.subsolve_iters, .nbr_key = params.nbr_key }) catch continue;
                defer sres.deinit();
                var old_dist: u64 = 0;
                for ([2]usize{ pa, pb }) |r| {
                    const sp = best.routeSpan(r);
                    var prev: usize = 0;
                    for (best.order[sp.start..sp.end]) |c| {
                        old_dist += best.inst.d(prev, c);
                        prev = c;
                    }
                    old_dist += best.inst.d(prev, 0);
                }
                if (sres.routes.len > 2 or sres.total_cost >= old_dist) continue;
                // splice: rewrite the flat order with the pair replaced (route
                // indices and count stay stable; pb may become empty)
                const old_ends = try allocator.dupe(usize, best.route_end[0..nr]);
                defer allocator.free(old_ends);
                var w: usize = 0;
                for (0..nr) |r| {
                    if (r == pa or r == pb) {
                        const which: usize = if (r == pa) 0 else 1;
                        if (which < sres.routes.len) {
                            for (sres.routes[which]) |sc2| {
                                best.scratch[w] = globals[sc2 - 1];
                                w += 1;
                            }
                        }
                    } else {
                        const os = if (r == 0) 0 else old_ends[r - 1];
                        for (best.order[os..old_ends[r]]) |c| {
                            best.scratch[w] = c;
                            w += 1;
                        }
                    }
                    best.route_end[r] = w;
                }
                @memcpy(best.order[0..w], best.scratch[0..w]);
                best.recompute();
                any_spliced = true;
            }
            if (any_spliced) {
                // educate across the new boundaries, then leave links synced
                var prev_dist = best.distance;
                while (true) {
                    try best.localSearch();
                    best.flushLinks();
                    if (best.distance >= prev_dist) break;
                    prev_dist = best.distance;
                }
            }
        }
    }
    if (params.kicks > 0) {
        // Iterated kicks: zero-temperature perturb-educate-accept. A small
        // ruin+recreate jolts inter-route structure the monotone refiners
        // cannot move, education drains it back to a local optimum, and only
        // strict improvements are kept — otherwise the next kick restarts
        // from best. Deterministic: fixed count, seeded rng.
        const kn = best.inst.n;
        const present = try allocator.alloc(bool, kn + 1);
        defer allocator.free(present);
        @memset(present, true);
        const removed = try allocator.alloc(usize, kn);
        defer allocator.free(removed);
        const rprev = try allocator.alloc(usize, kn);
        defer allocator.free(rprev);
        const rroute = try allocator.alloc(usize, kn);
        defer allocator.free(rroute);
        const ins = try allocator.alloc(usize, kn);
        defer allocator.free(ins);
        const touched = try allocator.alloc(usize, kn);
        defer allocator.free(touched);
        const rmark = try allocator.alloc(bool, kn);
        defer allocator.free(rmark);
        @memset(rmark, false);
        var prng = std.Random.DefaultPrng.init(options.seed ^ 0x6B1C6B1C);
        var ctx = SisrCtx{ .present = present, .removed = removed, .rprev = rprev, .rroute = rroute, .ins = ins, .touched = touched, .rmark = rmark, .blink = params.blink, .l_max = params.l_max, .cbar = params.cbar, .split_rate = 0, .split_alpha = params.split_alpha, .regret_rate = 0, .prng = &prng };
        const rng = prng.random();
        var work = try best.clone();
        defer work.deinit();
        for (0..params.kicks) |_| {
            work.copyLiveFrom(best);
            work.sisrRuin(&ctx, rng);
            work.sisrRecreate(&ctx);
            work.flushLinks();
            var prev_dist = work.distance;
            while (true) {
                try work.localSearch();
                work.flushLinks();
                if (work.distance >= prev_dist) break;
                prev_dist = work.distance;
            }
            if (work.distance < best.distance) best.copyLiveFrom(&work);
        }
        // Accepted kicks land via copyLiveFrom (links only) — resync arrays so
        // callers can read order/route_end directly.
        best.flushLinks();
    }
}

// --- Best-of-K parallel SISR -------------------------------------------------
// K independent SISR chains (seed + i) run concurrently, each on its own arena
// over the thread-safe page allocator; the lowest-cost feasible result wins. This
// is the compute-bound engine's speed/accuracy lever: on K cores it costs ~one
// chain's wall but searches K independent trajectories (SISR is stochastic, so the
// best of K beats a single chain). Falls back to inline when a spawn fails.
const SisrSlot = struct {
    inst: CvrpInstance,
    options: solver.SolveOptions,
    params: SisrParams,
    seed: u64,
    order: []usize, // parent-owned flat customer order (size n)
    ends: []usize, // parent-owned route-end boundaries (size n)
    nroutes: usize = 0,
    cost: u64 = std.math.maxInt(u64),
    ok: bool = false,
};

fn sisrWorker(slot: *SisrSlot) void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    var opts = slot.options;
    opts.seed = slot.seed;
    const res = solveCvrpSisr(arena.allocator(), slot.inst, opts, slot.params) catch {
        slot.ok = false;
        return;
    };
    // flatten arena-owned routes into the parent-owned slot buffers before teardown
    var w: usize = 0;
    for (res.routes, 0..) |route, ri| {
        @memcpy(slot.order[w .. w + route.len], route);
        w += route.len;
        slot.ends[ri] = w;
    }
    slot.nroutes = res.routes.len;
    slot.cost = res.total_cost;
    slot.ok = true;
}

/// Run `threads` independent SISR chains and return the best. threads<=1 is the
/// plain serial path. Each chain uses options.seed + chain index.
///
/// Reproducibility caveat: `threads == 0` resolves to the host CPU count, which
/// sets the chain count and therefore each chain's seed (options.seed + index),
/// so the winning result depends on the machine's core count, and the same seed
/// yields different routes across machines. For output reproducible across
/// machines, pass an explicit non-zero `threads`.
pub fn solveCvrpSisrParallel(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, params: SisrParams, threads: usize) !CvrpResult {
    const cpus = std.Thread.getCpuCount() catch 1;
    const k = if (threads == 0) @max(@as(usize, 1), cpus -| 1) else threads;
    if (k <= 1 or inst.n <= 2) return solveCvrpSisr(allocator, inst, options, params);
    const n = inst.n;

    const slots = try allocator.alloc(SisrSlot, k);
    defer allocator.free(slots);
    var allocated: usize = 0;
    errdefer for (slots[0..allocated]) |s| {
        allocator.free(s.order);
        allocator.free(s.ends);
    };
    // Chains run raw; the refine pipeline runs once on the winning chain below
    // (one refine instead of k, and the route sweep uses the idle threads).
    var chain_params = params;
    chain_params.final_ls = false;
    chain_params.route_atsp = false;
    chain_params.kicks = params.chain_kicks;
    chain_params.subsolve_iters = 0;
    chain_params.chain_kicks = 0;
    for (slots, 0..) |*s, i| {
        s.* = .{
            .inst = inst,
            .options = options,
            .params = chain_params,
            .seed = options.seed +% i,
            .order = try allocator.alloc(usize, n),
            .ends = try allocator.alloc(usize, n),
        };
        allocated += 1;
    }

    const ths = try allocator.alloc(std.Thread, k);
    defer allocator.free(ths);
    var spawned: usize = 0;
    for (0..k) |i| {
        ths[i] = std.Thread.spawn(.{}, sisrWorker, .{&slots[i]}) catch break;
        spawned += 1;
    }
    for (spawned..k) |i| sisrWorker(&slots[i]); // inline fallback if a spawn failed
    for (0..spawned) |i| ths[i].join();

    var best: ?usize = null;
    for (slots, 0..) |s, i| {
        if (!s.ok) continue;
        if (best == null or s.cost < slots[best.?].cost) best = i;
    }
    const winner = best orelse return error.AllChainsFailed;

    if (params.final_ls or params.route_atsp or params.kicks > 0 or params.subsolve_iters > 0) {
        const ws = slots[winner];
        var rsol = try Solution.fromFlat(allocator, inst, ws.order[0..n], ws.ends[0..ws.nroutes]);
        defer rsol.deinit();
        const gkv: usize = @min(if (params.gk == 0) @as(usize, 20) else params.gk, n - 1);
        const rgran = try buildCvrpNeighborsKeyed(allocator, inst, gkv, params.nbr_key);
        defer allocator.free(rgran);
        rsol.gran = rgran;
        rsol.gk = gkv;
        rsol.buildLinks();
        try refineBest(allocator, &rsol, options, params, k);
        var result = try rsol.toResult(allocator);
        errdefer result.deinit();
        if (validate(inst, result.routes) == null) return error.Infeasible;
        for (slots) |s| {
            allocator.free(s.order);
            allocator.free(s.ends);
        }
        return result;
    }

    // build the result from the winning slot's flat order/ends in the parent allocator
    const bs = slots[winner];
    const routes = try allocator.alloc([]usize, bs.nroutes);
    var filled: usize = 0;
    errdefer {
        for (routes[0..filled]) |rt| allocator.free(rt);
        allocator.free(routes);
    }
    var start: usize = 0;
    for (0..bs.nroutes) |ri| {
        const end = bs.ends[ri];
        routes[ri] = try allocator.dupe(usize, bs.order[start..end]);
        filled += 1;
        start = end;
    }
    if (validate(inst, routes) == null) return error.Infeasible;
    for (slots) |s| {
        allocator.free(s.order);
        allocator.free(s.ends);
    }
    return .{ .allocator = allocator, .routes = routes, .total_cost = bs.cost };
}

test "CVRP SISR marathon: below the 1M-iter gate is a no-op (bit-identical)" {
    const allocator = std.testing.allocator;
    const n = 9;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0x515A);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 60);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 5);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 12 };
    const opts = solver.SolveOptions{ .seed = 7 };

    var r_off = try solveCvrpSisr(allocator, inst, opts, .{ .iters = 50_000, .marathon = false });
    defer r_off.deinit();
    var r_on = try solveCvrpSisr(allocator, inst, opts, .{ .iters = 50_000, .marathon = true });
    defer r_on.deinit();
    try std.testing.expectEqual(r_off.total_cost, r_on.total_cost);
    try std.testing.expectEqual(r_off.routes.len, r_on.routes.len);
    for (r_off.routes, r_on.routes) |a, b| try std.testing.expectEqualSlices(usize, a, b);
}

test "CVRP SISR marathon: at 1M+ iters, feasible and changes the trajectory" {
    if (builtin.mode == .Debug) return error.SkipZigTest;
    const allocator = std.testing.allocator;
    const n = 60;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0xC0DE15);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 100);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 5);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 12 };
    const opts = solver.SolveOptions{ .seed = 3 };

    var r_off = try solveCvrpSisr(allocator, inst, opts, .{ .iters = 1_000_000, .marathon = false });
    defer r_off.deinit();
    var r_on = try solveCvrpSisr(allocator, inst, opts, .{ .iters = 1_000_000, .marathon = true });
    defer r_on.deinit();
    try std.testing.expect(validate(inst, r_off.routes) != null);
    try std.testing.expect(validate(inst, r_on.routes) != null);
    // Different constants (cbar/l_max/tf_factor/blink) must move the trajectory;
    // a truly inert flag would collapse this to an equality, which is the bug
    // this test exists to catch.
    try std.testing.expect(r_off.total_cost != r_on.total_cost);
}

test "CVRP SISR nbr_key=min and gk: feasible, self-consistent, default bit-identical" {
    const allocator = std.testing.allocator;
    const n = 60;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0xA51CE);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 100);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 5);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 12 };
    // default nbr_key/gk must be bit-identical to the pre-knob engine
    var a = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000 });
    defer a.deinit();
    var b = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .nbr_key = .sum, .gk = 20 });
    defer b.deinit();
    try std.testing.expectEqual(a.total_cost, b.total_cost);
    // min key + larger list: still feasible and self-consistent
    var c = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .nbr_key = .min, .gk = 40 });
    defer c.deinit();
    const checked = validate(inst, c.routes) orelse return error.TestInfeasibleResult;
    try std.testing.expectEqual(c.total_cost, checked);
}

test "CVRP SISR final_ls: never worse than without it, feasible, default bit-identical" {
    const allocator = std.testing.allocator;
    const n = 60;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0xF17A1);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 100);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 5);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 12 };
    var off = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000 });
    defer off.deinit();
    var off2 = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .final_ls = false });
    defer off2.deinit();
    try std.testing.expectEqual(off.total_cost, off2.total_cost);
    var on = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .final_ls = true });
    defer on.deinit();
    const checked = validate(inst, on.routes) orelse return error.TestInfeasibleResult;
    try std.testing.expectEqual(on.total_cost, checked);
    try std.testing.expect(on.total_cost <= off.total_cost);
}

test "CVRP SISR route_atsp: never worse than without it, feasible, default bit-identical" {
    const allocator = std.testing.allocator;
    const n = 60;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0xA75A1);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 100);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 5);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 30 };
    var off = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000 });
    defer off.deinit();
    var off2 = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .route_atsp = false });
    defer off2.deinit();
    try std.testing.expectEqual(off.total_cost, off2.total_cost);
    var on = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .route_atsp = true });
    defer on.deinit();
    const checked = validate(inst, on.routes) orelse return error.TestInfeasibleResult;
    try std.testing.expectEqual(on.total_cost, checked);
    try std.testing.expect(on.total_cost <= off.total_cost);
}

test "CVRP SISR kicks: never worse than without them, feasible, default bit-identical" {
    const allocator = std.testing.allocator;
    const n = 60;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0x6B1C1);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 100);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 5);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 12 };
    var off = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000 });
    defer off.deinit();
    var off2 = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .kicks = 0 });
    defer off2.deinit();
    try std.testing.expectEqual(off.total_cost, off2.total_cost);
    var on = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .final_ls = true, .kicks = 30 });
    defer on.deinit();
    const checked = validate(inst, on.routes) orelse return error.TestInfeasibleResult;
    try std.testing.expectEqual(on.total_cost, checked);
    try std.testing.expect(on.total_cost <= off.total_cost);
}

test "CVRP SISR subsolve: never worse than without it, feasible, default bit-identical" {
    const allocator = std.testing.allocator;
    const n = 60;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0x5B5B1);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 100);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 5);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 12 };
    var off = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000 });
    defer off.deinit();
    var off2 = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .subsolve_iters = 0 });
    defer off2.deinit();
    try std.testing.expectEqual(off.total_cost, off2.total_cost);
    var on = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .subsolve_iters = 30000, .subsolve_pairs = 4 });
    defer on.deinit();
    const checked = validate(inst, on.routes) orelse return error.TestInfeasibleResult;
    try std.testing.expectEqual(on.total_cost, checked);
    try std.testing.expect(on.total_cost <= off.total_cost);
}

test "CVRP SISR chain_kicks: parallel never worse, feasible, default bit-identical" {
    const allocator = std.testing.allocator;
    const n = 60;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0xC41C1);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 100);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 5);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 12 };
    var off = try solveCvrpSisrParallel(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000 }, 2);
    defer off.deinit();
    var off2 = try solveCvrpSisrParallel(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .chain_kicks = 0 }, 2);
    defer off2.deinit();
    try std.testing.expectEqual(off.total_cost, off2.total_cost);
    var on = try solveCvrpSisrParallel(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .chain_kicks = 20 }, 2);
    defer on.deinit();
    const checked = validate(inst, on.routes) orelse return error.TestInfeasibleResult;
    try std.testing.expectEqual(on.total_cost, checked);
    try std.testing.expect(on.total_cost <= off.total_cost);
}

test "CVRP SISR rounds: never worse than one round, feasible, default bit-identical" {
    const allocator = std.testing.allocator;
    const n = 60;
    const dim = n + 1;
    var prng = std.Random.DefaultPrng.init(0x20B1D5);
    const rng = prng.random();
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| matrix[i * dim + j] = if (i == j) 0 else rng.intRangeAtMost(u32, 1, 100);
    }
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    demand[0] = 0;
    for (1..dim) |i| demand[i] = rng.intRangeAtMost(u32, 1, 5);
    const inst = CvrpInstance{ .n = n, .matrix = matrix, .demand = demand, .capacity = 12 };
    var off = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000 });
    defer off.deinit();
    var one = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .rounds = 1 });
    defer one.deinit();
    try std.testing.expectEqual(off.total_cost, one.total_cost);
    var on = try solveCvrpSisr(allocator, inst, .{ .seed = 3 }, .{ .iters = 20000, .rounds = 3 });
    defer on.deinit();
    const checked = validate(inst, on.routes) orelse return error.TestInfeasibleResult;
    try std.testing.expectEqual(on.total_cost, checked);
    try std.testing.expect(on.total_cost <= off.total_cost);
}
