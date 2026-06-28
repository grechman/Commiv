const std = @import("std");
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
};

/// SISR solver for (symmetric or asymmetric) CVRP, uncapped fleet. Builds a feasible
/// start from the ATSP-seed giant tour, then runs `params.iters` ruin+recreate steps
/// under a geometric SA schedule, returning the best solution found.
pub fn solveCvrpSisr(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, params: SisrParams) !CvrpResult {
    const n = inst.n;
    if (inst.demand.len != n + 1 or inst.matrix.len != (std.math.mul(usize, n + 1, n + 1) catch return error.InvalidInstance)) return error.InvalidInstance;
    if (n <= 2) return solveCvrpImpl(allocator, inst, options, 10, 1, 0);

    const gk: usize = @min(@as(usize, 20), n - 1);
    const gran = try buildCvrpNeighbors(allocator, inst, gk);
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
    var ctx = SisrCtx{ .present = present, .removed = removed, .rprev = rprev, .rroute = rroute, .ins = ins, .touched = touched, .rmark = rmark, .blink = params.blink, .l_max = params.l_max, .cbar = params.cbar, .split_rate = eff_split, .split_alpha = params.split_alpha, .regret_rate = eff_regret };

    var prng = std.Random.DefaultPrng.init(options.seed);
    const rng = prng.random();

    const unit = @as(f64, @floatFromInt(cur.distance)) / @as(f64, @floatFromInt(n));
    const t0 = @max(1e-9, params.t0_factor * unit);
    const tf = @max(1e-9, params.tf_factor * unit);
    const iters = @max(@as(usize, 1), params.iters);
    const cf = std.math.pow(f64, tf / t0, 1.0 / @as(f64, @floatFromInt(iters)));
    var temp = t0;

    // UCB1 bandit over {plain, split} ruin: learns the best mix online instead of the
    // static n>=250 gate. Q = mean reward (move improved), N = pulls, exploration ~sqrt2.
    var bq = [2]f64{ 0, 0 };
    var bn = [2]f64{ 1, 1 };
    var bt: f64 = 2;

    // In-place ruin+recreate with O(removed) rollback on reject (no snapshot copy).
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
        if (use_regret) cur.sisrRecreateRegret(&ctx) else cur.sisrRecreate(&ctx, rng);
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

    best.flushLinks(); // order/route_end/load/distance/cost from the linked rep
    var result = try best.toResult(allocator);
    errdefer result.deinit();
    if (validate(inst, result.routes) == null) return error.Infeasible;
    return result;
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
    for (slots, 0..) |*s, i| {
        s.* = .{
            .inst = inst,
            .options = options,
            .params = params,
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
