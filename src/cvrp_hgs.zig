const std = @import("std");
const asymmetric = @import("asymmetric.zig");
const solver = @import("solver.zig");
const hgs_core = @import("hgs_core.zig");
const cvrp_types = @import("cvrp_types.zig");
const cvrp_split = @import("cvrp_split.zig");
const cvrp_solution = @import("cvrp_solution.zig");
const CvrpInstance = cvrp_types.CvrpInstance;
const CvrpResult = cvrp_types.CvrpResult;
const Solution = cvrp_solution.Solution;
const educateGiant = cvrp_solution.educateGiant;
const buildCvrpNeighbors = cvrp_solution.buildCvrpNeighbors;
const solveCvrpImpl = cvrp_solution.solveCvrpImpl;
const validate = cvrp_solution.validate;
const POP_CROSSOVER_N = cvrp_split.POP_CROSSOVER_N;


// =========================== HGS population layer ============================
// Hybrid Genetic Search (Vidal) for CVRP/ACVRP: a population of giant tours
// evolved by order crossover (OX) + education (Split + local search), with
// biased-fitness parent selection and diversity-aware survivor selection. This
// is the GLOBAL search the single-incumbent ILS lacks — crossover recombines
// whole route assignments across individuals, reaching solutions a local search
// cannot (e.g. repacking a near-full fleet that single-route elimination can't).

pub const HgsParams = struct {
    mu: usize = 0, // population floor (survivors per generation); 0 = auto by n
    lambda: usize = 0, // offspring generated per generation; 0 = auto by n
    generations: usize = 100,
    n_elite: usize = 10, // elites whose cost rank dominates biased fitness
    n_close: usize = 5, // neighbours averaged for the diversity contribution
    restart_gens: usize = 20, // gentle diversification after this many stagnant gens (0 = never)
    infeasible_search: bool = true, // explore capacity-infeasible space under an adaptive penalty
};

const Individual = struct {
    giant: []usize, // educated giant tour (the chromosome)
    edges: []u64, // sorted giant-cycle edges, for the broken-pairs diversity metric
    cost: u64, // penalized cost (distance + fleet penalty), the fitness
    fn deinit(self: *Individual, allocator: std.mem.Allocator) void {
        allocator.free(self.giant);
        allocator.free(self.edges);
    }
};

/// Biased fitness = costRank + (1 - n_elite/N) * diversityRank (Vidal). Lower is
/// better; clones (identical edge set to an at-least-as-good member) get a big
/// penalty so they are culled first.
fn cvrpComputeBiased(allocator: std.mem.Allocator, pop: []const Individual, out: []f64, params: HgsParams, n: usize) !void {
    const N = pop.len;
    const cost_rank = try allocator.alloc(usize, N);
    defer allocator.free(cost_rank);
    const div_rank = try allocator.alloc(usize, N);
    defer allocator.free(div_rank);
    const idxbuf = try allocator.alloc(usize, N);
    defer allocator.free(idxbuf);
    const div = try allocator.alloc(f64, N);
    defer allocator.free(div);
    const dists = try allocator.alloc(usize, N);
    defer allocator.free(dists);
    const clone = try allocator.alloc(bool, N);
    defer allocator.free(clone);

    for (0..N) |k| idxbuf[k] = k;
    std.sort.pdq(usize, idxbuf, pop, struct {
        fn lt(p: []const Individual, a: usize, b: usize) bool {
            return p[a].cost < p[b].cost or (p[a].cost == p[b].cost and a < b);
        }
    }.lt);
    for (0..N) |r| cost_rank[idxbuf[r]] = r;

    const nc = @min(params.n_close, if (N > 0) N - 1 else 0);
    for (0..N) |i| {
        var m: usize = 0;
        var is_clone = false;
        for (0..N) |jj| {
            if (jj == i) continue;
            const dist = n - hgs_core.edgeOverlap(pop[i].edges, pop[jj].edges);
            dists[m] = dist;
            m += 1;
            if (dist == 0 and (pop[jj].cost < pop[i].cost or (pop[jj].cost == pop[i].cost and jj < i))) is_clone = true;
        }
        clone[i] = is_clone;
        std.sort.pdq(usize, dists[0..m], {}, std.sort.asc(usize));
        var sum: f64 = 0;
        const take = @min(nc, m);
        for (0..take) |t| sum += @floatFromInt(dists[t]);
        div[i] = if (take > 0) sum / @as(f64, @floatFromInt(take)) else 0;
    }
    for (0..N) |k| idxbuf[k] = k;
    std.sort.pdq(usize, idxbuf, div, struct {
        fn lt(d: []const f64, a: usize, b: usize) bool {
            return d[a] > d[b] or (d[a] == d[b] and a < b);
        }
    }.lt);
    for (0..N) |r| div_rank[idxbuf[r]] = r;

    const elite_frac = 1.0 - @as(f64, @floatFromInt(params.n_elite)) / @as(f64, @floatFromInt(N));
    const big: f64 = @floatFromInt(2 * N);
    for (0..N) |i| {
        out[i] = @as(f64, @floatFromInt(cost_rank[i])) + elite_frac * @as(f64, @floatFromInt(div_rank[i]));
        if (clone[i]) out[i] += big;
    }
}

// Binary tournament over the first `count` individuals (the parents whose biased
// fitness was computed this generation — NOT the offspring being appended, whose
// biased[] slots are still uninitialized).
fn cvrpTournament(biased: []const f64, count: usize, rng: std.Random) usize {
    const a = rng.uintLessThan(usize, count);
    const b = rng.uintLessThan(usize, count);
    return if (biased[a] <= biased[b]) a else b;
}

/// Educate one giant tour, append it to the population, and update the incumbent.
fn cvrpInsert(allocator: std.mem.Allocator, inst: CvrpInstance, max_vehicles: usize, giant: []const usize, ebuf: []usize, n: usize, pop: *std.ArrayList(Individual), best: *CvrpResult, best_cost: *u64, have_best: *bool, gran: []const usize, gk: usize, pen_coeff: u64) !bool {
    var sol = try educateGiant(allocator, inst, giant, max_vehicles, gran, gk, pen_coeff);
    defer sol.deinit();
    @memcpy(ebuf, sol.order); // .order is the giant tour grouped by routes
    {
        const giant_dup = try allocator.dupe(usize, ebuf);
        errdefer allocator.free(giant_dup);
        const edges = try hgs_core.buildEdges(allocator, ebuf, n);
        errdefer allocator.free(edges);
        // ensureUnusedCapacity + appendAssumeCapacity makes the append itself
        // infallible; the block bounds the errdefers so they discharge once pop
        // owns the two allocations (a later `try` must not free pop's memory).
        try pop.ensureUnusedCapacity(allocator, 1);
        pop.appendAssumeCapacity(.{
            .giant = giant_dup,
            .edges = edges,
            .cost = sol.cost,
        });
    }
    const feasible = sol.cap_excess == 0;
    // Only a capacity-feasible solution may become the reported incumbent. When
    // feasible, cost = distance + fleet-count penalty (no capacity term), so the
    // comparison still prefers fewer vehicles under a fleet cap.
    if (feasible and (!have_best.* or sol.cost < best_cost.*)) {
        if (have_best.*) best.deinit();
        best.* = try sol.toResult(allocator);
        best_cost.* = sol.cost;
        have_best.* = true;
    }
    return feasible;
}

/// HGS solve for CVRP/ACVRP. `max_vehicles` (0 = unlimited) is the fleet cap; the
/// population's crossover reaches tight-fleet repackings the local search can't.
pub fn solveCvrpHgs(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, params: HgsParams, max_vehicles: usize) !CvrpResult {
    const n = inst.n;
    if (inst.demand.len != n + 1 or inst.matrix.len != (std.math.mul(usize, n + 1, n + 1) catch return error.InvalidInstance)) return error.InvalidInstance;
    if (n <= 2) return solveCvrpImpl(allocator, inst, options, 10, 1, max_vehicles);

    // Population size is n-adaptive (0 = auto): large instances cannot afford enough
    // generations to evolve a big population within budget, so they get a lean pop
    // (more generations per unit work — measured 3.45%->2.92% on X-n1001). Small,
    // multi-modal instances need the diversity of a full pop (a lean pop collapsed
    // X-n153 to 2.7%). Crossover ~n=250. Explicit non-zero params override.
    const mu = if (params.mu != 0) params.mu else (if (n <= POP_CROSSOVER_N) @as(usize, 25) else 10);
    const lambda = if (params.lambda != 0) params.lambda else (if (n <= POP_CROSSOVER_N) @as(usize, 40) else 20);

    // seed giant tour from the ATSP core
    const sub = try allocator.alloc(u32, n * n);
    defer allocator.free(sub);
    for (0..n) |a| {
        for (0..n) |b| sub[a * n + b] = inst.matrix[(a + 1) * (n + 1) + (b + 1)];
    }
    var atsp = try asymmetric.solveAtsp(allocator, sub, n, options);
    defer atsp.deinit();
    const seed_giant = try allocator.alloc(usize, n);
    defer allocator.free(seed_giant);
    for (atsp.tour, 0..) |c, idx| seed_giant[idx] = c + 1;

    const gk: usize = @min(@as(usize, 20), n - 1);
    const gran = try buildCvrpNeighbors(allocator, inst, gk);
    defer allocator.free(gran);

    // Adaptive capacity penalty for infeasible search (HGS-CVRP's key feature): the
    // local search may overload routes at a per-unit price, crossing infeasible
    // space the feasibility-gated search cannot. Initialise ~ mean arc cost over the
    // granular neighbourhoods (a representative move scale), then steer it each
    // generation toward a target feasible fraction.
    // Gated to the uncapped fleet (max_vehicles == 0): under a hard vehicle cap the
    // capacity-overload search collides with the route-count repair, and that joint
    // regime needs its own tuning — keep it on the proven strict-feasible path.
    // Infeasible search only pays off when the instance CONVERGES enough to exploit
    // the extra search space. It broke the ceiling for n<=~500 but is pure 2x-cost
    // overhead at n=1000 (which never converges in budget): X-n1001 lean+penalty
    // 4.08% vs lean+no-penalty 2.78%. So gate it off past ~600 customers.
    const use_infeasible = max_vehicles == 0 and params.infeasible_search and n <= 600;
    var pen_coeff: u64 = if (!use_infeasible) 0 else blk: {
        var sum: u64 = 0;
        var cnt: u64 = 0;
        for (1..n + 1) |a| {
            for (gran[(a - 1) * gk ..][0..gk]) |b| {
                if (b == 0) continue;
                sum += inst.d(a, b);
                cnt += 1;
            }
        }
        break :blk @max(@as(u64, 1), if (cnt > 0) sum / cnt else 1);
    };

    var prng = std.Random.DefaultPrng.init(options.seed);
    const rng = prng.random();
    const ebuf = try allocator.alloc(usize, n);
    defer allocator.free(ebuf);

    var pop: std.ArrayList(Individual) = .empty;
    defer {
        for (pop.items) |*ind| ind.deinit(allocator);
        pop.deinit(allocator);
    }

    var best: CvrpResult = undefined;
    var have_best = false;
    var best_cost: u64 = std.math.maxInt(u64);
    errdefer if (have_best) best.deinit();

    // seed individual 0 from the ATSP giant; the rest from random permutations
    _ = try cvrpInsert(allocator, inst, max_vehicles, seed_giant, ebuf, n, &pop, &best, &best_cost, &have_best, gran, gk, pen_coeff);
    var s: usize = 1;
    while (s < mu) : (s += 1) {
        for (0..n) |k| ebuf[k] = k + 1;
        var t = n;
        while (t > 1) {
            t -= 1;
            const r = rng.uintLessThan(usize, t + 1);
            std.mem.swap(usize, &ebuf[t], &ebuf[r]);
        }
        _ = try cvrpInsert(allocator, inst, max_vehicles, ebuf, ebuf, n, &pop, &best, &best_cost, &have_best, gran, gk, pen_coeff);
    }

    const cap = mu + lambda;
    const biased = try allocator.alloc(f64, cap);
    defer allocator.free(biased);
    const order = try allocator.alloc(usize, cap);
    defer allocator.free(order);
    const keep = try allocator.alloc(bool, cap);
    defer allocator.free(keep);

    var gen: usize = 0;
    var stagnation: usize = 0;
    while (gen < params.generations) : (gen += 1) {
        const before = best_cost;
        try cvrpComputeBiased(allocator, pop.items, biased[0..pop.items.len], params, n);
        const nparents = pop.items.len; // freeze: offspring are not eligible as parents
        var off: usize = 0;
        var feas: usize = 0;
        while (off < lambda) : (off += 1) {
            const p1 = cvrpTournament(biased[0..nparents], nparents, rng);
            const p2 = cvrpTournament(biased[0..nparents], nparents, rng);
            const cg = try hgs_core.oxCrossover(allocator, pop.items[p1].giant, pop.items[p2].giant, n, rng);
            defer allocator.free(cg);
            if (try cvrpInsert(allocator, inst, max_vehicles, cg, ebuf, n, &pop, &best, &best_cost, &have_best, gran, gk, pen_coeff)) feas += 1;
        }
        // Adapt the capacity penalty toward a ~25% feasible-offspring target: too few
        // feasible => raise the price of overload; too many => lower it to explore
        // deeper into infeasible space. Bounded to avoid runaway.
        if (use_infeasible) {
            const frac = @as(f64, @floatFromInt(feas)) / @as(f64, @floatFromInt(lambda));
            if (frac < 0.2) {
                pen_coeff = @min(pen_coeff + pen_coeff / 5 + 1, 1 << 33);
            } else if (frac > 0.3) {
                pen_coeff = @max(@as(u64, 1), pen_coeff - pen_coeff / 8);
            }
        }
        // survivor selection: keep mu best by biased fitness; always keep the
        // lowest-cost individual (elitism keeps the incumbent's genes in the pool).
        try cvrpComputeBiased(allocator, pop.items, biased[0..pop.items.len], params, n);
        var best_idx: usize = 0;
        for (1..pop.items.len) |k| {
            if (pop.items[k].cost < pop.items[best_idx].cost) best_idx = k;
        }
        for (0..pop.items.len) |k| order[k] = k;
        std.sort.pdq(usize, order[0..pop.items.len], biased, struct {
            fn lt(bf: []const f64, a: usize, b: usize) bool {
                return bf[a] < bf[b] or (bf[a] == bf[b] and a < b);
            }
        }.lt);
        @memset(keep[0..pop.items.len], false);
        var kept: usize = 0;
        keep[best_idx] = true;
        kept += 1;
        for (order[0..pop.items.len]) |idx| {
            if (kept >= mu) break;
            if (!keep[idx]) {
                keep[idx] = true;
                kept += 1;
            }
        }
        var w: usize = 0;
        for (0..pop.items.len) |idx| {
            if (keep[idx]) {
                pop.items[w] = pop.items[idx];
                w += 1;
            } else {
                var m = pop.items[idx];
                m.deinit(allocator);
            }
        }
        pop.items.len = w;

        // gentle diversification: on stagnation, keep the better half (by cost)
        // and refill with fresh random individuals — diversity without discarding
        // the good genes a full restart would. (An aggressive keep-1 restart was
        // tried and regressed: the short cycles between frequent restarts don't
        // re-converge deeply enough. The stochastic plateaus are better escaped by
        // external multi-start than by disrupting a single run mid-convergence.)
        if (best_cost < before) stagnation = 0 else stagnation += 1;
        if (params.restart_gens > 0 and stagnation >= params.restart_gens) {
            for (0..pop.items.len) |k| order[k] = k;
            std.sort.pdq(usize, order[0..pop.items.len], pop.items, struct {
                fn lt(p: []const Individual, a: usize, b: usize) bool {
                    return p[a].cost < p[b].cost or (p[a].cost == p[b].cost and a < b);
                }
            }.lt);
            const surv = @max(@as(usize, 1), mu / 2);
            @memset(keep[0..pop.items.len], false);
            for (order[0..surv]) |idx| keep[idx] = true;
            var w2: usize = 0;
            for (0..pop.items.len) |idx| {
                if (keep[idx]) {
                    pop.items[w2] = pop.items[idx];
                    w2 += 1;
                } else {
                    var m = pop.items[idx];
                    m.deinit(allocator);
                }
            }
            pop.items.len = w2;
            while (pop.items.len < mu) {
                for (0..n) |k| ebuf[k] = k + 1;
                var t = n;
                while (t > 1) {
                    t -= 1;
                    const r = rng.uintLessThan(usize, t + 1);
                    std.mem.swap(usize, &ebuf[t], &ebuf[r]);
                }
                _ = try cvrpInsert(allocator, inst, max_vehicles, ebuf, ebuf, n, &pop, &best, &best_cost, &have_best, gran, gk, pen_coeff);
            }
            stagnation = 0;
        }
    }
    if (!have_best) return error.NoFeasibleSplit;
    if (validate(inst, best.routes) == null) return error.Infeasible;
    return best;
}

// --- Best-of-K parallel HGS (same idea, for the fleet-capped / asymmetric path) ---
const HgsSlot = struct {
    inst: CvrpInstance,
    options: solver.SolveOptions,
    params: HgsParams,
    max_vehicles: usize,
    seed: u64,
    order: []usize,
    ends: []usize,
    nroutes: usize = 0,
    cost: u64 = std.math.maxInt(u64),
    ok: bool = false,
};

fn hgsWorker(slot: *HgsSlot) void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    var opts = slot.options;
    opts.seed = slot.seed;
    const res = solveCvrpHgs(arena.allocator(), slot.inst, opts, slot.params, slot.max_vehicles) catch {
        slot.ok = false;
        return;
    };
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

/// Run `threads` independent HGS searches (seed + i) and return the best. threads<=1
/// is the plain serial path. Brings best-of-K to the fleet-capped CVRP/ACVRP path.
///
/// Reproducibility caveat: `threads == 0` resolves to the host CPU count, which
/// sets the search count and therefore each search's seed (options.seed + i), so
/// the winning result depends on the machine's core count, and the same seed
/// yields different routes across machines. For output reproducible across
/// machines, pass an explicit non-zero `threads`.
pub fn solveCvrpHgsParallel(allocator: std.mem.Allocator, inst: CvrpInstance, options: solver.SolveOptions, params: HgsParams, max_vehicles: usize, threads: usize) !CvrpResult {
    const cpus = std.Thread.getCpuCount() catch 1;
    const k = if (threads == 0) @max(@as(usize, 1), cpus -| 1) else threads;
    if (k <= 1 or inst.n <= 2) return solveCvrpHgs(allocator, inst, options, params, max_vehicles);
    const n = inst.n;

    const slots = try allocator.alloc(HgsSlot, k);
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
            .max_vehicles = max_vehicles,
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
        ths[i] = std.Thread.spawn(.{}, hgsWorker, .{&slots[i]}) catch break;
        spawned += 1;
    }
    for (spawned..k) |i| hgsWorker(&slots[i]);
    for (0..spawned) |i| ths[i].join();

    var best: ?usize = null;
    for (slots, 0..) |s, i| {
        if (!s.ok) continue;
        if (best == null or s.cost < slots[best.?].cost) best = i;
    }
    const winner = best orelse return error.AllChainsFailed;
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
