const std = @import("std");
const commiv = @import("commiv");

// CVRP gap benchmark on the standard Augerat A-set (CVRPLIB, EUC_2D, all solved
// to proven optimality — the optimal value is embedded in each file's COMMENT).
// Reports cost, optimum, gap%, route count and wall time per instance so we can
// see the engine's real distance-to-optimal, not just an improvement over a
// weak baseline. Env: CB_DIR (default vendor/cvrp), CB_FILES (comma list),
// CB_ROUNDS (ILS rounds, default scales with n), CB_SEED.
pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const env = init.environ_map;
    const dir = env.get("CB_DIR") orelse "vendor/cvrp";
    const files = env.get("CB_FILES") orelse
        "A-n32-k5,A-n33-k5,A-n37-k5,A-n39-k5,A-n45-k7,A-n48-k7,A-n55-k9,A-n60-k9,A-n62-k8,A-n63-k9,A-n65-k9,A-n80-k10";
    const seed = try std.fmt.parseInt(u64, env.get("CB_SEED") orelse "12345", 10);

    std.debug.print("instance,n,opt,commiv,gap_pct,routes,k_opt,ms,feasible\n", .{});
    var total_gap: f64 = 0;
    var count: usize = 0;
    var it = std.mem.tokenizeScalar(u8, files, ',');
    while (it.next()) |name| {
        var path_buf: [512]u8 = undefined;
        const path = try std.fmt.bufPrint(&path_buf, "{s}/{s}.vrp", .{ dir, name });
        const bytes = try std.Io.Dir.cwd().readFileAlloc(init.io, path, allocator, .limited(1 << 20));
        defer allocator.free(bytes);

        var inst_owned = try parseCvrplib(allocator, bytes);
        defer inst_owned.deinit(allocator);
        const inst = inst_owned.inst();
        const n = inst.n;

        const rounds: usize = blk: {
            if (env.get("CB_ROUNDS")) |r| break :blk try std.fmt.parseInt(usize, r, 10);
            break :blk if (n <= 40) @as(usize, 120) else if (n <= 60) 100 else 80;
        };
        const restarts: usize = try std.fmt.parseInt(usize, env.get("CB_RESTARTS") orelse "12", 10);
        const use_vrptw = std.mem.eql(u8, env.get("CB_VRPTW") orelse "0", "1");

        const solve_opts = commiv.SolveOptions{
            .seed = seed,
            .budget = .{ .trials = @min(n, 100), .trial_extension_factor = 2, .max_passes = 60 },
            .candidates = .{ .candidate_count = @min(@as(usize, 10), n - 1), .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
            .search = .{ .enable_lk = true, .lk_max_depth = 5 },
        };

        // ASYMMETRIC ROAD TEST (CB_ASYM = alpha): turn a symmetric CVRP into a directed
        // one with a "6pm rush-hour" perturbation — moving TOWARD the depot/hub (the
        // congested inbound direction) costs more. Potential phi(i) = depot->i distance;
        // dmat[i][j] = base[i][j] + alpha*max(0, phi(i)-phi(j)). Then compare native-
        // asymmetric SISR (handles direction) vs symmetric-treatment (symmetrize, solve,
        // score on the TRUE directed matrix) across the asymmetry dial.
        if (env.get("CB_ASYM")) |as_| {
            const alpha = try std.fmt.parseFloat(f64, as_);
            // CB_ASYMMODE: gradient (conservative rush-hour, default), random (uniform
            // non-conservative noise), or oneway (sparse strong one-way streets — the
            // most road-realistic non-conservative source). CB_ONEWAY_P = fraction of
            // arcs made one-way (default 0.2).
            const amode = env.get("CB_ASYMMODE") orelse (if (std.mem.eql(u8, env.get("CB_ASYMRAND") orelse "0", "1")) "random" else "gradient");
            const dim = n + 1;
            const dmat = try allocator.alloc(u32, dim * dim);
            defer allocator.free(dmat);
            if (std.mem.eql(u8, amode, "random")) {
                var prng = std.Random.DefaultPrng.init(seed);
                const rng = prng.random();
                for (0..dim) |i| dmat[i * dim + i] = 0;
                for (0..dim) |i| for (i + 1..dim) |j| {
                    const b: f64 = @floatFromInt(inst.matrix[i * dim + j]);
                    dmat[i * dim + j] = inst.matrix[i * dim + j] + @as(u32, @intFromFloat(@round(alpha * b * rng.float(f64))));
                    dmat[j * dim + i] = inst.matrix[j * dim + i] + @as(u32, @intFromFloat(@round(alpha * b * rng.float(f64))));
                };
            } else if (std.mem.eql(u8, amode, "oneway")) {
                const frac = try std.fmt.parseFloat(f64, env.get("CB_ONEWAY_P") orelse "0.2");
                var prng = std.Random.DefaultPrng.init(seed);
                const rng = prng.random();
                for (0..dim) |i| dmat[i * dim + i] = 0;
                for (0..dim) |i| for (i + 1..dim) |j| {
                    const base = inst.matrix[i * dim + j];
                    const pen = base + @as(u32, @intFromFloat(@round(alpha * @as(f64, @floatFromInt(base)))));
                    if (rng.float(f64) < frac) { // one-way: penalise a random direction
                        if (rng.boolean()) {
                            dmat[i * dim + j] = base;
                            dmat[j * dim + i] = pen;
                        } else {
                            dmat[i * dim + j] = pen;
                            dmat[j * dim + i] = base;
                        }
                    } else {
                        dmat[i * dim + j] = base;
                        dmat[j * dim + i] = base;
                    }
                };
            } else for (0..dim) |i| for (0..dim) |j| {
                if (i == j) {
                    dmat[i * dim + j] = 0;
                    continue;
                }
                const phi_i: f64 = @floatFromInt(inst.matrix[i]); // matrix[0*dim+i] = depot->i
                const phi_j: f64 = @floatFromInt(inst.matrix[j]);
                const extra = alpha * @max(0.0, phi_i - phi_j);
                dmat[i * dim + j] = inst.matrix[i * dim + j] + @as(u32, @intFromFloat(@round(extra)));
            };
            var rs: f64 = 0;
            var cc: usize = 0;
            for (0..dim) |i| for (0..dim) |j| {
                if (i == j) continue;
                const x = dmat[i * dim + j];
                const y = dmat[j * dim + i];
                if (x > 0 and y > 0) {
                    rs += @as(f64, @floatFromInt(@max(x, y))) / @as(f64, @floatFromInt(@min(x, y)));
                    cc += 1;
                }
            };
            const rho = rs / @as(f64, @floatFromInt(cc));
            const sp = commiv.CvrpSisrParams{ .iters = try std.fmt.parseInt(usize, env.get("CB_ITERS") orelse "500000", 10) };
            const thr = try std.fmt.parseInt(usize, env.get("CB_THREADS") orelse "3", 10);
            const t_a = nanos();
            var ra = try commiv.solveCvrpSisrParallel(allocator, .{ .n = n, .matrix = dmat, .demand = inst.demand, .capacity = inst.capacity }, solve_opts, sp, thr);
            defer ra.deinit();
            const ms_a = @as(f64, @floatFromInt(nanos() - t_a)) / 1e6;
            const sym = try allocator.alloc(u32, dim * dim);
            defer allocator.free(sym);
            for (0..dim) |i| for (0..dim) |j| {
                sym[i * dim + j] = if (i == j) 0 else @intCast((@as(u64, dmat[i * dim + j]) + dmat[j * dim + i]) / 2);
            };
            var rsr = try commiv.solveCvrpSisrParallel(allocator, .{ .n = n, .matrix = sym, .demand = inst.demand, .capacity = inst.capacity }, solve_opts, sp, thr);
            defer rsr.deinit();
            var cost_sym: u64 = 0; // score the symmetric solution on the TRUE directed matrix
            for (rsr.routes) |route| {
                var prev: usize = 0;
                for (route) |c| {
                    cost_sym += dmat[prev * dim + c];
                    prev = c;
                }
                cost_sym += dmat[prev * dim + 0];
            }
            const pen = 100.0 * (@as(f64, @floatFromInt(cost_sym)) - @as(f64, @floatFromInt(ra.total_cost))) / @as(f64, @floatFromInt(ra.total_cost));
            std.debug.print("{s} alpha={d:.2} rho={d:.4} | native_asym={} ({d:.0}ms) | sym_treat={} | ignore-direction penalty={d:.2}%\n", .{ name, alpha, rho, ra.total_cost, ms_a, cost_sym, pen });
            continue;
        }

        const t0 = nanos();
        var total_cost: u64 = undefined;
        var nroutes: usize = undefined;
        var feasible: bool = undefined;
        if (use_vrptw) {
            // CVRP = VRPTW with infinite windows / zero service: exercises the
            // richer VRPTW move set (2-opt*, elimination) on the same instances.
            const dimN = n + 1;
            const ready = try allocator.alloc(u32, dimN);
            defer allocator.free(ready);
            const due = try allocator.alloc(u32, dimN);
            defer allocator.free(due);
            const service = try allocator.alloc(u32, dimN);
            defer allocator.free(service);
            @memset(ready, 0);
            @memset(due, std.math.maxInt(u32) / 4);
            @memset(service, 0);
            const vinst = commiv.VrptwInstance{
                .n = n,
                .matrix = inst.matrix,
                .demand = inst.demand,
                .capacity = inst.capacity,
                .ready = ready,
                .due = due,
                .service = service,
            };
            const use_hgs = std.mem.eql(u8, env.get("CB_HGS") orelse "0", "1");
            var vres = if (use_hgs) try commiv.solveVrptwHgs(allocator, vinst, solve_opts, .{
                .mu = try std.fmt.parseInt(usize, env.get("CB_MU") orelse "25", 10),
                .lambda = try std.fmt.parseInt(usize, env.get("CB_LAMBDA") orelse "40", 10),
                .generations = try std.fmt.parseInt(usize, env.get("CB_GENS") orelse "30", 10),
            }, 0) else try commiv.solveVrptw(allocator, vinst, solve_opts, rounds, restarts, 0);
            defer vres.deinit();
            total_cost = vres.total_cost;
            nroutes = vres.vehicles;
            const vchk = commiv.internal.vrptw.validate(vinst, vres.routes);
            feasible = vchk != null and vchk.? == vres.total_cost;
            const ms = @as(f64, @floatFromInt(nanos() - t0)) / 1e6;
            const opt = bksFor(name) orelse inst_owned.opt;
            const gap = 100.0 * (@as(f64, @floatFromInt(total_cost)) - @as(f64, @floatFromInt(opt))) / @as(f64, @floatFromInt(opt));
            if (feasible) {
                total_gap += gap;
                count += 1;
            }
            std.debug.print("{s},{},{},{},{d:.3},{},{},{d:.0},{}\n", .{ name, n, opt, total_cost, gap, nroutes, inst_owned.k, ms, feasible });
            continue;
        }
        const use_sisr = std.mem.eql(u8, env.get("CB_SISR") orelse "0", "1");
        const native_hgs = std.mem.eql(u8, env.get("CB_HGS") orelse "1", "1");
        const sisr_params = commiv.CvrpSisrParams{
            .iters = try std.fmt.parseInt(usize, env.get("CB_ITERS") orelse "300000", 10),
            .l_max = try std.fmt.parseInt(usize, env.get("CB_LMAX") orelse "10", 10),
            .cbar = try std.fmt.parseFloat(f64, env.get("CB_CBAR") orelse "10"),
            .blink = try std.fmt.parseFloat(f64, env.get("CB_BLINK") orelse "0.01"),
            .t0_factor = try std.fmt.parseFloat(f64, env.get("CB_T0") orelse "1.0"),
            .tf_factor = try std.fmt.parseFloat(f64, env.get("CB_TF") orelse "0.01"),
            .split_rate = try std.fmt.parseFloat(f64, env.get("CB_SPLIT") orelse "-1"),
            .split_alpha = try std.fmt.parseFloat(f64, env.get("CB_ALPHA") orelse "0.5"),
            .bandit = std.mem.eql(u8, env.get("CB_BANDIT") orelse "0", "1"),
            .regret_rate = try std.fmt.parseFloat(f64, env.get("CB_REGRET") orelse "-1"),
        };
        const sisr_threads = try std.fmt.parseInt(usize, env.get("CB_THREADS") orelse "1", 10);
        var res = if (use_sisr)
            try commiv.solveCvrpSisrParallel(allocator, inst, solve_opts, sisr_params, sisr_threads)
        else if (native_hgs)
            try commiv.solveCvrpHgs(allocator, inst, solve_opts, .{
                .generations = try std.fmt.parseInt(usize, env.get("CB_GENS") orelse "100", 10),
                .infeasible_search = !std.mem.eql(u8, env.get("CB_INFEAS") orelse "1", "0"),
                .mu = try std.fmt.parseInt(usize, env.get("CB_MU") orelse "0", 10),
                .lambda = try std.fmt.parseInt(usize, env.get("CB_LAMBDA") orelse "0", 10),
            }, 0)
        else
            try commiv.internal.vrp.solveCvrpMulti(allocator, inst, solve_opts, rounds, restarts);
        defer res.deinit();
        const ms = @as(f64, @floatFromInt(nanos() - t0)) / 1e6;

        const checked = commiv.internal.vrp.validate(inst, res.routes);
        feasible = checked != null and checked.? == res.total_cost;
        // Authoritative proven-optimal value (CVRPLIB Augerat A). The file COMMENT
        // values in some mirrors are wrong, so we never trust them.
        const opt = bksFor(name) orelse inst_owned.opt;
        const gap = 100.0 * (@as(f64, @floatFromInt(res.total_cost)) - @as(f64, @floatFromInt(opt))) /
            @as(f64, @floatFromInt(opt));
        if (feasible) {
            total_gap += gap;
            count += 1;
        }
        std.debug.print("{s},{},{},{},{d:.3},{},{},{d:.0},{}\n", .{
            name, n, opt, res.total_cost, gap, res.routes.len, inst_owned.k, ms, feasible,
        });
    }
    if (count > 0) {
        std.debug.print("# mean gap over {} feasible: {d:.3}%\n", .{ count, total_gap / @as(f64, @floatFromInt(count)) });
    }
}

/// Proven-optimal values for the Augerat A-set (CVRPLIB / Augerat et al. 1995,
/// all closed by branch-and-cut). Source of truth for gap; file COMMENTs are not.
fn bksFor(name: []const u8) ?u64 {
    const table = [_]struct { []const u8, u64 }{
        .{ "A-n32-k5", 784 },  .{ "A-n33-k5", 661 },  .{ "A-n33-k6", 742 },
        .{ "A-n34-k5", 778 },  .{ "A-n36-k5", 799 },  .{ "A-n37-k5", 669 },
        .{ "A-n37-k6", 949 },  .{ "A-n38-k5", 730 },  .{ "A-n39-k5", 822 },
        .{ "A-n39-k6", 831 },  .{ "A-n44-k6", 937 },  .{ "A-n45-k6", 944 },
        .{ "A-n45-k7", 1146 }, .{ "A-n46-k7", 914 },  .{ "A-n48-k7", 1073 },
        .{ "A-n53-k7", 1010 }, .{ "A-n54-k7", 1167 }, .{ "A-n55-k9", 1073 },
        .{ "A-n60-k9", 1354 }, .{ "A-n61-k9", 1034 }, .{ "A-n62-k8", 1288 },
        .{ "A-n63-k9", 1616 }, .{ "A-n63-k10", 1314 }, .{ "A-n64-k9", 1401 },
        .{ "A-n65-k9", 1174 }, .{ "A-n69-k9", 1159 }, .{ "A-n80-k10", 1763 },
        // Uchoa X set (the hard CVRP benchmark; BKS/optimal from CVRPLIB).
        .{ "X-n101-k25", 27591 }, .{ "X-n153-k22", 21220 }, .{ "X-n200-k36", 58578 },
        .{ "X-n303-k21", 21736 }, .{ "X-n502-k39", 69226 }, .{ "X-n1001-k43", 72355 },
    };
    for (table) |row| {
        if (std.mem.eql(u8, row[0], name)) return row[1];
    }
    return null;
}

const OwnedInstance = struct {
    matrix: []u32,
    demand: []u32,
    n: usize,
    capacity: u32,
    opt: u64,
    k: usize,

    fn inst(self: *const OwnedInstance) commiv.CvrpInstance {
        return .{ .n = self.n, .matrix = self.matrix, .demand = self.demand, .capacity = self.capacity };
    }
    fn deinit(self: *OwnedInstance, allocator: std.mem.Allocator) void {
        allocator.free(self.matrix);
        allocator.free(self.demand);
        self.* = undefined;
    }
};

/// Parse a CVRPLIB EUC_2D instance: DIMENSION, CAPACITY, NODE_COORD_SECTION,
/// DEMAND_SECTION. Node 1 (1-indexed) is the depot -> internal index 0. Builds a
/// rounded-Euclidean symmetric matrix (the engine treats it as a directional
/// matrix that happens to be symmetric). Extracts the optimal value + truck
/// count from the COMMENT for gap reporting.
fn parseCvrplib(allocator: std.mem.Allocator, bytes: []const u8) !OwnedInstance {
    var dimension: usize = 0;
    var capacity: u32 = 0;
    var opt: u64 = 0;
    var k: usize = 0;
    var xs: []f64 = &.{};
    var ys: []f64 = &.{};
    var demand: []u32 = &.{};
    errdefer {
        if (xs.len > 0) allocator.free(xs);
        if (ys.len > 0) allocator.free(ys);
        if (demand.len > 0) allocator.free(demand);
    }

    const Section = enum { none, coords, demand, depot };
    var section: Section = .none;
    var lines = std.mem.tokenizeAny(u8, bytes, "\r\n");
    while (lines.next()) |raw| {
        const line = std.mem.trim(u8, raw, " \t");
        if (line.len == 0) continue;
        if (std.mem.startsWith(u8, line, "DIMENSION")) {
            dimension = try parseTrailingInt(usize, line);
            xs = try allocator.alloc(f64, dimension);
            ys = try allocator.alloc(f64, dimension);
            demand = try allocator.alloc(u32, dimension);
            continue;
        }
        if (std.mem.startsWith(u8, line, "CAPACITY")) {
            capacity = try parseTrailingInt(u32, line);
            continue;
        }
        if (std.mem.startsWith(u8, line, "COMMENT")) {
            opt = scanLabeledInt(line, "Optimal value:") orelse scanLabeledInt(line, "Best value:") orelse 0;
            k = scanLabeledInt(line, "Min no of trucks:") orelse 0;
            continue;
        }
        if (std.mem.startsWith(u8, line, "NODE_COORD_SECTION")) {
            section = .coords;
            continue;
        }
        if (std.mem.startsWith(u8, line, "DEMAND_SECTION")) {
            section = .demand;
            continue;
        }
        if (std.mem.startsWith(u8, line, "DEPOT_SECTION")) {
            section = .depot;
            continue;
        }
        if (std.mem.startsWith(u8, line, "EOF")) break;
        // section keywords like EDGE_WEIGHT_TYPE / TYPE / NAME fall through to here
        switch (section) {
            .coords => {
                var t = std.mem.tokenizeAny(u8, line, " \t");
                const idx = std.fmt.parseInt(usize, t.next() orelse continue, 10) catch continue;
                const x = try std.fmt.parseFloat(f64, t.next() orelse return error.BadCoord);
                const y = try std.fmt.parseFloat(f64, t.next() orelse return error.BadCoord);
                if (idx >= 1 and idx <= dimension) {
                    xs[idx - 1] = x;
                    ys[idx - 1] = y;
                }
            },
            .demand => {
                var t = std.mem.tokenizeAny(u8, line, " \t");
                const idx = std.fmt.parseInt(usize, t.next() orelse continue, 10) catch continue;
                const dem = try std.fmt.parseInt(u32, t.next() orelse return error.BadDemand, 10);
                if (idx >= 1 and idx <= dimension) demand[idx - 1] = dem;
            },
            else => {},
        }
    }
    if (dimension < 2 or dimension > 100_000) return error.BadInstance;

    // Rounded-Euclidean symmetric matrix; depot is node 1 -> index 0. Checked
    // multiply + cap so an unbounded DIMENSION can't wrap dimension*dimension
    // (silent in ReleaseFast) into an undersized buffer and OOB-write.
    const cells = std.math.mul(usize, dimension, dimension) catch return error.BadInstance;
    const matrix = try allocator.alloc(u32, cells);
    errdefer allocator.free(matrix);
    for (0..dimension) |i| {
        for (0..dimension) |j| {
            const dx = xs[i] - xs[j];
            const dy = ys[i] - ys[j];
            matrix[i * dimension + j] = @intFromFloat(@round(@sqrt(dx * dx + dy * dy)));
        }
    }
    allocator.free(xs);
    allocator.free(ys);
    return .{
        .matrix = matrix,
        .demand = demand,
        .n = dimension - 1,
        .capacity = capacity,
        .opt = opt,
        .k = k,
    };
}

fn parseTrailingInt(comptime T: type, line: []const u8) !T {
    // formats like "DIMENSION : 32" or "CAPACITY : 100"
    var t = std.mem.tokenizeAny(u8, line, " \t:");
    var last: ?[]const u8 = null;
    while (t.next()) |tok| last = tok;
    return std.fmt.parseInt(T, last orelse return error.NoInt, 10);
}

fn scanLabeledInt(line: []const u8, label: []const u8) ?u64 {
    const pos = std.mem.indexOf(u8, line, label) orelse return null;
    var i = pos + label.len;
    while (i < line.len and (line[i] == ' ' or line[i] == '\t')) i += 1;
    var j = i;
    while (j < line.len and line[j] >= '0' and line[j] <= '9') j += 1;
    if (j == i) return null;
    return std.fmt.parseInt(u64, line[i..j], 10) catch null;
}

fn nanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    _ = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}
