const std = @import("std");
const commiv = @import("commiv");

// PDPTW gap benchmark on the Li & Lim 100-series (SINTEF) against the
// published best-known solutions. Both sides are scored on the SAME integer
// matrix (round(euclid * 1000)), and the BKS routes from the .sol files are
// re-validated through validatePdptw first — so the parser, the oracle, and
// the gap are all checked against each other every run.
//
// Objective is hierarchical like the literature: min vehicles, then distance
// (veh_penalty = 10M scaled units). Report: vehicles vs BKS vehicles, distance
// vs BKS distance (on our matrix), gap%, wall.
//
// Env: PB_DIR (default vendor/pdptw), PB_FILES (comma list, default all 56),
// PB_TIME_MS (per instance, default 10000), PB_SEED, PB_ITERS (cap, default
// huge — time-bound), PB_VEH_PEN (default 10_000_000).
pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const env = init.environ_map;
    const dir = env.get("PB_DIR") orelse "vendor/pdptw";
    const files = env.get("PB_FILES") orelse
        "lc101,lc102,lc103,lc104,lc105,lc106,lc107,lc108,lc109," ++
            "lc201,lc202,lc203,lc204,lc205,lc206,lc207,lc208," ++
            "lr101,lr102,lr103,lr104,lr105,lr106,lr107,lr108,lr109,lr110,lr111,lr112," ++
            "lr201,lr202,lr203,lr204,lr205,lr206,lr207,lr208,lr209,lr210,lr211," ++
            "lrc101,lrc102,lrc103,lrc104,lrc105,lrc106,lrc107,lrc108," ++
            "lrc201,lrc202,lrc203,lrc204,lrc205,lrc206,lrc207,lrc208";
    const time_ms = try std.fmt.parseInt(u64, env.get("PB_TIME_MS") orelse "10000", 10);
    const seed = try std.fmt.parseInt(u64, env.get("PB_SEED") orelse "1", 10);
    const iters = try std.fmt.parseInt(usize, env.get("PB_ITERS") orelse "1000000000", 10);
    const veh_pen = try std.fmt.parseInt(u64, env.get("PB_VEH_PEN") orelse "10000000", 10);
    const fleet_min = std.mem.eql(u8, env.get("PB_FLEET") orelse "0", "1");
    const cap = try std.fmt.parseInt(usize, env.get("PB_CAP") orelse "0", 10); // fixed fleet target (bank mode)
    const cbar = try std.fmt.parseFloat(f64, env.get("PB_CBAR") orelse "10");
    const l_max = try std.fmt.parseInt(usize, env.get("PB_LMAX") orelse "10", 10);
    const t0f = try std.fmt.parseFloat(f64, env.get("PB_T0") orelse "1");
    const blink = try std.fmt.parseFloat(f64, env.get("PB_BLINK") orelse "0.01");
    const n_threads = try std.fmt.parseInt(usize, env.get("PB_THREADS") orelse "1", 10); // >1: parallel fleet-min waves
    const eval_threads = try std.fmt.parseInt(usize, env.get("PB_EVAL_THREADS") orelse "1", 10); // >=2: intra-search depth (recreate hot loop); use XOR PB_THREADS, never both
    if (n_threads > 1 and eval_threads > 1) { // width*depth = n_threads*eval_threads live threads; refuse the footgun
        std.debug.print("PB_THREADS ({d}) and PB_EVAL_THREADS ({d}) both >1: oversubscribes cores; set one to 1\n", .{ n_threads, eval_threads });
        return error.ThreadKnobConflict;
    }
    const gran_gaps = try std.fmt.parseInt(u2, env.get("PB_GRAN") orelse "0", 10); // granular gaps mask: 1 pickup, 2 dropoff, 3 both
    const eject = std.mem.eql(u8, env.get("PB_EJECT") orelse "0", "1"); // GES squeeze fallback in capped runs
    const eject_k = try std.fmt.parseInt(u8, env.get("PB_EJECTK") orelse "1", 10); // max residents ejected per squeeze (1..3)
    const swap_kick = try std.fmt.parseInt(usize, env.get("PB_SWAP") orelse "0", 10); // pair-exchange kick period (0 = off)
    const p0_pct = try std.fmt.parseInt(u8, env.get("PB_P0") orelse "40", 10); // uncapped phase % of fleet-min budget
    const pin_mode = std.mem.eql(u8, env.get("PB_PIN") orelse "0", "1"); // pinned-fleet driver: pin = PB_CAP, or BKS fleet if unset
    const time_pen = try std.fmt.parseInt(u64, env.get("PB_TIMEPEN") orelse "0", 10); // money mode: cost per time unit of route duration
    const max_dur = try std.fmt.parseInt(u64, env.get("PB_MAXDUR") orelse "0", 10); // shift-length cap (route duration); 0 = uncapped

    std.debug.print("instance,n_pairs,bks_veh,veh,bks_dist,dist,gap_pct,ms,dur,wait\n", .{});
    var sum_gap: f64 = 0;
    var count: usize = 0;
    var veh_wins: usize = 0;
    var veh_ties: usize = 0;
    var veh_losses: usize = 0;

    var it = std.mem.tokenizeScalar(u8, files, ',');
    while (it.next()) |name| {
        var path_buf: [512]u8 = undefined;
        const ipath = try std.fmt.bufPrint(&path_buf, "{s}/{s}.txt", .{ dir, name });
        const ibytes = try std.Io.Dir.cwd().readFileAlloc(init.io, ipath, allocator, .limited(1 << 22));
        defer allocator.free(ibytes);
        var parsed = try parseLiLim(allocator, ibytes);
        defer parsed.deinit(allocator);
        const inst = parsed.inst();

        var spath_buf: [512]u8 = undefined;
        const spath = try std.fmt.bufPrint(&spath_buf, "{s}/{s}.sol", .{ dir, name });
        const sbytes = try std.Io.Dir.cwd().readFileAlloc(init.io, spath, allocator, .limited(1 << 22));
        defer allocator.free(sbytes);
        var bks = try parseSolRoutes(allocator, sbytes);
        defer bks.deinit(allocator);

        // BKS routes must pass our own oracle — parser/rounding sanity gate.
        const bks_dist = commiv.validatePdptw(inst, bks.routes) orelse {
            std.debug.print("{s},BKS_INVALID_UNDER_OUR_MATRIX\n", .{name});
            continue;
        };
        const bks_veh = bks.routes.len;

        const t0 = nanos();
        const base_params = commiv.PdpSisrParams{
            .seed = seed,
            .iters = iters,
            .time_ms = time_ms,
            .veh_penalty = veh_pen,
            .max_vehicles = cap,
            .cbar = cbar,
            .l_max = l_max,
            .t0_factor = t0f,
            .blink = blink,
            .gran_gaps = gran_gaps,
            .eject = eject,
            .eject_k = eject_k,
            .swap_kick = swap_kick,
            .fleet_p0_pct = p0_pct,
            .time_penalty = time_pen,
            .max_route_dur = max_dur,
            .eval_threads = eval_threads,
        };
        var res = if (pin_mode) blk: {
            const pin: usize = if (cap > 0) cap else bks_veh;
            break :blk commiv.internal.pdptw_sisr.solvePdptwSisrPinned(allocator, inst, base_params, time_ms, pin) catch |err| switch (err) {
                error.NoCompleteSolution => {
                    std.debug.print("{s},NO_PIN_AT_{d}\n", .{ name, pin });
                    continue;
                },
                else => return err,
            };
        } else if (fleet_min and n_threads > 1)
            try commiv.internal.pdptw_sisr.solvePdptwSisrFleetMinParallel(allocator, inst, base_params, time_ms, n_threads)
        else if (fleet_min)
            try commiv.internal.pdptw_sisr.solvePdptwSisrFleetMin(allocator, inst, base_params, time_ms)
        else
            commiv.solvePdptwSisr(allocator, inst, base_params) catch |err| switch (err) {
                error.NoCompleteSolution => {
                    std.debug.print("{s},NO_COMPLETE_SOLUTION_AT_CAP_{d}\n", .{ name, cap });
                    continue;
                },
                else => return err,
            };
        defer res.deinit();
        const ms = (nanos() - t0) / 1_000_000;

        const rc = try allocator.alloc([]const usize, res.routes.len);
        defer allocator.free(rc);
        for (res.routes, 0..) |r, i| rc[i] = r;
        const vc = commiv.validatePdptw(inst, rc) orelse {
            std.debug.print("{s},SOLVER_OUTPUT_INFEASIBLE\n", .{name});
            continue;
        };
        if (vc != res.total_cost) {
            std.debug.print("{s},COST_MISMATCH\n", .{name});
            continue;
        }

        const gap = (@as(f64, @floatFromInt(res.total_cost)) - @as(f64, @floatFromInt(bks_dist))) / @as(f64, @floatFromInt(bks_dist)) * 100.0;
        if (res.vehicles < bks_veh) veh_wins += 1 else if (res.vehicles == bks_veh) veh_ties += 1 else veh_losses += 1;
        sum_gap += gap;
        count += 1;
        // duration = departure-time-optimized (Tws algebra); wait = the part
        // of duration that is neither driving nor service
        var dur_total: u64 = 0;
        var service_total: u64 = 0;
        for (res.routes) |r| {
            dur_total += commiv.internal.pdptw_sisr.routeDuration(inst, r);
            for (r) |c| service_total += inst.service[c];
        }
        const wait_total = dur_total - res.total_cost - service_total;
        std.debug.print("{s},{d},{d},{d},{d:.3},{d:.3},{d:.2},{d},{d:.3},{d:.3}\n", .{
            name,                                        inst.n_pairs,                                     bks_veh, res.vehicles,
            @as(f64, @floatFromInt(bks_dist)) / 1000.0,  @as(f64, @floatFromInt(res.total_cost)) / 1000.0, gap,     ms,
            @as(f64, @floatFromInt(dur_total)) / 1000.0, @as(f64, @floatFromInt(wait_total)) / 1000.0,
        });
    }
    if (count > 0) {
        std.debug.print("summary: {d} instances, mean dist gap {d:.2}%, vehicles win/tie/loss {d}/{d}/{d}\n", .{ count, sum_gap / @as(f64, @floatFromInt(count)), veh_wins, veh_ties, veh_losses });
    }
}

fn nanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    _ = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}

const Parsed = struct {
    n_pairs: usize,
    capacity: i64,
    matrix: []u32,
    pair_of: []usize,
    is_pickup: []bool,
    demand_signed: []i64,
    ready: []u32,
    due: []u32,
    service: []u32,

    fn inst(self: *const Parsed) commiv.PdpInstance {
        return .{
            .n_pairs = self.n_pairs,
            .matrix = self.matrix,
            .capacity = self.capacity,
            .pair_of = self.pair_of,
            .is_pickup = self.is_pickup,
            .demand_signed = self.demand_signed,
            .ready = self.ready,
            .due = self.due,
            .service = self.service,
        };
    }

    fn deinit(self: *Parsed, allocator: std.mem.Allocator) void {
        allocator.free(self.matrix);
        allocator.free(self.pair_of);
        allocator.free(self.is_pickup);
        allocator.free(self.demand_signed);
        allocator.free(self.ready);
        allocator.free(self.due);
        allocator.free(self.service);
        self.* = undefined;
    }
};

const SCALE = 1000.0;

/// Li & Lim format: line 1 = "K Q speed"; then one line per node:
/// id x y demand ready due service pickup-sibling delivery-sibling
/// (depot is id 0; pickups have demand > 0 and name their delivery).
fn parseLiLim(allocator: std.mem.Allocator, bytes: []const u8) !Parsed {
    var lines = std.mem.tokenizeAny(u8, bytes, "\r\n");
    const header = lines.next() orelse return error.BadFormat;
    var hf = std.mem.tokenizeAny(u8, header, " \t");
    _ = hf.next() orelse return error.BadFormat; // vehicle count (unused: hierarchical objective)
    const capacity = try std.fmt.parseInt(i64, hf.next() orelse return error.BadFormat, 10);

    var xs: std.ArrayList(f64) = .empty;
    defer xs.deinit(allocator);
    var ys: std.ArrayList(f64) = .empty;
    defer ys.deinit(allocator);
    var dem: std.ArrayList(i64) = .empty;
    defer dem.deinit(allocator);
    var rdy: std.ArrayList(u32) = .empty;
    defer rdy.deinit(allocator);
    var due_l: std.ArrayList(u32) = .empty;
    defer due_l.deinit(allocator);
    var srv: std.ArrayList(u32) = .empty;
    defer srv.deinit(allocator);
    var deliv: std.ArrayList(usize) = .empty;
    defer deliv.deinit(allocator);
    var pick: std.ArrayList(usize) = .empty;
    defer pick.deinit(allocator);

    while (lines.next()) |line| {
        var f = std.mem.tokenizeAny(u8, line, " \t");
        const id = try std.fmt.parseInt(usize, f.next() orelse break, 10);
        if (id != xs.items.len) return error.BadFormat; // ids must be dense 0..N
        try xs.append(allocator, try std.fmt.parseFloat(f64, f.next() orelse return error.BadFormat));
        try ys.append(allocator, try std.fmt.parseFloat(f64, f.next() orelse return error.BadFormat));
        try dem.append(allocator, try std.fmt.parseInt(i64, f.next() orelse return error.BadFormat, 10));
        const rd = try std.fmt.parseInt(u64, f.next() orelse return error.BadFormat, 10);
        const du = try std.fmt.parseInt(u64, f.next() orelse return error.BadFormat, 10);
        const sv = try std.fmt.parseInt(u64, f.next() orelse return error.BadFormat, 10);
        try rdy.append(allocator, @intCast(rd * @as(u64, SCALE)));
        try due_l.append(allocator, @intCast(du * @as(u64, SCALE)));
        try srv.append(allocator, @intCast(sv * @as(u64, SCALE)));
        try pick.append(allocator, try std.fmt.parseInt(usize, f.next() orelse return error.BadFormat, 10));
        try deliv.append(allocator, try std.fmt.parseInt(usize, f.next() orelse return error.BadFormat, 10));
    }

    const dim = xs.items.len;
    if (dim < 3 or (dim - 1) % 2 != 0) return error.BadFormat;
    const n_pairs = (dim - 1) / 2;

    const pair_of = try allocator.alloc(usize, dim);
    errdefer allocator.free(pair_of);
    const is_pickup = try allocator.alloc(bool, dim);
    errdefer allocator.free(is_pickup);
    pair_of[0] = 0;
    is_pickup[0] = false;
    for (1..dim) |c| {
        if (dem.items[c] > 0) {
            is_pickup[c] = true;
            pair_of[c] = deliv.items[c];
            if (pair_of[c] == 0 or pair_of[c] >= dim) return error.BadFormat;
        } else if (dem.items[c] < 0) {
            is_pickup[c] = false;
            pair_of[c] = pick.items[c];
            if (pair_of[c] == 0 or pair_of[c] >= dim) return error.BadFormat;
        } else return error.BadFormat;
    }
    for (1..dim) |c| if (pair_of[pair_of[c]] != c) return error.BadFormat;

    const matrix = try allocator.alloc(u32, dim * dim);
    errdefer allocator.free(matrix);
    for (0..dim) |a| {
        for (0..dim) |b| {
            const dx = xs.items[a] - xs.items[b];
            const dy = ys.items[a] - ys.items[b];
            matrix[a * dim + b] = @intFromFloat(@round(@sqrt(dx * dx + dy * dy) * SCALE));
        }
    }

    return .{
        .n_pairs = n_pairs,
        .capacity = capacity,
        .matrix = matrix,
        .pair_of = pair_of,
        .is_pickup = is_pickup,
        .demand_signed = try allocator.dupe(i64, dem.items),
        .ready = try allocator.dupe(u32, rdy.items),
        .due = try allocator.dupe(u32, due_l.items),
        .service = try allocator.dupe(u32, srv.items),
    };
}

const BksRoutes = struct {
    routes: [][]const usize,
    fn deinit(self: *BksRoutes, allocator: std.mem.Allocator) void {
        for (self.routes) |r| allocator.free(r);
        allocator.free(self.routes);
        self.* = undefined;
    }
};

/// SINTEF .sol format: lines "Route N : c1 c2 c3 ...".
fn parseSolRoutes(allocator: std.mem.Allocator, bytes: []const u8) !BksRoutes {
    var out: std.ArrayList([]const usize) = .empty;
    errdefer {
        for (out.items) |r| allocator.free(r);
        out.deinit(allocator);
    }
    var lines = std.mem.tokenizeAny(u8, bytes, "\r\n");
    while (lines.next()) |line| {
        const trimmed = std.mem.trim(u8, line, " \t");
        if (!std.mem.startsWith(u8, trimmed, "Route")) continue;
        const colon = std.mem.indexOfScalar(u8, trimmed, ':') orelse continue;
        var route: std.ArrayList(usize) = .empty;
        errdefer route.deinit(allocator);
        var f = std.mem.tokenizeAny(u8, trimmed[colon + 1 ..], " \t");
        while (f.next()) |tok| try route.append(allocator, try std.fmt.parseInt(usize, tok, 10));
        if (route.items.len > 0) {
            try out.append(allocator, try route.toOwnedSlice(allocator));
        } else route.deinit(allocator);
    }
    return .{ .routes = try out.toOwnedSlice(allocator) };
}
