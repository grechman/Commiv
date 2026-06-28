const std = @import("std");
const commiv = @import("commiv");

// ASYMMETRIC CVRP benchmark — the real test of commiv's actual problem on real
// directed cost matrices. Fischetti/Toth/Vigo ACVRP instances (from Helsgaun's
// LKH-3 distribution), EXPLICIT FULL_MATRIX asymmetric costs. The reference is
// LKH-3 (the field's best on ACVRP); its tour cost is the comparison baseline, so
// gap-to-reference is gap-to-the-field on directed CVRP. Env: KB_DIR, KB_FILES.
pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const env = init.environ_map;
    const dir = env.get("KB_DIR") orelse "vendor/acvrp";
    const files = env.get("KB_FILES") orelse
        "A034-02f,A034-04f,A034-08f,A034-14f,A036-03f,A036-05f,A036-10f,A036-18f," ++
        "A039-03f,A039-06f,A039-12f,A039-20f,A045-03f,A045-06f,A045-11f,A045-18f," ++
        "A048-03f,A048-05f,A048-10f,A048-16f,A056-03f,A056-05f,A056-10f,A056-17f," ++
        "A065-03f,A065-06f,A065-12f,A065-19f,A071-03f,A071-05f,A071-10f,A071-17f";
    const seed = try std.fmt.parseInt(u64, env.get("KB_SEED") orelse "12345", 10);

    std.debug.print("instance,n,lkh,commiv,gap_pct,routes,veh,ms,feasible,fleet_ok\n", .{});
    var sum_gap: f64 = 0;
    var count: usize = 0;
    var over_fleet: usize = 0;
    var it = std.mem.tokenizeScalar(u8, files, ',');
    while (it.next()) |name| {
        var path_buf: [512]u8 = undefined;
        const path = try std.fmt.bufPrint(&path_buf, "{s}/{s}.vrp", .{ dir, name });
        const bytes = try std.Io.Dir.cwd().readFileAlloc(init.io, path, allocator, .limited(8 << 20));
        defer allocator.free(bytes);

        var owned = try parseAcvrp(allocator, bytes);
        defer owned.deinit(allocator);
        const inst = owned.inst();
        const n = inst.n;

        const rounds: usize = try std.fmt.parseInt(usize, env.get("KB_ROUNDS") orelse (if (n <= 50) "300" else "200"), 10);
        const restarts: usize = try std.fmt.parseInt(usize, env.get("KB_RESTARTS") orelse "12", 10);
        const gens: usize = try std.fmt.parseInt(usize, env.get("KB_GENS") orelse "100", 10);
        const use_hgs = !std.mem.eql(u8, env.get("KB_HGS") orelse "1", "0");
        const opts = commiv.SolveOptions{
            .seed = seed,
            .budget = .{ .trials = @min(n, 100), .trial_extension_factor = 2, .max_passes = 60 },
            .candidates = .{ .candidate_count = @min(@as(usize, 10), n - 1), .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
            .search = .{ .enable_lk = true, .lk_max_depth = 5 },
        };
        // Respect the instance's fixed fleet: at most `vehicles` routes.
        const t0 = nanos();
        const kb_threads = try std.fmt.parseInt(usize, env.get("KB_THREADS") orelse "1", 10);
        var res = if (use_hgs)
            try commiv.solveCvrpHgsParallel(allocator, inst, opts, .{ .generations = gens }, owned.vehicles, kb_threads)
        else
            try commiv.solveCvrpFleet(allocator, inst, opts, .{ .rounds = rounds, .restarts = restarts, .max_vehicles = owned.vehicles });
        defer res.deinit();
        const ms = @as(f64, @floatFromInt(nanos() - t0)) / 1e6;

        const checked = commiv.internal.vrp.validate(inst, res.routes);
        const feasible = checked != null and checked.? == res.total_cost;
        const lkh = lkhFor(name) orelse 0;
        const gap = if (lkh > 0) 100.0 * (@as(f64, @floatFromInt(res.total_cost)) - @as(f64, @floatFromInt(lkh))) / @as(f64, @floatFromInt(lkh)) else 0;
        // Fair comparison only when we respect the instance's fleet size: our
        // free-vehicle engine can otherwise use extra routes to undercut LKH's
        // distance, which is infeasible for ACVRP's fixed vehicle count.
        const fleet_ok = res.routes.len <= owned.vehicles;
        if (lkh > 0 and feasible and fleet_ok) {
            sum_gap += gap;
            count += 1;
        } else if (feasible and !fleet_ok) {
            over_fleet += 1;
        }
        std.debug.print("{s},{},{},{},{d:.3},{},{},{d:.0},{},{}\n", .{ name, n, lkh, res.total_cost, gap, res.routes.len, owned.vehicles, ms, feasible, fleet_ok });
    }
    if (count > 0) {
        std.debug.print("# mean gap over {} fleet-respecting vs LKH-3: {d:.3}%  ({} used extra vehicles, excluded)\n", .{ count, sum_gap / @as(f64, @floatFromInt(count)), over_fleet });
    }
}

/// LKH-3 reference tour costs (best of the bundled .tour files) — the field's
/// best on these asymmetric CVRP instances.
fn lkhFor(name: []const u8) ?u64 {
    const table = [_]struct { []const u8, u64 }{
        .{ "A034-02f", 1406 }, .{ "A034-04f", 1773 }, .{ "A034-08f", 2672 }, .{ "A034-14f", 4046 },
        .{ "A036-03f", 1644 }, .{ "A036-05f", 2110 }, .{ "A036-10f", 3338 }, .{ "A036-18f", 5296 },
        .{ "A039-03f", 1654 }, .{ "A039-06f", 2289 }, .{ "A039-12f", 3705 }, .{ "A039-20f", 5903 },
        .{ "A045-03f", 1740 }, .{ "A045-06f", 2303 }, .{ "A045-11f", 3544 }, .{ "A045-18f", 6399 },
        .{ "A048-03f", 1891 }, .{ "A048-05f", 2283 }, .{ "A048-10f", 3325 }, .{ "A048-16f", 4955 },
        .{ "A056-03f", 1739 }, .{ "A056-05f", 2165 }, .{ "A056-10f", 3263 }, .{ "A056-17f", 4998 },
        .{ "A065-03f", 1974 }, .{ "A065-06f", 2567 }, .{ "A065-12f", 3902 }, .{ "A065-19f", 6014 },
        .{ "A071-03f", 2054 }, .{ "A071-05f", 2457 }, .{ "A071-10f", 3486 }, .{ "A071-17f", 5006 },
    };
    for (table) |row| if (std.mem.eql(u8, row[0], name)) return row[1];
    return null;
}

const Owned = struct {
    matrix: []u32,
    demand: []u32,
    n: usize,
    capacity: u32,
    vehicles: usize,
    fn inst(self: *const Owned) commiv.CvrpInstance {
        return .{ .n = self.n, .matrix = self.matrix, .demand = self.demand, .capacity = self.capacity };
    }
    fn deinit(self: *Owned, allocator: std.mem.Allocator) void {
        allocator.free(self.matrix);
        allocator.free(self.demand);
        self.* = undefined;
    }
};

/// Parse a TSPLIB ACVRP instance: DIMENSION D (incl. depot), CAPACITY, VEHICLES,
/// EDGE_WEIGHT_SECTION (D*D directional FULL_MATRIX), DEMAND_SECTION (node demand),
/// DEPOT_SECTION (depot node, usually the LAST node). Remaps the depot to internal
/// index 0 and customers to 1..n, producing a directional CvrpInstance matrix.
fn parseAcvrp(allocator: std.mem.Allocator, bytes: []const u8) !Owned {
    var dim: usize = 0;
    var capacity: u32 = 0;
    var vehicles: usize = 0;
    var depot: usize = 0;
    var m: []u32 = &.{}; // raw D*D, 0-indexed
    var dem_by_node: []u32 = &.{}; // 1-indexed length D+1
    errdefer {
        if (m.len > 0) allocator.free(m);
        if (dem_by_node.len > 0) allocator.free(dem_by_node);
    }
    const Sec = enum { none, weights, demand, depot };
    var sec: Sec = .none;
    var wk: usize = 0;
    var lines = std.mem.tokenizeAny(u8, bytes, "\r\n");
    while (lines.next()) |raw| {
        const line = std.mem.trim(u8, raw, " \t");
        if (line.len == 0) continue;
        if (std.mem.startsWith(u8, line, "DIMENSION")) {
            dim = try trailingInt(usize, line);
            if (dim < 2 or dim > 100_000) return error.BadInstance;
            m = try allocator.alloc(u32, std.math.mul(usize, dim, dim) catch return error.BadInstance);
            dem_by_node = try allocator.alloc(u32, dim + 1);
            continue;
        }
        if (std.mem.startsWith(u8, line, "CAPACITY")) {
            capacity = try trailingInt(u32, line);
            continue;
        }
        if (std.mem.startsWith(u8, line, "VEHICLES")) {
            vehicles = try trailingInt(usize, line);
            continue;
        }
        if (std.mem.indexOf(u8, line, "EDGE_WEIGHT_SECTION") != null) {
            sec = .weights;
            continue;
        }
        if (std.mem.startsWith(u8, line, "DEMAND_SECTION")) {
            sec = .demand;
            continue;
        }
        if (std.mem.startsWith(u8, line, "DEPOT_SECTION")) {
            sec = .depot;
            continue;
        }
        if (std.mem.startsWith(u8, line, "EOF")) break;
        // any other keyword line resets to none unless we're mid weight matrix
        if (std.mem.indexOfScalar(u8, line, ':') != null and sec != .weights) {
            sec = .none;
            continue;
        }
        switch (sec) {
            .weights => {
                var t = std.mem.tokenizeAny(u8, line, " \t");
                while (t.next()) |tok| {
                    if (wk >= dim * dim) break;
                    m[wk] = std.fmt.parseInt(u32, tok, 10) catch continue;
                    wk += 1;
                }
            },
            .demand => {
                var t = std.mem.tokenizeAny(u8, line, " \t");
                const node = std.fmt.parseInt(usize, t.next() orelse continue, 10) catch continue;
                const d = std.fmt.parseInt(u32, t.next() orelse continue, 10) catch continue;
                if (node >= 1 and node <= dim) dem_by_node[node] = d;
            },
            .depot => {
                const v = std.fmt.parseInt(isize, line, 10) catch continue;
                if (v > 0 and depot == 0) depot = @intCast(v);
            },
            .none => {},
        }
    }
    if (dim < 2 or depot == 0 or depot > dim) return error.BadInstance;
    const n = dim - 1;
    // remap: depot -> 0, others -> 1..n in node order
    const remap = try allocator.alloc(usize, dim + 1);
    defer allocator.free(remap);
    remap[depot] = 0;
    var idx: usize = 1;
    var node: usize = 1;
    while (node <= dim) : (node += 1) {
        if (node == depot) continue;
        remap[node] = idx;
        idx += 1;
    }
    const matrix = try allocator.alloc(u32, (n + 1) * (n + 1));
    errdefer allocator.free(matrix);
    @memset(matrix, 0);
    var i: usize = 1;
    while (i <= dim) : (i += 1) {
        var j: usize = 1;
        while (j <= dim) : (j += 1) {
            if (i == j) continue; // leave diagonal 0 (never queried)
            matrix[remap[i] * (n + 1) + remap[j]] = m[(i - 1) * dim + (j - 1)];
        }
    }
    const demand = try allocator.alloc(u32, n + 1);
    errdefer allocator.free(demand);
    demand[0] = 0;
    node = 1;
    while (node <= dim) : (node += 1) demand[remap[node]] = dem_by_node[node];
    allocator.free(m);
    allocator.free(dem_by_node);
    return .{ .matrix = matrix, .demand = demand, .n = n, .capacity = capacity, .vehicles = vehicles };
}

fn trailingInt(comptime T: type, line: []const u8) !T {
    var t = std.mem.tokenizeAny(u8, line, " \t:");
    var last: ?[]const u8 = null;
    while (t.next()) |tok| last = tok;
    return std.fmt.parseInt(T, last orelse return error.NoInt, 10);
}

fn nanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    _ = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}
