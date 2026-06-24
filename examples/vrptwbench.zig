const std = @import("std");
const commiv = @import("commiv");

// VRPTW gap benchmark on the standard Solomon 100-customer instances. Solomon's
// objective is hierarchical: (1) minimize vehicles, (2) minimize distance — so we
// bias Split/search with a large per-vehicle penalty, then report both numbers
// and the distance gap vs the SINTEF best-known solution. CONVENTION: coordinates,
// time windows and service times are integers; travel time = distance = round(
// euclid * 100) centi-units (distance printed /100). This tracks the unrounded-
// Euclidean BKS to ~0.05%; the comparison is indicative at the 2nd decimal.
// Env: VT_DIR (default vendor/vrptw), VT_FILES, VT_ROUNDS, VT_RESTARTS, VT_SEED.
const SCALE: u32 = 100;

pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const env = init.environ_map;
    const dir = env.get("VT_DIR") orelse "vendor/vrptw";
    const files = env.get("VT_FILES") orelse "c101,c201,r101,r201,rc101,rc201";
    const seed = try std.fmt.parseInt(u64, env.get("VT_SEED") orelse "12345", 10);
    const rounds = try std.fmt.parseInt(usize, env.get("VT_ROUNDS") orelse "100", 10);
    const restarts = try std.fmt.parseInt(usize, env.get("VT_RESTARTS") orelse "10", 10);

    std.debug.print("instance,n,bks_veh,bks_dist,veh,dist,dist_gap_pct,ms,feasible,veh_match\n", .{});
    var sum_gap: f64 = 0;
    var ngap: usize = 0;
    var it = std.mem.tokenizeScalar(u8, files, ',');
    while (it.next()) |name| {
        var path_buf: [512]u8 = undefined;
        const path = try std.fmt.bufPrint(&path_buf, "{s}/{s}.txt", .{ dir, name });
        const bytes = try std.Io.Dir.cwd().readFileAlloc(init.io, path, allocator, .limited(1 << 20));
        defer allocator.free(bytes);

        var owned = try parseSolomon(allocator, bytes);
        defer owned.deinit(allocator);
        const inst = owned.inst();
        const n = inst.n;

        // Strong per-vehicle penalty: prioritize the minimum vehicle count
        // (Solomon's primary objective), distance as the tiebreaker.
        const veh_penalty: u64 = 10_000_000;
        const use_hgs = std.mem.eql(u8, env.get("VT_HGS") orelse "0", "1");
        const solve_opts = commiv.SolveOptions{
            .seed = seed,
            .budget = .{ .trials = @min(n, 100), .trial_extension_factor = 2, .max_passes = 60 },
            .candidates = .{ .candidate_count = @min(@as(usize, 10), n - 1), .candidate_mode = .alpha_nearness, .sparse_min_dimension = 0 },
            .search = .{ .enable_lk = true, .lk_max_depth = 5 },
        };
        const t0 = nanos();
        var res = if (use_hgs) try commiv.solveVrptwHgs(allocator, inst, solve_opts, .{
            .mu = try std.fmt.parseInt(usize, env.get("VT_MU") orelse "25", 10),
            .lambda = try std.fmt.parseInt(usize, env.get("VT_LAMBDA") orelse "40", 10),
            .generations = try std.fmt.parseInt(usize, env.get("VT_GENS") orelse "30", 10),
        }, veh_penalty) else try commiv.solveVrptw(allocator, inst, solve_opts, rounds, restarts, veh_penalty);
        defer res.deinit();
        const ms = @as(f64, @floatFromInt(nanos() - t0)) / 1e6;

        const checked = commiv.vrptw.validate(inst, res.routes);
        const feasible = checked != null and checked.? == res.total_cost;
        const dist = @as(f64, @floatFromInt(res.total_cost)) / @as(f64, @floatFromInt(SCALE));

        const bks = bksFor(name);
        const bks_veh: usize = if (bks) |b| b.veh else 0;
        const bks_dist: f64 = if (bks) |b| b.dist else 0;
        const veh_match = bks != null and res.vehicles == bks_veh;
        const gap = if (bks != null and bks_dist > 0) 100.0 * (dist - bks_dist) / bks_dist else 0;
        if (feasible and veh_match) {
            sum_gap += gap;
            ngap += 1;
        }
        std.debug.print("{s},{},{},{d:.2},{},{d:.2},{d:.3},{d:.0},{},{}\n", .{
            name, n, bks_veh, bks_dist, res.vehicles, dist, gap, ms, feasible, veh_match,
        });
    }
    if (ngap > 0) {
        std.debug.print("# mean dist gap over {} vehicle-matched: {d:.3}%\n", .{ ngap, sum_gap / @as(f64, @floatFromInt(ngap)) });
    }
}

const Bks = struct { veh: usize, dist: f64 };
fn bksFor(name: []const u8) ?Bks {
    const table = [_]struct { []const u8, usize, f64 }{
        .{ "c101", 10, 828.94 }, .{ "c201", 3, 591.56 },
        .{ "r101", 19, 1650.80 }, .{ "r201", 4, 1252.37 },
        .{ "rc101", 14, 1696.95 }, .{ "rc201", 4, 1406.94 },
    };
    for (table) |row| {
        if (eqIgnoreCase(row[0], name)) return .{ .veh = row[1], .dist = row[2] };
    }
    return null;
}
fn eqIgnoreCase(a: []const u8, b: []const u8) bool {
    if (a.len != b.len) return false;
    for (a, b) |ca, cb| {
        if (std.ascii.toLower(ca) != std.ascii.toLower(cb)) return false;
    }
    return true;
}

const Owned = struct {
    matrix: []u32,
    demand: []u32,
    ready: []u32,
    due: []u32,
    service: []u32,
    n: usize,
    capacity: u32,

    fn inst(self: *const Owned) commiv.vrptw.VrptwInstance {
        return .{
            .n = self.n,
            .matrix = self.matrix,
            .demand = self.demand,
            .capacity = self.capacity,
            .ready = self.ready,
            .due = self.due,
            .service = self.service,
        };
    }
    fn deinit(self: *Owned, allocator: std.mem.Allocator) void {
        allocator.free(self.matrix);
        allocator.free(self.demand);
        allocator.free(self.ready);
        allocator.free(self.due);
        allocator.free(self.service);
        self.* = undefined;
    }
};

/// Parse a Solomon VRPTW instance. Columns: CUST X Y DEMAND READY DUE SERVICE,
/// customer 0 = depot. Coordinates/times scaled by SCALE so travel time =
/// distance = round(euclid*SCALE), keeping the schedule in integer centi-units.
fn parseSolomon(allocator: std.mem.Allocator, bytes: []const u8) !Owned {
    var capacity: u32 = 0;
    var xs: std.ArrayList(f64) = .empty;
    defer xs.deinit(allocator);
    var ys: std.ArrayList(f64) = .empty;
    defer ys.deinit(allocator);
    var dem: std.ArrayList(u32) = .empty;
    defer dem.deinit(allocator);
    var rdy: std.ArrayList(u32) = .empty;
    defer rdy.deinit(allocator);
    var due: std.ArrayList(u32) = .empty;
    defer due.deinit(allocator);
    var svc: std.ArrayList(u32) = .empty;
    defer svc.deinit(allocator);

    var in_customers = false;
    var seen_capacity = false;
    var lines = std.mem.tokenizeAny(u8, bytes, "\r\n");
    while (lines.next()) |raw| {
        const line = std.mem.trim(u8, raw, " \t");
        if (line.len == 0) continue;
        if (!in_customers) {
            if (std.mem.indexOf(u8, line, "CUST NO.") != null) {
                in_customers = true;
                continue;
            }
            if (std.mem.startsWith(u8, line, "NUMBER")) continue; // header before cap line
            // the VEHICLE number/capacity numeric line: two ints
            if (!seen_capacity) {
                var t = std.mem.tokenizeAny(u8, line, " \t");
                const a = t.next() orelse continue;
                const b = t.next() orelse continue;
                if (t.next() != null) continue; // not the 2-column line
                const v0 = std.fmt.parseInt(u32, a, 10) catch continue;
                const cap = std.fmt.parseInt(u32, b, 10) catch continue;
                _ = v0;
                capacity = cap;
                seen_capacity = true;
            }
            continue;
        }
        // customer rows: cust x y demand ready due service
        var t = std.mem.tokenizeAny(u8, line, " \t");
        const id_s = t.next() orelse continue;
        _ = std.fmt.parseInt(usize, id_s, 10) catch continue;
        const x = try std.fmt.parseFloat(f64, t.next() orelse return error.BadRow);
        const y = try std.fmt.parseFloat(f64, t.next() orelse return error.BadRow);
        const d = try std.fmt.parseInt(u32, t.next() orelse return error.BadRow, 10);
        const r = try std.fmt.parseInt(u32, t.next() orelse return error.BadRow, 10);
        const dd = try std.fmt.parseInt(u32, t.next() orelse return error.BadRow, 10);
        const s = try std.fmt.parseInt(u32, t.next() orelse return error.BadRow, 10);
        try xs.append(allocator, x);
        try ys.append(allocator, y);
        try dem.append(allocator, d);
        try rdy.append(allocator, r * SCALE);
        try due.append(allocator, dd * SCALE);
        try svc.append(allocator, s * SCALE);
    }
    const dim = xs.items.len;
    if (dim < 2 or dim > 100_000) return error.BadInstance;
    // Checked multiply + cap, consistent with the other bench parsers, so a
    // pathological instance can't wrap dim*dim into an undersized buffer.
    const cells = std.math.mul(usize, dim, dim) catch return error.BadInstance;
    const matrix = try allocator.alloc(u32, cells);
    errdefer allocator.free(matrix);
    for (0..dim) |i| {
        for (0..dim) |j| {
            const dx = xs.items[i] - xs.items[j];
            const dy = ys.items[i] - ys.items[j];
            matrix[i * dim + j] = @intFromFloat(@round(@sqrt(dx * dx + dy * dy) * @as(f64, @floatFromInt(SCALE))));
        }
    }
    return .{
        .matrix = matrix,
        .demand = try dem.toOwnedSlice(allocator),
        .ready = try rdy.toOwnedSlice(allocator),
        .due = try due.toOwnedSlice(allocator),
        .service = try svc.toOwnedSlice(allocator),
        .n = dim - 1,
        .capacity = capacity,
    };
}

fn nanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    _ = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}
