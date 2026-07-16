const std = @import("std");
const commiv = @import("commiv");

// ---------------------------------------------------------------------------
// PDPTW MONEY objective on a REAL directed OSRM road matrix (M3, real-data
// money proof). The academic money bench (examples/pdptwbench.zig, money mode)
// runs on Li & Lim instances whose matrix is round(euclid*1000) over synthetic
// Euclidean coords. This one runs the SAME money objective on the REAL directed
// road travel-time matrices in vendor/road/*.road (nyc/moscow/berlin).
//
// WHAT IS REAL vs SYNTHETIC (do not overclaim):
//   REAL   - the directed road travel-TIME matrix (OSRM, asymmetric), and the
//            route DURATIONS it induces (travel + service + waiting). This is
//            exactly what the money objective prices and what VROOM cannot
//            express. The PRICES below are real-world-sourced.
//   SYNTH  - the pickup/delivery PAIRING and the TIME WINDOWS. Road instances
//            are plain delivery points with no pairing or windows, so we pair
//            customers deterministically (seeded) and synthesize windows from a
//            provably-feasible reference schedule (or load <name>.tw if present).
//   The money conclusion rests on the REAL half: directed durations under real
//            asymmetric road times, priced at real rates.
//
// MATRIX UNIT: seconds. Confirmed against vendor/road/nyc-2000.tw, whose depot
// horizon is 32400 (= 9h) and per-stop service 300 (= 5min); the .road arcs
// (~600-1300 for NYC point pairs = 10-22 min drives) are consistent with that.
// So "distance" here IS directed travel time in seconds, and route duration is
// also in seconds. There is NO kilometre column in a duration matrix, so the
// dollar model prices per-vehicle + per-minute-of-duration only (no $/km term).
//
// MONEY MODEL (real-world-sourced, same as BENCHMARKS.md money bench):
//   $140 per vehicle  +  $0.50 per minute of route duration.
// The $0.50/km term of the academic model has no analogue on a time matrix and
// is deliberately dropped rather than faked. veh_penalty / time_penalty are the
// solver's internal objective weights (MR_VEH_PEN / MR_TIMEPEN); the reported
// USD figure is computed from the physical outputs with the prices above so it
// is independent of the internal penalty scale.
//
// Env knobs:
//   MR_DIR      road dir (default vendor/road)
//   MR_TW_DIR   .tw overlay dir (default vendor/road)
//   MR_FILE     instance name (default nyc-100)
//   MR_PAIRS    cap on n_pairs (default (DIM-1)/2, i.e. as many as fit)
//   MR_SERVICE  per-stop service seconds (default 300 = 5min)
//   MR_SLACK    window slack seconds added past the reference arrival (default 3600)
//   MR_TIMEPEN  solver time_penalty: internal cost per second of duration (default 30)
//   MR_VEH_PEN  solver veh_penalty: internal cost per vehicle (default 8400)
//   MR_TIME_MS  fleet-min wall budget in ms (default 5000)
//   MR_SEED     pairing + solver seed (default 1)
//   MR_USD_VEH  dollars per vehicle for the reported USD (default 140)
//   MR_USD_MIN  dollars per minute of duration for the reported USD (default 0.50)
//   MR_USE_TW   1 = load <name>.tw if present instead of synthesizing (default 0)
//   MR_STAGGER  >0 = staggered pickup openings, ready[p] in [0,MR_STAGGER] seeded,
//               ready[q]=ready[p]; makes waiting bite. 0 = ready=0 (default, bit-id)
//   MR_DUMP     path: write the synthesized instance as JSON before solving
//   MR_SOLVE    0 = with MR_DUMP set, exit after dumping (no solve). default 1
//
// The default MR_TIMEPEN/MR_VEH_PEN keep the internal money weights at real
// ratio: 30 per sec = $0.50/min at a $1 == 60 internal-unit scale, and
// 8400 = $140 at the same scale. They are only the SEARCH weights; the reported
// USD uses MR_USD_VEH / MR_USD_MIN directly.
//
// Prints a CSV header then one line per run:
//   instance,n_pairs,vehicles,dist_sec,dur_sec,service_sec,wait_sec,usd_total,ms
// or "<name>,FAIL_INFEASIBLE" if the solver output does not pass validatePdptw.

pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const env = init.environ_map;
    const dir = env.get("MR_DIR") orelse "vendor/road";
    const tw_dir = env.get("MR_TW_DIR") orelse "vendor/road";
    const name = env.get("MR_FILE") orelse "nyc-100";
    const service_s = try std.fmt.parseInt(u32, env.get("MR_SERVICE") orelse "300", 10);
    const slack_s = try std.fmt.parseInt(u32, env.get("MR_SLACK") orelse "3600", 10);
    const time_pen = try std.fmt.parseInt(u64, env.get("MR_TIMEPEN") orelse "30", 10);
    const veh_pen = try std.fmt.parseInt(u64, env.get("MR_VEH_PEN") orelse "8400", 10);
    const time_ms = try std.fmt.parseInt(u64, env.get("MR_TIME_MS") orelse "5000", 10);
    const seed = try std.fmt.parseInt(u64, env.get("MR_SEED") orelse "1", 10);
    const usd_veh = try std.fmt.parseFloat(f64, env.get("MR_USD_VEH") orelse "140");
    const usd_min = try std.fmt.parseFloat(f64, env.get("MR_USD_MIN") orelse "0.50");
    const use_tw = std.mem.eql(u8, env.get("MR_USE_TW") orelse "0", "1");
    const stagger_s = try std.fmt.parseInt(u32, env.get("MR_STAGGER") orelse "0", 10);
    const dump_path = env.get("MR_DUMP");
    const do_solve = !std.mem.eql(u8, env.get("MR_SOLVE") orelse "1", "0");

    // --- read the REAL directed road matrix ---
    var path_buf: [512]u8 = undefined;
    const path = try std.fmt.bufPrint(&path_buf, "{s}/{s}.road", .{ dir, name });
    const bytes = try std.Io.Dir.cwd().readFileAlloc(init.io, path, allocator, std.Io.Limit.limited(512 << 20));
    defer allocator.free(bytes);
    var road = try parseRoad(allocator, bytes);
    defer road.deinit(allocator);
    const DIM = road.dim;
    if (DIM < 3) return error.InstanceTooSmall;

    // --- decide instance size: dim(PdpInstance) = 2*n_pairs+1 must equal the
    //     compacted matrix dimension. Even DIM leaves a leftover customer we
    //     drop; MR_PAIRS caps it smaller for quick runs. ---
    const max_pairs = (DIM - 1) / 2;
    var n_pairs = max_pairs;
    if (env.get("MR_PAIRS")) |mp| {
        const req = try std.fmt.parseInt(usize, mp, 10);
        if (req > 0 and req < n_pairs) n_pairs = req;
    }
    const K = 2 * n_pairs + 1; // kept nodes: depot 0 + customers 1..K-1

    // --- compact the directed matrix to K*K (copy the top-left block; kept
    //     nodes are the original 0..K-1, so the road times stay UNTOUCHED) ---
    const matrix = try allocator.alloc(u32, K * K);
    defer allocator.free(matrix);
    for (0..K) |a| {
        for (0..K) |b| matrix[a * K + b] = road.matrix[a * DIM + b];
    }

    // --- deterministic seeded pairing of the 2*n_pairs kept customers ---
    var prng = std.Random.DefaultPrng.init(seed ^ 0x9E37_79B9_7F4A_7C15);
    const rng = prng.random();
    const custs = try allocator.alloc(usize, 2 * n_pairs);
    defer allocator.free(custs);
    for (0..2 * n_pairs) |i| custs[i] = i + 1;
    rng.shuffle(usize, custs);

    const pair_of = try allocator.alloc(usize, K);
    defer allocator.free(pair_of);
    const is_pickup = try allocator.alloc(bool, K);
    defer allocator.free(is_pickup);
    const demand_signed = try allocator.alloc(i64, K);
    defer allocator.free(demand_signed);
    const ready = try allocator.alloc(u32, K);
    defer allocator.free(ready);
    const due = try allocator.alloc(u32, K);
    defer allocator.free(due);
    const service = try allocator.alloc(u32, K);
    defer allocator.free(service);

    pair_of[0] = 0;
    is_pickup[0] = false;
    demand_signed[0] = 0;
    ready[0] = 0;
    service[0] = 0;

    for (0..n_pairs) |i| {
        const p = custs[2 * i]; // pickup
        const q = custs[2 * i + 1]; // its dropoff
        pair_of[p] = q;
        pair_of[q] = p;
        is_pickup[p] = true;
        is_pickup[q] = false;
        // pair quantity q_amt from the pickup node's real road demand (>=1);
        // dropoff mirrors it so the pair nets to zero load.
        var q_amt: i64 = @intCast(road.demand[p]);
        if (q_amt < 1) q_amt = 1;
        if (q_amt > road.capacity) q_amt = @intCast(@max(@as(u32, 1), road.capacity));
        demand_signed[p] = q_amt;
        demand_signed[q] = -q_amt;
        service[p] = service_s;
        service[q] = service_s;
    }

    // --- time windows ---
    if (use_tw) {
        try loadTw(init, allocator, tw_dir, name, K, ready, due, service);
    } else {
        // Synthesize windows from a provably-feasible reference schedule: each
        // pair on its own vehicle depot->p->q->depot is feasible by
        // construction, so the whole instance is feasible and the money search
        // consolidates from there. due = reference arrival + slack. Depot horizon
        // covers every reference return.
        //
        // MR_STAGGER > 0: staggered pickup openings so waiting bites during
        // consolidation. ready[p] drawn deterministically in [0, MR_STAGGER],
        // ready[q] = ready[p] (>= ready[p]). The reference schedule departs the
        // depot at 0 and WAITS at each stop until its window opens, so arrive =
        // max(ready, earliest-travel-arrival) and due = arrive + slack stays
        // feasible for any stagger. MR_STAGGER == 0 draws no RNG and is
        // bit-identical to the original ready=0 construction.
        var horizon: u64 = 0;
        for (0..n_pairs) |i| {
            const p = custs[2 * i];
            const q = custs[2 * i + 1];
            const rp: u32 = if (stagger_s > 0) rng.intRangeAtMost(u32, 0, stagger_s) else 0;
            const rq: u32 = rp; // ready[q] >= ready[p]
            const arrive_p: u64 = @max(@as(u64, rp), @as(u64, matrix[0 * K + p]));
            const depart_p: u64 = arrive_p + service[p];
            const arrive_q: u64 = @max(@as(u64, rq), depart_p + matrix[p * K + q]);
            const ret: u64 = arrive_q + service[q] + matrix[q * K + 0];
            ready[p] = rp;
            ready[q] = rq;
            due[p] = @intCast(arrive_p + slack_s);
            due[q] = @intCast(arrive_q + slack_s);
            if (ret > horizon) horizon = ret;
        }
        ready[0] = 0;
        due[0] = @intCast(horizon + slack_s);
    }

    // --- assert pairing/precedence well-formed before we solve ---
    std.debug.assert(pair_of[0] == 0 and !is_pickup[0]);
    for (1..K) |c| {
        std.debug.assert(pair_of[c] >= 1 and pair_of[c] < K);
        std.debug.assert(pair_of[pair_of[c]] == c);
        std.debug.assert(is_pickup[c] != is_pickup[pair_of[c]]);
        std.debug.assert(is_pickup[c] == (demand_signed[c] > 0));
    }

    // --- optional instance dump: the EXACT arrays the solver sees, as JSON, so
    //     a second engine (VROOM via tools/vroom_road_pdptw.py) gets a bit-for-bit
    //     identical instance. MR_SOLVE=0 exits here without burning solver time. ---
    if (dump_path) |dp| {
        try dumpInstance(init, allocator, dp, name, n_pairs, road.capacity, K, matrix, custs, demand_signed, ready, due, service);
    }
    if (!do_solve) return;

    const inst = commiv.PdpInstance{
        .n_pairs = n_pairs,
        .matrix = matrix,
        .capacity = @intCast(road.capacity),
        .pair_of = pair_of,
        .is_pickup = is_pickup,
        .demand_signed = demand_signed,
        .ready = ready,
        .due = due,
        .service = service,
    };

    // --- money-mode fleet-min solve, SINGLE THREAD ---
    const params = commiv.PdpSisrParams{
        .seed = seed,
        .iters = 1_000_000_000,
        .time_ms = time_ms,
        .veh_penalty = veh_pen,
        .time_penalty = time_pen,
    };
    const t0 = nanos();
    var res = try commiv.internal.pdptw_sisr.solvePdptwSisrFleetMin(allocator, inst, params, time_ms);
    defer res.deinit();
    const ms = (nanos() - t0) / 1_000_000;

    // --- feasibility gate: solver output must pass the independent oracle.
    //     validatePdptw returns the pure directed travel-time total (no
    //     penalties, no duration) = our real dist_sec. ---
    const rc = try allocator.alloc([]const usize, res.routes.len);
    defer allocator.free(rc);
    for (res.routes, 0..) |r, i| rc[i] = r;
    const dist_sec = commiv.validatePdptw(inst, rc) orelse {
        std.debug.print("instance,n_pairs,vehicles,dist_sec,dur_sec,service_sec,wait_sec,usd_total,ms\n", .{});
        std.debug.print("{s},FAIL_INFEASIBLE\n", .{name});
        return error.SolverOutputInfeasible;
    };

    // --- physical outputs: duration (travel+service+wait), service, wait ---
    var dur_sec: u64 = 0;
    var service_sec: u64 = 0;
    for (res.routes) |r| {
        dur_sec += commiv.internal.pdptw_sisr.routeDuration(inst, r);
        for (r) |c| service_sec += inst.service[c];
    }
    const wait_sec = dur_sec - dist_sec - service_sec;

    // --- real-price dollars from the physical outputs ---
    const usd_total = @as(f64, @floatFromInt(res.vehicles)) * usd_veh +
        (@as(f64, @floatFromInt(dur_sec)) / 60.0) * usd_min;

    std.debug.print("instance,n_pairs,vehicles,dist_sec,dur_sec,service_sec,wait_sec,usd_total,ms\n", .{});
    std.debug.print("{s},{d},{d},{d},{d},{d},{d},{d:.2},{d}\n", .{
        name, n_pairs, res.vehicles, dist_sec, dur_sec, service_sec, wait_sec, usd_total, ms,
    });
}

fn nanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    _ = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}

// --- instance dump (JSON) --------------------------------------------------
// Writes exactly the arrays the solver receives so both engines solve the same
// instance. Schema:
//   {"name","n_pairs","capacity","matrix":[[KxK directed seconds]],
//    "pairs":[{"pickup","delivery","amount"}], "ready":[K],"due":[K],"service":[K]}

fn appendUint(buf: *std.ArrayList(u8), allocator: std.mem.Allocator, v: anytype) !void {
    var tmp: [24]u8 = undefined;
    const s = std.fmt.bufPrint(&tmp, "{d}", .{v}) catch unreachable;
    try buf.appendSlice(allocator, s);
}

fn appendArr(buf: *std.ArrayList(u8), allocator: std.mem.Allocator, comptime T: type, arr: []const T) !void {
    try buf.append(allocator, '[');
    for (arr, 0..) |v, i| {
        if (i != 0) try buf.append(allocator, ',');
        try appendUint(buf, allocator, v);
    }
    try buf.append(allocator, ']');
}

fn dumpInstance(
    init: std.process.Init,
    allocator: std.mem.Allocator,
    path: []const u8,
    name: []const u8,
    n_pairs: usize,
    capacity: u32,
    K: usize,
    matrix: []const u32,
    custs: []const usize,
    demand_signed: []const i64,
    ready: []const u32,
    due: []const u32,
    service: []const u32,
) !void {
    var buf: std.ArrayList(u8) = .empty;
    defer buf.deinit(allocator);

    try buf.appendSlice(allocator, "{\"name\":\"");
    try buf.appendSlice(allocator, name);
    try buf.appendSlice(allocator, "\",\"n_pairs\":");
    try appendUint(&buf, allocator, n_pairs);
    try buf.appendSlice(allocator, ",\"capacity\":");
    try appendUint(&buf, allocator, capacity);

    try buf.appendSlice(allocator, ",\"matrix\":[");
    for (0..K) |a| {
        if (a != 0) try buf.append(allocator, ',');
        try buf.append(allocator, '[');
        for (0..K) |b| {
            if (b != 0) try buf.append(allocator, ',');
            try appendUint(&buf, allocator, matrix[a * K + b]);
        }
        try buf.append(allocator, ']');
    }
    try buf.append(allocator, ']');

    try buf.appendSlice(allocator, ",\"pairs\":[");
    for (0..n_pairs) |i| {
        if (i != 0) try buf.append(allocator, ',');
        const p = custs[2 * i];
        const q = custs[2 * i + 1];
        const amt: u64 = @intCast(demand_signed[p]);
        try buf.appendSlice(allocator, "{\"pickup\":");
        try appendUint(&buf, allocator, p);
        try buf.appendSlice(allocator, ",\"delivery\":");
        try appendUint(&buf, allocator, q);
        try buf.appendSlice(allocator, ",\"amount\":");
        try appendUint(&buf, allocator, amt);
        try buf.append(allocator, '}');
    }
    try buf.append(allocator, ']');

    try buf.appendSlice(allocator, ",\"ready\":");
    try appendArr(&buf, allocator, u32, ready[0..K]);
    try buf.appendSlice(allocator, ",\"due\":");
    try appendArr(&buf, allocator, u32, due[0..K]);
    try buf.appendSlice(allocator, ",\"service\":");
    try appendArr(&buf, allocator, u32, service[0..K]);
    try buf.appendSlice(allocator, "}\n");

    try std.Io.Dir.cwd().writeFile(init.io, .{ .sub_path = path, .data = buf.items });
}

/// Load a <name>.tw overlay (same format as examples/twroadbench.zig: first
/// line is dim, then per-node "ready due service"), into the first K nodes.
/// Note: with a synthesized pairing this may be INFEASIBLE; the feasibility
/// gate catches that. Only sound when the .tw dim matches K exactly.
fn loadTw(
    init: std.process.Init,
    allocator: std.mem.Allocator,
    tw_dir: []const u8,
    name: []const u8,
    K: usize,
    ready: []u32,
    due: []u32,
    service: []u32,
) !void {
    var tw_buf: [512]u8 = undefined;
    const tw_path = try std.fmt.bufPrint(&tw_buf, "{s}/{s}.tw", .{ tw_dir, name });
    const tw_bytes = try std.Io.Dir.cwd().readFileAlloc(init.io, tw_path, allocator, std.Io.Limit.limited(16 << 20));
    defer allocator.free(tw_bytes);
    var lines = std.mem.tokenizeScalar(u8, tw_bytes, '\n');
    const dline = lines.next() orelse return error.BadTw;
    var dtok = std.mem.tokenizeAny(u8, dline, " \t\r");
    const tw_dim = try std.fmt.parseInt(usize, dtok.next() orelse return error.BadTw, 10);
    if (tw_dim < K) return error.TwDimTooSmall;
    var i: usize = 0;
    while (lines.next()) |line| : (i += 1) {
        if (i >= K) break;
        var tok = std.mem.tokenizeAny(u8, line, " \t\r");
        ready[i] = try std.fmt.parseInt(u32, tok.next() orelse return error.BadTw, 10);
        due[i] = try std.fmt.parseInt(u32, tok.next() orelse return error.BadTw, 10);
        service[i] = try std.fmt.parseInt(u32, tok.next() orelse return error.BadTw, 10);
    }
    if (i < K) return error.BadTw;
}

// --- .road parser, identical logic to examples/twroadbench.zig parseRoad ---

const Road = struct {
    dim: usize,
    capacity: u32,
    demand: []u32,
    matrix: []u32,
    fn deinit(self: *Road, allocator: std.mem.Allocator) void {
        allocator.free(self.demand);
        allocator.free(self.matrix);
        self.* = undefined;
    }
};

fn parseRoad(allocator: std.mem.Allocator, bytes: []const u8) !Road {
    const Mode = enum { header, coords, matrix };
    var mode: Mode = .header;
    var dim: usize = 0;
    var capacity: u32 = 0;
    var demand: []u32 = &.{};
    var matrix: []u32 = &.{};
    errdefer if (demand.len > 0) allocator.free(demand);
    errdefer if (matrix.len > 0) allocator.free(matrix);
    var row: usize = 0;
    var lines = std.mem.tokenizeScalar(u8, bytes, '\n');
    while (lines.next()) |line| {
        var tok = std.mem.tokenizeAny(u8, line, " \t\r");
        const first = tok.next() orelse continue;
        if (std.mem.eql(u8, first, "NAME")) {
            continue;
        } else if (std.mem.eql(u8, first, "DIM")) {
            dim = try std.fmt.parseInt(usize, tok.next() orelse return error.BadFormat, 10);
            if (dim < 1 or dim > 100_000) return error.BadFormat;
            const cells = std.math.mul(usize, dim, dim) catch return error.BadFormat;
            demand = try allocator.alloc(u32, dim);
            @memset(demand, 0);
            matrix = try allocator.alloc(u32, cells);
            @memset(matrix, 0);
        } else if (std.mem.eql(u8, first, "CAPACITY")) {
            capacity = try std.fmt.parseInt(u32, tok.next() orelse return error.BadFormat, 10);
        } else if (std.mem.eql(u8, first, "COORDS")) {
            mode = .coords;
        } else if (std.mem.eql(u8, first, "MATRIX")) {
            mode = .matrix;
        } else switch (mode) {
            .coords => {
                const idx = try std.fmt.parseInt(usize, first, 10);
                _ = tok.next();
                _ = tok.next();
                const dem = try std.fmt.parseInt(u32, tok.next() orelse "0", 10);
                if (idx >= dim) return error.BadFormat;
                demand[idx] = dem;
            },
            .matrix => {
                if (row >= dim) return error.BadFormat;
                var col: usize = 0;
                var t = first;
                while (true) {
                    if (col >= dim) return error.BadFormat;
                    matrix[row * dim + col] = try std.fmt.parseInt(u32, t, 10);
                    col += 1;
                    t = tok.next() orelse break;
                }
                if (col != dim) return error.BadFormat;
                row += 1;
            },
            .header => {},
        }
    }
    if (dim == 0 or row != dim) return error.BadFormat;
    return .{ .dim = dim, .capacity = capacity, .demand = demand, .matrix = matrix };
}
