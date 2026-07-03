const std = @import("std");
const commiv = @import("commiv");

// Synthetic VRPTW-SISR throughput + bit-identity probe. Deterministically builds
// a directed n=1000 instance (fixed PRNG seed, every customer constructed to be
// singleton-feasible by design — due[c] is derived FROM d(0,c)/ready[c] rather than
// checked after the fact, and due[0] is derived from the worst-case customer so the
// solver's singleton precheck can never reject it), runs solveVrptwSisr once at a
// fixed seed/iters, and prints final cost + a sequence-sensitive hash of the routes
// + wall time. Run this binary multiple times: cost and hash must be byte-identical
// across runs and across code changes that claim to be mechanical (no behavior
// change); wall time is the speed signal.
//
// Env: TP_N (default 1000), TP_ITERS (default 300000), TP_SEED (default 12345).

const GEN_SEED: u64 = 0xC0FFEE_1000;

pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const env = init.environ_map;
    const n = try std.fmt.parseInt(usize, env.get("TP_N") orelse "1000", 10);
    const iters = try std.fmt.parseInt(usize, env.get("TP_ITERS") orelse "300000", 10);
    const seed = try std.fmt.parseInt(u64, env.get("TP_SEED") orelse "12345", 10);
    const adjacent_gaps = std.mem.eql(u8, env.get("TP_ADJ_GAPS") orelse "0", "1");
    // TP_SCALE=100000 reproduces the pathological tight-window regime described
    // in the comment below (default 5000 keeps the canonical probe instance).
    const scale = try std.fmt.parseInt(u32, env.get("TP_SCALE") orelse "5000", 10);

    const dim = n + 1;
    const matrix = try allocator.alloc(u32, dim * dim);
    defer allocator.free(matrix);
    const demand = try allocator.alloc(u32, dim);
    defer allocator.free(demand);
    const ready = try allocator.alloc(u32, dim);
    defer allocator.free(ready);
    const due = try allocator.alloc(u32, dim);
    defer allocator.free(due);
    const service = try allocator.alloc(u32, dim);
    defer allocator.free(service);

    var prng = std.Random.DefaultPrng.init(GEN_SEED);
    const rng = prng.random();

    // Random coordinates in a scale x scale square, then a directed distance
    // matrix: base Euclidean distance plus an independent per-*ordered-pair*
    // integer perturbation, so d(a,b) != d(b,a) in general (real road matrices
    // are not symmetric and don't respect the triangle inequality; see the SISR
    // gapDelta comment for why insertion deltas are kept signed). At the default
    // scale 5000, time-window widths (3k-20k) are generous relative to typical
    // arc lengths (~avg 1300) and fleet size is capacity-driven (~30 routes).
    // TP_SCALE=100000 makes almost every window narrower than a single arc,
    // fragmenting into ~80-230 near-singleton routes out of 1000 customers —
    // the regime that exposed the string-removal feasibility bug (removing a
    // string can delay downstream arrivals when the bridging arc violates the
    // triangle inequality; fixed via SisrTw.removalSafe, regression-tested in
    // src/vrptw.zig "emits valid solutions in the tight-window curl regime").
    const xs = try allocator.alloc(f64, dim);
    defer allocator.free(xs);
    const ys = try allocator.alloc(f64, dim);
    defer allocator.free(ys);
    const fscale: f64 = @floatFromInt(scale);
    for (0..dim) |i| {
        xs[i] = rng.float(f64) * fscale;
        ys[i] = rng.float(f64) * fscale;
    }
    for (0..dim) |a| {
        for (0..dim) |b| {
            if (a == b) {
                matrix[a * dim + b] = 0;
                continue;
            }
            const dx = xs[a] - xs[b];
            const dy = ys[a] - ys[b];
            const base: i64 = @intFromFloat(@round(@sqrt(dx * dx + dy * dy)));
            const noise = rng.intRangeAtMost(i64, 0, @divTrunc(base, 5) + 1); // up to ~20% curl, one-sided per direction
            matrix[a * dim + b] = @intCast(@max(base + noise, 1));
        }
    }

    demand[0] = 0;
    ready[0] = 0;
    service[0] = 0;
    const capacity: u32 = 200;

    // Per-customer demand/time-window generation. due[c] is set FROM the actual
    // required arrival floor (max(d(0,c), ready[c])) so singleton feasibility
    // (customer c served alone, round trip from the depot) holds by construction;
    // due[0] is then raised to the worst-case (due[c] + service[c] + d(c,0)) over
    // all customers, again by construction rather than by rejection-sampling.
    var horizon_floor: u64 = 0;
    for (1..dim) |c| {
        demand[c] = rng.intRangeAtMost(u32, 1, 10);
        const fwd = matrix[0 * dim + c];
        const bwd = matrix[c * dim + 0];
        const r = rng.intRangeAtMost(u32, 0, 50_000);
        const width = rng.intRangeAtMost(u32, 3_000, 20_000);
        const floor_arrival = @max(fwd, r);
        const dd = floor_arrival + width;
        const svc = rng.intRangeAtMost(u32, 0, 50);
        ready[c] = r;
        due[c] = dd;
        service[c] = svc;
        const needed: u64 = @as(u64, dd) + svc + bwd;
        if (needed > horizon_floor) horizon_floor = needed;
    }
    due[0] = @intCast(horizon_floor + 10); // +10 margin, not load-bearing

    const inst = commiv.VrptwInstance{
        .n = n,
        .matrix = matrix,
        .demand = demand,
        .capacity = capacity,
        .ready = ready,
        .due = due,
        .service = service,
    };

    const t0 = nanos();
    var res = try commiv.solveVrptwSisr(allocator, inst, .{ .seed = seed }, .{ .iters = iters, .adjacent_gaps = adjacent_gaps });
    defer res.deinit();
    const ms = @as(f64, @floatFromInt(nanos() - t0)) / 1e6;

    const checked = commiv.internal.vrptw.validate(inst, res.routes);
    const feasible = checked != null and checked.? == res.total_cost;

    const hash = hashRoutes(res.routes);
    std.debug.print("n={},iters={},seed={},cost={},vehicles={},hash={x:0>16},ms={d:.1},feasible={}\n", .{
        n, iters, seed, res.total_cost, res.vehicles, hash, ms, feasible,
    });
}

/// FNV-1a 64-bit over the emitted routes: order- and sequence-sensitive (route
/// boundaries and within-route customer order both affect the hash), so any
/// reordering of routes or customers within a route changes it.
fn hashRoutes(routes: []const []const usize) u64 {
    var h: u64 = 0xcbf29ce484222325;
    const mix = struct {
        fn byte(hh: *u64, b: u8) void {
            hh.* ^= b;
            hh.* *%= 0x100000001b3;
        }
        fn int(hh: *u64, v: u64) void {
            var buf: [8]u8 = undefined;
            std.mem.writeInt(u64, &buf, v, .little);
            for (buf) |b| byte(hh, b);
        }
    };
    mix.int(&h, routes.len);
    for (routes) |r| {
        mix.int(&h, r.len);
        for (r) |c| mix.int(&h, @intCast(c));
    }
    return h;
}

fn nanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    _ = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}
