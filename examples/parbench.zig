const std = @import("std");
const commiv = @import("commiv");

// Focused parallel-vs-serial benchmark for the README. Runs ONLY the headline
// config (alpha-nearness, dimension-scaled trials with stagnation extension --
// the same budget bench.zig's `alpha-w8-kick` mode uses) so it finishes in a
// couple of minutes, not the full CSV suite's half hour.
//
// For each fixture it runs:
//   serial   = solve()         at seed 12345                 (1 core, 1 seed)
//   parallel = solveParallel() best-of-K islands, seed 12345 (K cores, K seeds)
// and prints len / gap-to-optimum / wall-time for both, so the parallel column
// drops straight into the README beside the existing LKH numbers.
//
// Env:
//   BENCH_THREADS  island count (default 0 = auto = cpuCount-1, one core free)
// Pin with taskset so the machine stays usable, e.g.
//   BENCH_THREADS=3 taskset -c 0-2 nice -n 10 zig build parbench -Doptimize=ReleaseFast

const Fixture = struct {
    name: []const u8,
    path: []const u8,
    optimum: u64,
};

// n <= 1577; rl11849 / d18512 are deferred (big-runtime, separate session).
const fixtures = [_]Fixture{
    .{ .name = "berlin52", .path = "vendor/tsplib/berlin52.tsp", .optimum = 7542 },
    .{ .name = "eil76", .path = "vendor/tsplib/eil76.tsp", .optimum = 538 },
    .{ .name = "kroA100", .path = "vendor/tsplib/kroA100.tsp", .optimum = 21282 },
    .{ .name = "bier127", .path = "vendor/tsplib/bier127.tsp", .optimum = 118282 },
    .{ .name = "rat195", .path = "vendor/tsplib/rat195.tsp", .optimum = 2323 },
    .{ .name = "ts225", .path = "vendor/tsplib/ts225.tsp", .optimum = 126643 },
    .{ .name = "a280", .path = "vendor/tsplib/a280.tsp", .optimum = 2579 },
    .{ .name = "lin318", .path = "vendor/tsplib/lin318.tsp", .optimum = 42029 },
    .{ .name = "rd400", .path = "vendor/tsplib/rd400.tsp", .optimum = 15281 },
    .{ .name = "fl417", .path = "vendor/tsplib/fl417.tsp", .optimum = 11861 },
    .{ .name = "pcb442", .path = "vendor/tsplib/pcb442.tsp", .optimum = 50778 },
    .{ .name = "att532", .path = "vendor/tsplib/att532.tsp", .optimum = 27686 },
    .{ .name = "u574", .path = "vendor/tsplib/u574.tsp", .optimum = 36905 },
    .{ .name = "rat575", .path = "vendor/tsplib/rat575.tsp", .optimum = 6773 },
    .{ .name = "d657", .path = "vendor/tsplib/d657.tsp", .optimum = 48912 },
    .{ .name = "pr1002", .path = "vendor/tsplib/pr1002.tsp", .optimum = 259045 },
    .{ .name = "fl1577", .path = "vendor/tsplib/fl1577.tsp", .optimum = 22249 },
};

const seed: u64 = 12345;

pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;

    const threads = blk: {
        const s = init.environ_map.get("BENCH_THREADS") orelse "0";
        break :blk std.fmt.parseInt(usize, s, 10) catch 0;
    };
    const island_count = commiv.parallel.resolveThreadCount(threads);

    std.debug.print("# parallel benchmark: {} islands (split), seed={}\n", .{ island_count, seed });
    std.debug.print("# {s:<10} {s:>5} | {s:>9} {s:>8} {s:>9} | {s:>9} {s:>8} {s:>9}\n", .{
        "instance", "n", "ser_len", "ser_gap", "ser_ms", "par_len", "par_gap", "par_ms",
    });

    for (fixtures) |fixture| {
        const bytes = std.Io.Dir.cwd().readFileAlloc(init.io, fixture.path, allocator, .limited(16 * 1024 * 1024)) catch |err| switch (err) {
            error.FileNotFound => {
                std.debug.print("# missing fixture: {s}\n", .{fixture.path});
                continue;
            },
            else => return err,
        };
        defer allocator.free(bytes);

        var p = commiv.parseTsplib(allocator, bytes, .{ .max_dimension = 20_000, .max_matrix_weights = 25_000_000 }) catch |err| {
            std.debug.print("# failed fixture: {s}\n", .{fixture.name});
            return err;
        };
        defer p.deinit();

        const opts = headlineOptions(p.dimension);

        // Serial.
        const ser_start = monotonicNanos();
        var ser = try commiv.solve(allocator, &p, opts);
        const ser_ms = nanosToMs(monotonicNanos() - ser_start);
        defer ser.deinit();
        try p.validateTour(ser.tour);

        // Parallel.
        const par_start = monotonicNanos();
        var par = try commiv.solveParallel(allocator, &p, opts, .{ .threads = threads });
        const par_ms = nanosToMs(monotonicNanos() - par_start);
        defer par.deinit();
        try p.validateTour(par.tour);

        std.debug.print("  {s:<10} {d:>5} | {d:>9} {d:>7.3}% {d:>8.0} | {d:>9} {d:>7.3}% {d:>8.0}\n", .{
            fixture.name,
            p.dimension,
            ser.length,
            gap(ser.length, fixture.optimum),
            ser_ms,
            par.length,
            gap(par.length, fixture.optimum),
            par_ms,
        });
    }
}

fn headlineOptions(n: usize) commiv.SolveOptions {
    return .{
        .seed = seed,
        .budget = .{
            .trials = n,
            .trial_extension_factor = if (n >= 1000) @as(usize, 2) else @as(usize, 4),
            .max_passes = 64,
            // Default 16 MB budget: small n cached, large n on-the-fly (never a
            // multi-GB matrix -- which would also be replicated per island).
            .max_distance_cache_bytes = (commiv.SolveOptions.Budget{}).max_distance_cache_bytes,
        },
        .candidates = .{
            .candidate_count = if (n >= 1000) @as(usize, 5) else @as(usize, 8),
            .candidate_mode = .alpha_nearness,
        },
        .search = .{
            .enable_lk = true,
            .lk_max_depth = 5,
            .lk_backtrack_limit = 80_000,
        },
    };
}

fn gap(len: u64, optimum: u64) f64 {
    return 100.0 * (@as(f64, @floatFromInt(len)) - @as(f64, @floatFromInt(optimum))) / @as(f64, @floatFromInt(optimum));
}

fn nanosToMs(ns: u64) f64 {
    return @as(f64, @floatFromInt(ns)) / 1_000_000.0;
}

fn monotonicNanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    const rc = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    std.debug.assert(rc == 0);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}
