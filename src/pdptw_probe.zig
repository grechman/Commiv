const std = @import("std");
const pdptw = @import("pdptw.zig");

// Optimality-gap probe for the PDPTW baseline: solvePdptw vs bruteForceOptimum
// on the deterministic random-instance family. Not part of the library or the
// test suite (kept compiling via the refAllDecls test below). Reproduce the
// reported table with:
//   zig run -OReleaseFast src/pdptw_probe.zig 2>&1
pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;

    inline for ([_]usize{ 3, 4 }) |np| {
        const seeds: u64 = if (np == 3) 200 else 60;
        for ([_]usize{ 1, 8, 16, 64 }) |restarts| {
            var matches: usize = 0;
            var gap_sum: f64 = 0;
            var veh_sum: usize = 0;
            var seed: u64 = 1;
            while (seed <= seeds) : (seed += 1) {
                var ri = try pdptw.randomInstance(allocator, np, seed, true);
                defer ri.deinit(allocator);
                const inst = ri.inst();
                const opt = try pdptw.bruteForceOptimum(allocator, inst);
                var res = try pdptw.solvePdptw(allocator, inst, .{ .restarts = restarts });
                defer res.deinit();
                if (res.total_cost == opt) matches += 1;
                gap_sum += (@as(f64, @floatFromInt(res.total_cost)) - @as(f64, @floatFromInt(opt))) / @as(f64, @floatFromInt(opt)) * 100.0;
                veh_sum += res.vehicles;
            }
            const ft: f64 = @floatFromInt(seeds);
            std.debug.print("np={d} R={d:>2}: match {d:>3}/{d} ({d:5.1}%)  mean gap {d:6.3}%  mean veh {d:.2}\n", .{ np, restarts, matches, seeds, 100.0 * @as(f64, @floatFromInt(matches)) / ft, gap_sum / ft, @as(f64, @floatFromInt(veh_sum)) / ft });
        }
    }
}

test {
    // keeps the probe compiling whenever pdptw.zig's tests build
    std.testing.refAllDecls(@This());
}
