const std = @import("std");

/// Geometric threshold-accepting schedule shared by the three SISR engines:
/// thresholds scale with a per-customer cost unit (each engine derives `unit`
/// from its own seed-distance definition), decaying t0 -> tf over `iters`
/// with a constant factor cf per iteration. Inlined at comptime — identical
/// operations in identical order to the per-engine copies this replaces.
pub const Schedule = struct { t0: f64, tf: f64, cf: f64, iters: usize };

pub fn geometric(unit: f64, t0_factor: f64, tf_factor: f64, iters_in: usize) Schedule {
    const t0 = @max(1e-9, t0_factor * unit);
    const tf = @max(1e-9, tf_factor * unit);
    const iters = @max(@as(usize, 1), iters_in);
    const cf = std.math.pow(f64, tf / t0, 1.0 / @as(f64, @floatFromInt(iters)));
    return .{ .t0 = t0, .tf = tf, .cf = cf, .iters = iters };
}
