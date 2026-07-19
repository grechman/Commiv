const std = @import("std");
const builtin = @import("builtin");

/// Monotonic clock in nanoseconds.
pub fn nanos() u64 {
    // Linux: raw vDSO syscall, libc-free (the historic path, byte-identical).
    // Elsewhere (macOS wheels): libc clock_gettime — Zig always links
    // libSystem on macOS, so this is safe even in the "libc-free" library.
    if (builtin.os.tag == .linux) {
        var ts: std.os.linux.timespec = undefined;
        _ = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
        return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
    } else {
        var ts: std.c.timespec = undefined;
        _ = std.c.clock_gettime(.MONOTONIC, &ts);
        return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
    }
}
