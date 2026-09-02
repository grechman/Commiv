const std = @import("std");

pub fn draw(prng: *std.Random.DefaultPrng) f64 {
    const rand = prng.next();
    var rand_lz: u64 = @clz(rand);
    if (rand_lz >= 12) {
        rand_lz = 12;
        while (true) {
            const addl_rand_lz: u64 = @clz(prng.next());
            rand_lz += addl_rand_lz;
            if (addl_rand_lz != 64) break;
            if (rand_lz >= 1022) {
                rand_lz = 1022;
                break;
            }
        }
    }
    const mantissa = rand & 0xFFFFFFFFFFFFF;
    const exponent = (1022 - rand_lz) << 52;
    return @bitCast(exponent | mantissa);
}

pub fn hash(seed: u64, gen: u64, ri: usize, p: usize, a: usize, b: usize) f64 {
    const w: u64 = (@as(u64, @intCast(ri)) << 48) ^ (@as(u64, @intCast(p)) << 32) ^ (@as(u64, @intCast(a)) << 16) ^ @as(u64, @intCast(b));
    var x = (seed ^ gen) *% 0x9E3779B97F4A7C15;
    x ^= w *% 0xD1B54A32D192ED03;
    x ^= x >> 30;
    x *%= 0xBF58476D1CE4E5B9;
    x ^= x >> 27;
    x *%= 0x94D049BB133111EB;
    x ^= x >> 31;
    return @as(f64, @floatFromInt(x >> 11)) * 0x1.0p-53;
}

test "hash is uniform in [0,1) and distinct per site" {
    var n_low: usize = 0;
    var prev: f64 = -1;
    var same: usize = 0;
    for (0..1_000_000) |i| {
        const v = hash(1, i, 3, 7, 5, 9);
        try std.testing.expect(v >= 0 and v < 1);
        if (v < 0.01) n_low += 1;
        if (v == prev) same += 1;
        prev = v;
    }
    try std.testing.expect(n_low > 9_000 and n_low < 11_000);
    try std.testing.expect(same == 0);
    try std.testing.expect(hash(1, 1, 1, 1, 1, 2) != hash(1, 1, 1, 1, 2, 1));
}

test "draw reproduces std.Random.float(f64) draw for draw" {
    var a = std.Random.DefaultPrng.init(0x50_44_50_7457);
    var b = std.Random.DefaultPrng.init(0x50_44_50_7457);
    const rb = b.random();
    for (0..2_000_000) |_| try std.testing.expectEqual(rb.float(f64), draw(&a));
    var c = std.Random.DefaultPrng.init(0);
    var d = std.Random.DefaultPrng.init(0);
    const rd = d.random();
    for (0..2_000_000) |_| try std.testing.expectEqual(rd.float(f64), draw(&c));
}
