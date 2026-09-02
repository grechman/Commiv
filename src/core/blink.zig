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
