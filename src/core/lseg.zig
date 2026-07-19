/// Load-segment algebra: O(1)-concat capacity primitive. `lo`/`hi` are the
/// min/max prefix sums over the segment, including the empty prefix (0).
/// PDPTW-only today (pickup/delivery signed demands); generic over the
/// instance type so future engines with signed loads can share it. `Inst`
/// must provide `demand_signed`. Comptime-instantiated — inlines to exactly
/// the code the pdptw.zig copy compiled to.
pub fn Lseg(comptime Inst: type) type {
    return struct {
        const Self = @This();

        delta: i64, // net load change over the segment
        lo: i64, // min prefix sum (<= 0)
        hi: i64, // max prefix sum (>= 0)

        pub fn empty() Self {
            return .{ .delta = 0, .lo = 0, .hi = 0 };
        }

        pub fn node(inst: Inst, c: usize) Self {
            const q = inst.demand_signed[c];
            return .{ .delta = q, .lo = @min(0, q), .hi = @max(0, q) };
        }

        /// Concatenate `a` then `b`.
        pub fn merge(a: Self, b: Self) Self {
            return .{
                .delta = a.delta + b.delta,
                .lo = @min(a.lo, a.delta + b.lo),
                .hi = @max(a.hi, a.delta + b.hi),
            };
        }

        /// Feasible from a starting load of 0 under `capacity`.
        pub fn feasible(self: Self, capacity: i64) bool {
            return self.lo >= 0 and self.hi <= capacity;
        }
    };
}
