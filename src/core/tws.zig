// Time-Window Segment (Vidal / PyVRP): a constant-size summary of a contiguous
// node sequence that lets two segments be CONCATENATED in O(1), giving the merged
// segment's total time warp (0 == time-window-feasible) without rescheduling. This
// is what turns every local-search move from O(route length) into O(1): a candidate
// route is a concatenation of a few precomputed prefix/suffix segments + the moved
// nodes. Distance and load are tracked separately (both O(1) via deltas).
//
// Generic over the instance type: `Inst` must provide `service`, `ready`, `due`
// slices (index 0 = depot, due[0] = horizon). Comptime instantiation — each
// engine's Tws(Inst) inlines to exactly the code the per-engine copies compiled
// to before extraction; no vtables, no indirection.

pub fn Tws(comptime Inst: type) type {
    return struct {
        const Self = @This();

        dur: i64, // duration spanned (travel + service + forced waiting)
        tw: i64, // accumulated time warp (infeasibility); 0 == feasible
        early: i64, // earliest feasible start at the segment's first node
        late: i64, // latest feasible start at the segment's first node

        pub fn client(inst: Inst, c: usize) Self {
            return .{ .dur = @intCast(inst.service[c]), .tw = 0, .early = @intCast(inst.ready[c]), .late = @intCast(inst.due[c]) };
        }
        pub fn depotNode(inst: Inst) Self {
            return .{ .dur = 0, .tw = 0, .early = 0, .late = @intCast(inst.due[0]) };
        }

        /// Feasibility-only fast path for prefix + one node + suffix. For
        /// warp-free segments, `early + dur` is the earliest completion of the
        /// prefix and `late` is the latest feasible start of the suffix. This is
        /// algebraically identical to checking the two merges' final `tw`, but
        /// avoids constructing either summary when their duration is not needed.
        pub inline fn insertFeasible(prefix: Self, edge_before: i64, node: Self, edge_after: i64, suffix: Self) bool {
            if (prefix.tw != 0 or node.tw != 0 or suffix.tw != 0) return false;
            if (prefix.early > prefix.late or node.early > node.late or suffix.early > suffix.late) return false;
            const start = @max(prefix.early + prefix.dur + edge_before, node.early);
            return start <= node.late and start + node.dur + edge_after <= suffix.late;
        }

        /// Merge `left` then `right`, connected by an edge of travel time `edge`.
        pub fn merge(left: Self, edge: i64, right: Self) Self {
            const delta = left.dur - left.tw + edge;
            const d_wait = @max(right.early - delta - left.late, 0);
            const d_tw = @max(left.early + delta - right.late, 0);
            return .{
                .dur = left.dur + right.dur + edge + d_wait,
                .tw = left.tw + right.tw + d_tw,
                .early = @max(right.early - delta, left.early) - d_wait,
                .late = @min(right.late - delta, left.late) + d_tw,
            };
        }
    };
}
