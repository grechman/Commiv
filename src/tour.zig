const std = @import("std");

pub const TourEdge = struct {
    a: usize,
    b: usize,
};

pub const MovePlan = struct {
    removed: []const TourEdge,
    added: []const TourEdge,
    component_count: usize = 0,
    smallest_component_size: usize = 0,

    pub fn init(removed: []const TourEdge, added: []const TourEdge) MovePlan {
        return .{ .removed = removed, .added = added };
    }

    pub fn validate(
        self: *MovePlan,
        view: *const TourView,
        degree_delta: []i8,
        neighbor0: []usize,
        neighbor1: []usize,
        component: []usize,
        component_size: []usize,
        seen: []bool,
    ) bool {
        const n = view.len();
        if (self.removed.len == 0 or self.removed.len != self.added.len) return false;
        @memset(degree_delta, 0);
        @memset(component, std.math.maxInt(usize));
        @memset(component_size, 0);

        for (self.removed, 0..) |edge, i| {
            if (!validEdge(edge, n)) return false;
            if (!view.isTourEdge(edge.a, edge.b)) return false;
            if (tourEdgeInSlice(edge, self.removed[0..i])) return false;
            degree_delta[edge.a] -= 1;
            degree_delta[edge.b] -= 1;
        }

        for (self.added, 0..) |edge, i| {
            if (!validEdge(edge, n)) return false;
            if (view.isTourEdge(edge.a, edge.b)) return false;
            if (tourEdgeInSlice(edge, self.added[0..i])) return false;
            if (tourEdgeInSlice(edge, self.removed)) return false;
            degree_delta[edge.a] += 1;
            degree_delta[edge.b] += 1;
        }

        for (degree_delta) |delta| {
            if (delta != 0) return false;
        }
        if (!self.analyzeComponents(view, neighbor0, neighbor1, component, component_size, seen)) return false;
        return true;
    }

    pub fn analyzeComponents(
        self: *MovePlan,
        view: *const TourView,
        neighbor0: []usize,
        neighbor1: []usize,
        component: []usize,
        component_size: []usize,
        seen: []bool,
    ) bool {
        const n = view.len();
        for (0..n) |node| {
            neighbor0[node] = view.prev(node);
            neighbor1[node] = view.next(node);
        }
        for (self.removed) |edge| {
            if (!removePlanEdge(edge, neighbor0, neighbor1)) return false;
        }
        for (self.added) |edge| {
            if (!addPlanEdge(edge, neighbor0, neighbor1)) return false;
        }

        @memset(seen, false);
        self.component_count = 0;
        self.smallest_component_size = std.math.maxInt(usize);
        for (0..n) |start| {
            if (seen[start]) continue;
            var current = start;
            var previous: usize = std.math.maxInt(usize);
            var size: usize = 0;
            while (true) {
                if (current >= n) return false;
                if (seen[current]) {
                    if (current != start) return false;
                    break;
                }
                seen[current] = true;
                component[current] = self.component_count;
                size += 1;
                const a = neighbor0[current];
                const b = neighbor1[current];
                if (a == std.math.maxInt(usize) or b == std.math.maxInt(usize)) return false;
                const next_node = if (previous == std.math.maxInt(usize) or previous == b) a else b;
                previous = current;
                current = next_node;
            }
            component_size[self.component_count] = size;
            self.smallest_component_size = @min(self.smallest_component_size, size);
            self.component_count += 1;
        }
        return self.component_count > 0;
    }

    pub fn removePlanEdge(edge: TourEdge, neighbor0: []usize, neighbor1: []usize) bool {
        return removePlanNeighbor(edge.a, edge.b, neighbor0, neighbor1) and removePlanNeighbor(edge.b, edge.a, neighbor0, neighbor1);
    }

    pub fn removePlanNeighbor(a: usize, b: usize, neighbor0: []usize, neighbor1: []usize) bool {
        if (neighbor0[a] == b) {
            neighbor0[a] = std.math.maxInt(usize);
            return true;
        }
        if (neighbor1[a] == b) {
            neighbor1[a] = std.math.maxInt(usize);
            return true;
        }
        return false;
    }

    pub fn addPlanEdge(edge: TourEdge, neighbor0: []usize, neighbor1: []usize) bool {
        if (edge.a == edge.b) return false;
        if (neighbor0[edge.a] == edge.b or neighbor1[edge.a] == edge.b) return false;
        if (neighbor0[edge.b] == edge.a or neighbor1[edge.b] == edge.a) return false;
        return addPlanNeighbor(edge.a, edge.b, neighbor0, neighbor1) and addPlanNeighbor(edge.b, edge.a, neighbor0, neighbor1);
    }

    pub fn addPlanNeighbor(a: usize, b: usize, neighbor0: []usize, neighbor1: []usize) bool {
        if (neighbor0[a] == std.math.maxInt(usize)) {
            neighbor0[a] = b;
            return true;
        }
        if (neighbor1[a] == std.math.maxInt(usize)) {
            neighbor1[a] = b;
            return true;
        }
        return false;
    }

    pub fn validEdge(edge: TourEdge, n: usize) bool {
        return edge.a < n and edge.b < n and edge.a != edge.b;
    }
};

pub fn tourEdgeInSlice(edge: TourEdge, edges: []const TourEdge) bool {
    for (edges) |existing| {
        if (sameUndirectedEdge(edge.a, edge.b, existing.a, existing.b)) return true;
    }
    return false;
}

pub fn removeTourEdgeFromSlice(edges: []const TourEdge, edge: TourEdge) ?usize {
    for (edges, 0..) |existing, idx| {
        if (sameUndirectedEdge(edge.a, edge.b, existing.a, existing.b)) return idx;
    }
    return null;
}

// Flat array tour representation: the tour order plus a position index and a
// cached next/prev adjacency. This is the solver's only tour ADT. (A two-level
// segment variant was prototyped behind this same interface for R1 but never
// paid off — see plans/commiv/14.md — and was removed; if revived it slots back
// in as an alternate backend exposing this exact method set.)
pub const TourView = struct {
    tour: []usize,
    pos: []usize,
    next_nodes: []usize,
    prev_nodes: []usize,
    scratch_neighbor0: []usize,
    scratch_neighbor1: []usize,
    scratch_seen: []bool,

    pub fn initFlat(
        tour: []usize,
        pos: []usize,
        next_nodes: []usize,
        prev_nodes: []usize,
        scratch_neighbor0: []usize,
        scratch_neighbor1: []usize,
        scratch_seen: []bool,
    ) TourView {
        return .{
            .tour = tour,
            .pos = pos,
            .next_nodes = next_nodes,
            .prev_nodes = prev_nodes,
            .scratch_neighbor0 = scratch_neighbor0,
            .scratch_neighbor1 = scratch_neighbor1,
            .scratch_seen = scratch_seen,
        };
    }

    pub fn len(self: *const TourView) usize {
        return self.tour.len;
    }

    pub fn isTourEdge(self: *const TourView, a: usize, b: usize) bool {
        return self.next(a) == b or self.prev(a) == b;
    }

    pub fn rebuild(self: *TourView) void {
        const n = self.tour.len;
        for (self.tour, 0..) |node, idx| {
            self.pos[node] = idx;
            self.prev_nodes[node] = self.tour[(idx + n - 1) % n];
            self.next_nodes[node] = self.tour[(idx + 1) % n];
        }
    }

    pub fn next(self: *const TourView, node: usize) usize {
        return self.next_nodes[node];
    }

    pub fn prev(self: *const TourView, node: usize) usize {
        return self.prev_nodes[node];
    }

    pub fn between(self: *const TourView, a: usize, b: usize, c: usize) bool {
        if (a == b or b == c) return false;
        const pa = self.pos[a];
        const pb = self.pos[b];
        const pc = self.pos[c];
        if (pa <= pc) return pa < pb and pb < pc;
        return pb > pa or pb < pc;
    }

    pub fn flipPath(self: *TourView, first_node: usize, last_node: usize) void {
        var first = self.pos[first_node];
        var last = self.pos[last_node];
        if (first > last) std.mem.swap(usize, &first, &last);
        std.mem.reverse(usize, self.tour[first .. last + 1]);
        for (first..last + 1) |idx| {
            self.pos[self.tour[idx]] = idx;
        }
        self.rebuild();
    }

    pub fn applyEdges(self: *TourView, removed: []const TourEdge, added: []const TourEdge) bool {
        const n = self.tour.len;
        for (0..n) |node| {
            self.scratch_neighbor0[node] = self.prev_nodes[node];
            self.scratch_neighbor1[node] = self.next_nodes[node];
        }

        for (removed) |edge| {
            if (!self.removeScratchEdge(edge.a, edge.b)) return false;
        }
        for (added) |edge| {
            if (!self.addScratchEdge(edge.a, edge.b)) return false;
        }

        @memset(self.scratch_seen, false);
        const start = self.tour[0];
        var previous: usize = std.math.maxInt(usize);
        var current = start;
        for (0..n) |idx| {
            if (self.scratch_seen[current]) return false;
            self.scratch_seen[current] = true;
            self.tour[idx] = current;
            const a = self.scratch_neighbor0[current];
            const b = self.scratch_neighbor1[current];
            if (a == std.math.maxInt(usize) or b == std.math.maxInt(usize)) return false;
            const next_node = if (idx == 0)
                self.preferredFirstNeighbor(start, a, b)
            else if (a == previous)
                b
            else if (b == previous)
                a
            else
                return false;
            previous = current;
            current = next_node;
        }
        if (current != start) return false;
        self.rebuild();
        return true;
    }

    pub fn materialize(self: *const TourView, out: []usize) void {
        std.debug.assert(out.len == self.tour.len);
        @memcpy(out, self.tour);
    }

    pub fn preferredFirstNeighbor(self: *const TourView, start: usize, a: usize, b: usize) usize {
        if (self.next_nodes[start] == a) return a;
        if (self.next_nodes[start] == b) return b;
        return @min(a, b);
    }

    pub fn removeScratchEdge(self: *TourView, a: usize, b: usize) bool {
        return self.removeScratchNeighbor(a, b) and self.removeScratchNeighbor(b, a);
    }

    pub fn removeScratchNeighbor(self: *TourView, a: usize, b: usize) bool {
        if (self.scratch_neighbor0[a] == b) {
            self.scratch_neighbor0[a] = std.math.maxInt(usize);
            return true;
        }
        if (self.scratch_neighbor1[a] == b) {
            self.scratch_neighbor1[a] = std.math.maxInt(usize);
            return true;
        }
        return false;
    }

    pub fn addScratchEdge(self: *TourView, a: usize, b: usize) bool {
        if (a == b) return false;
        if (self.scratch_neighbor0[a] == b or self.scratch_neighbor1[a] == b) return false;
        if (self.scratch_neighbor0[b] == a or self.scratch_neighbor1[b] == a) return false;
        return self.addScratchNeighbor(a, b) and self.addScratchNeighbor(b, a);
    }

    pub fn addScratchNeighbor(self: *TourView, a: usize, b: usize) bool {
        if (self.scratch_neighbor0[a] == std.math.maxInt(usize)) {
            self.scratch_neighbor0[a] = b;
            return true;
        }
        if (self.scratch_neighbor1[a] == std.math.maxInt(usize)) {
            self.scratch_neighbor1[a] = b;
            return true;
        }
        return false;
    }
};

pub fn sameUndirectedEdge(a: usize, b: usize, c: usize, d: usize) bool {
    return (a == c and b == d) or (a == d and b == c);
}

// --- Differential tour harness (architecture M4) -----------------------------
// Validates TourView against an independent ground-truth array over random
// tours x random reversals, asserting next/prev/between/materialize agree after
// every move. Against the flat array impl this is a near-tautology ("green at a
// no-op"), but it is the reusable oracle that would turn any future alternate
// tour backend (e.g. a two-level list) from "hope it's bit-identical" into
// "proven on ~10^6 random ops": drop the new rep in as a second case and it
// must match this same reference.
const DiffOracle = struct {
    ref: []usize, // ground-truth tour order
    pos: []usize, // ref[pos[node]] == node, kept in sync

    fn syncPos(self: *DiffOracle) void {
        for (self.ref, 0..) |node, i| self.pos[node] = i;
    }
    fn next(self: *const DiffOracle, node: usize) usize {
        const n = self.ref.len;
        return self.ref[(self.pos[node] + 1) % n];
    }
    fn prev(self: *const DiffOracle, node: usize) usize {
        const n = self.ref.len;
        return self.ref[(self.pos[node] + n - 1) % n];
    }
    // Forward walk from a: reach b before c => true. Independent of any position
    // formula (the point of the cross-check), so kept to small n for cost.
    fn betweenWalk(self: *const DiffOracle, a: usize, b: usize, c: usize) bool {
        if (a == b or b == c or a == c) return false;
        var cur = self.next(a);
        while (cur != a) : (cur = self.next(cur)) {
            if (cur == b) return true;
            if (cur == c) return false;
        }
        return false;
    }
    fn reverse(self: *DiffOracle, i: usize, j: usize) void {
        std.mem.reverse(usize, self.ref[i .. j + 1]);
        self.syncPos();
    }
};

fn diffExpectAgreement(view: *const TourView, oracle: *const DiffOracle, mat: []usize, rng: std.Random) !void {
    const n = oracle.ref.len;
    for (0..n) |node| {
        try std.testing.expectEqual(oracle.next(node), view.next(node));
        try std.testing.expectEqual(oracle.prev(node), view.prev(node));
    }
    // Order is fully pinned by next/prev; materialize must reproduce ref exactly.
    view.materialize(mat);
    try std.testing.expectEqualSlices(usize, oracle.ref, mat);
    // Independent betweenness cross-check (forward walk), small n only.
    if (n >= 3 and n <= 64) {
        for (0..8) |_| {
            const a = rng.uintLessThan(usize, n);
            const b = rng.uintLessThan(usize, n);
            const c = rng.uintLessThan(usize, n);
            try std.testing.expectEqual(oracle.betweenWalk(a, b, c), view.between(a, b, c));
        }
    }
}

test "differential tour harness: flat view matches array oracle over random reversals" {
    const allocator = std.testing.allocator;
    var prng = std.Random.DefaultPrng.init(0x7f4a7c15d3e91b02);
    const rng = prng.random();
    const sizes = [_]usize{ 4, 7, 16, 63, 256, 511 };
    const tours_per_size = 12;
    const moves_per_tour = 80;

    for (sizes) |n| {
        const tour = try allocator.alloc(usize, n);
        defer allocator.free(tour);
        const pos = try allocator.alloc(usize, n);
        defer allocator.free(pos);
        const next_nodes = try allocator.alloc(usize, n);
        defer allocator.free(next_nodes);
        const prev_nodes = try allocator.alloc(usize, n);
        defer allocator.free(prev_nodes);
        const sn0 = try allocator.alloc(usize, n);
        defer allocator.free(sn0);
        const sn1 = try allocator.alloc(usize, n);
        defer allocator.free(sn1);
        const seen = try allocator.alloc(bool, n);
        defer allocator.free(seen);
        const ref = try allocator.alloc(usize, n);
        defer allocator.free(ref);
        const ref_pos = try allocator.alloc(usize, n);
        defer allocator.free(ref_pos);
        const mat = try allocator.alloc(usize, n);
        defer allocator.free(mat);

        for (0..tours_per_size) |_| {
            for (0..n) |i| tour[i] = i;
            rng.shuffle(usize, tour);
            @memcpy(ref, tour);
            var view = TourView.initFlat(tour, pos, next_nodes, prev_nodes, sn0, sn1, seen);
            view.rebuild();
            var oracle = DiffOracle{ .ref = ref, .pos = ref_pos };
            oracle.syncPos();
            try diffExpectAgreement(&view, &oracle, mat, rng);

            for (0..moves_per_tour) |_| {
                var i = rng.uintLessThan(usize, n);
                var j = rng.uintLessThan(usize, n);
                if (i > j) std.mem.swap(usize, &i, &j);
                const first_node = oracle.ref[i];
                const last_node = oracle.ref[j];
                oracle.reverse(i, j);
                view.flipPath(first_node, last_node);
                try diffExpectAgreement(&view, &oracle, mat, rng);
            }
        }
    }
}
