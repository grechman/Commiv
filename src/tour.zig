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

pub const FlatTourView = struct {
    tour: []usize,
    pos: []usize,
    next_nodes: []usize,
    prev_nodes: []usize,
    scratch_neighbor0: []usize,
    scratch_neighbor1: []usize,
    scratch_seen: []bool,

    pub fn rebuild(self: *FlatTourView) void {
        const n = self.tour.len;
        for (self.tour, 0..) |node, idx| {
            self.pos[node] = idx;
            self.prev_nodes[node] = self.tour[(idx + n - 1) % n];
            self.next_nodes[node] = self.tour[(idx + 1) % n];
        }
    }

    pub fn next(self: *const FlatTourView, node: usize) usize {
        return self.next_nodes[node];
    }

    pub fn prev(self: *const FlatTourView, node: usize) usize {
        return self.prev_nodes[node];
    }

    pub fn between(self: *const FlatTourView, a: usize, b: usize, c: usize) bool {
        if (a == b or b == c) return false;
        const pa = self.pos[a];
        const pb = self.pos[b];
        const pc = self.pos[c];
        if (pa <= pc) return pa < pb and pb < pc;
        return pb > pa or pb < pc;
    }

    pub fn flipPath(self: *FlatTourView, first_node: usize, last_node: usize) void {
        var first = self.pos[first_node];
        var last = self.pos[last_node];
        if (first > last) std.mem.swap(usize, &first, &last);
        std.mem.reverse(usize, self.tour[first .. last + 1]);
        for (first..last + 1) |idx| {
            self.pos[self.tour[idx]] = idx;
        }
        self.rebuild();
    }

    pub fn applyEdges(self: *FlatTourView, removed: []const TourEdge, added: []const TourEdge) bool {
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

    pub fn materialize(self: *const FlatTourView, out: []usize) void {
        std.debug.assert(out.len == self.tour.len);
        @memcpy(out, self.tour);
    }

    pub fn preferredFirstNeighbor(self: *const FlatTourView, start: usize, a: usize, b: usize) usize {
        if (self.next_nodes[start] == a) return a;
        if (self.next_nodes[start] == b) return b;
        return @min(a, b);
    }

    pub fn removeScratchEdge(self: *FlatTourView, a: usize, b: usize) bool {
        return self.removeScratchNeighbor(a, b) and self.removeScratchNeighbor(b, a);
    }

    pub fn removeScratchNeighbor(self: *FlatTourView, a: usize, b: usize) bool {
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

    pub fn addScratchEdge(self: *FlatTourView, a: usize, b: usize) bool {
        if (a == b) return false;
        if (self.scratch_neighbor0[a] == b or self.scratch_neighbor1[a] == b) return false;
        if (self.scratch_neighbor0[b] == a or self.scratch_neighbor1[b] == a) return false;
        return self.addScratchNeighbor(a, b) and self.addScratchNeighbor(b, a);
    }

    pub fn addScratchNeighbor(self: *FlatTourView, a: usize, b: usize) bool {
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

pub const SegmentTourView = struct {
    flat: FlatTourView,
    segment_of_node: []usize,
    rank_in_segment: []usize,
    segment_start: []usize,
    segment_len: []usize,
    segment_reversed: []bool,
    target_segment_size: usize,
    segment_count: usize = 0,

    pub fn rebuild(self: *SegmentTourView) void {
        self.flat.rebuild();
        self.rebuildSegments();
    }

    pub fn next(self: *const SegmentTourView, node: usize) usize {
        return self.flat.next(node);
    }

    pub fn prev(self: *const SegmentTourView, node: usize) usize {
        return self.flat.prev(node);
    }

    pub fn between(self: *const SegmentTourView, a: usize, b: usize, c: usize) bool {
        return self.flat.between(a, b, c);
    }

    pub fn flipPath(self: *SegmentTourView, first_node: usize, last_node: usize) void {
        self.flat.flipPath(first_node, last_node);
        self.rebuildSegments();
    }

    pub fn applyEdges(self: *SegmentTourView, removed: []const TourEdge, added: []const TourEdge) bool {
        if (!self.flat.applyEdges(removed, added)) return false;
        self.rebuildSegments();
        return true;
    }

    pub fn materialize(self: *const SegmentTourView, out: []usize) void {
        self.flat.materialize(out);
    }

    pub fn rebuildSegments(self: *SegmentTourView) void {
        // Roadmap item 8 / per-move-rebuild kill: the two-level segment structure
        // (segment_of_node/rank_in_segment/segment_*) is NEVER read by solver
        // logic — between()/next()/prev() all use pos[] and the flat next/prev
        // arrays. Its only reader is debugSegmentMatchesFlatMaterialization, which
        // runs only under runtime_safety. So in release it is pure O(n)/move
        // waste; skip it. Debug builds still maintain + validate it. Bit-identical
        // either way because the structure never influences a move decision.
        if (!std.debug.runtime_safety) return;
        const n = self.flat.tour.len;
        const size = @max(self.target_segment_size, 1);
        self.segment_count = 0;
        var start: usize = 0;
        while (start < n) : (self.segment_count += 1) {
            const len = @min(size, n - start);
            self.segment_start[self.segment_count] = start;
            self.segment_len[self.segment_count] = len;
            self.segment_reversed[self.segment_count] = false;
            for (0..len) |rank| {
                const node = self.flat.tour[start + rank];
                self.segment_of_node[node] = self.segment_count;
                self.rank_in_segment[node] = rank;
            }
            start += len;
        }
    }
};

pub const TourView = union(enum) {
    flat: FlatTourView,
    segment: SegmentTourView,

    pub fn initFlat(
        tour: []usize,
        pos: []usize,
        next_nodes: []usize,
        prev_nodes: []usize,
        scratch_neighbor0: []usize,
        scratch_neighbor1: []usize,
        scratch_seen: []bool,
    ) TourView {
        return .{ .flat = .{
            .tour = tour,
            .pos = pos,
            .next_nodes = next_nodes,
            .prev_nodes = prev_nodes,
            .scratch_neighbor0 = scratch_neighbor0,
            .scratch_neighbor1 = scratch_neighbor1,
            .scratch_seen = scratch_seen,
        } };
    }

    pub fn initSegment(
        tour: []usize,
        pos: []usize,
        next_nodes: []usize,
        prev_nodes: []usize,
        scratch_neighbor0: []usize,
        scratch_neighbor1: []usize,
        scratch_seen: []bool,
        segment_of_node: []usize,
        rank_in_segment: []usize,
        segment_start: []usize,
        segment_len: []usize,
        segment_reversed: []bool,
    ) TourView {
        return .{ .segment = .{
            .flat = .{
                .tour = tour,
                .pos = pos,
                .next_nodes = next_nodes,
                .prev_nodes = prev_nodes,
                .scratch_neighbor0 = scratch_neighbor0,
                .scratch_neighbor1 = scratch_neighbor1,
                .scratch_seen = scratch_seen,
            },
            .segment_of_node = segment_of_node,
            .rank_in_segment = rank_in_segment,
            .segment_start = segment_start,
            .segment_len = segment_len,
            .segment_reversed = segment_reversed,
            .target_segment_size = segmentTargetSize(tour.len),
        } };
    }

    pub fn rebuild(self: *TourView) void {
        switch (self.*) {
            .flat => |*view| view.rebuild(),
            .segment => |*view| view.rebuild(),
        }
    }

    pub fn next(self: *const TourView, node: usize) usize {
        return switch (self.*) {
            .flat => |*view| view.next(node),
            .segment => |*view| view.next(node),
        };
    }

    pub fn prev(self: *const TourView, node: usize) usize {
        return switch (self.*) {
            .flat => |*view| view.prev(node),
            .segment => |*view| view.prev(node),
        };
    }

    pub fn len(self: *const TourView) usize {
        return switch (self.*) {
            .flat => |*view| view.tour.len,
            .segment => |*view| view.flat.tour.len,
        };
    }

    pub fn isTourEdge(self: *const TourView, a: usize, b: usize) bool {
        return self.next(a) == b or self.prev(a) == b;
    }

    pub fn between(self: *const TourView, a: usize, b: usize, c: usize) bool {
        return switch (self.*) {
            .flat => |*view| view.between(a, b, c),
            .segment => |*view| view.between(a, b, c),
        };
    }

    pub fn flipPath(self: *TourView, first_node: usize, last_node: usize) void {
        switch (self.*) {
            .flat => |*view| view.flipPath(first_node, last_node),
            .segment => |*view| view.flipPath(first_node, last_node),
        }
    }

    pub fn applyEdges(self: *TourView, removed: []const TourEdge, added: []const TourEdge) bool {
        return switch (self.*) {
            .flat => |*view| view.applyEdges(removed, added),
            .segment => |*view| view.applyEdges(removed, added),
        };
    }

    pub fn materialize(self: *const TourView, out: []usize) void {
        switch (self.*) {
            .flat => |*view| view.materialize(out),
            .segment => |*view| view.materialize(out),
        }
    }
};

pub const segmentTourThreshold: usize = 512;

pub fn useSegmentTour(n: usize) bool {
    return n >= segmentTourThreshold;
}

pub fn segmentTargetSize(n: usize) usize {
    var size: usize = 1;
    while (size * size < n) : (size += 1) {}
    return @max(size, 1);
}

pub fn sameUndirectedEdge(a: usize, b: usize, c: usize, d: usize) bool {
    return (a == c and b == d) or (a == d and b == c);
}
