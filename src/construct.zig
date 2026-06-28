const std = @import("std");
const distance = @import("distance.zig");
const candidates_mod = @import("candidates.zig");
const DistanceOracle = distance.DistanceOracle;
const Candidates = candidates_mod.Candidates;

pub fn nearestNeighborTour(
    dist_oracle: *DistanceOracle,
    candidates: *const Candidates,
    random: *std.Random,
    trial: usize,
    randomized: bool,
    tour: []usize,
    used: []bool,
) void {
    const n = dist_oracle.p.dimension;
    std.debug.assert(used.len == n);
    @memset(used, false);

    var current = trial % n;
    if (randomized and trial > 0) current = random.intRangeLessThan(usize, 0, n);

    for (0..n) |idx| {
        tour[idx] = current;
        used[current] = true;
        if (idx + 1 == n) break;

        var best_nodes: [4]usize = undefined;
        var best_dist: [4]u64 = .{ std.math.maxInt(u64), std.math.maxInt(u64), std.math.maxInt(u64), std.math.maxInt(u64) };
        var found: usize = 0;

        for (candidates.row(current)) |candidate| {
            if (!used[candidate]) {
                insertCandidate(candidate, dist_oracle.distance(current, candidate), &best_nodes, &best_dist, &found);
            }
        }
        if (found == 0) {
            for (0..n) |node| {
                if (!used[node]) {
                    insertCandidate(node, dist_oracle.distance(current, node), &best_nodes, &best_dist, &found);
                }
            }
        }
        std.debug.assert(found != 0);

        const choice_count = if (randomized and trial > 0) @min(found, 3) else 1;
        const chosen_idx = if (choice_count > 1) random.intRangeLessThan(usize, 0, choice_count) else 0;
        current = best_nodes[chosen_idx];
    }
}

pub fn farthestInsertionTour(
    dist_oracle: *DistanceOracle,
    tour: []usize,
    scratch: []usize,
    used: []bool,
) void {
    const n = dist_oracle.p.dimension;
    std.debug.assert(tour.len == n);
    std.debug.assert(scratch.len == n);
    std.debug.assert(used.len == n);
    @memset(used, false);

    var first: usize = 0;
    var second: usize = 1;
    var best_dist: u32 = 0;
    for (0..n) |a| {
        for (a + 1..n) |b| {
            const d = dist_oracle.distance(a, b);
            if (d > best_dist) {
                best_dist = d;
                first = a;
                second = b;
            }
        }
    }

    tour[0] = first;
    tour[1] = second;
    used[first] = true;
    used[second] = true;
    var len: usize = 2;

    while (len < n) {
        var next_node: usize = std.math.maxInt(usize);
        var farthest: u32 = 0;
        for (0..n) |node| {
            if (used[node]) continue;
            var nearest_to_tour: u32 = std.math.maxInt(u32);
            for (tour[0..len]) |tour_node| {
                nearest_to_tour = @min(nearest_to_tour, dist_oracle.distance(node, tour_node));
            }
            if (nearest_to_tour > farthest or (nearest_to_tour == farthest and node < next_node)) {
                farthest = nearest_to_tour;
                next_node = node;
            }
        }
        std.debug.assert(next_node != std.math.maxInt(usize));

        var insert_after: usize = 0;
        var best_delta: i64 = std.math.maxInt(i64);
        for (0..len) |idx| {
            const a = tour[idx];
            const b = tour[(idx + 1) % len];
            const delta = @as(i64, @intCast(dist_oracle.distance(a, next_node))) +
                @as(i64, @intCast(dist_oracle.distance(next_node, b))) -
                @as(i64, @intCast(dist_oracle.distance(a, b)));
            if (delta < best_delta or (delta == best_delta and a < tour[insert_after])) {
                best_delta = delta;
                insert_after = idx;
            }
        }

        @memcpy(scratch[0..len], tour[0..len]);
        const insert_at = insert_after + 1;
        @memcpy(tour[0..insert_at], scratch[0..insert_at]);
        tour[insert_at] = next_node;
        @memcpy(tour[insert_at + 1 .. len + 1], scratch[insert_at..len]);
        used[next_node] = true;
        len += 1;
    }
}

fn insertCandidate(
    node: usize,
    dist: u32,
    best_nodes: *[4]usize,
    best_dist: *[4]u64,
    found: *usize,
) void {
    const candidate_dist = @as(u64, dist);
    var slot: ?usize = null;
    for (0..best_dist.len) |i| {
        if (candidate_dist < best_dist[i] or (candidate_dist == best_dist[i] and (found.* <= i or node < best_nodes[i]))) {
            slot = i;
            break;
        }
    }
    const pos = slot orelse return;
    var i = best_dist.len - 1;
    while (i > pos) : (i -= 1) {
        best_dist[i] = best_dist[i - 1];
        best_nodes[i] = best_nodes[i - 1];
    }
    best_dist[pos] = candidate_dist;
    best_nodes[pos] = node;
    found.* = @min(best_dist.len, found.* + 1);
}

// Double-bridge perturbation: split the tour into A B C D and reconnect as
// A C B D (segment exchange, no reversals). Exactly three tour edges change;
// their six endpoints are reported so the LK queue can be seeded with only the
// perturbed neighborhood (cf. LKH's kick + InBestTour activation gating).
pub fn segmentExchangeKick(tour: []usize, random: *std.Random, touched: *[6]usize) void {
    const n = tour.len;
    std.debug.assert(n >= 8);
    const i = random.intRangeLessThan(usize, 1, n - 2);
    const j = random.intRangeLessThan(usize, i + 1, n - 1);
    const k = random.intRangeLessThan(usize, j + 1, n);
    touched.* = .{ tour[i - 1], tour[i], tour[j - 1], tour[j], tour[k - 1], tour[k] };
    std.mem.rotate(usize, tour[i..k], j - i);
}

// Plateau kick: apply up to `moves` zero-delta reconnections anchored at
// random tour positions. On degenerate integer geometries (rattled grids,
// drilling patterns) locally optimal tours sit on broad cost-equal plateaus:
// the residual gap to the optimum hides in scattered micro-sections whose
// better variant is cost-equal until a neighboring section also changes
// (measured on rat575: 67 differing edges vs the optimum in 59 sections of
// size <= 2). Length-preserving moves walk the plateau without giving up
// quality, so the follow-up descent starts from a genuinely different tour
// of equal length. With `or_opt` set, half the attempts are zero-delta
// segment relocations (1-3 nodes, both orientations) — the scattered
// residual sections are size <= 2, i.e. relocations, which reversal drift
// cannot express; the stride of `touched` grows from 4 to 6 per move and
// the RNG consumption changes, so callers gate it to keep small-instance
// trajectories untouched. Returns the number of applied moves.
pub fn plateauKick(
    dist_oracle: *DistanceOracle,
    candidates: *const Candidates,
    tour: []usize,
    pos: []usize,
    random: *std.Random,
    moves: usize,
    or_opt: bool,
    touched: []usize,
) usize {
    const n = tour.len;
    const stride: usize = if (or_opt) 6 else 4;
    std.debug.assert(touched.len >= stride * moves);
    for (tour, 0..) |node, idx| pos[node] = idx;

    var applied: usize = 0;
    var attempts: usize = 0;
    const max_attempts = 8 * moves;
    while (applied < moves and attempts < max_attempts) : (attempts += 1) {
        if (or_opt and random.intRangeLessThan(usize, 0, 2) == 1) {
            applied += plateauOrOptMove(dist_oracle, candidates, tour, pos, random, touched[stride * applied ..]);
            continue;
        }
        const i = random.intRangeLessThan(usize, 0, n);
        const a = tour[i];
        const b = tour[(i + 1) % n];
        const d_ab = @as(i64, @intCast(dist_oracle.distance(a, b)));
        for (candidates.row(a)) |c| {
            if (c == b or c == a) continue;
            const j = pos[c];
            const d = tour[(j + 1) % n];
            if (d == a) continue;
            const delta = @as(i64, @intCast(dist_oracle.distance(a, c))) +
                @as(i64, @intCast(dist_oracle.distance(b, d))) -
                d_ab - @as(i64, @intCast(dist_oracle.distance(c, d)));
            if (delta != 0) continue;
            // 2-opt: reverse the path b..c (positions i+1..j, possibly
            // wrapping); normalize so the reversed span lies in-bounds.
            var lo = (i + 1) % n;
            var hi = j;
            if (lo > hi) {
                // Reverse the complementary span d..a instead; same cycle.
                lo = (j + 1) % n;
                hi = i;
                if (lo > hi) continue;
            }
            var x = lo;
            var y = hi;
            while (x < y) : ({
                x += 1;
                y -= 1;
            }) {
                std.mem.swap(usize, &tour[x], &tour[y]);
                pos[tour[x]] = x;
                pos[tour[y]] = y;
            }
            pos[tour[x]] = x;
            touched[stride * applied + 0] = a;
            touched[stride * applied + 1] = b;
            touched[stride * applied + 2] = c;
            touched[stride * applied + 3] = d;
            if (or_opt) {
                touched[stride * applied + 4] = a;
                touched[stride * applied + 5] = b;
            }
            applied += 1;
            break;
        }
    }
    return applied;
}

// One zero-delta Or-opt drift attempt: relocate the 1-3 node segment at a
// random position to sit after a candidate neighbor of its head node, in
// either orientation, when the relocation is exactly cost-neutral. Returns
// 1 and records 6 endpoints into `touched` on success, 0 otherwise.
// Wrapping segments and wrapping insertions are skipped (the array splice
// stays a single contiguous shift; the anchor position is uniform anyway).
fn plateauOrOptMove(
    dist_oracle: *DistanceOracle,
    candidates: *const Candidates,
    tour: []usize,
    pos: []usize,
    random: *std.Random,
    touched: []usize,
) usize {
    const n = tour.len;
    const seg_len = random.intRangeLessThan(usize, 1, 4);
    const i = random.intRangeLessThan(usize, 0, n);
    if (i == 0 or i + seg_len >= n) return 0;
    const p = tour[i - 1];
    const s0 = tour[i];
    const s1 = tour[i + seg_len - 1];
    const q = tour[i + seg_len];
    const removal_gain = @as(i64, @intCast(dist_oracle.distance(p, s0))) +
        @as(i64, @intCast(dist_oracle.distance(s1, q))) -
        @as(i64, @intCast(dist_oracle.distance(p, q)));
    if (removal_gain == 0) return 0;

    for (candidates.row(s0)) |c| {
        const jc = pos[c];
        // c inside the segment or directly before it (re-insertion no-op).
        if (jc + 1 >= i and jc < i + seg_len) continue;
        if (jc == n - 1) continue;
        const cn = tour[jc + 1];
        const d_ccn = @as(i64, @intCast(dist_oracle.distance(c, cn)));
        const ins_fwd = @as(i64, @intCast(dist_oracle.distance(c, s0))) +
            @as(i64, @intCast(dist_oracle.distance(s1, cn))) - d_ccn;
        const ins_rev = @as(i64, @intCast(dist_oracle.distance(c, s1))) +
            @as(i64, @intCast(dist_oracle.distance(s0, cn))) - d_ccn;
        const fwd = ins_fwd == removal_gain;
        if (!fwd and ins_rev != removal_gain) continue;

        var seg: [3]usize = undefined;
        @memcpy(seg[0..seg_len], tour[i .. i + seg_len]);
        if (!fwd) std.mem.reverse(usize, seg[0..seg_len]);
        if (jc > i) {
            // Shift the gap left, drop the segment in after c.
            std.mem.copyForwards(usize, tour[i .. jc + 1 - seg_len], tour[i + seg_len .. jc + 1]);
            @memcpy(tour[jc + 1 - seg_len .. jc + 1], seg[0..seg_len]);
            for (i..jc + 1) |idx| pos[tour[idx]] = idx;
        } else {
            // Shift the gap right, drop the segment in after c.
            std.mem.copyBackwards(usize, tour[jc + 1 + seg_len .. i + seg_len], tour[jc + 1 .. i]);
            @memcpy(tour[jc + 1 .. jc + 1 + seg_len], seg[0..seg_len]);
            for (jc + 1..i + seg_len) |idx| pos[tour[idx]] = idx;
        }
        touched[0] = p;
        touched[1] = q;
        touched[2] = c;
        touched[3] = cn;
        touched[4] = s0;
        touched[5] = s1;
        return 1;
    }
    return 0;
}

// Guided restart construction, ported from LKH's ChooseInitialTour cases
// C/D/E (Trial > 1): starting at a random node, repeatedly extend with
//   (C) a random unchosen candidate neighbor whose edge has alpha == 0 and
//       lies in the best or next-best reference tour, else
//   (D) a random unchosen candidate neighbor, else
//   (E) the nearest unchosen node.
// Retaining the alpha-zero backbone keeps the tour near-elite while every
// alpha>0 stretch diverges through case D — structurally different parents
// that double-bridge kicks cannot produce and that give EAX merging
// independent differing sections to recombine.
// On return `used` no longer means "chosen": it flags the endpoints of every
// tour edge absent from both reference tours, i.e. the only neighborhoods a
// follow-up LK descent needs to reactivate.
pub fn guidedBackboneTour(
    dist_oracle: *DistanceOracle,
    candidates: *const Candidates,
    refs: [2]?[]const usize,
    ref_next: [2][]usize,
    ref_prev: [2][]usize,
    max_divergence: usize,
    random: *std.Random,
    tour: []usize,
    used: []bool,
) void {
    const n = dist_oracle.p.dimension;
    std.debug.assert(used.len == n);
    @memset(used, false);
    for (refs, ref_next, ref_prev) |maybe_ref, rn, rp| {
        const ref = maybe_ref orelse continue;
        std.debug.assert(ref.len == n);
        for (ref, 0..) |node, idx| {
            rn[node] = ref[(idx + 1) % n];
            rp[node] = ref[(idx + n - 1) % n];
        }
    }

    var current = random.intRangeLessThan(usize, 0, n);
    var divergences: usize = 0;
    for (0..n) |idx| {
        tour[idx] = current;
        used[current] = true;
        if (idx + 1 == n) break;

        const row = candidates.row(current);
        const alpha_row = candidates.alphaRow(current);
        var count: usize = 0;
        var pick: usize = 0;
        // Case C: alpha-zero candidate edges on a reference tour, picked
        // uniformly via reservoir sampling (LKH picks uniformly among the
        // collected alternatives).
        for (row, alpha_row) |cand, alpha| {
            if (used[cand] or alpha != 0) continue;
            if (!guideEdgeOnRefs(refs, ref_next, ref_prev, current, cand)) continue;
            count += 1;
            if (count == 1 or random.intRangeLessThan(usize, 0, count) == 0) pick = cand;
        }
        // Case C-and-a-half (deviation from LKH, active only when a finite
        // divergence budget is set): a candidate reference-tour edge
        // regardless of alpha. While under the budget it is followed with
        // probability 3/4 so that divergence strikes at random alpha>0
        // stretches; once the budget is spent it is followed
        // unconditionally. Unbounded case-D divergence makes the follow-up
        // light descent nearly as expensive as a cold one; the budget keeps
        // guided tours near-elite — few neighborhoods to reactivate, and
        // localized independent differences, which is exactly what EAX
        // merging wants. With max_divergence == maxInt the construction is
        // the faithful LKH C/D/E ladder for full-descent callers.
        if (count == 0 and max_divergence != std.math.maxInt(usize) and
            (divergences >= max_divergence or random.intRangeLessThan(usize, 0, 4) != 0))
        {
            for (row) |cand| {
                if (used[cand]) continue;
                if (!guideEdgeOnRefs(refs, ref_next, ref_prev, current, cand)) continue;
                count += 1;
                if (count == 1 or random.intRangeLessThan(usize, 0, count) == 0) pick = cand;
            }
        }
        // Case D: any unchosen candidate edge.
        if (count == 0) {
            divergences += 1;
            for (row) |cand| {
                if (used[cand]) continue;
                count += 1;
                if (count == 1 or random.intRangeLessThan(usize, 0, count) == 0) pick = cand;
            }
        }
        // Case E: nearest unchosen node.
        if (count == 0) {
            var best_dist: u64 = std.math.maxInt(u64);
            for (0..n) |node| {
                if (used[node]) continue;
                const d = @as(u64, dist_oracle.distance(current, node));
                if (d < best_dist) {
                    best_dist = d;
                    pick = node;
                }
            }
        }
        current = pick;
    }

    @memset(used, false);
    for (tour, 0..) |a, idx| {
        const b = tour[(idx + 1) % n];
        if (!guideEdgeOnRefs(refs, ref_next, ref_prev, a, b)) {
            used[a] = true;
            used[b] = true;
        }
    }
}

fn guideEdgeOnRefs(refs: [2]?[]const usize, ref_next: [2][]usize, ref_prev: [2][]usize, a: usize, b: usize) bool {
    for (refs, ref_next, ref_prev) |maybe_ref, rn, rp| {
        if (maybe_ref == null) continue;
        if (rn[a] == b or rp[a] == b) return true;
    }
    return false;
}
