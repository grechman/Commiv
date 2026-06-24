const std = @import("std");
const commiv = @import("commiv");

// ASYMMETRIC TSP benchmark on the standard TSPLIB ATSP set (Fischetti / Repetto /
// Reinelt instances) — the canonical asymmetric benchmark, all with PROVEN optima.
// This is the real test of commiv's asymmetric core (solveAtsp): the SOTA solvers
// we'd compare against on symmetric instances (HGS-CVRP, FILO, SISR) are symmetric-
// only; on ATSP the reference is LKH-3, which solves every one of these to
// OPTIMALITY. So gap-to-optimum here is gap-to-the-best-in-the-field.
// Env: AB_DIR (default vendor/atsp), AB_FILES, AB_SEED, AB_TRIALS.
pub fn main(init: std.process.Init) !void {
    const allocator = init.gpa;
    const env = init.environ_map;
    const dir = env.get("AB_DIR") orelse "vendor/atsp";
    const files = env.get("AB_FILES") orelse
        "br17,ftv33,ftv35,ftv38,p43,ftv44,ftv47,ry48p,ft53,ftv55,ftv64,ft70,ftv70,kro124p";
    const seed = try std.fmt.parseInt(u64, env.get("AB_SEED") orelse "12345", 10);

    std.debug.print("instance,n,opt,commiv,gap_pct,ms,valid\n", .{});
    var sum_gap: f64 = 0;
    var count: usize = 0;
    var it = std.mem.tokenizeScalar(u8, files, ',');
    while (it.next()) |name| {
        var path_buf: [512]u8 = undefined;
        const path = try std.fmt.bufPrint(&path_buf, "{s}/{s}.atsp", .{ dir, name });
        const bytes = try std.Io.Dir.cwd().readFileAlloc(init.io, path, allocator, .limited(8 << 20));
        defer allocator.free(bytes);

        const parsed = try parseAtsp(allocator, bytes);
        defer allocator.free(parsed.matrix);
        const n = parsed.n;

        // ROUTING TEST (AB_BLEND b in [0,0.5]): blend the matrix toward symmetric,
        // M'[i][j] = (1-b)*d_ij + b*d_ji (b=0.5 -> fully symmetric). At each blend,
        // measure the asymmetry ratio rho (mean max/min over arc pairs) and compare
        // two treatments on the SAME blended matrix: "asym" (solveAtsp on M') vs "sym"
        // (symmetrize M', solve, evaluate the resulting tour's TRUE directed length).
        // The crossover rho is where it starts paying to handle direction.
        if (env.get("AB_BLEND")) |bs| {
            const b = try std.fmt.parseFloat(f64, bs);
            const M = try allocator.alloc(u32, n * n);
            defer allocator.free(M);
            for (0..n) |i| for (0..n) |j| {
                const x: f64 = @floatFromInt(parsed.matrix[i * n + j]);
                const y: f64 = @floatFromInt(parsed.matrix[j * n + i]);
                M[i * n + j] = if (i == j) 0 else @intFromFloat(@round((1 - b) * x + b * y));
            };
            var rho_sum: f64 = 0;
            var cnt: usize = 0;
            for (0..n) |i| for (0..n) |j| {
                if (i == j) continue;
                const x = M[i * n + j];
                const y = M[j * n + i];
                if (x > 0 and y > 0) {
                    rho_sum += @as(f64, @floatFromInt(@max(x, y))) / @as(f64, @floatFromInt(@min(x, y)));
                    cnt += 1;
                }
            };
            const rho = rho_sum / @as(f64, @floatFromInt(cnt));
            const opt = commiv.SolveOptions{ .seed = seed, .budget = .{ .trials = @min(n, 200), .trial_extension_factor = 2, .max_passes = 64 }, .candidates = .{ .candidate_count = 16, .sparse_min_dimension = 0 }, .search = .{ .enable_lk = true, .lk_max_depth = 6 } };
            const ta = nanos();
            var ra = try commiv.solveAtsp(allocator, M, n, opt);
            defer ra.deinit();
            const ms_a = @as(f64, @floatFromInt(nanos() - ta)) / 1e6;
            const S = try allocator.alloc(u32, n * n);
            defer allocator.free(S);
            for (0..n) |i| for (0..n) |j| {
                S[i * n + j] = if (i == j) 0 else @intCast((@as(u64, M[i * n + j]) + M[j * n + i]) / 2);
            };
            const ts = nanos();
            var rs = try commiv.solveAtsp(allocator, S, n, opt);
            defer rs.deinit();
            const ms_s = @as(f64, @floatFromInt(nanos() - ts)) / 1e6;
            var len_sym: u64 = 0;
            for (0..n) |idx| len_sym += M[rs.tour[idx] * n + rs.tour[(idx + 1) % n]];
            const pen = 100.0 * (@as(f64, @floatFromInt(len_sym)) - @as(f64, @floatFromInt(ra.length))) / @as(f64, @floatFromInt(ra.length));
            std.debug.print("{s} blend={d:.2} rho={d:.3} | asym={} ({d:.0}ms) | sym_treat={} ({d:.0}ms) | sym penalty={d:.3}% | winner={s}\n", .{ name, b, rho, ra.length, ms_a, len_sym, ms_s, pen, if (ra.length <= len_sym) "ASYM" else "SYM" });
            continue;
        }

        const trials_env = env.get("AB_TRIALS");
        const mode: commiv.CandidateMode = if (std.mem.eql(u8, env.get("AB_MODE") orelse "nearest", "alpha")) .alpha_nearness else .nearest_distance;
        const t0 = nanos();
        const ab_threads = try std.fmt.parseInt(usize, env.get("AB_THREADS") orelse "1", 10);
        const ab_btdepth = env.get("AB_BTDEPTH"); // force lk_backtrack_depth (null=auto)
        const ab_native = std.mem.eql(u8, env.get("AB_NATIVE") orelse "0", "1");
        if (ab_native) {
            const trials = if (trials_env) |s| try std.fmt.parseInt(usize, s, 10) else @min(n, 200);
            var nres = try commiv.solveAtspNative(allocator, parsed.matrix, n, .{ .seed = seed, .budget = .{ .trials = trials } });
            defer nres.deinit();
            const nms = @as(f64, @floatFromInt(nanos() - t0)) / 1e6;
            const nvalid = validTour(parsed.matrix, n, nres.tour, nres.length);
            const nopt = optFor(name) orelse 0;
            const ngap = if (nopt > 0) 100.0 * (@as(f64, @floatFromInt(nres.length)) - @as(f64, @floatFromInt(nopt))) / @as(f64, @floatFromInt(nopt)) else 0;
            if (nopt > 0 and nvalid) {
                sum_gap += ngap;
                count += 1;
            }
            std.debug.print("{s},{},{},{},{d:.3},{d:.0},{}\n", .{ name, n, nopt, nres.length, ngap, nms, nvalid });
            continue;
        }
        var res = try commiv.solveAtspParallel(allocator, parsed.matrix, n, .{
            .seed = seed,
            .budget = .{
                .trials = if (trials_env) |s| try std.fmt.parseInt(usize, s, 10) else @min(n, 200),
                .trial_extension_factor = 2,
                .max_passes = try std.fmt.parseInt(usize, env.get("AB_PASSES") orelse "64", 10),
            },
            .candidates = .{ .candidate_count = try std.fmt.parseInt(usize, env.get("AB_WIDTH") orelse "12", 10), .candidate_mode = mode, .sparse_min_dimension = 0 },
            .search = .{
                .enable_lk = true,
                .lk_max_depth = try std.fmt.parseInt(usize, env.get("AB_DEPTH") orelse "6", 10),
                .lk_backtrack_depth = if (ab_btdepth) |s| try std.fmt.parseInt(usize, s, 10) else null,
                .nonseq_max_dimension = try std.fmt.parseInt(usize, env.get("AB_NONSEQMAX") orelse "512", 10),
            },
        }, ab_threads);
        defer res.deinit();
        const ms = @as(f64, @floatFromInt(nanos() - t0)) / 1e6;

        // independent validity: permutation + reported length matches a fresh scan
        const valid = validTour(parsed.matrix, n, res.tour, res.length);
        const opt = optFor(name) orelse 0;
        const gap = if (opt > 0) 100.0 * (@as(f64, @floatFromInt(res.length)) - @as(f64, @floatFromInt(opt))) / @as(f64, @floatFromInt(opt)) else 0;
        if (opt > 0 and valid) {
            sum_gap += gap;
            count += 1;
        }
        std.debug.print("{s},{},{},{},{d:.3},{d:.0},{}\n", .{ name, n, opt, res.length, gap, ms, valid });
    }
    if (count > 0) {
        std.debug.print("# mean gap over {} valid: {d:.3}%\n", .{ count, sum_gap / @as(f64, @floatFromInt(count)) });
    }
}

/// Proven optima for the TSPLIB ATSP instances (Heidelberg TSPLIB / DIMACS).
fn optFor(name: []const u8) ?u64 {
    const table = [_]struct { []const u8, u64 }{
        .{ "br17", 39 },       .{ "ftv33", 1286 },    .{ "ftv35", 1473 },
        .{ "ftv38", 1530 },    .{ "p43", 5620 },      .{ "ftv44", 1613 },
        .{ "ftv47", 1776 },    .{ "ry48p", 14422 },   .{ "ft53", 6905 },
        .{ "ftv55", 1608 },    .{ "ftv64", 1839 },    .{ "ft70", 38673 },
        .{ "ftv70", 1950 },    .{ "kro124p", 36230 }, .{ "ftv170", 2755 },
        .{ "rbg323", 1326 },   .{ "rbg358", 1163 },   .{ "rbg403", 2465 },
        .{ "rbg443", 2720 },
    };
    for (table) |row| {
        if (std.mem.eql(u8, row[0], name)) return row[1];
    }
    return null;
}

const Parsed = struct { matrix: []u32, n: usize };

/// Parse a TSPLIB ATSP file: DIMENSION + EDGE_WEIGHT_SECTION FULL_MATRIX (n*n
/// row-major integers, possibly line-wrapped). The diagonal (a forbidding 9999
/// etc.) is read as-is; solveAtsp ignores it.
fn parseAtsp(allocator: std.mem.Allocator, bytes: []const u8) !Parsed {
    var n: usize = 0;
    // dimension
    var lines = std.mem.tokenizeAny(u8, bytes, "\r\n");
    var section_off: usize = 0;
    while (lines.next()) |raw| {
        const line = std.mem.trim(u8, raw, " \t");
        if (std.mem.startsWith(u8, line, "DIMENSION")) {
            var t = std.mem.tokenizeAny(u8, line, " \t:");
            var last: ?[]const u8 = null;
            while (t.next()) |tok| last = tok;
            n = try std.fmt.parseInt(usize, last orelse return error.NoDim, 10);
        }
        if (std.mem.indexOf(u8, line, "EDGE_WEIGHT_SECTION") != null) {
            section_off = lines.index; // bytes offset right after this line
            break;
        }
    }
    if (n == 0) return error.NoDim;
    const matrix = try allocator.alloc(u32, n * n);
    errdefer allocator.free(matrix);
    var nums = std.mem.tokenizeAny(u8, bytes[section_off..], " \t\r\n");
    var k: usize = 0;
    while (k < n * n) {
        const tok = nums.next() orelse return error.TooFewWeights;
        const val = std.fmt.parseInt(u32, tok, 10) catch continue; // skip stray tokens (EOF, etc.)
        matrix[k] = val;
        k += 1;
    }
    return .{ .matrix = matrix, .n = n };
}

fn validTour(matrix: []const u32, n: usize, tour: []const usize, reported: u64) bool {
    if (tour.len != n) return false;
    const seen = std.heap.page_allocator.alloc(bool, n) catch return false;
    defer std.heap.page_allocator.free(seen);
    @memset(seen, false);
    for (tour) |c| {
        if (c >= n or seen[c]) return false;
        seen[c] = true;
    }
    var total: u64 = 0;
    for (0..n) |i| {
        const a = tour[i];
        const b = tour[(i + 1) % n];
        total += matrix[a * n + b];
    }
    return total == reported;
}

fn nanos() u64 {
    var ts: std.os.linux.timespec = undefined;
    _ = std.os.linux.clock_gettime(std.os.linux.CLOCK.MONOTONIC, &ts);
    return @as(u64, @intCast(ts.sec)) * 1_000_000_000 + @as(u64, @intCast(ts.nsec));
}
