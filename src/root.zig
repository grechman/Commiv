const std = @import("std");

pub const problem = @import("problem.zig");
pub const tsplib = @import("tsplib.zig");
pub const exact = @import("exact.zig");
pub const solver = @import("solver.zig");
pub const parallel = @import("parallel.zig");
pub const spatial = @import("spatial.zig");

pub const Coord = problem.Coord;
pub const DistanceKind = problem.DistanceKind;
pub const Problem = problem.Problem;
pub const TourResult = problem.TourResult;

pub const ParseDiagnostic = tsplib.ParseDiagnostic;
pub const ParseOptions = tsplib.ParseOptions;
pub const parseTsplib = tsplib.parse;

pub const bruteForce = exact.bruteForce;
pub const ExactOptions = exact.ExactOptions;

pub const solve = solver.solve;
pub const SolveOptions = solver.SolveOptions;
pub const SolveResult = solver.SolveResult;
pub const SolveStats = solver.SolveStats;
pub const CandidateMode = solver.CandidateMode;

pub const solveParallel = parallel.solveParallel;
pub const ParallelOptions = parallel.ParallelOptions;

test {
    std.testing.refAllDecls(@This());
}
