const std = @import("std");

// =============================================================================
// commiv public API
//
// Everything an application needs is a curated `pub const` below, grouped by
// problem family. The implementation modules are NOT part of the stable API;
// they live under `internal` (see the bottom of this file) purely so the
// in-repo benchmarks and power users can reach solver guts. Treat anything under
// `commiv.internal.*` as unstable.
// =============================================================================

// Internal module handles used to build the curated surface below. Referenced
// through `internal.*` so root has no bare module decls leaking into the API.
const problem = internal.problem;
const tsplib = internal.tsplib;
const exact = internal.exact;
const solver = internal.solver;
const parallel = internal.parallel;
const asymmetric = internal.asymmetric;
const vrp = internal.vrp;
const vrptw = internal.vrptw;

// ---- Parsing ----------------------------------------------------------------
pub const parseTsplib = tsplib.parse;
pub const ParseOptions = tsplib.ParseOptions;
pub const ParseDiagnostic = tsplib.ParseDiagnostic;

// ---- Problem definition -----------------------------------------------------
pub const Problem = problem.Problem;
pub const Coord = problem.Coord;
pub const DistanceKind = problem.DistanceKind;

// ---- Shared solve options / result ------------------------------------------
pub const SolveOptions = solver.SolveOptions;
pub const SolveResult = solver.SolveResult; // returned by solve / solveAtsp* / bruteForce
pub const SolveStats = solver.SolveStats;
pub const CandidateMode = solver.CandidateMode;

// ---- TSP (symmetric) --------------------------------------------------------
pub const solve = solver.solve;

// ---- ATSP (directed) --------------------------------------------------------
pub const solveAtsp = asymmetric.solveAtsp; // 2n Jonker-Volgenant transform
pub const solveAtspNative = asymmetric.solveAtspNative; // direct directed search
pub const solveAtspParallel = asymmetric.solveAtspParallel;

// ---- Exact (tiny n) ---------------------------------------------------------
pub const bruteForce = exact.bruteForce;
pub const ExactOptions = exact.ExactOptions;

// ---- CVRP / ACVRP -----------------------------------------------------------
pub const CvrpInstance = vrp.CvrpInstance;
pub const CvrpResult = vrp.CvrpResult;
pub const solveCvrp = vrp.solveCvrp; // no-config default (SISR)
pub const solveCvrpFleet = vrp.solveCvrpFleet; // fixed vehicle cap
pub const solveCvrpHgs = vrp.solveCvrpHgs;
pub const solveCvrpHgsParallel = vrp.solveCvrpHgsParallel;
pub const solveCvrpSisr = vrp.solveCvrpSisr;
pub const solveCvrpSisrParallel = vrp.solveCvrpSisrParallel;
pub const CvrpHgsParams = vrp.HgsParams;
pub const CvrpSisrParams = vrp.SisrParams;

// ---- VRPTW ------------------------------------------------------------------
pub const VrptwInstance = vrptw.VrptwInstance;
pub const VrptwResult = vrptw.VrptwResult;
pub const solveVrptw = vrptw.solveVrptw;
pub const solveVrptwHgs = vrptw.solveVrptwHgs;
pub const VrptwHgsParams = vrptw.HgsParams;

// ---- Asymmetry analysis -----------------------------------------------------
pub const conservativeness = asymmetric.conservativeness;
pub const Conservativeness = asymmetric.Conservativeness;

// ---- Parallel driver --------------------------------------------------------
pub const solveParallel = parallel.solveParallel;
pub const ParallelOptions = parallel.ParallelOptions;

// ---- Internals (unstable; not the public API) -------------------------------
// Exposed only so the in-repo benchmarks/probes (and power users who accept the
// instability) can reach solver internals like the candidate builder or the
// distance oracle. Nothing here is covered by the README API contract.
pub const internal = struct {
    pub const problem = @import("problem.zig");
    pub const tsplib = @import("tsplib.zig");
    pub const exact = @import("exact.zig");
    pub const result = @import("result.zig");
    pub const solver = @import("solver.zig");
    pub const parallel = @import("parallel.zig");
    pub const spatial = @import("spatial.zig");
    pub const asymmetric = @import("asymmetric.zig");
    pub const vrp = @import("vrp.zig");
    pub const vrptw = @import("vrptw.zig");
};

test {
    std.testing.refAllDecls(@This());
    // Pull every implementation module into the compilation graph so its unit
    // tests are discovered (the curated re-exports only reference some modules).
    _ = @import("problem.zig");
    _ = @import("tsplib.zig");
    _ = @import("exact.zig");
    _ = @import("result.zig");
    _ = @import("solver.zig");
    _ = @import("parallel.zig");
    _ = @import("spatial.zig");
    _ = @import("asymmetric.zig");
    _ = @import("vrp.zig");
    _ = @import("vrptw.zig");
}
