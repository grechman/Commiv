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
const pdptw = internal.pdptw;
const pdptw_sisr = internal.pdptw_sisr;

pub const version = @import("version.zig").string;

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
pub const solveWithStats = solver.solveWithStats;

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
pub const CvrpFleetParams = vrp.CvrpFleetParams; // tuning for solveCvrpFleet (rounds/restarts/max_vehicles)
pub const validateCvrp = vrp.validate; // feasibility + true-cost checker for a CvrpResult's routes

// ---- VRPTW ------------------------------------------------------------------
pub const VrptwInstance = vrptw.VrptwInstance;
pub const VrptwResult = vrptw.VrptwResult;
pub const solveVrptwSisr = vrptw.solveVrptwSisr; // flagship engine + time windows
pub const solveVrptwSisrFrom = vrptw.solveVrptwSisrFrom;
pub const solveVrptwSisrFleetMin = vrptw.solveVrptwSisrFleetMin;
pub const solveVrptwSisrParallel = vrptw.solveVrptwSisrParallel;
pub const solveVrptwHgs = vrptw.solveVrptwHgs;
pub const VrptwSisrParams = vrptw.VrptwSisrParams;
pub const VrptwNbrKey = vrptw.NbrKey;
pub const VrptwHgsParams = vrptw.HgsParams;
pub const validateVrptw = vrptw.validate; // feasibility + true-cost checker for a VrptwResult's routes

// ---- PDPTW (pickup & delivery with time windows) ----------------------------
pub const PdpInstance = pdptw.PdpInstance;
pub const PdpResult = pdptw.PdpResult;
pub const PdpPairing = pdptw.Pairing;
pub const PdpPairingError = pdptw.PairingError;
pub const buildPdpPairing = pdptw.buildPairing;
pub const validatePdptw = pdptw.validate;
pub const validatePdptwTyped = pdptw.validateTyped; // per-route vehicle-type capacities
pub const validatePdptwWithBreak = pdptw.validateWithBreak; // driver-break contract oracle
pub const solvePdptwSisr = pdptw_sisr.solvePdptwSisr; // flagship SISR engine for PDPTW
pub const solvePdptwSisrFleetMin = pdptw_sisr.solvePdptwSisrFleetMin;
pub const solvePdptwSisrFleetMinParallel = pdptw_sisr.solvePdptwSisrFleetMinParallel;
pub const solvePdptwSisrPinned = pdptw_sisr.solvePdptwSisrPinned;
pub const solvePdptwSisrDispatch = pdptw_sisr.solvePdptwSisrDispatch;
pub const PdpSisrParams = pdptw_sisr.PdpSisrParams;
pub const PdpVehType = pdptw_sisr.VehType;
pub const PdpBreak = pdptw_sisr.Break;
pub const pdpMaxVehTypes = pdptw_sisr.MAX_VEH_TYPES;

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
    pub const solver = @import("solver.zig");
    pub const parallel = @import("parallel.zig");
    pub const spatial = @import("spatial.zig");
    pub const asymmetric = @import("asymmetric.zig");
    pub const vrp = @import("vrp.zig");
    pub const vrptw = @import("vrptw.zig");
    pub const pdptw = @import("pdptw.zig");
    pub const pdptw_sisr = @import("pdptw_sisr.zig");
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
    _ = @import("pdptw.zig");
    _ = @import("pdptw_sisr.zig");
    _ = @import("capi.zig");
}
