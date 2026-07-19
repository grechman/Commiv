const std = @import("std");
const time = @import("time.zig");

// SISR ruin-and-recreate loop skeleton (Phase 3 of the cleanup campaign):
// the iterate / wall-check-every-256 / save / ruin / recreate / threshold-
// accept-or-rollback / temperature-decay / post-iteration-hook discipline that
// every engine previously duplicated. The engine is a comptime duck-typed
// adapter, so every hook body inlines exactly where the engine's inline code
// used to be — same operations, same order, no vtables, no indirect calls.
//
// Contract (`Eng` methods, all inlined at comptime):
//   Saved     save(it)              snapshot scalars + ledger, begin the iter
//   !void     ruin(rng, it)
//   !void     recreate(rng)
//   i64       delta(saved)          effective-cost delta incl. penalty terms
//   !void     accept(saved, it)     commit path: best-capture and its gates
//   !void     reject(saved)         rollback to the snapshot
//   !void     afterIter(it, rng)    periodic kicks etc.; empty when unused
//
// Ruin/recreate/eval stay ENGINE-SPECIFIC (pair-atomic vs string vs CVRP
// semantics differ for real); this skeleton unifies the loop and state
// discipline, not the neighborhood math.
pub const LoopCfg = struct {
    iters: usize,
    time_ms: u64, // 0 = iteration-bound only (no clock reads in the loop)
    t0: f64,
    cf: f64,
};

pub fn run(comptime Eng: type, eng: *Eng, cfg: LoopCfg, rng: std.Random) !void {
    var temp = cfg.t0;
    const t_start = time.nanos();
    var it: usize = 0;
    while (it < cfg.iters) : (it += 1) {
        if (cfg.time_ms > 0 and it % 256 == 0 and (time.nanos() - t_start) / std.time.ns_per_ms >= cfg.time_ms) break;
        const saved = eng.save(it);
        try eng.ruin(rng, it);
        try eng.recreate(rng);
        const dt = eng.delta(saved);
        if (@as(f64, @floatFromInt(dt)) < temp) {
            try eng.accept(saved, it);
        } else {
            try eng.reject(saved);
        }
        temp *= cfg.cf;
        try eng.afterIter(it, rng);
    }
}
