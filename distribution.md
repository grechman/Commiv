# distribution.md — the build order for turning the benchmark lead into adoption

Written 2026-07-16, off the competitive-intelligence read of the 2026-07-13/14
campaign + 2026-07-15 rebench.

## The one fact this plan exists to fix

commiv wins the benchmark war (3x HGS on Uchoa X, +11.6%/$1.27M money vs VROOM,
48x faster than VROOM at n=5000, 352/352 PDPTW complete where VROOM drops
shipments) and reaches **zero external users**. The competitors that lose on
quality — VROOM, OR-Tools — win the only metric that ships: they are already
wired into everyone's stack.

And here is the kicker, verified in the source today: **the entire competitive
wedge is unreachable through the doors.** The C ABI (`src/capi.zig`), Python
(`bindings/python`), and REST (`server/main.zig`) each expose ONLY cvrp, vrptw,
atsp. PDPTW, the money objective, fleet-min, pinned fleet, and dispatch —
everything that beats VROOM — cannot be called by an outside user. They only run
inside `pdptwbench`/`vrptwbench`, calling internal Zig directly.

So the strategic task is not algorithmic (we're already at the top of the
quality axis; squeezing rbg323 or lc1_10_3 returns nothing commercially). It is
distributional: move right on the adoption axis. **Every milestone below is
distribution, not search. Do not add search machinery in this plan.**

Acceptance criterion for the whole plan (one line): a routing engineer who has
never seen this repo can, in one afternoon, run their own directed instance with
a money objective and get a complete plan back — via pip or curl, without a Zig
toolchain.

---

## M1 — Expose the wedge through the C ABI  (do this first; nothing else matters until it lands)

The differentiators exist and are benchmark-proven; this is plumbing, not
invention, which is why it is low-risk despite being the biggest unlock.

Build, in `src/capi.zig` + `include/commiv.h`:
- `commiv_solve_pdptw` — pickup/delivery pairs, TW, capacity; the fleet-min
  driver (`solvePdptwSisrFleetMinParallel`) behind it. This is the 352/352
  completeness win.
- A `money` option block on the solve calls: `time_penalty` + `veh_penalty`
  (+ dist implicit) exposed as fields, so a caller can price driver time. This
  is the +11.6% structural win vs VROOM — it MUST be reachable.
- `commiv_solve_vrptw` gains a `fleet_min` + `max_vehicles` path
  (`solveVrptwSisrFleetMin`, shipped 1a0d4cf). Currently only the uncapped SISR
  is wired.
- Pinned fleet (`max_vehicles` hard cap) and a `wall_ms` budget field on every
  solve (time-boxed, not iters — external users think in wall time).

Explicitly DEFER from M1: dispatch/rolling-horizon (stateful, needs a session
API — its own milestone, M6).

Acceptance: a C program links `libcommiv`, feeds a Li&Lim PDPTW instance with a
money objective, and gets back the same fleet/cost the `pdptwbench` harness
produces for that instance and seed (assert equality against a known cell).
Errors are typed, never a crash. Checkpoint-commit before starting.

## M2 — Python + REST follow mechanically

Both wrap the C ABI, so once M1 lands this is thin.

- Python (`bindings/python`): add `solve_pdptw`, `money=` kwargs on all solvers,
  `fleet_min=`/`max_vehicles=`, `wall_ms=`. numpy-in / list-out, typed
  exceptions, docstrings that show the money objective in the first example.
- REST (`server/main.zig`): add `/solve/pdptw`, accept a `money` block and
  `fleet_min`/`max_vehicles`/`wall_ms` on all four endpoints. JSON in/out,
  same schema shape as VROOM's request where possible (see M4).

Acceptance: `test_smoke.py` covers all four solvers incl. PDPTW + money and
passes; a `curl` against each REST endpoint returns a validated plan. The Python
money example reproduces a known `pdptwbench` cost.

## M3 — Credibility: independent reproduction + one real-data money proof

This is the moat when you are the unknown challenger, and it is currently the
weakest point (every number is one machine, one author; a rounding artifact —
r1_4_1=39 — already slipped through once).

- **Independent reproduction**: run the campaign's headline cells (Uchoa X,
  directed road CVRP/VRPTW, PDPTW money) on a *second* machine (not the winserver
  or the zenbook — a cloud box or a colleague's) via the documented
  `zig build ... -Doptimize=ReleaseFast` + campaign driver. Publish the diff.
  Converts "one person's claims" into "verified."
- **One real-data money proof**: the money bench is synthetic-caveated
  (declared prices, academic instances, VROOM scored on an objective it can't
  express). Run the money objective ONCE on a real directed OSRM matrix (we
  already have moscow/nyc/berlin road instances) with a real driver wage +
  real per-km cost, commiv vs VROOM, and report it honestly next to the
  synthetic number. This makes the headline claim defensible instead of
  "a capability demonstration."

Acceptance: a `REPRODUCTION.md` with the second-machine numbers within wall
noise of `BENCHMARKS.md`, and one real-matrix money cell with the cost model
stated and the command next to the number (per the "no unreproducible number"
rule).

## M4 — The VROOM migration path (the wedge, weaponized)

VROOM's exact weaknesses are commiv's exact strengths: it can't price time, it
drops shipments, it loses on directed matrices. Make switching trivial.

- A `vroom-compat` adapter: accept VROOM's JSON request format (jobs/shipments/
  vehicles/matrices) directly on the REST server, so an existing VROOM caller
  repoints the URL and it just works.
- A one-command demo: feed a VROOM request that includes waiting-heavy routes,
  show commiv's plan is complete + cheaper under a money objective, side by side.
  This is the landing-page artifact.

Acceptance: an unmodified VROOM `-i request.json` payload solved by commiv via
the compat endpoint, returning a complete plan; the demo shows the dollar delta
with the honest caveat inline.

## M5 — Zero-friction install + task-oriented docs

"Try it" must be 30 seconds, not a Zig build.

- Prebuilt Python wheels (`pip install commiv`) for linux x86_64 (manylinux) +
  macOS arm64, CI-built. This is the single biggest friction cut for the
  Python-native audience where the users actually are.
- A REST container (`docker run commiv-serve`) for the ops audience.
- Docs that sell the wedge, not API-reference sludge: "price your driver time",
  "never drop a shipment", "directed real-road routing" — each a runnable
  10-line example, money objective in the first one. NOT a symbol dump.

Acceptance: `pip install commiv` on a clean machine, then a 10-line money-
objective script runs green with no toolchain. `docker run` serves `/solve/pdptw`.

## M6 — Dispatch / rolling-horizon session API (the real-ops feature, last)

The stateful one, deferred out of M1 on purpose. Rolling-horizon re-solve
(`solvePdptwSisrDispatch`, locked prefixes) is what real logistics runs on — but
it needs a session/state API, which is a different shape from the stateless
solve calls. Do it once the stateless wedge is proven and adopted, not before.

Acceptance: a session accepts an initial fleet + committed (locked) prefixes,
re-optimizes around new orders, and never moves a locked stop; a demo simulates
a shift with mid-day order arrivals.

## M7 — ABI stability contract

Only once the surface is proven by real integrators, so we're not versioning a
shape we still change.

- Semver on the C ABI, a documented stability guarantee, `commiv_version()`
  gated compatibility. So an integrator can depend on it without fear.

Acceptance: a versioned `commiv.h` with a written no-break policy and a CI check
that flags ABI changes.

---

## Order rationale (why this sequence)

1. **M1 is the gate.** Every downstream milestone (Python, REST, migration,
   docs) exposes or wraps what M1 makes callable. Until the money objective and
   PDPTW are reachable in C, the product a user can actually touch is the
   commoditized cvrp/vrptw/atsp — exactly where the entrenched incumbents win.
2. **M2 is nearly free once M1 lands** (thin wrappers), so it rides immediately.
3. **M3 before marketing anything.** The reproduction + real-data proof
   inoculate against the one real reputational threat (single-machine, single-
   author numbers) *before* the migration story (M4) puts those numbers in front
   of skeptics.
4. **M4 is the actual sales wedge** — but it's worthless pointed at numbers that
   haven't survived scrutiny (M3) or at features that aren't exposed (M1/M2).
5. **M5 removes the last friction** so the wedge can spread.
6. **M6/M7 are for when there are integrators to serve** — building them earlier
   is polishing a door nobody has opened.

## What this plan deliberately does NOT include

- No new search machinery. The quality lead is decisive and commercially banked.
- No chasing LKH on symmetric TSP (parity is the win, zero return).
- No promoting multi-eject (measured net-neutral, stays gated off).
- No fixing rbg323/lc1_10_3 (records nobody feels; cuOpt/supercomputer turf).

The gap to close is distribution. Nothing in `src/`'s solver core closes it.
