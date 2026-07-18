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

## M6 — Dispatch / rolling-horizon session API (PULLED UP 2026-07-16) — BUILT 2026-07-18, commit pending validation

`commiv_solve_pdptw_dispatch` (C ABI), `solve_pdptw_dispatch` + `DispatchSession`
(Python), and `POST /solve/pdptw/dispatch` (REST) now expose
`solvePdptwSisrDispatch` through all three doors, stateless at the C/REST
level as designed (`DispatchSession` is the Python-only convenience session).
`examples/dispatch_shift_demo.py` simulates a shift with mid-day order
arrivals. Written by inspection against the plan below, not yet run through
`zig build`/the validation ladder on this machine — see the build+test pass
before treating the acceptance criterion as met.

REPRIORITIZED by owner decision 2026-07-16: M6 moves up the line, to be built
immediately after the planned bench campaign lands (tightness sweep, cuOpt
round, equal-compute n=2000 rerun). Rationale: the Russian market target runs
express delivery with 15–30 min promises measured from ORDER TIME — that is
structurally a rolling-horizon dispatch problem (orders arriving live, locked
prefixes, re-solve), not a pre-booked-slot problem. The static tightness sweep
only proxies it; the real product for that segment IS the dispatch API.
`solvePdptwSisrDispatch` (locked prefixes) already exists in the engine,
benchmark-proven internally — this is exposure work, same shape as M1 was.

The stateful one, deferred out of M1 on purpose. Rolling-horizon re-solve
(`solvePdptwSisrDispatch`, locked prefixes) is what real logistics runs on — but
it needs a session/state API, which is a different shape from the stateless
solve calls.

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

UPDATE 2026-07-16 (status + reorder): M1 DONE (7f5290b, C ABI 0.3.0, verified).
M2 DONE (6aa588a, Python + REST + docs, smoke-verified). M3 substantially done
(MONEY_REAL.md: real prices, real matrices, audited VROOM head-to-heads both
regimes; remaining: tightness sweep + equal-compute n=2000 rerun in flight).
NEW ORDER for the rest: finish bench campaign -> **M6 (dispatch API, pulled up —
express-delivery market)** -> M4 (VROOM migration + waiting-cost demo) -> M5 ->
M7. M4's demo gains the measured waiting numbers from M3 either way.

UPDATE 2026-07-18 (gap-closing round): M6 server-validated bench-grade (all
gates PASS; dispatch-quality number on nyc-1000 window: warm 5s re-solve
$8686.62 vs cold 5s $9218.99 = 5.8% cheaper at equal wall, and 1.07% cheaper
than cold 30s at 1/6th the wall, all 253 committed stops preserved). M4 DONE
(cfd1eb6): POST /compat/vroom accepts an unmodified VROOM request (matrix-based
v1, shipments->PDPTW / jobs->VRPTW) and answers in VROOM's response shape, with
an optional "commiv" block for the money objective. Feature gaps closed the
same night, each behind an identity+speed+smoke perf gate: max route duration
cap (80f8d9d, C ABI 0.4.0), heterogeneous fleet v1 (vehicle types via
commiv_solve_pdptw_typed / vehicle_types= kwarg), driver breaks v1
(break_duration/earliest/latest options; PDPTW-only). Multi-depot: designed,
not built — docs/multidepot-design.md. M5 remains blocked on PyPI/CI accounts.

UPDATE 2026-07-19 (M5 + M7, commit ed4d82e): M5 BUILT — platform wheels
(tools/build_wheels.sh: linux x86_64 manylinux2014 + macOS arm64/x86_64, Zig
cross-compiled, libc-free on Linux so no glibc pinning; clean-venv pip install
smoke green incl. vehicle_types/break_), from-scratch docker image (static
musl commiv-serve, 9.8 MB, health + typed-solve smoked), GH Actions workflow
(.github/workflows/wheels.yml: builds + smokes wheels on every push, publishes
to PyPI + GHCR on v* tags). macOS wheels are cross-compiled and NOT
runtime-tested (no Apple hardware) — nanos() was made portable (comptime
linux-syscall / libSystem clock_gettime split; linux twprobe identity
verified unchanged) but say "untested on macOS" wherever published until
someone runs test_smoke on a Mac. PUBLISHING blocked on exactly one owner
action: PyPI account + PYPI_API_TOKEN repo secret, then `git tag v0.4.0 &&
git push --tags` does the rest. M7 DONE — docs/abi.md (semver + append-only
options contract), capi comptime layout canaries (104 B + every field
offset), tools/abi_check.sh + committed symbol baseline (13 exports).
The 7-milestone plan is now M1-M7 all built; only the PyPI/GHCR publish
button and the multi-depot build remain.

UPDATE 2026-07-19 later: PUBLISHED. Owner added the PyPI token, tag v0.4.0
pushed, CI green end-to-end, and `pip install commiv` from the real index
verified working (fresh venv, money-objective PDPTW solve). THE PLAN'S
ACCEPTANCE CRITERION IS MET. The docker image pushed to
ghcr.io/grechman/commiv-serve:latest — note GHCR packages start PRIVATE:
make it public in the package settings for anonymous `docker pull`.

## What this plan deliberately does NOT include

- No new search machinery. The quality lead is decisive and commercially banked.
- No chasing LKH on symmetric TSP (parity is the win, zero return).
- No promoting multi-eject (measured net-neutral, stays gated off).
- No fixing rbg323/lc1_10_3 (records nobody feels; cuOpt/supercomputer turf).

The gap to close is distribution. Nothing in `src/`'s solver core closes it.
