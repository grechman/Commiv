# Multi-depot PDPTW — design (v1, not yet implemented)

Written 2026-07-18 as the design half of the gap-closing round that shipped the
max-route-duration cap, heterogeneous fleet v1, and driver breaks v1. This doc
grounds the next feature in the actual code so implementation is mechanical.

## The model (v1 scope)

Depots ride on the vehicle-type table shipped tonight: `VehType` gains
`depot_node: u32` (default 0). A route's start AND end depot is its type's
depot — no open routes, no start!=end in v1 (that generality is cheap to add
later but doubles the test matrix now).

The instance carries depot rows as ORDINARY matrix nodes: for D depots and n
pairs, dim = D + 2n, nodes `0..D-1` are depots, the rest are stops. `pair_of`
/ `is_pickup` / `demand_signed` get depot entries like node 0 has today
(self-pair, not pickup, zero demand). `ready/due/service` per depot: each
depot's own horizon — `due[depot]` bounds that depot's routes, which the Tws
algebra picks up for free once the boundary anchors are parameterized.

Why type-table, not per-route free choice: the type ledger (count accounting,
snapshot/rollback via `saved_type_used` + `Snap.rtype`) already restores a
route's type across accept/reject. Depot-per-type rides that machinery with
ZERO new rollback state — the single most dangerous part of any fleet feature
is already paid for.

## Every depot-anchored site (from tonight's tree, `src/pdptw_sisr.zig`)

The engine hardcodes depot = node 0 at exactly these anchors. Each becomes a
`dep = s.routeDepot(ri)` parameter (one branch per route, off-path returns 0 —
same pattern as `routeCap`/`penOf` shipped tonight):

Distance / schedule walks (replace literal `0` end anchors):
- `routeDuration` — `pdptw_sisr.zig:180` (`var prev: usize = 0`) and `:185`
  (`d(prev, 0)` + `Tws.depotNode` tail). Needs a `dep` parameter.
- `walkWithBreak` — `:203`/`:215` and `:235` (both passes start `prev = 0`,
  end `+ d(prev, 0)`); same `dep` parameter.
- `arcSum` — `:370` (`d(0, items[0])` ... `d(last, 0)`).
- `seqFeasible` — `:560`/`:566`.
- Freshen prefix/suffix builds — `:533` (`pre_t[0] = depotNode`), `:536`
  (`prev = 0`), `:543` (`suf_t[L] = depotNode`), `:549` (`nxt = ... else 0`),
  `:552` (`tail_d` tail arc). These make ALL O(1) insertion evals
  depot-correct automatically, because `pre_t`/`suf_t`/`pre_d`/`tail_d`
  already carry the boundary.
- Insertion evals only touch the depot through `prev_a`/`nxt` when a == 0 or
  b == L — `:683`/`:687`/`:695` (evalPairInsert), `:745..762` (break
  variant), `:807..813` (viol), `:1609..1618` (parallel copy). Those all read
  the freshened arrays plus `it[...] else 0` — the `else 0` becomes
  `else dep`.
- Singleton opening estimate — `:1237` and `:1699`
  (`d(0,p) + d(p,q) + d(q,0)`): evaluate per candidate type (the depot is a
  consequence of the type choice, so `chooseType` returns (type, depot) and
  the singleton is priced with that depot's arcs — cheapest-type-first stays,
  ties can extend to nearest-depot).
- Oracles: `pdptw.validate` step-5 walk and `routeCost` assume depot 0 —
  add `validateMultiDepot(inst, routes, depots_per_route)` alongside
  (untouched `validate`, same pattern as `validateTyped` /
  `validateWithBreak` tonight).
- Construction (`pdp.construct`) stays single-depot on node 0: the seed is
  depot-agnostic-bad but the search reassigns (routes empty and reopen with
  their cheapest type/depot); measured-if-needed later.
- kNN `buildNeighbors` iterates customers `1..n` — with D depots it iterates
  `D..dim-1`; the `(c - 1) * gk` indexing becomes `(c - D) * gk`. Mechanical
  but easy to fumble: every `gran[(x - 1) * s.gk + t]` site (ruin `:1119`,
  squeeze, recreate discovery, kick) must shift the same way — grep
  `* s.gk` and fix all of them together.

## Off-path bit-identity strategy

Same discipline as tonight's two features, both of which passed EXACT
identity gates:
- `routeDepot(ri)` short-circuits to 0 when no multi-depot table is present;
  every literal-0 anchor is replaced by a value that IS 0 on the off path, so
  the arithmetic is unchanged instruction-for-instruction modulo one
  predictable branch per route-level call.
- The freshen arrays are rebuilt per route anyway; seeding them from
  `depotNodeAt(inst, dep)` with dep = 0 produces identical Tws values.
- Gate: twprobe identity + iteration-bound pdptwbench/moneyroadbench equality
  + wall medians within ±4%, exactly tonight's PERF_GATE.

## Door surface

- capi: the typed entry family grows `veh_type_depot: ?[*]const u32`
  (NULL = all depot 0, which also keeps `commiv_solve_pdptw_typed` callers
  from tonight source-compatible; the ABI gets a NEW entry
  `commiv_solve_pdptw_typed_v2`-style only if the arg-append breaks someone —
  it doesn't, C has no overloads and we control the header). Result side:
  `commiv_routes_type` already returns the type index; the caller maps type
  -> depot itself, no new accessor.
- Python: `vehicle_types=[(cap, fixed, count, depot?)]` — 3-tuples keep
  today's meaning, 4-tuples opt into depots.
- REST/vroom-compat: VROOM `vehicles[].start_index/end_index` currently 422s
  when != 0 — multi-depot removes that 422 by mapping start_index to a
  depot-typed fleet entry (start MUST equal end in v1). This is the feature's
  real sales surface: VROOM requests with per-vehicle depots start working.

## Perf-gate plan

1. Baseline = the commit before the multi-depot diff; identity suite exactly
   as tonight (twprobe, lc101/lr112 iteration-bound, money nyc-1000
   iteration-bound, 3-rep medians ±4%).
2. Feature smoke: synthesize a 2-depot nyc-100 (split the window dump's
   customers around two depot rows), assert every route starts/ends at its
   type's depot via the independent oracle, and that forcing all types onto
   depot 0 reproduces the single-depot cost for the same seed.
3. The `(c - D)` kNN re-indexing gets its own unit test (off-by-one here
   corrupts ruin/recreate silently — it would still produce feasible plans,
   just bad ones, which no oracle catches; only the identity gate does).

## Honest effort estimate

The mechanical surface is ~25 sites in pdptw_sisr.zig + one oracle + door
plumbing. With tonight's ledger machinery already in place: one focused
session (4-6 h) for engine + oracle + unit tests, plus a second session for
doors + vroom-compat depot mapping + the full gate. The risky 20% is the kNN
re-indexing and the construct-seed interaction under scarce depot-typed
counts (seed routes that fit no available type all land in the request bank —
fine at nyc-100 scale, needs a look at n=2000 before claiming production).
