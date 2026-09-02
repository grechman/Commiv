# C ABI stability contract (M7)

Effective from libcommiv **0.5.0** (2026-07-19; 0.4.0 contract superseded by the authorized 0.5.0 break below). This is the promise an
integrator can build against.

## Versioning

`commiv_version()` returns "MAJOR.MINOR.PATCH" (static storage). Semantics:

- **PATCH**: no ABI or behavior contract changes. Solver internals may change
  (results for a given seed MAY differ between patch releases — determinism
  is guaranteed within one binary, not across releases).
- **MINOR**: ABI grows, never breaks. Concretely: new exported functions may
  appear; `commiv_options` may grow by APPENDING fields (zero value of every
  new field = previous behavior, so callers that memset and set only the
  fields they know keep working — this is why the struct has no version tag);
  existing function signatures, error codes, and struct field offsets never
  change.
- **MAJOR**: anything else (removing/renaming exports, reordering fields,
  changing semantics of an existing zero-default). None planned.

## The rules that make appends safe

1. Callers MUST zero-initialize `commiv_options` (memset or `= {0}`), never
   partially initialize by position. Documented in commiv.h since 0.1.
2. New `commiv_options` fields are appended only, each with "0 = previous
   behavior", keeping 8-byte alignment (pad explicitly, as `reserved` does).
3. `commiv_routes` stays opaque; access only through `commiv_routes_*`.
   New accessors may appear (e.g. `commiv_routes_type` in 0.4.0); existing
   ones never change signature or semantics.
4. Error codes are append-only; existing negative values keep their meaning.

## History

| version | options size | change |
|---|---|---|
| 0.3.0 | 80 B | M1/M2 surface: cvrp/vrptw/pdptw/atsp + money knobs |
| 0.4.0 | 88 B | + `max_route_duration` |
| 0.4.0 | 104 B | + `break_duration/earliest/latest` + pad; `commiv_solve_pdptw_typed`, `commiv_solve_pdptw_dispatch`, `commiv_routes_type` (all additions within one release cycle, pre-first-publish) |
| 0.5.0 | 88 B | BREAKING (authorized pre-adoption cleanup): removed `vrptw_rounds`/`vrptw_restarts` fields (legacy VRPTW ILS engine deleted; SISR is the only VRPTW engine), all later field offsets shift down 16 B. Also removed from the Zig API: `solveVrptw`/`VrptwParams`, `solvePdptw`/`PdpParams`, `solveCvrpMulti`/`CvrpMultiParams`, PDPTW `eval_threads`, `eject_k` (both engines). REST: `engine:"ils"` and `rounds`/`restarts` request fields gone. |
| 0.5.1 | 88 B | No ABI change: same exports, same `commiv_options` layout. Performance/memory pass (see [bench/README.md](../bench/README.md)). Behavior note, allowed by the PATCH rule above: a large native ATSP (above about n=2900) on a degenerate matrix no longer multiplies the requested `trials` by 100, and its candidate build breaks key ties differently, so a given seed can produce a different tour than 0.5.0. |
| 0.6.0 | 88 B | No ABI change: same 13 exports, same `commiv_options` layout (`tools/abi_check.sh` clean). MINOR for the Zig API: root now exports `commiv.version`, the PDPTW fleet-min/pinned/dispatch drivers, `VehType`, `MAX_VEH_TYPES`, and `pdptw.buildPairing`. Behavior note: PDPTW blinks are now a per-gap hash instead of draws from the shared PRNG, so a given seed produces a different PDPTW solution than 0.5.1 (equal-wall quality unchanged, see bench/README.md). |

## Enforcement

- `src/capi.zig` carries comptime tests pinning `@sizeOf(CommivOptions)`
  and every field offset — an accidental reorder/resize fails `zig build test`.
- `tools/abi_check.sh` diffs the shared library's exported `commiv_*`
  symbols against the committed baseline `tools/abi_symbols.txt`; a removed
  symbol fails, a new one demands a baseline update in the same commit
  (which is what makes the addition deliberate and reviewed).
- CI (`.github/workflows/wheels.yml`) builds wheels from the same tree, so a
  published wheel can never drift from the checked ABI.
