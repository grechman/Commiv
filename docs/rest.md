# commiv REST API

`commiv-serve` is a single static binary: JSON over HTTP, no runtime
dependencies. Every language that can speak HTTP gets near-optimal directed
routes without linking anything.

Four solver families are exposed: `/solve/cvrp`, `/solve/vrptw`, `/solve/pdptw`
(pickup-and-delivery with time windows and a money objective), and `/solve/atsp`.
`/solve/pdptw/dispatch` adds rolling-horizon re-solve around a committed plan.

```sh
zig build serve -Doptimize=ReleaseFast
COMMIV_PORT=8080 ./zig-out/bin/commiv-serve
```

Configuration is environment variables: `COMMIV_HOST` (default `127.0.0.1`),
`COMMIV_PORT` (default `8080`), `COMMIV_MAX_BODY_MB` (request size cap,
default 256).

Requests are handled sequentially — a solve occupies the CPU budget anyway.
Front it with your own queue/replicas if you need concurrency; parallelism
*inside* one solve comes from the `threads` field. The server binds to
localhost by default and has no auth: treat it as an internal service, do not
expose it to the internet as-is.

## Conventions

- Cost matrices are directed: `matrix[a][b]` = cost of going `a -> b`. Rows of
  equal length, values are non-negative integers (e.g. travel seconds).
  A symmetric matrix is just the special case where `m[a][b] == m[b][a]`.
- CVRP/VRPTW: node `0` is the depot, customers are `1..n`, so the matrix is
  `(n+1) x (n+1)` and `demand` has `n+1` entries with `demand[0] = 0`.
- Routes in responses list customer indices in visit order; the depot is
  implied at both ends.
- `seed` (default 1) makes runs reproducible; `threads > 1` (CVRP only) runs
  parallel islands — faster, but the result then depends on the thread count.
- Errors are `{"error": "..."}` with 400 (bad JSON), 404/405, 413 (body cap),
  422 (bad shape or infeasible instance), 500.

## GET /health

```sh
curl http://127.0.0.1:8080/health
# {"status":"ok","version":"0.3.0"}
```

## POST /solve/cvrp

Directed capacitated VRP, uncapped fleet (SISR solver).

| field | type | required | meaning |
|---|---|---|---|
| `matrix` | `int[n+1][n+1]` | yes | directed costs, depot = row/col 0 |
| `demand` | `int[n+1]` | yes | `demand[0] = 0` |
| `capacity` | int | yes | per-vehicle capacity |
| `seed` | int | no (1) | RNG seed |
| `iters` | int | no (300000) | SISR ruin-and-recreate iterations |
| `threads` | int | no (1) | >1 = parallel islands |

```sh
curl -X POST http://127.0.0.1:8080/solve/cvrp -d '{
  "matrix": [[0,10,14,12],[11,0,9,20],[15,8,0,7],[13,18,6,0]],
  "demand": [0,4,6,5],
  "capacity": 10
}'
# {"total_cost":58,"vehicles":2,"routes":[[2,1],[3]]}
```

## POST /solve/vrptw

CVRP fields plus per-node arrays of `n+1` entries. Windows constrain the
**start of service**: a vehicle leaves the depot at t=0, waits for free if it
arrives before `ready[i]`, must start service by `due[i]`, spends `service[i]`
at the stop, and must be back at the depot by `due[0]` (the horizon).

| field | type | meaning |
|---|---|---|
| `ready` | `int[n+1]` | earliest service start, `ready[0] = 0` |
| `due` | `int[n+1]` | latest service start, `due[0]` = depot horizon |
| `service` | `int[n+1]` | service duration, `service[0] = 0` |
| `iters` | int | SISR iterations (0 = 300000 default) |
| `threads` | int | >1 = best-of-K parallel SISR chains |
| `veh_penalty` | int | per-route penalty; > 0 biases toward fewer vehicles |
| `fleet_min` | bool | `true` runs the hierarchical fleet-minimization driver (minimize vehicles first, then distance); needs a wall budget and takes precedence over `threads` |
| `max_vehicles` | int | hard cap on the number of routes (0 = uncapped) |
| `wall_ms` | int | wall-clock budget in ms for the SISR / fleet-min search (0 = bounded by `iters` only; `fleet_min` defaults to 10000 if unset) |
| `max_route_duration` | int | shift-length cap: no route's duration (travel + service + waiting) may exceed it; 0 = uncapped |
| `engine` | string | `"sisr"` — the only engine; field kept for compat |

There is no `time_penalty` (money objective) on VRPTW — that knob is PDPTW-only.

```sh
curl -X POST http://127.0.0.1:8080/solve/vrptw -d '{
  "matrix": [[0,10,14,12],[11,0,9,20],[15,8,0,7],[13,18,6,0]],
  "demand": [0,4,6,5], "capacity": 10,
  "ready": [0,0,0,0], "due": [1000,500,500,500], "service": [0,5,5,5]
}'
# {"total_cost":58,"vehicles":2,"routes":[[2,1],[3]]}
```

## POST /solve/pdptw

Directed pickup-and-delivery with time windows (SISR solver), with an optional
**money objective**. Nodes are `dim = 2*n_pairs + 1`: node `0` is the depot,
and each of the `n_pairs` requests is one pickup node and one delivery node
carried on the **same route, pickup before delivery**, capacity respected along
the whole route.

| field | type | required | meaning |
|---|---|---|---|
| `matrix` | `int[dim][dim]` | yes | directed costs, depot = row/col 0, `dim = 2*n_pairs+1` |
| `pickups` | `int[n_pairs]` | yes | pickup node id of each request (in `1..dim-1`) |
| `deliveries` | `int[n_pairs]` | yes | delivery node id of each request; every node `1..dim-1` must appear exactly once across `pickups`+`deliveries` |
| `demand` | `int[n_pairs]` | yes | load of each request (`> 0`, `<= capacity`) |
| `capacity` | int | yes | per-vehicle capacity |
| `ready` | `int[dim]` | yes | earliest service start, `ready[0] = 0` |
| `due` | `int[dim]` | yes | latest service start, `due[0]` = depot horizon |
| `service` | `int[dim]` | yes | service duration, `service[0] = 0` |
| `seed` | int | no (1) | RNG seed |
| `iters` | int | no (default) | SISR iterations |
| `veh_penalty` | int | no (0) | per-route penalty; > 0 biases toward fewer vehicles |
| `time_penalty` | int | no (0) | **money objective**: cost charged per matrix time-unit of route *duration* (travel + service + unavoidable waiting), on top of distance and `veh_penalty`. 0 = pure distance |
| `fleet_min` | bool | no (false) | run the hierarchical vehicle-minimization driver |
| `max_vehicles` | int | no (0) | positive = the **pinned** driver: target EXACTLY this many vehicles (enterprise fixed fleet), not merely an upper bound |
| `wall_ms` | int | no | wall-clock budget in ms for the wall-driven drivers (0 defaults to 10000) |
| `max_route_duration` | int | no (0) | shift-length cap: no route's duration (travel + service + waiting) may exceed it; 0 = uncapped |
| `vehicle_types` | `int[][3]` | no (`[]`) | heterogeneous fleet (1..8 types): each entry `[capacity, fixed_cost, count]`, count 0 = unlimited. Each route is served by one type (its capacity bounds the route load; its fixed cost replaces `veh_penalty`); the response gains `"types"` = route -> type index. `capacity` above is ignored; incompatible with `fleet_min`/`max_vehicles` |
| `driver_break` | `int[3]` | no (`[]`) | `[duration, earliest, latest]`: every route whose schedule finishes after `earliest` must contain one break of `duration` starting within `[earliest, latest]`. Absorbs waiting first, counts into route duration. Incompatible with `fleet_min`/`max_vehicles` |

Note: `threads` is ignored for PDPTW. The money objective (`time_penalty > 0`)
trades fuel for driver hours wherever waiting exists; VROOM cannot price waiting
time at all.

```sh
curl -X POST http://127.0.0.1:8080/solve/pdptw -d '{
  "matrix": [[0,10,12,14,16],[10,0,6,8,10],[12,6,0,7,9],[14,8,7,0,6],[16,10,9,6,0]],
  "pickups": [1,2], "deliveries": [3,4], "demand": [4,5], "capacity": 10,
  "ready": [0,0,0,0,0], "due": [10000,10000,10000,10000,10000], "service": [0,3,3,3,3],
  "time_penalty": 3
}'
# {"total_cost":45,"vehicles":1,"routes":[[1,2,4,3]]}
```

## POST /solve/pdptw/dispatch

Rolling-horizon PDPTW re-solve: the same instance fields as `/solve/pdptw`,
plus the **current plan** and its **locked prefixes**. This is the
express-delivery / live-dispatch shape — orders arrive mid-shift, some stops
are already committed (in progress or served), and the rest re-optimizes
around them.

| field | type | required | meaning |
|---|---|---|---|
| ...all `/solve/pdptw` fields except `fleet_min`/`max_vehicles`/`vehicle_types`... | | | dispatch keeps the current fleet shape (uniform fleet in v1); `max_route_duration` and `driver_break` are accepted |
| `current` | `int[][]` | no (`[]`) | one array of node ids per existing route, in visit order; `[]` is a legal cold start |
| `locked` | `int[]` | no (`[]`) | one entry per route in `current`; `locked[i]` = how many of that route's **leading** stops are committed and must not move in the result |

The locked-prefix contract: a locked delivery's pickup must also be locked,
in the **same** route (422 otherwise). A node absent from `current` is fine
(unrouted, or a brand-new order). `fleet_min` and `max_vehicles` are not
accepted here — dispatch never resizes the fleet on its own; it only opens or
closes routes as ruin-and-recreate naturally does around the locks.

Node-id stability: when new orders arrive, rebuild the request with a LARGER
`matrix`/`pickups`/`deliveries`/etc. that keeps every existing node's index
unchanged and appends the new pairs at the end — the old `current` plan stays
valid input for the next call.

```sh
curl -X POST http://127.0.0.1:8080/solve/pdptw/dispatch -d '{
  "matrix": [[0,10,12,14,16],[10,0,6,8,10],[12,6,0,7,9],[14,8,7,0,6],[16,10,9,6,0]],
  "pickups": [1,2], "deliveries": [3,4], "demand": [4,5], "capacity": 10,
  "ready": [0,0,0,0,0], "due": [10000,10000,10000,10000,10000], "service": [0,3,3,3,3],
  "time_penalty": 3,
  "current": [[1,2,4,3]], "locked": [4]
}'
# {"total_cost":45,"vehicles":1,"routes":[[1,2,4,3]]}
# locked:[4] locks the WHOLE route, and both requests are already in it, so
# the result is guaranteed to reproduce it verbatim regardless of iters/seed.
```

## POST /compat/vroom

Drop-in [VROOM](https://github.com/VROOM-Project/vroom) compatibility: send a
VROOM request JSON, get a VROOM-shaped response back. An existing VROOM client
points at this URL unchanged and picks up the money objective (route-duration +
waiting pricing) that VROOM cannot express. v1 is **matrix-based and
single-depot on a homogeneous fleet** — it maps the request onto commiv's PDPTW
(from `shipments`) or VRPTW (from `jobs`) and answers in VROOM's own shape.

Request fields honored:

| VROOM field | requirement | maps to |
|---|---|---|
| `matrices.car.durations` | **required**, square `int[dim][dim]` | commiv cost/time matrix (no geocoding; a missing matrix is a 422) |
| `matrices.car.costs` | optional, square `int[dim][dim]` | basis for the reported `cost`; defaults to `durations` |
| `vehicles[].start_index` / `end_index` | `0` or absent | single depot (any other value is a 422) |
| `vehicles[].capacity` | `[c]`, same for all | per-vehicle capacity (heterogeneous fleet is a 422) |
| `vehicles[].time_window` | `[a,b]` | `vehicles[0].time_window[1]` is the depot horizon (`due[0]`) |
| `vehicles[].costs.fixed` | same `F` for all (or all absent) | `veh_penalty = F` (heterogeneous fixed cost is a 422) |
| `shipments[]` | `{amount:[q], pickup{...}, delivery{...}}` | **PDPTW**: one pickup + one delivery per request, same route, pickup first |
| `jobs[]` | `{location_index, delivery/amount, time_windows, service}` | **VRPTW**: one delivery stop per job |
| `pickup`/`delivery`/job `time_windows` | `[[a,b],...]` | first window is used; absent → `[0, horizon]` |
| `commiv` | `{wall_ms, time_penalty, seed}` (optional) | opt into the money objective (`time_penalty` is shipments/PDPTW-only) |

`shipments` and `jobs` are mutually exclusive (both → 422). The task
`location_index` values **are** the matrix rows: v1 requires the matrix to cover
exactly the depot plus every task node once (no unused rows), which is precisely
what `tools/vroom_road_pdptw.py` produces from a `moneyroadbench` dump.

Response is VROOM's shape — `{code, summary, routes, unassigned}` — so a VROOM
client parses it as-is. Every summary/route/step time number is derived by
walking the schedule recurrence over the returned routes (depart depot at
`t=0`, travel, wait until `ready`, serve, return): `summary.duration` and each
`route.duration` are travel time only; `service` and `waiting_time` are broken
out separately; `cost` is travel cost over the cost matrix plus `fixed` per used
vehicle. Errors are VROOM-shaped: `{"code":1,"error":"..."}` with a 4xx status.

```sh
curl -X POST http://127.0.0.1:8080/compat/vroom -d '{
  "matrices": {"car": {"durations": [[0,10,12,14,16],[10,0,6,8,10],
    [12,6,0,7,9],[14,8,7,0,6],[16,10,9,6,0]]}},
  "vehicles": [{"id":1,"start_index":0,"end_index":0,"capacity":[10],
    "time_window":[0,10000],"costs":{"fixed":1000}}],
  "shipments": [
    {"amount":[4],"pickup":{"id":11,"location_index":1,"service":3},
     "delivery":{"id":12,"location_index":3,"service":3}},
    {"amount":[5],"pickup":{"id":21,"location_index":2,"service":3},
     "delivery":{"id":22,"location_index":4,"service":3}}]
}'
# {"code":0,"summary":{"cost":...,"routes":1,"unassigned":0,"duration":...,
#   "service":...,"waiting_time":...},"routes":[{"vehicle":1,"steps":[
#   {"type":"start","location_index":0,...}, {"type":"pickup",...}, ...,
#   {"type":"end","location_index":0,...}],...}],"unassigned":[]}
```

Acceptance self-test: `tools/vroom_compat_selftest.py` POSTs a real
`vroom_road_pdptw.py` request unmodified, plus a hand-built `jobs` request and a
no-matrix negative case, and asserts `code:0`, full service, and that every
response number reconciles with an independent schedule walk.

## POST /solve/atsp

Pure directed ordering, no capacity. `matrix` is `n x n`; optional `seed`,
`trials` (search budget, 0 = default).

```sh
curl -X POST http://127.0.0.1:8080/solve/atsp -d '{
  "matrix": [[0,1,9,9],[9,0,1,9],[9,9,0,1],[1,9,9,0]]
}'
# {"cost":4,"tour":[0,1,2,3]}
```

## Long-run quality levers (VRPTW)

`POST /solve/vrptw` accepts four optional fields, all default-off: `polish` (bool),
`stress_rate` (float), `tabu_tenure` (int), `marathon` (bool). Measured best together
("combo": `polish=true, stress_rate=0.5, tabu_tenure=10000, marathon=true`) on runs of
1M+ iterations (~30 s and up at n=1000); below that budget leave them off. `marathon`
is also accepted by `/solve/cvrp`. Both `/solve/cvrp` and `/solve/vrptw` accept
`nbr_key` ("sum" default | "min" | "out") and `gk` (int, 0 = auto) - the granular
neighbor-list key and size.
`nbr_key: "min"` is the measured lever for strongly one-way street networks (NYC-like);
near-symmetric or mildly directed cities measured best on the default. Same seed + same
flags stays fully deterministic.

## Binary matrix framing (CMV1)

The matrix is 90%+ of a large body, and JSON-parsing it dominates request
handling from n≈2000 (at n=5000 the matrix alone is ~175 MB of JSON text).
Every `/solve/*` path also accepts a binary framing on the **same URL** —
the server sniffs the leading magic, no header or path changes needed:

```
"CMV1"                      4 bytes, magic
header_len                  u32, little-endian
header                      header_len bytes: the endpoint's JSON body, minus "matrix"
matrix                      dim*dim u32 values, little-endian, row-major
```

`dim` is inferred from the byte count (which must be exactly `4*dim*dim`);
for CVRP/VRPTW `dim = n+1` with the depot at index 0, for ATSP `dim = n`.
Responses and results are byte-identical to the JSON encoding — same seed,
same routes. Measured at n=2000 (moscow-2000): body 22.2 → 16.0 MB,
server peak RSS 75 → 47 MB, and the parse stage drops from ~0.25 s of JSON
to one memcpy.

Python with numpy (`matrix` any integer array-like):

```python
import json, struct, numpy as np, requests

header = json.dumps({"demand": demand, "capacity": 10, "seed": 1}).encode()
body = (b"CMV1" + struct.pack("<I", len(header)) + header
        + np.ascontiguousarray(matrix, dtype="<u4").tobytes())
resp = requests.post("http://127.0.0.1:8080/solve/cvrp", data=body)
```

## Clients

Python (or use the native binding in [`bindings/python/`](../bindings/python/),
which skips HTTP entirely):

```python
import requests

resp = requests.post("http://127.0.0.1:8080/solve/cvrp", json={
    "matrix": matrix, "demand": demand, "capacity": 10, "seed": 1,
})
resp.raise_for_status()
sol = resp.json()  # {"total_cost": ..., "vehicles": ..., "routes": [[...], ...]}
```

JavaScript / TypeScript:

```js
const resp = await fetch("http://127.0.0.1:8080/solve/cvrp", {
  method: "POST",
  body: JSON.stringify({ matrix, demand, capacity: 10, seed: 1 }),
});
if (!resp.ok) throw new Error((await resp.json()).error);
const { total_cost, routes } = await resp.json();
```

Go:

```go
body, _ := json.Marshal(map[string]any{
    "matrix": matrix, "demand": demand, "capacity": 10, "seed": 1,
})
resp, err := http.Post("http://127.0.0.1:8080/solve/cvrp",
    "application/json", bytes.NewReader(body))
// decode: struct { TotalCost uint64 `json:"total_cost"`; Routes [][]int `json:"routes"` }
```

Anything else: it is one POST with a JSON body. If your language can `curl`,
it can route.
