# commiv REST API

`commiv-serve` is a single static binary: JSON over HTTP, no runtime
dependencies. Every language that can speak HTTP gets near-optimal directed
routes without linking anything.

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
# {"status":"ok","version":"0.2.0"}
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
| `engine` | string | `"sisr"` (default) or `"ils"` (legacy; uses `rounds`/`restarts`) |

```sh
curl -X POST http://127.0.0.1:8080/solve/vrptw -d '{
  "matrix": [[0,10,14,12],[11,0,9,20],[15,8,0,7],[13,18,6,0]],
  "demand": [0,4,6,5], "capacity": 10,
  "ready": [0,0,0,0], "due": [1000,500,500,500], "service": [0,5,5,5]
}'
# {"total_cost":58,"vehicles":2,"routes":[[2,1],[3]]}
```

## POST /solve/atsp

Pure directed ordering, no capacity. `matrix` is `n x n`; optional `seed`,
`trials` (search budget, 0 = default).

```sh
curl -X POST http://127.0.0.1:8080/solve/atsp -d '{
  "matrix": [[0,1,9,9],[9,0,1,9],[9,9,0,1],[1,9,9,0]]
}'
# {"cost":4,"tour":[0,1,2,3]}
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
