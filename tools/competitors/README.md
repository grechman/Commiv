# Competitor harness — commiv vs the field on the Moscow benchmark

Adapters that run external VRP solvers on the **same directed `moscow-*.road` matrices**
commiv is benchmarked on, scoring every solution identically (route cost on the true
directed matrix, `depot -> customers -> depot`, capacity-validated by `roadlib.validate`).

`moscow-*` is a custom benchmark (real OSRM output, not a published instance set), so these
adapters exist to compare commiv against solvers that also accept arbitrary directed cost
matrices — there is no embedded optimum.

## Setup

```sh
python3 -m venv venv
./venv/bin/pip install ortools pyvroom scipy numpy
# LKH-3 (build from source):
curl -O http://webhotel4.ruc.dk/~keld/research/LKH-3/LKH-3.0.14.tgz
tar xzf LKH-3.0.14.tgz && cd LKH-3.0.14 && make    # produces ./LKH
```

## Run (one solver, one instance)

```sh
ROAD=../../vendor/road
./venv/bin/python3 ortools_cvrp.py $ROAD/moscow-1000.road 60          # OR-Tools, 60s budget
./venv/bin/python3 vroom_cvrp.py   $ROAD/moscow-1000.road 5           # VROOM, exploration level 5
./venv/bin/python3 lkh_warm.py     $ROAD/moscow-1000.road 300 ./LKH-3.0.14/LKH   # LKH-3 with a feasible sweep warmstart
./venv/bin/python3 ap_lb.py        $ROAD/moscow-1000.road 51 207406   # assignment lower bound (K routes, our cost)
```

Each prints a CSV row: `solver,instance,dim,cost,routes,seconds,valid`.

## Notes

- **LKH-3 needs the warmstart** (`lkh_warm.py`, not `lkh_cvrp.py`): on an explicit directed
  matrix its default WALK initial tour collapses into a few overloaded routes and never
  recovers feasibility at n >= 1000. `lkh_warm.py` builds a capacity-feasible sweep solution
  from the lng/lat coords and hands it to LKH as `INITIAL_TOUR_FILE`.
- **Cap memory** so a runaway never takes down the box (n=5000 holds a 25M-entry matrix):
  `systemd-run --user --scope -p MemoryMax=3G -p MemorySwapMax=0 -p CPUQuota=300% -- <cmd>`.
- `lkh_cvrp.py` is the plain (no-warmstart) LKH adapter, kept to show the failure mode.
