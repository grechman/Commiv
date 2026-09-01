#!/usr/bin/env python3
import hashlib
import json
import math
import os
import struct
import subprocess
import sys
import time
import urllib.error
import urllib.request

ROOT = sys.argv[1]
PORT = int(sys.argv[2])
OUT = sys.argv[3]
BASE = f"http://127.0.0.1:{PORT}"


def lilim(path, scale=1000):
    lines = [l.split() for l in open(path).read().splitlines() if l.strip()]
    xs, ys, dem, rdy, due, srv, pk, dl = [], [], [], [], [], [], [], []
    for f in lines[1:]:
        xs.append(float(f[1]))
        ys.append(float(f[2]))
        dem.append(int(f[3]))
        rdy.append(int(f[4]) * scale)
        due.append(int(f[5]) * scale)
        srv.append(int(f[6]) * scale)
        pk.append(int(f[7]))
        dl.append(int(f[8]))
    dim = len(xs)
    m = [
        [
            int(round(math.hypot(xs[a] - xs[b], ys[a] - ys[b]) * scale))
            for b in range(dim)
        ]
        for a in range(dim)
    ]
    pickups = [c for c in range(1, dim) if dem[c] > 0]
    deliveries = [dl[c] for c in pickups]
    demand = [dem[c] for c in pickups]
    return dict(
        matrix=m,
        pickups=pickups,
        deliveries=deliveries,
        demand=demand,
        capacity=int(lines[0][1]),
        ready=rdy,
        due=due,
        service=srv,
    )


def post(path, body, raw=False):
    data = body if raw else json.dumps(body).encode()
    req = urllib.request.Request(BASE + path, data=data, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=600) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


def get(path):
    req = urllib.request.Request(BASE + path, method="GET")
    try:
        with urllib.request.urlopen(req, timeout=60) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as e:
        return e.code, e.read()


M4 = [[0, 10, 14, 12], [11, 0, 9, 20], [15, 8, 0, 7], [13, 18, 6, 0]]
PDP = lilim(os.path.join(ROOT, "vendor/pdptw/lc101.txt"))
PDP_SMALL = dict(
    matrix=[
        [0, 10, 12, 14, 16],
        [10, 0, 6, 8, 10],
        [12, 6, 0, 7, 9],
        [14, 8, 7, 0, 6],
        [16, 10, 9, 6, 0],
    ],
    pickups=[1, 2],
    deliveries=[3, 4],
    demand=[4, 5],
    capacity=10,
    ready=[0] * 5,
    due=[10000] * 5,
    service=[0, 3, 3, 3, 3],
)

cases = []
cases.append(("health", "GET", "/health", None))
cases.append(
    (
        "cvrp_readme",
        "POST",
        "/solve/cvrp",
        dict(matrix=M4, demand=[0, 4, 6, 5], capacity=10),
    )
)
cases.append(
    (
        "cvrp_iters",
        "POST",
        "/solve/cvrp",
        dict(
            matrix=M4,
            demand=[0, 4, 6, 5],
            capacity=10,
            seed=7,
            iters=5000,
            nbr_key="min",
        ),
    )
)
cases.append(
    (
        "vrptw_readme",
        "POST",
        "/solve/vrptw",
        dict(
            matrix=M4,
            demand=[0, 4, 6, 5],
            capacity=10,
            ready=[0, 0, 0, 0],
            due=[1000, 500, 500, 500],
            service=[0, 5, 5, 5],
            seed=7,
            iters=5000,
        ),
    )
)
cases.append(
    (
        "vrptw_fleetmin",
        "POST",
        "/solve/vrptw",
        dict(
            matrix=M4,
            demand=[0, 4, 6, 5],
            capacity=10,
            ready=[0, 0, 0, 0],
            due=[1000, 500, 500, 500],
            service=[0, 5, 5, 5],
            fleet_min=True,
            wall_ms=300,
        ),
    )
)
cases.append(
    ("pdptw_small", "POST", "/solve/pdptw", dict(PDP_SMALL, seed=5, iters=2000))
)
cases.append(
    (
        "pdptw_small_money",
        "POST",
        "/solve/pdptw",
        dict(PDP_SMALL, seed=5, iters=2000, time_penalty=3),
    )
)
cases.append(
    (
        "pdptw_small_types",
        "POST",
        "/solve/pdptw",
        dict(PDP_SMALL, seed=5, iters=2000, vehicle_types=[[6, 100, 0], [10, 250, 1]]),
    )
)
cases.append(
    (
        "pdptw_small_break",
        "POST",
        "/solve/pdptw",
        dict(PDP_SMALL, seed=5, iters=2000, driver_break=[5, 10, 40]),
    )
)
cases.append(
    (
        "pdptw_lc101",
        "POST",
        "/solve/pdptw",
        dict(PDP, seed=1, iters=20000, veh_penalty=10_000_000),
    )
)
cases.append(
    (
        "pdptw_lc101_money",
        "POST",
        "/solve/pdptw",
        dict(PDP, seed=1, iters=20000, veh_penalty=280000, time_penalty=1),
    )
)
cases.append(
    (
        "pdptw_lc101_maxdur",
        "POST",
        "/solve/pdptw",
        dict(PDP, seed=1, iters=20000, max_route_duration=900000),
    )
)
cases.append(
    (
        "atsp_ring",
        "POST",
        "/solve/atsp",
        dict(matrix=[[0, 1, 9, 9], [9, 0, 1, 9], [9, 9, 0, 1], [1, 9, 9, 0]]),
    )
)
vroom = dict(
    matrices=dict(car=dict(durations=PDP_SMALL["matrix"])),
    vehicles=[dict(id=1, capacity=[10], time_window=[0, 10000], costs=dict(fixed=100))],
    shipments=[
        dict(
            amount=[4],
            pickup=dict(id=11, location_index=1, service=3),
            delivery=dict(id=12, location_index=3, service=3),
        ),
        dict(
            amount=[5],
            pickup=dict(id=21, location_index=2, service=3),
            delivery=dict(id=22, location_index=4, service=3),
        ),
    ],
    commiv=dict(seed=5, wall_ms=200, time_penalty=2),
)
cases.append(("compat_vroom_ships", "POST", "/compat/vroom", vroom))
vroom_jobs = dict(
    matrices=dict(car=dict(durations=M4)),
    vehicles=[dict(id=1, capacity=[10], time_window=[0, 1000])],
    jobs=[
        dict(id=1, location_index=1, delivery=[4], service=5),
        dict(id=2, location_index=2, delivery=[6], service=5),
        dict(id=3, location_index=3, delivery=[5], service=5),
    ],
    commiv=dict(seed=7, wall_ms=200),
)
cases.append(("compat_vroom_jobs", "POST", "/compat/vroom", vroom_jobs))

err = [
    ("err_notfound", "POST", "/nope", {}),
    ("err_health_post", "POST", "/health", {}),
    ("err_badjson", "POST", "/solve/cvrp", b"{not json", True),
    ("err_cvrp_nomatrix", "POST", "/solve/cvrp", dict(demand=[0, 1], capacity=1)),
    (
        "err_cvrp_notsquare",
        "POST",
        "/solve/cvrp",
        dict(matrix=[[0, 1], [1]], demand=[0, 1], capacity=1),
    ),
    (
        "err_cvrp_demandlen",
        "POST",
        "/solve/cvrp",
        dict(matrix=M4, demand=[0, 4], capacity=10),
    ),
    (
        "err_cvrp_infeasible",
        "POST",
        "/solve/cvrp",
        dict(matrix=M4, demand=[0, 4, 11, 5], capacity=10),
    ),
    (
        "err_vrptw_len",
        "POST",
        "/solve/vrptw",
        dict(
            matrix=M4,
            demand=[0, 4, 6, 5],
            capacity=10,
            ready=[0, 0, 0],
            due=[1, 1, 1, 1],
            service=[0, 0, 0, 0],
        ),
    ),
    ("err_pdptw_dim", "POST", "/solve/pdptw", dict(PDP_SMALL, matrix=M4)),
    ("err_pdptw_lens", "POST", "/solve/pdptw", dict(PDP_SMALL, deliveries=[3])),
    ("err_pdptw_tw_len", "POST", "/solve/pdptw", dict(PDP_SMALL, ready=[0, 0])),
    ("err_pdptw_cap0", "POST", "/solve/pdptw", dict(PDP_SMALL, capacity=0)),
    ("err_pdptw_depot", "POST", "/solve/pdptw", dict(PDP_SMALL, ready=[5, 0, 0, 0, 0])),
    (
        "err_pdptw_types9",
        "POST",
        "/solve/pdptw",
        dict(PDP_SMALL, vehicle_types=[[10, 1, 0]] * 9),
    ),
    (
        "err_pdptw_types_fleetmin",
        "POST",
        "/solve/pdptw",
        dict(PDP_SMALL, vehicle_types=[[10, 1, 0]], fleet_min=True),
    ),
    (
        "err_pdptw_break_shape",
        "POST",
        "/solve/pdptw",
        dict(PDP_SMALL, driver_break=[5, 10]),
    ),
    (
        "err_pdptw_break_fleetmin",
        "POST",
        "/solve/pdptw",
        dict(PDP_SMALL, driver_break=[5, 10, 40], max_vehicles=1),
    ),
    (
        "err_pdptw_break_order",
        "POST",
        "/solve/pdptw",
        dict(PDP_SMALL, driver_break=[5, 40, 10]),
    ),
    (
        "err_pdptw_type_cap0",
        "POST",
        "/solve/pdptw",
        dict(PDP_SMALL, vehicle_types=[[0, 1, 0]]),
    ),
    ("err_pdptw_selfpair", "POST", "/solve/pdptw", dict(PDP_SMALL, deliveries=[1, 4])),
    ("err_pdptw_dup", "POST", "/solve/pdptw", dict(PDP_SMALL, deliveries=[3, 3])),
    ("err_pdptw_demand", "POST", "/solve/pdptw", dict(PDP_SMALL, demand=[11, 5])),
    (
        "err_pdptw_coverage",
        "POST",
        "/solve/pdptw",
        dict(
            PDP_SMALL,
            pickups=[1, 2],
            deliveries=[3, 4],
            matrix=PDP_SMALL["matrix"],
            ready=[0] * 5,
            due=[10000] * 5,
            service=[0] * 5,
            demand=[1, 1],
            capacity=10,
            seed=1,
            iters=10,
            extra=1,
        ),
    ),
    (
        "err_dispatch_locklen",
        "POST",
        "/solve/pdptw/dispatch",
        dict(PDP_SMALL, current=[[1, 3]], locked=[]),
    ),
    (
        "err_dispatch_lockexceeds",
        "POST",
        "/solve/pdptw/dispatch",
        dict(PDP_SMALL, current=[[1, 3]], locked=[3]),
    ),
    (
        "err_dispatch_badnode",
        "POST",
        "/solve/pdptw/dispatch",
        dict(PDP_SMALL, current=[[1, 9]], locked=[0]),
    ),
    (
        "err_dispatch_lockpair",
        "POST",
        "/solve/pdptw/dispatch",
        dict(PDP_SMALL, current=[[3, 1]], locked=[1]),
    ),
    (
        "err_dispatch_break_shape",
        "POST",
        "/solve/pdptw/dispatch",
        dict(PDP_SMALL, current=[], locked=[], driver_break=[1]),
    ),
    ("err_atsp_n1", "POST", "/solve/atsp", dict(matrix=[[0]])),
    (
        "err_vroom_nodur",
        "POST",
        "/compat/vroom",
        dict(vehicles=[dict(capacity=[1])], jobs=[dict(location_index=1)]),
    ),
    (
        "err_vroom_both",
        "POST",
        "/compat/vroom",
        dict(vroom, jobs=[dict(location_index=1)]),
    ),
    (
        "err_vroom_hetero",
        "POST",
        "/compat/vroom",
        dict(vroom, vehicles=[dict(capacity=[10]), dict(capacity=[5])]),
    ),
    ("err_cmv1_short", "POST", "/solve/cvrp", b"CMV1\x01", True),
    ("err_cmv1_hlen", "POST", "/solve/cvrp", b"CMV1\xff\xff\xff\xff{}", True),
]
cases.append(
    (
        "dispatch_cold",
        "POST",
        "/solve/pdptw/dispatch",
        dict(PDP_SMALL, current=[], locked=[], seed=5, iters=2000),
    )
)
cases.append(
    (
        "dispatch_locked",
        "POST",
        "/solve/pdptw/dispatch",
        dict(PDP_SMALL, current=[[1, 3]], locked=[2], seed=5, iters=2000),
    )
)
hdr = json.dumps(dict(demand=[0, 4, 6, 5], capacity=10)).encode()

flat = b"".join(struct.pack("<I", v) for row in M4 for v in row)
cases.append(
    (
        "cvrp_cmv1",
        "POST",
        "/solve/cvrp",
        b"CMV1" + struct.pack("<I", len(hdr)) + hdr + flat,
        True,
    )
)
cases.extend(err)

proc = subprocess.Popen(
    [os.path.join(ROOT, "zig-out/bin/commiv-serve")],
    env=dict(os.environ, COMMIV_PORT=str(PORT)),
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
)
try:
    for _ in range(50):
        try:
            get("/health")
            break
        except Exception:
            time.sleep(0.1)
    results = {}
    for case in cases:
        name, method, path = case[0], case[1], case[2]
        if method == "GET":
            status, body = get(path)
        elif len(case) == 5:
            status, body = post(path, case[3], raw=True)
        else:
            status, body = post(path, case[3])
        results[name] = dict(
            status=status,
            sha=hashlib.sha256(body).hexdigest()[:16],
            body=body.decode()
            if len(body) < 400
            else body[:120].decode(errors="replace") + "...",
        )
        print(
            f"{name:28s} {status} {results[name]['sha']} {results[name]['body'][:100]}",
            flush=True,
        )
    json.dump(results, open(OUT, "w"), indent=1, sort_keys=True)
finally:
    proc.terminate()
    proc.wait()
