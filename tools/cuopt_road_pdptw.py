#!/usr/bin/env python3
# NVIDIA cuOpt (GPU) head-to-head on the REAL directed-road PDPTW money instance
# produced by examples/moneyroadbench.zig (MR_DUMP=...). Mirrors the contract of
# tools/vroom_road_pdptw.py: reads that dump JSON, builds the cuOpt DataModel,
# steers cuOpt with the SAME money objective commiv optimized, runs the solve at
# a fixed GPU time budget, then SCORES cuOpt's returned plan under the dump's
# MONEY_REAL Prices model with an INDEPENDENT feasibility walk (does not trust
# cuOpt's own arrival stamps or objective).
#
# THE MONEY-OBJECTIVE MAPPING (the whole point of this bench)
# ----------------------------------------------------------
# Prices model (identical to the commiv bench and the VROOM scorer):
#   usd = (drive_sec + time_pen*dur_sec + veh_pen*veh) * money_unit
#   drive_sec = travel only  (sum of cost-matrix arcs on the route)
#   dur_sec   = drive + service + waiting  (total route duration commiv prices)
#
# cuOpt CAN express this natively, and UNLIKE VROOM it CAN price waiting. Its
# objective is a linear combination of Objective terms (set_objective_function):
#   COST              = sum of cost-matrix arcs           = drive_sec
#   TRAVEL_TIME       = drive + service + WAIT time       = dur_sec
#                       (verbatim from cuOpt's own docstring, vehicle_routing_
#                        wrapper.pyx:109 "This includes drive, service and wait
#                        time" - VERIFIED on the installed 26.06, not guessed)
#   VEHICLE_FIXED_COST = sum of per-vehicle fixed costs   = veh_pen*veh
# So:
#   set_objective_function([COST, TRAVEL_TIME, VEHICLE_FIXED_COST],
#                          [1, time_pen, 1])
#   set_vehicle_fixed_costs([veh_pen]*fleet)
# yields internal objective = drive + time_pen*(drive+service+wait) + veh_pen*veh
# = drive_sec + time_pen*dur_sec + veh_pen*veh = the money model (money_unit is a
# constant scale that does not change the argmin). cuOpt's time term INCLUDES
# waiting, so cuOpt does NOT inherit VROOM's blind spot - recorded in the CSV's
# wait_included_in_objective note field as "yes".
#
# Usage:
#   # 1. dump the instance (no solve):
#   MR_FILE=nyc-100 MR_DUMP=/tmp/nyc100.json MR_SOLVE=0 ./zig-out/bin/commiv-moneyroadbench
#   # 2. score cuOpt on it (on the GPU box, inside the cuopt venv):
#   CUOPT_TIME=30 python3 tools/cuopt_road_pdptw.py /tmp/nyc100.json
#
# Env:
#   CUOPT_TIME   GPU solve budget seconds -> SolverSettings.set_time_limit (default 10)
#   MR_DUMP      dump path (alternative to argv[1])
#   MR_TIMEPEN / MR_VEH_PEN / MR_MONEY_UNIT
#                REQUIRED ONLY for legacy dumps lacking a "prices" object; refuses
#                to score such a dump without them (same anti-footgun rule as VROOM).
import json, os, sys, time

T = int(os.environ.get("CUOPT_TIME", "10"))


def resolve_prices(d):
    """Prices MUST come from the dump (written by the bench with the exact weights
    the solver optimized). Legacy dumps without a "prices" object are scored only
    when the operator explicitly supplies every price via env."""
    prices = d.get("prices")
    if prices:
        return float(prices["time_pen"]), float(prices["veh_pen"]), float(prices["money_unit"])
    try:
        return (float(os.environ["MR_TIMEPEN"]),
                float(os.environ["MR_VEH_PEN"]),
                float(os.environ.get("MR_MONEY_UNIT", "0.0015")))
    except KeyError:
        sys.stderr.write(
            "dump has no \"prices\" object (legacy dump). Re-dump with the current "
            "moneyroadbench, or set MR_TIMEPEN and MR_VEH_PEN explicitly to the "
            "weights that run optimized. Refusing to guess.\n")
        sys.exit(2)


def build_model(d, timepen, vehpen):
    """Construct the cuOpt DataModel from the moneyroadbench dump dict, steered by
    the money objective. Location layout: depot=0, order i lives at location i+1,
    so there are K = 2*n_pairs+1 locations and N_ORDERS = 2*n_pairs orders. Pairs
    in the dump are LOCATION ids; cuOpt pickup/delivery indices are ORDER ids
    (location L -> order L-1)."""
    import cudf
    from cuopt import routing

    n_pairs = d["n_pairs"]
    K = len(d["matrix"])
    assert K == 2 * n_pairs + 1, f"matrix K={K} != 2*n_pairs+1={2*n_pairs+1}"
    n_orders = 2 * n_pairs
    fleet = n_pairs  # one vehicle per pair is always enough

    dm = routing.DataModel(K, fleet, n_orders)
    # cost matrix = directed seconds; COST objective accumulates it = drive_sec.
    dm.add_cost_matrix(cudf.DataFrame(d["matrix"]).astype("float32"))

    # order i at location i+1 (customers 1..K-1)
    dm.set_order_locations(cudf.Series(list(range(1, K))))
    dm.set_order_time_windows(
        cudf.Series([d["ready"][i] for i in range(1, K)]),
        cudf.Series([d["due"][i] for i in range(1, K)]))
    dm.set_order_service_times(cudf.Series([d["service"][i] for i in range(1, K)]))

    # pickup/delivery pairs as ORDER indices (location L -> order L-1)
    dm.set_pickup_delivery_pairs(
        cudf.Series([p["pickup"] - 1 for p in d["pairs"]]),
        cudf.Series([p["delivery"] - 1 for p in d["pairs"]]))

    demand = [0] * n_orders
    for p in d["pairs"]:
        demand[p["pickup"] - 1] = p["amount"]
        demand[p["delivery"] - 1] = -p["amount"]
    dm.add_capacity_dimension("demand", cudf.Series(demand),
                              cudf.Series([d["capacity"]] * fleet))

    dm.set_vehicle_time_windows(
        cudf.Series([0] * fleet), cudf.Series([d["due"][0]] * fleet))

    # THE MONEY OBJECTIVE (see header). money_unit is a constant scale -> omitted
    # from the weights (does not change the argmin); applied only in scoring.
    Obj = routing.Objective
    dm.set_objective_function(
        cudf.Series([Obj.COST, Obj.TRAVEL_TIME, Obj.VEHICLE_FIXED_COST]),
        cudf.Series([1.0, timepen, 1.0]))
    dm.set_vehicle_fixed_costs(cudf.Series([float(vehpen)] * fleet))
    return dm, K, fleet


def extract_routes(sol, K):
    """Turn sol.get_route() into {truck_id: [location, ...]} preserving stop order.
    The DataFrame has columns route/arrival_stamp/truck_id/location; we walk by the
    physical `location` column (index into the dump matrix) and ignore cuOpt's own
    arrival_stamp - scoring re-derives arrivals independently from the dump."""
    df = sol.get_route().to_pandas()
    routes = {}
    for _, row in df.iterrows():
        tid = int(row["truck_id"])
        loc = int(row["location"])
        routes.setdefault(tid, []).append(loc)
    return routes


def score(d, routes, timepen, vehpen, unit):
    """INDEPENDENT feasibility walk + money score. Walks each truck depot->stops->
    depot against the dump matrix and windows; checks TW, precedence, pairing (both
    ends same route), and capacity. Returns (line_fields_dict). usd is INCOMPLETE
    if any order is unserved or any hard constraint is violated."""
    mat = d["matrix"]
    ready, due, service, cap = d["ready"], d["due"], d["service"], d["capacity"]
    n_pairs = d["n_pairs"]
    K = len(mat)

    # pickup-location -> delivery-location, for precedence/pairing checks
    deliv_of = {p["pickup"]: p["delivery"] for p in d["pairs"]}
    pickups = set(deliv_of.keys())
    deliveries = set(deliv_of.values())

    drive_sec = wait_sec = service_sec = 0
    served = set()
    used = 0
    violations = []

    for tid, seq in routes.items():
        # strip to the real traversal: cuOpt lists depot at ends; keep as-is and
        # ensure we start and end at depot 0 for the drive accounting.
        stops = [s for s in seq]
        # drop leading/trailing that are already depot duplicates handled by walk
        real = [s for s in stops if s != 0]
        if not real:
            continue
        used += 1
        # build full path depot -> stops (in given order) -> depot
        path = [0] + real + [0]
        t = 0  # depart depot at time 0 (vehicle TW start)
        load = 0
        seen_pos = {}
        for idx in range(1, len(path)):
            a, b = path[idx - 1], path[idx]
            travel = mat[a][b]
            drive_sec += travel
            arrival = t + travel
            if b == 0:  # return to depot: closes the route span
                t = arrival
                continue
            # customer stop
            if b in served:
                violations.append(f"loc{b}_double_served")
            served.add(b)
            seen_pos[b] = idx
            w = ready[b] - arrival
            if w < 0:
                w = 0
            if arrival > due[b]:
                violations.append(f"loc{b}_late(arr{arrival}>due{due[b]})")
            wait_sec += w
            service_sec += service[b]
            t = arrival + w + service[b]
            # capacity: pickups add, deliveries subtract
            if b in pickups:
                load += _demand(d, b)
            elif b in deliveries:
                load -= _demand_pickup_for_delivery(d, b)
            if load > cap:
                violations.append(f"cap_exceeded(load{load}>{cap})")
            if load < 0:
                violations.append(f"neg_load({load})")
        # precedence + pairing within this route
        for pk, dl in deliv_of.items():
            in_route = pk in seen_pos or dl in seen_pos
            if not in_route:
                continue
            if (pk in seen_pos) != (dl in seen_pos):
                violations.append(f"pair_split(pk{pk},dl{dl})")
            elif seen_pos[pk] >= seen_pos[dl]:
                violations.append(f"precedence(pk{pk}after dl{dl})")

    dur_sec = drive_sec + wait_sec + service_sec
    # unserved orders: every customer location 1..K-1 must be served once
    all_customers = set(range(1, K))
    unserved = all_customers - served
    unassigned_pairs = sum(1 for p in d["pairs"]
                           if p["pickup"] in unserved or p["delivery"] in unserved)

    ok = (not unserved) and (not violations)
    usd = (drive_sec + timepen * dur_sec + vehpen * used) * unit
    return {
        "used": used, "drive": drive_sec, "dur": dur_sec, "wait": wait_sec,
        "usd": usd, "ok": ok, "unassigned": unassigned_pairs,
        "violations": violations[:6],
    }


def _demand(d, ploc):
    for p in d["pairs"]:
        if p["pickup"] == ploc:
            return p["amount"]
    return 0


def _demand_pickup_for_delivery(d, dloc):
    for p in d["pairs"]:
        if p["delivery"] == dloc:
            return p["amount"]
    return 0


def run(path):
    d = json.load(open(path))
    name = d.get("name", os.path.basename(path))
    n_pairs = d["n_pairs"]
    timepen, vehpen, unit = resolve_prices(d)
    note = "yes"  # cuOpt TRAVEL_TIME includes drive+service+wait (verified 26.06)

    try:
        from cuopt import routing
        dm, K, fleet = build_model(d, timepen, vehpen)
        ss = routing.SolverSettings()
        ss.set_time_limit(T)
        t0 = time.time()
        sol = routing.Solve(dm, ss)
        wall = time.time() - t0
    except Exception as e:
        import traceback
        traceback.print_exc(file=sys.stderr)
        return f"{name},{n_pairs},ERROR,0,0,0,0,0,0.0,{note}  # {type(e).__name__}: {str(e)[:120]}", 1

    status = sol.get_status()
    if sol.get_vehicle_count() == 0:
        return f"{name},{n_pairs},ERROR,0,0,0,0,0,{wall:.1f},{note}  # cuopt status={status} no route", 1

    routes = extract_routes(sol, K)
    r = score(d, routes, timepen, vehpen, unit)
    usd_field = f"{r['usd']:.2f}" if r["ok"] else "INCOMPLETE"

    line = (f"{name},{n_pairs},{r['used']},{r['drive']},{r['dur']},{r['wait']},"
            f"{usd_field},{r['unassigned']},{wall:.1f},{note}")
    if not r["ok"]:
        why = f"unserved={r['unassigned']}pairs" if r["unassigned"] else ""
        if r["violations"]:
            why += (" " if why else "") + "viol:" + ";".join(r["violations"])
        line += f"  # INCOMPLETE plan not scored ({why}; cuopt status={status})"
    return line, 0


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("MR_DUMP")
    if not path:
        sys.stderr.write("usage: cuopt_road_pdptw.py <dump.json>  (or MR_DUMP=...)\n")
        sys.exit(2)
    line, rc = run(path)
    print(line, flush=True)
    sys.exit(rc)
