"""VROOM (production VRP engine, github.com/VROOM-Project/vroom) on the TRUE directed
matrix. VROOM natively supports asymmetric duration matrices. Usage: vroom_cvrp.py <road> <expl>"""
import sys
import time
import math
import numpy as np
import vroom
import roadlib


def main():
    path = sys.argv[1]
    expl = int(sys.argv[2]) if len(sys.argv) > 2 else 5      # exploration level 0..5
    dim, cap, demand, M, coords = roadlib.parse_road(path)
    mat = np.frombuffer(M, dtype=np.int64).reshape(dim, dim).astype(np.uint32)

    inp = vroom.Input()
    inp.set_durations_matrix("car", mat)
    lb = math.ceil(sum(demand) / cap)
    K = int(lb * 1.3) + 5
    for v in range(K):
        inp.add_vehicle(vroom.Vehicle(v, start=0, end=0, capacity=[cap], profile="car"))
    for c in range(1, dim):
        inp.add_job(vroom.Job(c, location=c, delivery=[int(demand[c])]))

    t0 = time.time()
    sol = inp.solve(exploration_level=expl, nb_threads=3)
    el = time.time() - t0

    df = sol.routes
    routes = []
    for _, grp in df.groupby('vehicle_id'):
        r = [int(jid) for typ, jid in zip(grp['type'], grp['id']) if typ == 'job']
        if r:
            routes.append(r)

    cost = roadlib.score(M, dim, routes)
    err = roadlib.validate(dim, cap, demand, routes)
    print(f"vroom,{path},{dim},{cost},{len(routes)},{el:.1f},{'OK' if not err else err}")


if __name__ == '__main__':
    main()
