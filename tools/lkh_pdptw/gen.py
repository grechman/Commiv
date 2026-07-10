import math, os, sys, subprocess, time, re

VENDOR = "vendor/pdptw"
OUT = os.environ.get("LKH_OUT", "/tmp/lkh_pdptw_work"); os.makedirs(OUT, exist_ok=True)
LKH = os.path.expanduser("~/cbench/LKH-3.0.14/LKH")
SCALE = 1000

def parse(name):
    lines = open(f"{VENDOR}/{name}.txt").read().split("\n")
    k, cap, _ = map(int, lines[0].split())
    nodes = []
    for ln in lines[1:]:
        f = ln.split()
        if len(f) < 9: continue
        nodes.append(tuple(map(int, f)))
    return k, cap, nodes

def bks_veh(name):
    return sum(1 for ln in open(f"{VENDOR}/{name}.sol") if ln.strip().startswith("Route"))

def gen(name):
    k, cap, nodes = parse(name)
    dim = len(nodes)
    xs = [n[1] for n in nodes]; ys = [n[2] for n in nodes]
    with open(f"{OUT}/{name}.pdptw", "w") as f:
        f.write(f"NAME : {name}\nTYPE : PDPTW\nDIMENSION : {dim}\nCAPACITY : {cap}\n")
        f.write(f"VEHICLES : {bks_veh(name)}\n")
        f.write("EDGE_WEIGHT_TYPE : EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n")
        for a in range(dim):
            row = []
            for b in range(dim):
                d = round(math.dist((xs[a], ys[a]), (xs[b], ys[b])) * SCALE)
                row.append(str(d))
            f.write(" ".join(row) + "\n")
        f.write("PICKUP_AND_DELIVERY_SECTION\n")
        for n in nodes:
            nid, x, y, dem, rdy, due, srv, pick, deliv = n
            p = pick + 1 if pick else 0
            d = deliv + 1 if deliv else 0
            f.write(f"{nid+1} {dem} {rdy*SCALE} {due*SCALE} {srv*SCALE} {p} {d}\n")
        f.write("DEPOT_SECTION\n1\n-1\nEOF\n")
    with open(f"{OUT}/{name}.par", "w") as f:
        f.write(f"PROBLEM_FILE = {OUT}/{name}.pdptw\n")
        f.write("SPECIAL\n")
        import os as _os
        tl = _os.environ.get("LKH_TIME", "10")
        f.write(f"TOTAL_TIME_LIMIT = {tl}\nTIME_LIMIT = {tl}\nRUNS = 100\nTRACE_LEVEL = 0\n")
        f.write(f"TOUR_FILE = {OUT}/{name}.tour\n")

def run(name):
    gen(name)
    t0 = time.time()
    r = subprocess.run([LKH, f"{OUT}/{name}.par"], capture_output=True, text=True, timeout=int(os.environ.get("LKH_TIME","10"))*2+30)
    wall = time.time() - t0
    out = r.stdout
    # LKH prints Cost.Best or the tour file has the cost; parse "Cost.min = X" / "Best PDPTW solution:"
    m = re.findall(r"Cost\.min = (-?\d+)", out)
    cost = None; veh = None
    m2 = re.search(r"Best PDPTW solution:.*", out)
    # tour file: COMMENT lines carry cost like "COMMENT : Length = X"
    tf = f"{OUT}/{name}.tour"
    if os.path.exists(tf):
        for ln in open(tf):
            mm = re.search(r"Length = (-?\d+)", ln)
            if mm: cost = int(mm.group(1))
    return cost, wall, out[-600:]

if __name__ == "__main__":
    name = sys.argv[1]
    c, w, tail = run(name)
    print(name, c, f"{w:.1f}s")
    if c is None:
        print(tail)
