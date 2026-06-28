#!/usr/bin/env python3
"""Fetch a REAL directed road travel-time matrix and write a .road fixture.

This is the provenance tool for the asymmetric road benchmark (examples/roadbench.zig).
It samples points in a city bounding box, queries an OSRM `/table` service for the
directed (i->j) driving durations, and writes a self-contained .road file that the
Zig benchmark parses. The committed vendor/road/*.road files were produced by this
script; re-running it reproduces them (up to live map updates).

Why directed durations matter: OSRM driving times encode one-way streets, turn
restrictions and divided roads, so d(i,j) != d(j,i). That asymmetry is the whole
point of the benchmark.

Coordinate limit: the PUBLIC demo server (router.project-osrm.org) caps a /table
request at 100 coordinates. For n<=100 we issue one request. For larger n we tile:
partition the points into blocks of <=50 and request each pair of blocks together
(<=100 coords), filling the full NxN matrix. The public demo is rate-limited and
meant for light use, so large n (hundreds+) should point --server at a SELF-HOSTED
OSRM (osrm-routed on a Geofabrik extract, with --max-table-size raised) rather than
hammering the demo.

Usage:
  python3 tools/fetch_road_matrix.py --n 100 --out vendor/road/moscow-100.road
  python3 tools/fetch_road_matrix.py --n 300 --server http://localhost:5000 \
      --out vendor/road/moscow-300.road            # needs self-hosted OSRM

Output format (line-based, integers are seconds):
  NAME <name>
  DIM <total nodes incl depot>
  CAPACITY <vehicle capacity>
  COORDS
  <idx> <lng> <lat> <demand>      x DIM   (idx 0 = depot, demand 0)
  MATRIX
  <DIM ints per row>              x DIM   (row-major, M[i*DIM+j] = seconds i->j)
"""
import argparse, json, math, os, random, socket, time, urllib.request, urllib.error, sys

DEMO = "https://router.project-osrm.org"

# HTTP statuses worth retrying: rate-limit (429) and transient server errors (5xx).
RETRY_STATUS = {429, 500, 502, 503, 504}


def _get(url, timeout, attempts=5, base=1.0):
    """GET a URL and return the parsed JSON, with bounded exponential backoff.

    Retries on transient HTTP errors (429/500/502/503/504), connection-level
    urllib.error.URLError, and socket.timeout; sleeps base*2**attempt plus a
    small random jitter between tries and re-raises after the final attempt."""
    for attempt in range(attempts):
        try:
            with urllib.request.urlopen(url, timeout=timeout) as r:
                return json.load(r)
        except (urllib.error.URLError, socket.timeout) as e:
            # HTTPError is a subclass of URLError; only retry transient codes.
            if isinstance(e, urllib.error.HTTPError) and e.code not in RETRY_STATUS:
                raise
            if attempt == attempts - 1:
                raise
            delay = base * (2 ** attempt) + random.uniform(0, base)
            print(f"  transient fetch error ({e}); retry {attempt + 1}/{attempts - 1} "
                  f"in {delay:.1f}s", file=sys.stderr)
            time.sleep(delay)


def table(server, coords, timeout=60):
    """coords: list of (lng,lat). Returns NxN list of durations (seconds, float)."""
    s = ";".join(f"{lng},{lat}" for lng, lat in coords)
    url = f"{server}/table/v1/driving/{s}?annotations=duration"
    d = _get(url, timeout)
    if d.get("code") != "Ok":
        raise RuntimeError(f"OSRM error: {d}")
    return d["durations"]


def _load_checkpoint(path, n, block):
    """Return (M, done_set) from a matching partial checkpoint, else (None, None)."""
    if not path or not os.path.exists(path):
        return None, None
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None, None
    if data.get("n") != n or data.get("block") != block:
        print(f"  ignoring stale checkpoint {path} (n/block mismatch)", file=sys.stderr)
        return None, None
    done = {(a, b) for a, b in data.get("completed", [])}
    print(f"  resuming from {path}: {len(done)} tile(s) already fetched", file=sys.stderr)
    return data["M"], done


def _save_checkpoint(path, n, block, M, done):
    """Atomically persist the partial matrix and the set of completed block-pairs."""
    if not path:
        return
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"n": n, "block": block, "completed": sorted(done), "M": M}, f)
    os.replace(tmp, path)


def fetch_matrix(server, coords, max_table=100, pause=1.5, timeout=60, checkpoint=None):
    """Full directed NxN matrix. One request if n <= max_table (raise it for a
    self-hosted osrm-routed started with --max-table-size); otherwise tile into
    <=max_table-coord requests (blocks of max_table//2 so a block-pair fits).
    Tiling bounds peak memory and per-request time for very large n.

    When checkpoint is set, the partial matrix and the set of completed block
    pairs are persisted after every tile, so a run aborted by a transient
    failure resumes instead of refetching everything."""
    n = len(coords)
    if n <= max_table:
        return table(server, coords, timeout)
    block = max(1, max_table // 2)
    blocks = [list(range(i, min(i + block, n))) for i in range(0, n, block)]
    pairs = [(a, b) for a in range(len(blocks)) for b in range(a, len(blocks))]
    M, done = _load_checkpoint(checkpoint, n, block)
    if M is None:
        M = [[None] * n for _ in range(n)]
        done = set()
    for k, (a, b) in enumerate(pairs):
        if (a, b) in done:
            continue
        idx = blocks[a] if a == b else blocks[a] + blocks[b]
        sub = table(server, [coords[i] for i in idx], timeout)
        for p, gi in enumerate(idx):
            for q, gj in enumerate(idx):
                M[gi][gj] = sub[p][q]
        done.add((a, b))
        _save_checkpoint(checkpoint, n, block, M, done)
        print(f"  tile {k+1}/{len(pairs)} (blocks {a},{b}, {len(idx)} coords)", file=sys.stderr)
        if k + 1 < len(pairs):
            time.sleep(pause)
    return M


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=100, help="total nodes incl depot")
    ap.add_argument("--out", required=True)
    ap.add_argument("--name", default=None)
    ap.add_argument("--server", default=DEMO)
    ap.add_argument("--max-table", type=int, default=100,
                    help="max coords per /table request (public demo=100; self-hosted: "
                         "match osrm-routed --max-table-size to fetch in one shot)")
    ap.add_argument("--timeout", type=int, default=60, help="per-request HTTP timeout (s)")
    ap.add_argument("--seed", type=int, default=20260623, help="point-sampling seed")
    ap.add_argument("--demand-seed", type=int, default=42)
    ap.add_argument("--routes", type=int, default=10, help="target #routes -> capacity")
    # central Moscow service area; depot near the centre (Lubyanka-ish)
    ap.add_argument("--depot", default="37.6173,55.7558")
    ap.add_argument("--bbox", default="37.50,55.64,37.74,55.82", help="lng_min,lat_min,lng_max,lat_max")
    args = ap.parse_args()

    name = args.name or args.out.rsplit("/", 1)[-1].rsplit(".", 1)[0]
    dlng, dlat = (float(x) for x in args.depot.split(","))
    lng0, lat0, lng1, lat1 = (float(x) for x in args.bbox.split(","))

    rng = random.Random(args.seed)
    coords = [(round(dlng, 5), round(dlat, 5))]
    for _ in range(args.n - 1):
        coords.append((round(rng.uniform(lng0, lng1), 5), round(rng.uniform(lat0, lat1), 5)))

    print(f"fetching {args.n}x{args.n} directed matrix from {args.server} ...", file=sys.stderr)
    checkpoint = args.out + ".partial.json"
    dur = fetch_matrix(args.server, coords, max_table=args.max_table,
                       timeout=args.timeout, checkpoint=checkpoint)
    n = len(dur)

    finite = [v for row in dur for v in row if v is not None]
    big = int(max(finite) * 3) if finite else 1
    unreach = sum(1 for row in dur for v in row if v is None)
    if unreach:
        print(f"WARNING: {unreach} unreachable pairs capped at {big}s", file=sys.stderr)
    M = [[0 if i == j else (int(round(dur[i][j])) if dur[i][j] is not None else big)
          for j in range(n)] for i in range(n)]

    drng = random.Random(args.demand_seed)
    dem = [0] + [drng.randint(1, 9) for _ in range(n - 1)]
    cap = max(1, math.ceil(sum(dem) / max(1, args.routes)))

    with open(args.out, "w") as f:
        f.write(f"NAME {name}\nDIM {n}\nCAPACITY {cap}\nCOORDS\n")
        for i, (lng, lat) in enumerate(coords):
            f.write(f"{i} {lng} {lat} {dem[i]}\n")
        f.write("MATRIX\n")
        for i in range(n):
            f.write(" ".join(str(M[i][j]) for j in range(n)) + "\n")
    if os.path.exists(checkpoint):
        os.remove(checkpoint)
    print(f"wrote {args.out}: DIM={n} CAPACITY={cap} total_demand={sum(dem)} "
          f"(~{math.ceil(sum(dem)/cap)} routes)", file=sys.stderr)


if __name__ == "__main__":
    main()
