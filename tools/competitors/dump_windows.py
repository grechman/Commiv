#!/usr/bin/env python3
"""Dump the deterministic make_windows overlay for a .road instance to
vendor/road/<city>.tw so examples/twroadbench.zig can load it. Usage:
  python3 tools/competitors/dump_windows.py nyc-2000 [more cities...]
Run from the repo root. The overlay is a pure function of the matrix
(TW_SEED=42), identical to what every competitor adapter applies."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import roadlib
from vrptw_moscow import make_windows

for city in sys.argv[1:]:
    dim, cap, demand, M, coords = roadlib.parse_road(f"vendor/road/{city}.road")
    ready, due, service = make_windows(dim, M)
    with open(f"vendor/road/{city}.tw", "w") as f:
        f.write(f"{dim}\n")
        for i in range(dim):
            f.write(f"{ready[i]} {due[i]} {service[i]}\n")
    print(city, "ok", dim)
