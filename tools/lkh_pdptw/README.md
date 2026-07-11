# LKH-3 PDPTW head-to-head harness

Converts Li & Lim instances (vendor/pdptw) to LKH-3 PDPTW format with the SAME
integer matrix commiv's bench uses (round(euclid*1000), times scaled x1000) and
runs LKH-3 (SPECIAL) with VEHICLES = the BKS fleet size (Helsgaun's convention;
favors LKH). FAIL = LKH found no feasible solution within the budget
(Penalty.min > 0).

    LKH_TIME=10 python3 tools/lkh_pdptw/runall.py "lc101,lr101,lrc101"

Ours, same instances and wall:

    PB_FILES=lc101,lr101,lrc101 PB_TIME_MS=10000 ./zig-out/bin/commiv-pdptwbench 2>&1

Requires ~/cbench/LKH-3.0.14/LKH. 2026-07-10 result at 10 s equal wall,
single thread both: commiv 56/56 feasible, 53/56 exact BKS (fleet-min driver,
PB_FLEET=1); LKH 37/56 feasible, 26/56 exact, 0 cells better than commiv.
At 60 s LKH recovers 6 of its 19 misses (13 still infeasible). Its former
lone hierarchical win (forced-9-vehicle lc103) is gone: commiv's fleet
cap + request bank reaches the 9-vehicle records on lc103/lc109 too.
Not extended past n=100: LKH already fails feasibility on most 100-series
cells, and it only degrades with size.
