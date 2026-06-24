"""Assignment-relaxation lower bound for the directed CVRP. Relaxes capacity AND
subtour-connectivity: each node gets exactly one successor/predecessor, depot split
into K copies (K routes). AP_cost <= optimal <= our_cost, so (our-AP)/AP is a rigorous
UPPER bound on our true optimality gap. Usage: ap_lb.py <road> <K> <our_cost>"""
import sys
import numpy as np
from scipy.optimize import linear_sum_assignment
import roadlib

path, K, our = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
dim, cap, demand, M, coords = roadlib.parse_road(path)
Mn = np.frombuffer(M, dtype=np.int64).reshape(dim, dim)

ncust = dim - 1
size = ncust + K
BIG = np.int64(1 << 40)
C = np.full((size, size), BIG, dtype=np.int64)

# customers 0..ncust-1  <-> nodes 1..dim-1 ; depot copies ncust..ncust+K-1 <-> node 0
cc = Mn[1:dim, 1:dim].copy()          # customer->customer
np.fill_diagonal(cc, BIG)
C[:ncust, :ncust] = cc
ret = Mn[1:dim, 0][:, None]            # customer -> depot (return)
C[:ncust, ncust:] = ret
out = Mn[0, 1:dim][None, :]            # depot -> customer (leave)
C[ncust:, :ncust] = out
# depot-copy -> depot-copy stays BIG

r, c = linear_sum_assignment(C)
lb = int(C[r, c].sum())
print(f"{path.split('/')[-1]}: K={K}  AP-LB={lb}  our={our}  (our-LB)/LB={100*(our-lb)/lb:.2f}%  "
      f"(rigorous upper bound on our optimality gap)")
