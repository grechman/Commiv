import os
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen import run
names = sys.argv[1].split(",")
for n in names:
    try:
        c, w, _ = run(n)
        print(f"{n},{c if c is not None else 'FAIL'},{w:.1f}", flush=True)
    except Exception as e:
        print(f"{n},ERROR:{e}", flush=True)
