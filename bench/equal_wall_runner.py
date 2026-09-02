#!/usr/bin/env python3

from __future__ import annotations

import concurrent.futures as cf
import fcntl
import hashlib
import json
import os
import pathlib
import subprocess
import sys
import threading
import time

HOME = pathlib.Path.home()
CWD = HOME / "projects" / "commiv-baseline"
MONEY_CPUS = [0, 1, 3, 4, 5]
SIDE_A = "0,1,3,4,5"
SIDE_B = "6,7,9,10,11"
SIZES = [
    ("100", "vendor/pdptw", 10, (1, 2, 3)),
    ("200", "vendor/pdptw/200", 15, (1,)),
    ("400", "vendor/pdptw/400", 30, (1,)),
    ("600", "vendor/pdptw/600", 45, (1,)),
    ("800", "vendor/pdptw/800", 60, (1,)),
    ("1000", "vendor/pdptw/1000", 90, (1,)),
]

lock = threading.Lock()


def sha256(path: str) -> str:
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


class Campaign:
    def __init__(self, mode: str, old: str, new: str, out: pathlib.Path):
        self.mode, self.old, self.new, self.out = mode, old, new, out
        out.mkdir(parents=True, exist_ok=True)
        self.jsonl = out / f"{mode}-results.jsonl"
        self.journal = out / f"{mode}-cells.done"
        self.status = out / f"{mode}-status.txt"
        self.lockfile = (out / f"{mode}-runner.lock").open("w")
        fcntl.flock(self.lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.done = (
            set(self.journal.read_text().splitlines())
            if self.journal.exists()
            else set()
        )
        self.sha = {"old": sha256(old), "new": sha256(new)}
        self.bins = {"old": old, "new": new}

    def log(self, label: str) -> None:
        line = f"{time.strftime('%F %T')} {label}"
        with lock:
            self.status.write_text(line + "\n")
            print(line, flush=True)

    def append(self, obj: dict) -> None:
        with lock:
            with self.jsonl.open("a") as f:
                f.write(json.dumps(obj, sort_keys=True) + "\n")

    def finish(self, cid: str) -> None:
        with lock:
            with self.journal.open("a") as f:
                f.write(cid + "\n")
            self.done.add(cid)

    def launch(
        self, which: str, cpus: str, env: dict
    ) -> tuple[subprocess.Popen, float]:
        e = os.environ.copy()
        e.update(env)
        p = subprocess.Popen(
            ["taskset", "-c", cpus, self.bins[which]],
            cwd=CWD,
            env=e,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        return p, time.time()

    def collect(
        self, p: subprocess.Popen, t0: float, name: str, timeout: float
    ) -> dict:
        try:
            out, _ = p.communicate(timeout=timeout)
            rc = p.returncode
        except subprocess.TimeoutExpired:
            p.kill()
            out, _ = p.communicate()
            rc, out = 124, (out or "") + "\nTIMEOUT"
        wall = time.time() - t0
        rows = [
            l.strip()
            for l in (out or "").splitlines()
            if l.strip().startswith(name + ",")
        ]
        rec = {"rc": rc, "external_wall": round(wall, 3)}
        if rc == 0 and len(rows) == 1:
            rec["row"] = rows[0]
        else:
            rec["error"] = (out or "")[-400:]
        return rec

    def cells(self):
        for size, rel, wall, seeds in SIZES:
            d = CWD / rel
            names = sorted(
                p.stem for p in d.glob("*.txt") if p.with_suffix(".sol").exists()
            )
            for name in names:
                for seed in seeds if self.mode == "pdptw" else (1,):
                    yield size, rel, wall, name, seed

    def money_cell(
        self, idx: int, size: str, rel: str, wall: int, name: str, cpu: int
    ) -> None:
        cid = f"money/{size}/{name}"
        if cid in self.done:
            return
        env = {
            "PB_DIR": rel,
            "PB_FILES": name,
            "PB_TIME_MS": str(wall * 1000),
            "PB_THREADS": "1",
            "PB_SEED": "1",
            "PB_TIMEPEN": "1",
            "PB_VEH_PEN": "280000",
        }
        order = ("old", "new") if idx % 2 == 0 else ("new", "old")
        self.log(f"CELL {cid} cpu={cpu} order={order[0]}-first")
        for which in order:
            p, t0 = self.launch(which, str(cpu), env)
            rec = self.collect(p, t0, name, wall * 3 + 300)
            rec.update(
                family="money",
                size=size,
                instance=name,
                seed=1,
                wall_budget=wall,
                bin=which,
                sha256=self.sha[which],
                cpu=str(cpu),
                order=order[0] + "-first",
                _cell=cid,
            )
            self.append(rec)
        self.finish(cid)

    def pdptw_cell(
        self, idx: int, size: str, rel: str, wall: int, name: str, seed: int
    ) -> None:
        cid = f"pdptw/{size}/{name}/s{seed}"
        if cid in self.done:
            return
        env = {
            "PB_DIR": rel,
            "PB_FILES": name,
            "PB_TIME_MS": str(wall * 1000),
            "PB_FLEET": "1",
            "PB_EJECT": "1",
            "PB_THREADS": "5",
            "PB_SEED": str(seed),
        }
        if size != "100":
            env["PB_GRAN"] = "2"
        sides = (
            {"old": SIDE_A, "new": SIDE_B}
            if idx % 2 == 0
            else {"old": SIDE_B, "new": SIDE_A}
        )
        self.log(f"CELL {cid} old={sides['old']} new={sides['new']}")
        procs = {w: self.launch(w, sides[w], env) for w in ("old", "new")}
        for which, (p, t0) in procs.items():
            rec = self.collect(p, t0, name, wall * 3 + 300)
            rec.update(
                family="pdptw",
                size=size,
                instance=name,
                seed=seed,
                wall_budget=wall,
                bin=which,
                sha256=self.sha[which],
                cpu=sides[which],
                _cell=cid,
            )
            self.append(rec)
        self.finish(cid)

    def run(self) -> None:
        self.log(
            f"CAMPAIGN_START {self.mode} old={self.sha['old'][:12]} new={self.sha['new'][:12]}"
        )
        self.append(
            {
                "family": "meta",
                "mode": self.mode,
                "old": self.old,
                "new": self.new,
                "sha256_old": self.sha["old"],
                "sha256_new": self.sha["new"],
                "cwd": str(CWD),
            }
        )
        cells = list(self.cells())
        if self.mode == "money":
            with cf.ThreadPoolExecutor(len(MONEY_CPUS)) as pool:
                futs = [
                    pool.submit(
                        self.money_cell,
                        i,
                        size,
                        rel,
                        wall,
                        name,
                        MONEY_CPUS[i % len(MONEY_CPUS)],
                    )
                    for i, (size, rel, wall, name, _seed) in enumerate(cells)
                ]
                for f in futs:
                    f.result()
        else:
            for i, (size, rel, wall, name, seed) in enumerate(cells):
                self.pdptw_cell(i, size, rel, wall, name, seed)
        self.log(f"CAMPAIGN_COMPLETE {self.mode} {len(self.done)} cells")


def main() -> int:
    mode, old, new = sys.argv[1], sys.argv[2], sys.argv[3]
    out = (
        pathlib.Path(sys.argv[4])
        if len(sys.argv) > 4
        else HOME / "projects" / "equal-wall"
    )
    if mode not in ("money", "pdptw"):
        raise SystemExit("mode must be money or pdptw")
    Campaign(mode, old, new, out).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
