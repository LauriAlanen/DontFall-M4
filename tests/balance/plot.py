#!/usr/bin/env python3
"""Plot the balance simulator's runs (docs/archive/balance-plan.md).

Runs on the PC (needs matplotlib), on the log of a CSV build:

    scripts/test.sh -s dontfall.balance -x=BALANCE_CSV=1
    tests/balance/plot.py [run] [-o out.png]

The log is build/twister/.../dontfall.balance/handler.log; each
"csv,<run>,..." line is one sample of a run.
"""

import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt

LOG = (Path(__file__).resolve().parents[2] / "build/twister/native_sim_native_64"
       / "balance/dontfall.balance/handler.log")


def read_runs(log):
    runs = {}
    for line in log.read_text().splitlines():
        if not line.startswith("csv,"):
            continue
        _, run, *fields = line.strip().split(",")
        if fields[0] == "t":
            runs[run] = {name: [] for name in fields}
            continue
        for name, value in zip(runs[run], fields):
            runs[run][name].append(float(value))
    return runs


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("run", nargs="?", default="fall")
    ap.add_argument("--log", type=Path, default=LOG)
    ap.add_argument("-o", "--out", type=Path, help="save a PNG instead of showing it")
    args = ap.parse_args()

    runs = read_runs(args.log)
    if args.run not in runs:
        raise SystemExit(f"no run '{args.run}' in {args.log} (built with -x=BALANCE_CSV=1?)"
                         f", found: {', '.join(runs) or 'none'}")
    r = runs[args.run]

    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(8, 7))
    ax[0].plot(r["t"], [math.degrees(v) for v in r["theta"]])
    ax[0].set_ylabel("tilt θ (deg)")
    ax[1].plot(r["t"], [math.degrees(v) for v in r["theta_dot"]])
    ax[1].set_ylabel("tilt rate θ̇ (deg/s)")
    ax[2].plot(r["t"], r["omega"], label="wheel ω (rad/s)")
    ax[2].plot(r["t"], [1000 * v for v in r["torque"]], label="torque (mN m)")
    ax[2].legend()
    ax[2].set_xlabel("t (s)")
    for a in ax:
        a.grid(True)
    fig.suptitle(f"balance simulator: {args.run}")
    fig.tight_layout()

    if args.out:
        fig.savefig(args.out, dpi=100)
    else:
        plt.show()


if __name__ == "__main__":
    main()
