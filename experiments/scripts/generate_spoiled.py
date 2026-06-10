#!/usr/bin/env python3
"""Generate Vertigo random-outlier spoiled datasets, mirroring the IPC author's
protocol exactly (only the file paths and an added --seed differ).

Author's protocol (from the IPC paper + ipc/bash/ipc_experiments_2D.sh):
  - outlier count per rate:  n = round(rate/100 * inliers)   (100% => #outliers == #inliers)
  - rates 10..100 step 10,   10 Monte-Carlo runs per rate (00..09)
  - independent random outliers (generateDataset.py defaults: groupsize 1, info auto)

We call the UNMODIFIED ipc/scripts/generateDataset.py — no change to the author's tool.
The only addition is --seed = run index, so the spoiled files are reproducible and
REUSABLE across methods (IPC / baselines / TACO all read the same files). Averaging over
the 10 runs makes the result independent of the specific seeding.

Idempotent: existing files are skipped (so re-runs reuse, not regenerate).

Usage:
  generate_spoiled.py --clean experiments/datasets/2D/M3500/graph.g2o \
                      --inliers 1954 \
                      --out experiments/datasets/2D/M3500/SPOILED_DATA
"""
import argparse, os, subprocess, csv

RATES = range(10, 101, 10)   # 10,20,...,100  (% of inliers)
RUNS  = range(0, 10)         # 00..09  (run index == seed)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--clean",   required=True, help="clean source g2o")
    ap.add_argument("--inliers", required=True, type=int, help="canonic_inliers (true loop closures)")
    ap.add_argument("--out",     required=True, help="SPOILED_DATA output dir")
    ap.add_argument("--gen",     default="ipc/scripts/generateDataset.py", help="author's generator (unmodified)")
    args = ap.parse_args()

    os.makedirs(args.out, exist_ok=True)
    dataset = os.path.basename(os.path.dirname(args.clean))
    rows, made, skipped = [], 0, 0

    for rate in RATES:
        n = round(rate / 100 * args.inliers)
        rate_dir = os.path.join(args.out, str(rate))
        os.makedirs(rate_dir, exist_ok=True)
        for run in RUNS:
            out_file = os.path.join(rate_dir, f"{run:02d}.g2o")
            cmd = ["python3", args.gen, "-i", args.clean, "-n", str(n),
                   "--seed", str(run), "-o", out_file]
            if os.path.exists(out_file):
                skipped += 1
            else:
                subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
                made += 1
            rows.append(dict(dataset=dataset, rate=rate, run=f"{run:02d}", seed=run,
                             n_outliers=n, inliers=args.inliers, file=out_file,
                             command=" ".join(cmd)))

    manifest = os.path.join(args.out, "manifest.csv")
    with open(manifest, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["dataset","rate","run","seed","n_outliers","inliers","file","command"])
        w.writeheader(); w.writerows(rows)

    print(f"[{dataset}] generated={made} skipped(existing)={skipped} total={len(rows)}")
    print(f"manifest -> {manifest}")

if __name__ == "__main__":
    main()
