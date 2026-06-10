#!/usr/bin/env python3
"""Aggregate IPC .PR outputs into a precision/recall/F1 curve vs outlier rate.

This is OUR analysis of the author-faithful run (the run itself is untouched).
Reads <root>/<rate>/<run>.PR where each .PR is:
    line1:  precision recall
    line2:  total_time  avg_time_per_test

Writes:
    metrics_per_run.csv   (rate, run, precision, recall, f1, total_time)
    metrics_summary.csv   (rate, n_runs, mean_precision, mean_recall, mean_f1)
and prints the per-rate summary table.

Usage:
  aggregate_pr.py --root experiments/results/IPC/M3500/110125/G2O_IPC_REC_20
"""
import argparse, os, csv, glob, statistics as st

def f1(p, r):
    return 0.0 if (p + r) == 0 else 2 * p * r / (p + r)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", required=True, help="dir containing <rate>/<run>.PR")
    ap.add_argument("--out",  default=None, help="output dir for CSVs (default: --root)")
    args = ap.parse_args()
    out_dir = args.out or args.root

    per_run = []
    for pr_file in sorted(glob.glob(os.path.join(args.root, "*", "*.PR"))):
        rate = int(os.path.basename(os.path.dirname(pr_file)))
        run  = os.path.splitext(os.path.basename(pr_file))[0]
        with open(pr_file) as f:
            lines = f.read().split("\n")
        p, r = (float(x) for x in lines[0].split()[:2])
        ttime = float(lines[1].split()[0]) if len(lines) > 1 and lines[1].strip() else float("nan")
        per_run.append(dict(rate=rate, run=run, precision=p, recall=r, f1=f1(p, r), total_time=ttime))

    if not per_run:
        print(f"no .PR files under {args.root}"); return

    with open(os.path.join(out_dir, "metrics_per_run.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["rate","run","precision","recall","f1","total_time"]); w.writeheader(); w.writerows(per_run)

    rates = sorted({x["rate"] for x in per_run})
    summary = []
    for rate in rates:
        rows = [x for x in per_run if x["rate"] == rate]
        summary.append(dict(rate=rate, n_runs=len(rows),
                            mean_precision=st.mean(r["precision"] for r in rows),
                            mean_recall=st.mean(r["recall"] for r in rows),
                            mean_f1=st.mean(r["f1"] for r in rows)))
    with open(os.path.join(out_dir, "metrics_summary.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["rate","n_runs","mean_precision","mean_recall","mean_f1"]); w.writeheader(); w.writerows(summary)

    print(f"\nIPC on M3500 — mean over runs (root: {args.root})")
    print(f"{'rate%':>5} {'n':>3} {'precision':>10} {'recall':>8} {'F1':>8}")
    for s in summary:
        print(f"{s['rate']:>5} {s['n_runs']:>3} {s['mean_precision']:>10.4f} {s['mean_recall']:>8.4f} {s['mean_f1']:>8.4f}")
    print(f"\nwrote metrics_per_run.csv + metrics_summary.csv to {out_dir}")

if __name__ == "__main__":
    main()
