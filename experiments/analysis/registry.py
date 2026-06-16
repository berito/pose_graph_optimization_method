#!/usr/bin/env python3
"""ONE common registry across EVERY method (ipc, taco, dcipc, all baselines).

Keys off the universal artifact — every run writes a `.PR` — so it covers methods that
also drop a `meta.json` (the new runners) AND those that don't (the baseline campaign).
Per run it collects the common metrics:
    precision, recall, f1   <- <run>.PR
    ATE, RPE                <- <run>.TE   (if present; produced by analysis/evaluate.sh)
plus params/provenance from <run>.meta.json when available.

Result paths are  results/<METHOD>/<dataset>/<date>/<tag.../>/<rate>/<run>.PR
so (method, dataset, date, rate, run) come from the path; the bit between date and rate
is the config tag (e.g. `corr/lam50_kbl0_rec0`, or `BASE`).

Usage:
  registry.py [--root experiments/results] [--out experiments/results/registry.csv]
"""
import argparse, csv, glob, json, os, statistics as st


def f1(p, r):
    return 0.0 if not (p == p and r == r) or (p + r) == 0 else 2 * p * r / (p + r)


def read_pr(path):
    p = r = ttime = avg = float("nan")
    if os.path.exists(path):
        lines = open(path).read().split("\n")
        if lines and lines[0].strip():
            v = lines[0].split(); p, r = float(v[0]), float(v[1])
        if len(lines) > 1 and lines[1].strip():
            tv = lines[1].split(); ttime = float(tv[0])
            avg = float(tv[1]) if len(tv) > 1 else float("nan")
    return p, r, ttime, avg


def read_te(path):
    """`ATE_T <v> ABS_T <v> RPE_T <v> F` -> (ate, rpe)."""
    ate = rpe = float("nan")
    if os.path.exists(path):
        t = open(path).read().split()
        for i, tok in enumerate(t):
            if tok == "ATE_T" and i + 1 < len(t): ate = float(t[i + 1])
            if tok == "RPE_T" and i + 1 < len(t): rpe = float(t[i + 1])
    return ate, rpe


FIELDS = ["method", "config", "dataset", "date", "rate", "run",
          "precision", "recall", "f1", "ate", "rpe", "total_time", "avg_time",
          "lambda", "s_factor", "use_kbl", "k_buddies", "use_recovery", "wall_s",
          "bin_sha256", "src_sha256", "data_sha256", "data_file"]


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    default_root = os.path.normpath(os.path.join(here, "..", "results"))
    ap.add_argument("--root", default=default_root)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    out = args.out or os.path.join(args.root, "registry.csv")

    rows = []
    for pr in sorted(glob.glob(os.path.join(args.root, "**", "*.PR"), recursive=True)):
        rel = os.path.relpath(pr, args.root).split(os.sep)
        if len(rel) < 5:
            continue
        method, dataset, date = rel[0], rel[1], rel[2]
        rate, run = rel[-2], os.path.splitext(rel[-1])[0]
        tag = "/".join(rel[3:-2])

        p, r, ttime, avg = read_pr(pr)
        ate, rpe = read_te(pr[:-3] + ".TE")
        m = {}
        mp = pr[:-3] + ".meta.json"
        if os.path.exists(mp):
            m = json.load(open(mp))
        config = m.get("config") or tag or method
        rows.append({
            "method": method, "config": config, "dataset": dataset, "date": date,
            "rate": rate, "run": run,
            "precision": p, "recall": r, "f1": f1(p, r), "ate": ate, "rpe": rpe,
            "total_time": ttime, "avg_time": avg,
            "lambda": m.get("lambda"), "s_factor": m.get("s_factor"),
            "use_kbl": m.get("use_kbl"), "k_buddies": m.get("k_buddies"),
            "use_recovery": m.get("use_recovery"), "wall_s": m.get("wall_s"),
            "bin_sha256": m.get("bin_sha256"), "src_sha256": m.get("src_sha256"),
            "data_sha256": m.get("data_sha256"), "data_file": m.get("data_file"),
        })

    if not rows:
        print(f"no .PR files under {args.root}")
        return

    with open(out, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=FIELDS); w.writeheader(); w.writerows(rows)
    print(f"wrote {len(rows)} runs -> {out}")

    # Quick pivot: mean F1 by (rate, method/config) — the cross-method table at a glance.
    keyed = {}
    for x in rows:
        keyed.setdefault((x["rate"], f'{x["method"]}:{x["config"]}'), []).append(x["f1"])
    rates = sorted({k[0] for k in keyed}, key=lambda s: (len(s), s))
    cfgs = sorted({k[1] for k in keyed})
    print("\nmean F1 by (rate, method:config):")
    print("rate  " + "".join(f"{c:>30}" for c in cfgs))
    for rt in rates:
        cells = "".join(f"{st.mean(keyed[(rt, c)]):>30.3f}" if (rt, c) in keyed else f"{'-':>30}" for c in cfgs)
        print(f"{rt:>5} {cells}")


if __name__ == "__main__":
    main()
