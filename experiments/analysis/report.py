#!/usr/bin/env python3
"""Generic, registry-driven analysis + figures. Independent of which algorithms or datasets
exist — it only filters the common table (registry.csv) by what you select and plots.

Adding a new algorithm or dataset needs NO change here: it just appears as new rows in the
registry and becomes selectable. The benchmark metrics are always the same
(precision, recall, f1, ate, rpe).

  report.py --methods DCS --datasets M3500 --metric f1            # single report (one line)
  report.py --methods IPC,DCS,dcipc --datasets M3500 --metric f1 # comparison (line per method)
  report.py --methods DCS --datasets M3500,INTEL --metric ate    # line per dataset
  report.py --methods dcipc --datasets M3500 --by lambda --metric f1   # DC-IPC lambda ablation
  report.py --methods all --datasets M3500 --metric f1,ate,rpe   # multi-panel

x-axis = outlier rate (line plot); if only one rate is selected it draws a bar chart instead.
Each series is averaged over seeds (mean, with a ±std band). Writes a PNG and a summary CSV.
Run registry.py first (and evaluate.sh if you need ATE/RPE).
"""
import argparse, csv, os, statistics as st
from collections import defaultdict
os.environ.setdefault("MPLCONFIGDIR", "/tmp/mplconfig")
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

METRIC_ALIAS = {"prec": "precision", "rec": "recall", "p": "precision", "r": "recall"}


def sel(arg, present):
    if arg in (None, "", "all"):
        return present
    return [x for x in arg.replace(",", " ").split()]


def fnum(s):
    try:
        v = float(s); return v if v == v else None
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--registry", default=os.path.normpath(os.path.join(here, "..", "results", "registry.csv")))
    ap.add_argument("--methods", default="all")
    ap.add_argument("--datasets", default="all")
    ap.add_argument("--metric", default="f1", help="one or comma-list: precision recall f1 ate rpe")
    ap.add_argument("--by", default="auto", help="what each series is: method|dataset|config|lambda|<col>|auto")
    ap.add_argument("--rate-col", default="rate")
    ap.add_argument("--out", default=None, help="output PNG (default: experiments/figures/report/<auto>.png)")
    args = ap.parse_args()

    if not os.path.exists(args.registry):
        raise SystemExit(f"no registry at {args.registry} — run analysis/registry.py first")
    rows = list(csv.DictReader(open(args.registry)))
    if not rows:
        raise SystemExit("registry is empty")

    methods = sel(args.methods, sorted({r["method"] for r in rows}))
    datasets = sel(args.datasets, sorted({r["dataset"] for r in rows}))
    metrics = [METRIC_ALIAS.get(m, m) for m in args.metric.replace(",", " ").split()]

    data = [r for r in rows if r["method"] in methods and r["dataset"] in datasets]
    if not data:
        raise SystemExit(f"no rows for methods={methods} datasets={datasets}")

    by = args.by
    if by == "auto":
        by = "method" if len(methods) > 1 else "dataset" if len(datasets) > 1 else \
             ("config" if len({r.get("config") for r in data}) > 1 else "method")
    if by not in data[0]:
        raise SystemExit(f"--by '{by}' is not a registry column ({list(data[0])})")

    rates = sorted({int(r[args.rate_col]) for r in data if r[args.rate_col].isdigit()})
    single_rate = len(rates) <= 1

    # agg[(series, metric, rate)] -> list of values
    agg = defaultdict(list)
    series_set = set()
    for r in data:
        s = r.get(by) or "?"
        series_set.add(s)
        rt = int(r[args.rate_col]) if r[args.rate_col].isdigit() else None
        for m in metrics:
            v = fnum(r.get(m))
            if v is not None and rt is not None:
                agg[(s, m, rt)].append(v)
    series = sorted(series_set)

    # summary CSV
    figdir = os.path.normpath(os.path.join(here, "..", "figures", "report"))
    os.makedirs(figdir, exist_ok=True)
    tag = f"{'-'.join(methods)}__{'-'.join(datasets)}__{'_'.join(metrics)}__by-{by}"
    out_png = args.out or os.path.join(figdir, tag + ".png")
    out_csv = os.path.splitext(out_png)[0] + ".csv"
    with open(out_csv, "w", newline="") as fh:
        w = csv.writer(fh); w.writerow([by, "metric", "rate", "mean", "std", "n"])
        for (s, m, rt), vals in sorted(agg.items()):
            w.writerow([s, m, rt, f"{st.mean(vals):.6f}",
                        f"{(st.stdev(vals) if len(vals) > 1 else 0):.6f}", len(vals)])

    # plot
    fig, axes = plt.subplots(1, len(metrics), figsize=(5.6 * len(metrics), 4.6), squeeze=False)
    for k, m in enumerate(metrics):
        a = axes[0][k]
        if single_rate:
            rt = rates[0] if rates else 0
            ys = [st.mean(agg[(s, m, rt)]) if agg[(s, m, rt)] else float("nan") for s in series]
            a.bar(range(len(series)), ys, color="tab:blue", edgecolor="black", linewidth=0.4)
            a.set_xticks(range(len(series))); a.set_xticklabels(series, rotation=45, ha="right", fontsize=8)
            for i, y in enumerate(ys):
                if y == y: a.text(i, y, f"{y:.3f}", ha="center", va="bottom", fontsize=8)
            a.set_title(f"{m} @ rate {rt}")
        else:
            for s in series:
                xs = [rt for rt in rates if agg[(s, m, rt)]]
                mu = [st.mean(agg[(s, m, rt)]) for rt in xs]
                sd = [st.stdev(agg[(s, m, rt)]) if len(agg[(s, m, rt)]) > 1 else 0 for rt in xs]
                if not xs: continue
                a.plot(xs, mu, marker="o", lw=2, label=str(s))
                a.fill_between(xs, [m_ - s_ for m_, s_ in zip(mu, sd)],
                               [m_ + s_ for m_, s_ in zip(mu, sd)], alpha=0.15)
            a.set_xlabel("outliers [%]"); a.set_title(f"{m} vs outlier %")
            a.legend(fontsize=8, title=by)
        a.grid(alpha=0.3)
    fig.suptitle(f"methods={','.join(methods)}  datasets={','.join(datasets)}  (series = {by})")
    fig.tight_layout(); fig.savefig(out_png, dpi=120); plt.close(fig)
    print(f"wrote {out_png}\n      {out_csv}  ({len(series)} series, {len(rates)} rate(s))")


if __name__ == "__main__":
    main()
