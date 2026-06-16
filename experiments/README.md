# experiments/ — datasets, configs, runs, results

**To run an experiment, you do not need to open any script.** Pick the method's runner from the
table below. There is **exactly one runner per method** (no duplicates).

Four purpose-folders — generate, run, analyze, plus the data and results:
```
experiments/
├── datasets/   clean SOURCE graphs + ground truth (inputs)
├── datagen/    ALL dataset generation (the only thing here is making datasets)
│   ├── generate_spoiled.py          random/Vertigo outliers → SPOILED_DATA/
│   ├── generateCorrelatedDataset.py correlated grouped outliers → SPOILED_DATA_CORR/
│   └── genCorrCampaign.sh           driver: generate the full correlated grid
├── scripts/    the METHOD RUNNERS (one per method) + shared run-system
│   ├── run_ipc.sh · run_taco.sh · run_dcipc.sh · run_baseline.sh
│   └── lib/campaign_lib.sh   shared run system (atomic save / resume / provenance)
├── analysis/   ALL result analysis — COMMON to every method (same metrics for all)
│   ├── evaluate.sh   run the evaluator on any .TRJ → .TE (ATE/RPE)
│   ├── registry.py   scan ALL results → registry.csv (precision/recall/f1/ate/rpe + params)
│   └── make_figures.py · make_comparison.py
└── results/    outputs (.TRJ/.PR/.TE/meta.json)        — gitignored (.PR/.TE tracked)
```

## Which file runs which experiment

| I want to run… | Use this one file | Reads config from |
|---|---|---|
| **IPC** | `run_ipc.sh` | `ipc/cfg/2D/<dataset>_params.yaml` |
| **TACO** | `run_taco.sh` | `taco/cfg/2D/<dataset>_params.yaml` |
| **DC-IPC** (λ ablation) | `run_dcipc.sh` | `dc_ipc/cfg/2D/<dataset>_params.yaml` |
| **one baseline** (e.g. DCS) | `run_baseline.sh DCS` | `configs/baselines/<METHOD>.yaml` |
| **all baselines** | `run_baseline.sh all all all all` | `configs/baselines/*.yaml` |

All three method runners share the same run **system** via `lib/campaign_lib.sh`: each (dataset×rate×run)
is saved atomically the moment it finishes, finished runs are **skipped on re-run** (resume), and every
run drops a `meta.json` (git-free provenance: binary/source/data SHA) + a snapshot of its config.

### Run examples
```bash
# DC-IPC on correlated data, λ ablation (the contribution)
LAMBDAS="0 50" RATES="10 20 30 40 50" bash experiments/scripts/run_dcipc.sh

# IPC replication on random spoiled data
DATADIR_NAME=SPOILED_DATA RATES="10 50 100" bash experiments/scripts/run_ipc.sh

# IPC on the SAME correlated data (G1 — does the baseline fail?)
DATADIR_NAME=SPOILED_DATA_CORR SCEN_TAG=corr bash experiments/scripts/run_ipc.sh
```
### Analyze (same for every method — that's the point)
Every method ends in the same metrics, so analysis is one shared folder:
```bash
bash   experiments/analysis/evaluate.sh      # any .TRJ → .TE  (ATE/RPE);  run inside the devcontainer
python3 experiments/analysis/registry.py     # ALL results → experiments/results/registry.csv + an F1 pivot
```
`registry.py` keys off `.PR` (so it sees every method, with or without `meta.json`), pulls ATE/RPE
from `.TE`, and merges params/provenance from `meta.json` → one table with
`precision, recall, f1, ate, rpe` + the parameters per run.

## Results layout
```
results/<METHOD>/<dataset>/<date>/<scenario>/<signature>/<rate>/<run>.{TRJ,PR,meta.json,cfg.yaml}
  signature = ipc_s3 | taco_s10_k2_rec1 | lam50_kbl0_rec0
```

## canonic_inliers per dataset (true loop closures in the CLEAN graph)
Counted from `datasets/2D/<NAME>/graph.g2o` (non-consecutive `EDGE_SE2`). PR is scored against this.

| dataset | CSAIL | INTEL | M3500 | MIT | FRH | FR079 |
|---|---|---|---|---|---|---|
| canonic_inliers | 128 | 256 | 1954 | 20 | 1505 | 229 |

> `canonic_inliers` is a property of the clean dataset (spoiling adds outliers, doesn't change it).
