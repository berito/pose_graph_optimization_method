# experiments/ — our datasets, configs, runs, results

Scaffold for the experiment stages in `TASKS.md` (S1 reproduce IPC · S2 TACO · S3 prove the gap · …).

```
experiments/
├── datasets/   clean SOURCE graphs + ground truth (the raw IPC inputs)  — gitignored
│               e.g. datasets/2D/M3500/{graph.g2o, GT.txt}
├── configs/    experiment-specific run configs (e.g. one per dataset×method×regime)
├── results/    GENERATED spoiled graphs (.g2o + .groups) + metrics CSVs + per-run outputs  — gitignored
└── scripts/
    └── run_method.sh   run one method on one config (handles per-tool CLI)
```

> Keep the line clear: `datasets/` holds **clean source** graphs (inputs); generators write
> **spoiled** datasets into `results/` (outputs). Don't mix generated data into `datasets/`.

## How a run works (shared by IPC + all baselines)

All methods consume the **same** spoiled g2o + GT.txt + `canonic_inliers`, and each emits
`<output>.txt` (trajectory) + `<output>.PR` (precision, recall, avg convergence time).

The three tool families pass their config **differently**:

| Family | Binaries | Config flag |
|---|---|---|
| IPC | `ipc_tester_2D`, `ipc_tester_3D` | `-c <cfg>` |
| g2o baselines | `IN_SC_2D`, `IN_MAXMIX_2D`, `IN_DCS_2D`, `IN_GNC_2D`, `IN_HUBER_2D`, `IN_RRR_2D` (+ `_3D`, + offline) | `-cfg <cfg>` |
| GTSAM tier | `gtsam_DCS_2D`, `gtsam_GNC_2D`, `gtsam_PCM_2D`, `gtsam_HUBER_2D` (+ `_3D`) | `<cfg>` (positional) |

`run_method.sh` hides this — it locates the binary under `build/` and picks the right flag:

```bash
experiments/scripts/run_method.sh ipc_tester_2D cfg/2D/M3500_params.yaml experiments/results/M3500
```

> ⚠️ Untested until S0.1 build is green — binary paths/flags are from the READMEs; verify on first run.

## canonic_inliers per dataset (true loop closures in the CLEAN graph)

Counted from `experiments/datasets/2D/<NAME>/graph.g2o` (non-consecutive `EDGE_SE2` vertex ids).
Validated: M3500=1954 and INTEL=256 match the upstream configs.

| dataset | canonic_inliers |
|---|---|
| CSAIL | 128 |
| INTEL | 256 |
| M3500 | 1954 |
| MIT   | 20 |
| FRH   | 1505 |

(`canonic_inliers` is a property of the clean dataset; spoiling adds outliers but does not
change it. PR is scored against this count.)
