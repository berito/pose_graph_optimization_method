# DC-IPC experiment setup — repeatable ablation protocol

Parallel to `IPC_experiment_setup.md` / `TACO_experiment_setup.md`, but for **our** contribution.
Goal: every DC-IPC result is reproducible from (binary SHA + source SHA + data SHA + full param set +
config snapshot), and the study is a disciplined **one-factor-at-a-time ablation**, not ad-hoc runs.
**Provenance is git-free** — we hash what actually ran, which is immune to a dirty working tree.

Baseline of record is **relative**: success = DC-IPC (λ>0) beats DC-IPC (λ=0 ≡ IPC) on the *same*
binary, *same* data. Internally valid regardless of the IPC-paper reproduction (S1).

---

## 1. Parameter taxonomy (three classes — keep them separate)

### FIXED — inherited from the replicated IPC; never tuned in an ablation
Holding these constant is what makes the λ comparison clean.

| param | value | source |
|---|---|---|
| `s_factor` | 3 | IPC paper (Tier A, `PARAMETERS_AUTHOR_vs_OURS.md`) |
| `fast_reject_th` / `slow_reject_th` | 6.251 / 11.345 | IPC code (Tier B), χ² α=0.95 |
| `fast/slow_reject_iter_base` | 50 / 100 | IPC code |
| final `optimize(...)` | 1000 | IPC code |

### SCENARIO — defined by the generator, recorded in the data `manifest.csv`
These describe the *test case*; changing one means generating a new data set.

| param | default | meaning |
|---|---|---|
| `rate` | 10–50 | outliers as % of true loops |
| `group_len` | 5 | edges per correlated group |
| `disp_trans` / `disp_rot` | 5.0 m / 30° | shared misalignment magnitude (trap difficulty) |
| `min_sep` | 50 | min anchor separation (also bounds subgraph span / runtime) |
| `sigma_t` / `sigma_r` | 0.1 / 0.01 | inlier noise (sets injected-edge information) |
| `seed` | run index 00–09 | determinism |

Scenario data lives at `experiments/datasets/2D/<DS>/SPOILED_DATA_CORR[/<suffix>]/<rate>/<run>.{g2o,groups,gt_labels}`.
Generate with `experiments/datagen/genCorrCampaign.sh` (env-overridable: `GROUP_LEN`, `DISP_TRANS`, `DISP_ROT`, …).

### METHOD — the ablation variables (what we actually vary)

| param | values | role |
|---|---|---|
| **`lambda`** | 0, … | **headline knob** — correlation reward; 0 ⇒ IPC |
| `use_best_k_buddies` / `k_buddies` | off / {2} | kBL subgraph (borrowed from TACO) |
| `use_recovery` | off / on | SR-SC retrospective pass (borrowed from TACO) |

---

## 2. Baseline operating point (the control)
`lambda=0, use_best_k_buddies=false, use_recovery=false, s_factor=3` ⇒ **≡ IPC**.
Every ablation below changes **exactly one axis** from this point.

---

## 3. Ablation matrix (one-factor-at-a-time)

| id | varies | held fixed | question |
|---|---|---|---|
| **A1** | `lambda ∈ {0,5,20,50,100,200}` | rate=30, canonical scenario | λ sensitivity → pick **λ\*** |
| **A0** ⭐ | `lambda ∈ {0, λ*}` × `rate ∈ {10,20,30,40,50}` | scenario | **G2 verdict**: on vs off across difficulty |
| **A2** | `group_len ∈ {2,5,10}`, `disp_trans ∈ {2,5,10}` | λ=λ\*, rate=30 | does stronger correlation widen the gap? |
| **A3** | kBL on/off, recovery on/off | λ=λ\*, rate=30 | do the TACO modules add anything on top? |

**λ\* is chosen by A1 first (data-driven), then A0 runs at that λ\*.** Order: A1 → A0 → A2 → A3.

Control runs alongside every cell: the same scenario with **no `.groups` sidecar** (or λ=0) is the
correlation-blind IPC baseline; `SPOILED_DATA/` (random outliers) is the G1 random control.

---

## 4. Runners & reproducibility — DECOUPLED runners, SHARED system

Each method has its **own independent runner** (launch/edit one without touching the others):

| runner | binary | s default | extra knobs | default data |
|---|---|---|---|---|
| `run_ipc.sh` | `ipc_tester_2D` | 3 | — | `SPOILED_DATA` |
| `run_taco.sh` | `taco_tester_2D` | 10 | `use_best_k_buddies`,`k_buddies`,`use_recovery` | `SPOILED_DATA` |
| `run_dcipc.sh` | `dc_ipc_tester_2D` | 3 | + `lambda` | `SPOILED_DATA_CORR` |

They all `source experiments/scripts/lib/campaign_lib.sh` — the **shared experiment system** (the part
that must be *identical* for every method): worker pool, idempotent resume on valid `.PR`, atomic
scratch→final, per-job scratch CWD, git-free provenance. The runner only declares its binary, knobs,
`apply_overrides()`, and `signature()`. So re-running IPC/TACO later (S1/S2 replication, or G1 on
correlated data) gets the same save/resume/provenance — without one entangled mega-script.

**Atomic per-run persistence** (the failure-mode fix): each `(ds,rate,run,λ)` is a separate invocation,
saved to its final path the instant it succeeds. rate-10 finishing is on disk even if rate-90 later
fails; run 00 is saved even if run 09 fails; a re-launch skips every valid `.PR` and continues.

Per run it writes:

- `<run>.TRJ` — optimized poses, `<run>.PR` — `precision recall` / `total_time avg_time` (same format as IPC).
- `<run>.meta.json` — **binary SHA-256** (what executed), **source SHA-256** (algorithm version),
  **data file + SHA-256**, full resolved param set, wall-time. Git-free.
- `<run>.cfg.yaml` — snapshot of the exact resolved config used.

Result layout (keyed by method signature):
```
experiments/results/<IPC|TACO|DC_IPC>/<DS>/<DATE>/<SCEN_TAG>/<sig>/<rate>/<run>.{TRJ,PR,meta.json,cfg.yaml}
  sig:  ipc_s3  |  taco_s10_k2_rec0  |  lam50_kbl0_rec0
```

Env knobs (per runner): `DATASETS RATES RUNS LAMBDAS USE_KBL K_BUDDIES USE_REC S_FACTOR DATADIR_NAME SCEN_TAG CAP DATE`.
Single job: `<runner> --one <ds> <rate> <run> <lambda>`.

Examples:
```
LAMBDAS="0 50" RATES="10 20 30 40 50" bash experiments/scripts/run_dcipc.sh           # A0 (correlated, default data)
DATADIR_NAME=SPOILED_DATA_CORR SCEN_TAG=corr bash experiments/scripts/run_ipc.sh      # G1: IPC on correlated
DATADIR_NAME=SPOILED_DATA RATES="10 50 100" bash experiments/scripts/run_ipc.sh       # S1 replication (random)
```

## 5. Results registry & analysis
`experiments/analysis/registry.py` scans the whole `results/` tree (all methods) and emits
`experiments/results/registry.csv` — one row per run with `method` + **all params + metrics**
(`precision,recall,f1,total_time,wall_s,bin_sha256,src_sha256,data_sha256,lambda,rate,kbl,rec,…`).
The ablation study is then a pivot/plot over this single table. ATE/RPE/SR are added later by running
the `evaluator` on the `.TRJ` files (precision/recall come straight from `.PR`).

## 6. Logging discipline
One `RUN_LOG.md` line per session: which ablation cell, the key number (e.g. ΔF1 = λ\* vs λ=0), next step.
Record each gate verdict (G1/G2/G3) + key number in `RUN_LOG.md` and the decisions log.
