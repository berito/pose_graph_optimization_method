# Experiment run conditions

The exact environment, build, data, parameters, and code-state under which the IPC + baseline
experiments are run on this server. Update this if any condition changes.

## Hardware / host
- Server `aiserver`: Intel Xeon Gold 6230R — **26 physical cores / 52 threads**, ~**62 GiB RAM**, RTX A4000 (unused; CPU-only).
- Worker-pool concurrency: **CAP=26** (one per physical core; single-threaded binaries — `user≈real` confirmed).

## Execution environment
- All builds + runs happen **inside the VS Code devcontainer image** `vsc-robust_pgo-…` (deps baked in:
  g2o, GTSAM, Kimera-RPGO, TBB, yq v4.53.3, Python 3.10.12). Repo bind-mounted at `/workspaces/robust_pgo`.
- Campaigns run as a **detached Docker container** via `experiments/scripts/run_as_service.sh`
  (`--restart unless-stopped`; Docker enabled on boot) → survives closing VS Code and a power-cycle; resumable.

## Build
- `cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_GTSAM_BASELINES=ON && make -j52`.
- g2o tag `20201223_git` (`/usr/local`); GTSAM 4.2.0 (MKL-free, Eigen+TBB); Kimera-RPGO.
- IPC code verified **byte-identical to upstream** (`EmilioOlivastri/IPC`); solver = **Dog-Leg (`dl_var`)**.

## Datasets & spoiling (shared by IPC + all baselines)
- 6 datasets, `canonic_inliers`: CSAIL 128, INTEL 256, M3500 1954, MIT 20, FRH 1505, **FR079 229**
  (= clean loop-closure edges `b≠a+1`).
- Spoiled with the author's **Vertigo `generateDataset.py`** (unmodified): `n = round(rate/100 × inliers)`
  outliers, **rates 10–100 step 10**, **runs 00–09 (seed = run index → deterministic/byte-reproducible)**.
- Spoiled files: `experiments/datasets/2D/<ds>/SPOILED_DATA/<rate>/<run>.g2o` (gitignored; regenerate on server).
- **FR079 caveat:** its info matrices are degenerate to g2o (`q_θθ=0`); this is the author's exact released file
  — do NOT convert (see `HANDOFF.md` fact #6). Standalone recall ~0.33 is expected.

## IPC run
- Binary `ipc_tester_2D`; `s_factor = 3` (2024 paper); `fast/slow_reject_th = 6.251 / 11.345`
  (= χ²(0.90,3)/χ²(0.99,3)); `fast/slow_reject_iter_base = 50 / 100`. k_buddies/use_recovery present but inert.
- Runner: `experiments/scripts/run_ipc_campaign.sh` (tag `IPC_S3`; threshold variants via `FAST_TH`/`SLOW_TH`).

## Baseline run
- 7 comparators: PCM, GNC, DCS, HUBER, GM, ADAPT, MAXMIX. Runner: `run_baseline.sh <METHOD>` (tag `BASE`).
- Per-method params + binaries + CLI: see `experiments/configs/baselines/PARAMS.md` (defaults, **not yet tuned**).
- **HUBER** → g2o `HUBER_2D` for all datasets (gtsam HUBER diverges).
- **ADAPT** → gtsam, our reimplementation, **Levenberg-Marquardt** inner solver, **unverified** (provisional;
  degenerate on tiny MIT).

## Metrics
- Each run emits `.PR` (`precision recall` / `time`); the `evaluator` produces `.TE`
  (`ATE_T <m> ABS_T <m> RPE_T <m>`) from the trajectory vs `GT.txt`.
- Output layout: `experiments/results/<METHOD|IPC>/<ds>/<DATE>/<TAG>/<rate>/<run>.{TRJ,PR,TE}` (gitignored).
- Aggregate/plot: `aggregate_pr.py`, `make_figures.py <TAG>`.

## Current findings (context for these runs)
- **S1: IPC does not reproduce its paper** with the released artifacts (recall ~0.59 vs >0.80;
  F1 0.70/0.66 vs 0.91/0.89; RPE/ATE worse). Only precision and the FRH dataset reproduce. Root cause
  deferred. Baselines are being run on the **same setup** to check the *relative* comparison.
