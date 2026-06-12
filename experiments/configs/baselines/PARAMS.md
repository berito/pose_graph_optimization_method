# Baseline experiment — parameters used (S1 multi-method comparison)

Record of exactly what was run, so a re-run knows what to change/tune.
All methods run on the **same spoiled data as IPC** (`datasets/2D/<ds>/SPOILED_DATA/<rate>/<run>.g2o`),
same protocol (6 datasets × rates 10–100 × runs 00–09). Each config is a **superset** of the keys both
tiers read (each binary reads its subset, ignores the rest). The runner injects `dataset`, `output`,
`canonic_inliers` per run; everything else below is fixed.

## Methods, binaries, CLI

| Method | Binary | CLI | Tier | Source the paper used |
|---|---|---|---|---|
| PCM   | `gtsam_PCM_2D`   | `<cfg>` (positional) | gtsam | Kimera |
| GNC   | `gtsam_GNC_2D`   | positional | gtsam (loss = TLS) | gtsam |
| DCS   | `gtsam_DCS_2D`   | positional | gtsam | gtsam |
| GM    | `gtsam_GM_2D`    | positional | gtsam (our add) | gtsam |
| HUBER | g2o `HUBER_2D` (ALL datasets) | `-cfg` | g2o | gtsam* |
| ADAPT | `gtsam_ADAPT_2D` | positional | gtsam (our reimpl, LM) | custom g2o |
| MAXMIX| g2o `MAXMIX_2D`  | `-cfg` | g2o | OpenSLAM |

\* The paper used gtsam HUBER, but gtsam HUBER throws `IndeterminantLinearSystemException` on several
spoiled datasets (MIT, CSAIL...) — its GaussNewton diverges. We use the g2o Huber (same kernel; odom-init +
loop-only robust → stable) for **all** datasets.

## Parameters (default-first; NOT yet tuned — the paper used "best fixed set per method", unpublished)

**gtsam tier** (PCM, GNC, DCS, HUBER, GM, ADAPT) — `experiments/configs/baselines/<M>.yaml`:
| key | value | read by | meaning |
|---|---|---|---|
| max_iters | 1000 | all gtsam | optimizer iterations |
| inlier_th | 0.9 | all gtsam | χ² confidence for accept/eval (PCM: pairwise threshold) |
| alpha | 0.99 | DCS/GM/HUBER/GNC/ADAPT | χ² confidence → kernel/threshold |
| init_loop | false | all gtsam | initialise estimate from odometry only |

**g2o MAXMIX** — `MAXMIX.yaml`:
| key | value | meaning |
|---|---|---|
| max_iters | 50 | optimizer iterations |
| inlier_th | 0.5 | switch inlier threshold |
| switch_prior | 10.0 | switch prior weight |
| maxmix_weight | 0.1 | max-mixture param 1 |
| nu_constraint | 0.95 | max-mixture param 2 |
| nu_nullHypothesis | 1.0e-5 | max-mixture param 3 |
| batch_size | 10 | — |

Source of defaults: `baselines/robust_gtsam/cfg/params.yaml` (gtsam) and `baselines/robust_g2o/cfg/params_2D.yaml` (g2o),
plus `init_loop: false` (key the upstream gtsam cfg omitted).

## To tune later (per the paper's "best fixed parameter set")
- **PCM**: `inlier_th` (pairwise consistency threshold) is the main knob.
- **DCS/GM/HUBER/GNC**: `alpha` / `inlier_th` set the χ² kernel width.
- **MAXMIX**: `maxmix_weight`, `nu_constraint`, `nu_nullHypothesis`.
- Edit the single `<M>.yaml` to retune that method only.

## Known issues / caveats
- **HUBER**: gtsam HUBER crashes (`IndeterminantLinearSystemException`, GaussNewton divergence) on several
  spoiled datasets (MIT, CSAIL). Fix: runner uses the **g2o `HUBER_2D`** for ALL datasets (stable).
- **ADAPT**: our reimplementation (Barron IRLS), **not yet verified**. Originally crashed on MIT
  (GaussNewton); fixed by switching its inner optimizer to **Levenberg-Marquardt** (`adapt_2D.cpp`).
  Still degenerate on tiny MIT (P=R=0 — rejects all on 20 inliers); treat ADAPT numbers as provisional.
- Metrics: each run emits `.PR` (precision recall \n time) and, via the evaluator, `.TE` (ATE_T ABS_T RPE_T).
  Output layout: `experiments/results/<METHOD>/<ds>/<DATE>/<TAG>/<rate>/<run>.{TRJ,PR,TE}`.

## Author's released setup vs ours (checked against `ROS_upstream` = EmilioOlivastri/RobustOptimizationSLAM)
The author's released **scripts do NOT reproduce the paper's Fig.3/4** — they run a *partial* comparison,
and the param values come from one shared cfg per tier (no per-method tuning is in the release).

| Aspect | Author (released) | Ours | Status |
|---|---|---|---|
| g2o params (inlier_th 0.5, switch_prior 10, maxmix_weight 1e-1, nu_constraint 0.95, nu_nullHypothesis 1e-5, batch_size 10, max_iters 50) | `robust_g2o/cfg/params_2D.yaml` | same | ✅ matched |
| gtsam params (max_iters 1000, inlier_th 0.9, alpha 0.99) | `robust_gtsam/cfg/params.yaml` | same | ✅ matched |
| `init_loop` | **absent** from author's gtsam cfg (binary still reads it) | added `false` | ⚠️ ours (author unpinned) |
| `canonic_inliers` | per-dataset cfg in author's data dir (not in repo) | derived + validated | ✅ matched values |
| datasets | g2o script: CSAIL FR079 INTEL FRH (no MIT/M3500); gtsam: +MIT (no M3500) | all 6 | ⚠️ extended to paper's 6 |
| outlier rates | **g2o script: 10–50 only**; gtsam: 10–100 | 10–100 | ⚠️ paper protocol |
| methods | g2o: HUBER DCS MAXMIX RRR SC GNC; gtsam: HUBER DCS GNC (**PCM commented, GM absent**) | paper's 7 | ⚠️ paper's set |
| HUBER impl | gtsam HUBER | g2o HUBER | ⚠️ ours (gtsam crashes) |
| GM, ADAPT | not in released scripts | gtsam GM (our add) + ADAPT (our reimpl, LM) | ⚠️ ours |
| per-method params | one shared cfg per tier | per-method cfg files, same values | cosmetic (tunability) |

**Decision:** we follow the **paper protocol** (6 datasets × 10–100% × the 7 methods) using the author's
**committed parameter values**, since no complete author script exists to mirror verbatim.

## Run
```
bash experiments/scripts/run_baseline_campaign.sh                 # all 7, all datasets (TAG=BASE)
METHODS="PCM DCS" DATASETS="M3500" bash .../run_baseline_campaign.sh   # subset
```
Resumable (skips finished runs); launch via `run_as_service.sh` for VS-Code-independent execution.
