# TACO — experiment setup (the author's protocol, for S2 replication)

**Why this file:** the exact experimental conditions the author used for TACO, in one place, so our S2
run replicates it faithfully. **Source: Olivastri PhD thesis, Chapter 5 (§5.4), pages cited inline.**
TACO was **never released as code or run-scripts** (confirmed: no `EmilioOlivastri/TACO` repo; only
`IPC` and `RobustOptimizationSLAM` exist) — so this protocol is reconstructed from the chapter text.
Result targets (Tables 5.1–5.5) live in [`TACO.md`](TACO.md); this file is the **how-to-run**.

---

## 1. Datasets

### 2D (pose-graph SLAM) — our S2 scope
Intel, MIT, CSAIL, **FRH** (Freiburg Building 079 / hospital), **FR079** — openslam sources [116][117].
(M3500 is **Chapter-4 IPC only**, not in the TACO Ch.5 tables.) Originals are **outlier-free**; their
existing loop closures are the **inlier ground truth**. (§5.4.3, p.82.)

### 3D (Visual SLAM) — out of current scope, recorded for completeness
KITTI_00, KITTI_05 (KITTI visual-odometry seq. 00 & 05), TUM_FR1_DESK (TUM RGB-D). Odometry + loop
closures generated with **ORB-SLAM2**; ground-truth trajectory via **SE-Sync** on the clean version.
(§5.4.3, p.82.)

---

## 2. Outlier-injection protocol (§5.4.3, p.82) — the core of the setup

- Outliers added with the **Vertigo** package (grossly-wrong random loop closures).
- Count = a **percentage of the number of true inliers**, so **100% ⇒ #outliers = #inliers**.
- Sweep **10% → 100%, in steps of 10%** (10 operating points).
- **Ten distinct corrupted datasets** generated **per outlier percentage** → results are **averaged
  over the 10**.
- Datasets start outlier-free; injected loops are the negatives, original loops the positives.

> This is identical in spirit to the IPC Ch.4 protocol — same Vertigo generator, same %-of-inliers
> sweep, same 10-datasets-per-rate averaging. So our existing `SPOILED_DATA/{rate}/{00..09}.g2o`
> layout already matches; reuse it.

---

## 3. Methods / variants evaluated (Tables 5.1–5.5)

Comparators: **HUBER, DCS, PCM, GNC, MAXMIX** (we have these as baselines).
TACO family (our `taco/` project, config-selected):

| Variant | `use_best_k_buddies` | `k_buddies` | `use_recovery` |
|---|---|---|---|
| IPC (full subgraph)   | false | 0 | false |
| 2BL-IPC               | true  | 2 | false |
| TACO (full + SR-SC)   | false | 0 | true  |
| 2BL-TACO              | true  | 2 | true  |

k-sweep also reported: **k ∈ {2, 3, 5, 10}**, with **k = ∞ ≡ IPC** (Fig. 5.8). Headline = **k = 2**.

---

## 4. Metrics (§5.4.1–5.4.2)

- **SR — Success Rate (PRIMARY):** `SR = N_success / N_total`, a run is *successful* iff its
  **ATE < th_ATE = 0.75 m**. ↑ better. (ATE/RPE alone are called **unreliable** under heavy
  distortion — Fig 5.7 shows visually-worse trajectories scoring *better* ATE — so SR is the headline.)
- **Precision / Recall / F1** on loop classification (TP/FP/FN). ↑ better.
- **ATE / RPE** (RMSE, metres). ↓ better — report but treat as secondary.
- Inlier definition for methods without explicit classification: χ² error `< χ²_{α}` with **α = 0.95**.

---

## 5. Hyperparameters (§5.4, p.79) — set these in the configs

| Parameter | Symbol / our field | 2D value | 3D value | Notes |
|---|---|---|---|---|
| Odometry confidence scaling | `s` → `s_factor` | **10** | 30 | IPC Ch.4 used 3; TACO raised it |
| k-best loops cap | `k` → `k_buddies` (+`use_best_k_buddies`) | **2** | 2 | k=∞≡IPC |
| SR-SC trigger / batch size | `R` | **10** | 10 | run SR-SC every 10 detections |
| SR-SC voting rounds | `maxIter` | **20** | 20 | |
| Random prior-flip fraction | `ε` | **0.3** | 0.3 | 30% of labels flipped per round |
| Voting promotion threshold | `inlierTh` | **0.7** | 0.7 | promote if avg vote > 0.7 |
| Per-round switch cutoff | `s_ab` | **0.5** | 0.5 | counts as inlier that round |
| χ² confidence | `α` | **0.95** | 0.95 | inlier ⇔ χ² < χ²_α |
| Success-Rate ATE threshold | `th_ATE` | **0.75** | 0.75 | run "successful" if ATE < 0.75 m |
| Switch prior weight | `γ` info | 1.0 | 1.0 | SC-paper default (our SR-SC uses 1.0) |

Parameters held **constant across all datasets of the same dimensionality** (one 2D config, one 3D).

---

## 6. Solver & hardware (§5.4, p.79)

- **Framework:** C++, **g2o**, **Powell's Dog-Leg** optimization steps.
- **CPU:** single **Intel Xeon Gold 5220**.
- Our build uses the same g2o; note our SR-SC sub-optimizer currently uses **Levenberg-Marquardt**
  (BlockSolverX) for the switchable subproblem — a deviation to record (the thesis specifies Dog-Leg
  for the main solve; it does not separately specify the SR-SC inner solver).

---

## 7. Aggregation for our figures (matches how we did IPC/baselines)

Two-level averaging: (1) average each metric over the **10 corrupted datasets** at a given rate, then
(2) average across **datasets** for the per-rate summary curves. Tables 5.3/5.4 report ATE/RPE only at
**50% and 100%**; Tables 5.1/5.2 report SR/F1/PR at 20/40/60/80/100%.

---

## 8. Known gaps / our deviations (be honest in the write-up)

- **Iteration budgets not stated:** the thesis gives `maxIter=20` for SR-SC voting but does **not**
  publish the per-subproblem optimization iteration counts (our IPC `fast/slow_reject_iter_base`
  50/100 are carried over — an assumption, same open lead as the IPC S1 divergence).
- **χ² thresholds:** thesis specifies only α=0.95; our configs mirror the IPC per-dataset
  `fast/slow_reject_th` values (6.251 / 11.345). Any fast-vs-slow asymmetry is an implementation
  choice, not sourced from the thesis.
- **Minimal trusted subgraph:** thesis uses Dijkstra-minimal (Fig 5.5, ~half size); our SR-SC uses the
  transitive node-span superset (correct, larger). Affects speed, not correctness.
- **SR-SC inner solver:** LM (ours) vs Dog-Leg (thesis main solve).
- **Switch prior weight:** 1.0 (SC-paper default; thesis does not specify a TACO-specific value).

---

## 9. Reproduction targets (go/no-go) → see [`TACO.md`](TACO.md) §"reproduction targets" + Tables 5.1–5.5
Priority for S2: first reproduce **2BL-IPC** (Table 5.1, e.g. 2D@100%: F1≈0.92, PR≈0.96) with SR-SC
**off**, isolating Module A; then enable SR-SC and check **TACO** (2D@100%: SR≈0.38, F1≈0.93,
PR≈0.98; avg SR > 0.55). Use **SR + P/R/F1** as the verdict metrics, not raw ATE.
