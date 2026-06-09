# IPC Paper Reference

> Ground-truth extraction from the IPC paper PDF
> (`docs/2024_Olivastri_IPC_Incremental_Probabilistic_Consensus.pdf`).
> Use this file to diff our S1 reproduction against the published numbers.

## Citation

**Title:** IPC: Incremental Probabilistic Consensus-based Consistent Set Maximization for SLAM Backends
**Authors:** Emilio Olivastri and Alberto Pretto (Department of Information Engineering (DEI), University of Padova, Italy)
**Venue:** 2024 IEEE International Conference on Robotics and Automation (ICRA), pp. 10283–10289
**Year:** 2024
**DOI:** 10.1109/ICRA57147.2024.10611214
**arXiv:** 2405.08503v2 [cs.RO], 15 Aug 2024
**Code:** https://github.com/EmilioOlivastri/IPC

## Summary

IPC (Incremental Probabilistic Consensus) is an **online, consensus-based** robust PGO back-end that
approximates the combinatorial problem of finding the **maximally consistent set** of loop-closure
measurements, incrementally. For each incoming loop closure it isolates the **minimal independent subgraph**
(`G'`) the constraint influences, solves PGO on just that subgraph, and **accepts the new loop closure only
if all previously accepted measurements in that subgraph agree** with the new solution via a **χ² test**
(Eq. 4). Previously integrated inliers therefore hold **veto power** over new candidates; if accepted, the
subgraph solution is propagated to the rest of the graph (Sec. III-C / III-D, Algorithm 1). Odometry edges are
assumed to be inliers (A1) and their information matrix is up-weighted by a scaling factor `s > 1` (Eq. 6).

## Hyperparameters & values

| Parameter | Code name | Value in paper | Where stated |
|---|---|---|---|
| Scaling factor **S** | `s_factor` | **s = 3** (fixed; **NOT swept**) | Sec. IV, first paragraph: "...setting **s = 3** for all experiments." Also defined in Sec. III-C / Eq. (6) as `s > 1`. |
| χ² test (acceptance) | — | `eᵀ Ω e < χ²_{α,δ}` (Eq. 4) | Sec. III-C, Eq. (4): α = confidence parameter, δ = degrees of freedom of the error. **No numeric threshold value is printed in the paper.** |
| Inlier-definition confidence | — | **α_th = 0.95** | Sec. IV-C ("Evaluation"): "an inlier is defined as a measurement having a χ² error less than χ²_th with α_th = 0.95." |
| Optimizer | — | **Dog-Leg** steps, on **g2o** | Sec. IV, first paragraph. |
| "k best buddies" / #voters / k | `use_best_k_buddies`, `k_buddies` | **NOT mentioned anywhere in the paper.** | — (no such parameter exists in the paper; this appears to be a code-only knob) |

### Notes on the specific values you asked to confirm

- **`s_factor` (the "S = ..." runtime print): s = 3.** Stated explicitly and used for *all* experiments
  (Sec. IV). It is a **single fixed value, not swept.** The paper stresses `s` is IPC's *only* control
  parameter and that it is "fairly simple to configure" (Sec. V, Conclusions).
- **χ² rejection thresholds (`fast_reject_th` / `slow_reject_th`, e.g. 6.251 / 11.345):**
  **NOT stated numerically in the paper.** The paper only gives the *form* of the test (Eq. 4,
  `eᵀ Ω e < χ²_{α,δ}`) parameterised by confidence α and DOF δ, plus the evaluation-time inlier
  threshold α_th = 0.95. The concrete numbers 6.251 / 11.345 correspond to χ² critical values
  (e.g. α≈0.90 at 3 DOF ≈ 6.251; α≈0.99 at 3 DOF ≈ 11.345) and must be confirmed **from the code**,
  not the paper — the paper does not list them.
- **`k_buddies` / "k best buddies":** **does not appear in the paper at all.** There is no
  number-of-voters / k parameter described; acceptance requires **all** measurements in `G'` to agree
  (Algorithm 1, line 20: `∀ e_ab ∈ E^I`). So this is a code-side knob with **no paper reference value**.

## Experimental setup

- **Implementation:** C++ on the **g2o** framework, Dog-Leg optimization steps, `s = 3` (Sec. IV).
- **Compared methods (7 baselines):**
  - **GNC** [8] — Graduated Non-Convexity (gtsam)
  - **ADAPT** [5] — Adaptive robust loss kernel (implemented inside g2o)
  - **DCS** [6] — Dynamic Covariance Scaling (gtsam)
  - **GM** [7] — German-McClure / Geman-McClure (gtsam, "GM" in plots)
  - **HUBER** [7] — Huber kernel (gtsam)
  - **MAXMIX** [10] — Max-Mixture (OpenSLAM implementation)
  - **PCM** [9] — Pairwise Consistency Maximization (Kimera's implementation [23])
  - For each method, the best fixed parameter set was used; GNC and PCM results taken after a fixed
    number of iterations due to long convergence times.
- **Datasets** (Sec. IV-A): standard PGO benchmarks, both real and synthetic, all originally **outlier-free**:
  - **Real:** INTEL, MIT, CSAIL, Freiburg Building (FRH), Freiburg University Hospital / FR079
  - **Synthetic:** **M3500**
  - (Fig. 1 trajectory comparison is on the **FR079** dataset at 70% outliers.)
- **Outlier-injection protocol:** outliers added with the **Vertigo package** [26], as a **percentage of the
  total number of inliers** present in the dataset (to keep results balanced). Outlier rates range from
  **10% to 100% in increments of 10%**. A rate of 100% means #outliers == #inliers. **Ten distinct
  trajectories** were generated per outlier percentage for fair/robust testing (Sec. IV-C).
- **Metrics** (Sec. IV-B):
  - **Precision** = TP/(TP+FP), **Recall** = TP/(TP+FN), **F1** = 2·(P·R)/(P+R) — on loop-closure
    accept/reject (TP/FP/TN/FN defined over loop closures).
  - **ATE** (Absolute Trajectory Error) and **RPE** (Relative Pose Error) [28] — accuracy of the estimated
    trajectory vs. the outlier-free ground-truth trajectory; ground truth obtained via **SE-Sync** [27]
    on the outlier-free dataset.
  - **ACTxC** = Average Convergence Time per Constraint (runtime, seconds).
  - Precision/Recall/F1 (Fig. 3) and ATE/RPE/convergence (Fig. 4) are **averaged over all datasets**.

## Numerical results

### Table I — Summary of most important metrics (averaged), at 50% and 100% outliers

(Reproduced from Fig. 3 / Fig. 4. ↓ = lower is better. Bold = best per column in the original.)

| Method | 50% F1 | 50% RPE ↓ | 50% TIME ↓ | 100% F1 | 100% RPE ↓ | 100% TIME ↓ |
|---|---|---|---|---|---|---|
| GNC    | 0.747 | 32.20 | 155.86   | 0.604 | 35.53 | 349.04    |
| ADAPT  | 0.873 | 1.867 | 5457.07  | 0.738 | 1.856 | 12576.27  |
| MAXMIX | 0.77  | 0.324 | 70.99    | 0.66  | 0.43  | 137.29    |
| DCS    | 0.796 | 26.12 | **4.58** | 0.66  | 37.684| **8.28**  |
| GM     | 0.795 | 23.64 | 7.07     | 0.65  | 38.33 | 9.59      |
| HUBER  | 0.706 | 21.44 | 10.04    | 0.55  | 27.32 | 8.29      |
| PCM    | 0.53  | 25.01 | 8289.04  | 0.371 | 28.54 | 12655.41  |
| **IPC**| **0.91** | **0.05** | 321.37 | **0.89** | **0.068** | 442.733 |

Notes:
- IPC has the **best F1** at both 50% (0.91) and 100% (0.89), and the **best (lowest) RPE** at both
  (0.05 and 0.068). DCS is fastest. ATE column from Fig. 4 is not tabulated in Table I.

### Table II — IPC's ACTxC (Average Convergence Time per Constraint) per dataset, seconds

| Dataset | 20% | 40% | 60% | 80% | 100% |
|---|---|---|---|---|---|
| MIT   | 0.053 | 0.050 | 0.050 | 0.050 | 0.044 |
| INTEL | 0.056 | 0.051 | 0.058 | 0.054 | 0.056 |
| M3500 | 0.577 | 0.542 | 0.717 | 0.601 | 0.623 |
| CSAIL | 0.043 | 0.042 | 0.043 | 0.042 | 0.043 |
| FRH   | 0.098 | 0.095 | 0.100 | 0.098 | 0.109 |
| FR079 | 0.042 | 0.042 | 0.042 | 0.041 | 0.043 |

Note: M3500's per-constraint time is by far the highest because its outliers tend to capture a vast
section of the trajectory, producing high-dimensional subproblems.

### Figures 3 & 4 (plots, not tabulated — qualitative trends)

These are **averaged-over-all-datasets line plots** vs. outlier % (10–100%); exact per-point values are
not printed, only the 50%/100% slices in Table I. Key reported trends:

- **Fig. 3 (Precision / Recall / F1):**
  - **MAXMIX and IPC reject most outliers** across datasets (highest precision), closely followed by PCM.
  - For **GNC, DCS, GM**, precision **drops below 0.5 once outliers exceed 50%**.
  - **Recall** is nearly **constant** vs. outlier rate for all methods; **GM and DCS have the highest
    recall**, then ADAPT and IPC (**IPC recall consistently > 80%**).
  - In the plots, GM is completely covered by DCS (precision & recall); MAXMIX is nearly covered by IPC
    in the precision plot.
- **Fig. 4 (ATE / RPE / Convergence):**
  - **ATE:** most methods similar; **IPC clearly best (lowest) ATE** (the cyan IPC curve sits well below
    the others, ~20–35 m vs ~50–80 m for the pack).
  - **RPE:** **IPC, MAXMIX, ADAPT** have the best (lowest) RPE. (PCM's RPE plot is fully covered by ADAPT.)
  - **Convergence time:** IPC is **not** the fastest — robust estimators and DCS are faster — but IPC is
    comparable to / faster than more complex methods (GNC, MAXMIX, PCM, ADAPT). DCS/GM/HUBER are the
    cheapest (~10⁰–10¹ s region); PCM and ADAPT are the most expensive (~10³ s).

## Reproduction targets

Specific numbers our S1 (random-outlier, Vertigo) reproduction must match. Note: paper numbers are
**averaged over all 6 datasets × 10 trajectories**, so single-dataset M3500 runs will not match Table I
exactly — these are the right-shape targets / sanity bounds:

- **IPC scaling factor must be `s = 3`** (the runtime "S = ..." print) — fixed, not swept.
- **IPC averaged F1:** ≈ **0.91 at 50% outliers**, ≈ **0.89 at 100% outliers** (Table I) — IPC should be
  the **top F1** method, beating MAXMIX (0.77 / 0.66) and PCM (0.53 / 0.371).
- **IPC averaged RPE (lowest):** ≈ **0.05 at 50%**, ≈ **0.068 at 100%** (Table I).
- **IPC precision should be high (near top, with MAXMIX)** and **recall consistently > 80%** across
  10–100% outliers (Fig. 3).
- **IPC averaged convergence TIME:** ≈ **321 s at 50%**, ≈ **443 s at 100%** (Table I) — slower than DCS
  (~4.6 / 8.3 s) but far faster than PCM/ADAPT (~8000–12600 s).
- **IPC per-constraint time (ACTxC) on M3500** (synthetic, the one in our `datasets/2D/M3500/`):
  ≈ **0.54–0.72 s/constraint** across 20–100% (Table II); other datasets are ~0.04–0.11 s/constraint.
- **Inlier definition for evaluation:** χ² error < χ²_th with **α_th = 0.95** (use the same to label
  TP/FP/TN/FN).

### Caveats for diffing

- The paper does **not** publish per-dataset precision/recall/ATE numbers, only the all-dataset averages
  in Table I (50%/100% slices) and IPC's ACTxC in Table II. Per-dataset reproduction must be judged
  against the **averaged trends**, not exact per-dataset figures.
- The concrete **χ² thresholds (e.g. 6.251 / 11.345) are not in the paper** — confirm them from the IPC
  source code, not from this reference.
- **`k_buddies` / `use_best_k_buddies` has no paper counterpart** — there is no k / number-of-voters
  parameter in the published method (acceptance requires agreement of *all* edges in the subgraph).
