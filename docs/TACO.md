# TACO & IPC — Thesis Reference (extracted from Olivastri PhD thesis)

> **TACO IS PRESENT in this thesis** as a full chapter (Chapter 5).
> TACO = **"Test And Check Optimization"** (per the Glossary, p. xii). It is *not* about
> correlated/grouped outliers explicitly — it is a two-time-horizon robust PGO framework
> (online "test" module + offline "check"/error-recovery module). See notes on correlation below.

---

## Source

- **Thesis:** Emilio Olivastri, *"Robust Pose Graph Optimization for Calibration and Autonomous
  Robots Navigation"*, Ph.D. in Information Engineering (ICT, XXXVII Cycle), Università degli Studi
  di Padova (DEI). Supervisor: Alberto Pretto; Coordinator: Fabio Vandin.
- **Chapter 4 — IPC** (Incremental Probabilistic Consensus-based Consistent Set Maximization),
  pp. **49–63**. Method §4.3 (pp. 54–59), evaluation §4.4 (pp. 59–63).
- **Chapter 5 — TACO** (A Test and Check Framework for Robust PGO), pp. **65–90**.
  Method §5.3 (pp. 69–78): IPC recap §5.3.1, **kBL-IPC** §5.3.2 (pp. 72–74),
  Switchable Constraints §5.3.3 (pp. 74–75), **SR-SC** §5.3.4 (pp. 75–78);
  Experiments §5.4 (pp. 79–89).
- TACO is built **on top of IPC** (Ch. 4). TACO = **kBL-IPC** (short-horizon "test") **+ SR-SC**
  (long-horizon "check"). The thesis evaluates three variants: `kBL-TACO` (= kBL-IPC + SR-SC),
  `TACO` (= full IPC + SR-SC), and `kBL-IPC` alone.
- **Open-source (TACO):** `https://github.com/EmilioOlivastri/TACO` — but the thesis notes the code;
  for our purposes the implementation must be reproduced from the chapter text (the task states the
  TACO source was never released; the repo link is in the thesis but treat the method below as the
  spec).
- **Open-source (IPC):** `https://github.com/EmilioOlivastri/IPC`.

**Correlation note (relevant to our thesis bet):** TACO does **not** target *correlated / grouped*
outliers as a named problem. Its failure-mode analysis (§5.4, Fig. 5.6/5.7) is about trajectory
*distortion* types, and its outlier injection is the standard Vertigo random-outlier protocol
(per-edge random, not grouped). So TACO is a **correlation-blind** comparator, exactly the class our
correlation-aware idea aims to beat. Its decision unit is still per-candidate-loop-closure (with a
local subgraph consistency test), not group-joint.

---

## TACO — algorithm (step-by-step, enough to reimplement)

TACO has two modules operating at different time horizons (Fig. 5.1):

### Module A — kBL-IPC ("k-Best Loops IPC"), the short-horizon online TEST (§5.3.2)

This is IPC (Ch. 4) but with the *minimal independent subgraph* replaced by an **approximate
subgraph using only the `k` most-overlapping previously-accepted loop closures**. Plain IPC builds
the full minimal independent subgraph (which can grow toward the whole graph → slow); kBL-IPC caps
the test subgraph at `k` loops.

**Per incoming candidate loop closure `z_ab` (edge `e_ab`, with a<b):**

1. **Score overlap of the candidate against every accepted inlier loop** `e_ij ∈ ᶦE_l` using
   **Intersection-over-Union (IoU)** of their node spans (Eq. 5.6):
   - `V(a,b) = { x_i : a ≤ i ≤ b }` (set of nodes spanned by the candidate loop).
   - `V(c,d) = { x_i : c ≤ i ≤ d }` (span of an accepted loop `e_cd`).
   - `IoU(e_ab, e_cd) = |V(a,b) ∩ V(c,d)| / |V(a,b) ∪ V(c,d)|`.
2. **Select the `k` accepted loops with highest IoU** → set `E_l^k` (Algorithm 6 `findKBestLoops`:
   compute IoU of candidate vs every accepted loop, sort by descending score, pop the top `k`).
3. **Build the approximate minimal independent subgraph `G^k(a,b)`** from: the `k` selected loop
   edges `E_l^k`, the candidate edge `e_ij`, plus the relevant **odometry** edges spanning the
   subgraph.
4. **Optimize the subgraph PGO** using the IPC objective (Eq. 5.4):

   `Σ_{(a,a+1)∈E_o^I}  e_{a,a+1}^T · s·Ω_{a,a+1} · e_{a,a+1}  +  Σ_{(a,b)∈E_l^I}  e_{a,b}^T · Ω_{a,b} · e_{a,b}`

   — odometry information matrices are scaled by a **fixed confidence factor `s > 1`** (odometry is
   trusted). The scaling factor adds **no extra variables** (contrast switchable constraints).
5. **χ² consistency test** of the new solution against **all** edges in the subgraph (Eq. 5.5):

   `e_{a,b}^T · Ω_{a,b} · e_{a,b}  <  χ²_{α,δ} ,  ∀ e_{a,b} ∈ E_o^I ∪ ᶦE_l`

   where `α` = confidence, `δ` = DoF of the error. Same as IPC Eq. 4.4 but over `E^k` instead of the
   full subgraph.
6. **Accept / reject:**
   - If **any** test fails → candidate is an **outlier**: `ᵒE_l ← {(a,b)} ∪ ᵒE_l`, revert subgraph
     nodes to pre-optimization state.
   - If **all** tests pass → **inlier**: `ᶦE_l ← {(a,b)} ∪ ᶦE_l`, **propagate** the subgraph
     optimization to the rest of the graph.
7. **Every tested candidate** (whether labelled inlier or outlier) is also appended to an **unrevised
   set `Ē_l`**. When `|Ē_l|` reaches size **R**, trigger Module B (SR-SC).

`k` trades speed vs robustness. `k = ∞` (all intersected loops) ≡ plain IPC. Smaller `k` is faster
but less robust. The thesis settles on **k = 2** (called `2BL-IPC` / `2BL-TACO`).

### Module B — SR-SC ("Selective Randomized Switchable Constraints"), the long-horizon CHECK (§5.3.4)

A retrospective error-recovery pass that can **flip** earlier inlier/outlier decisions, built on the
**Switchable Constraints (SC)** framework with a **randomized voting** scheme, run only on a small
**trusted subgraph** for efficiency. Triggered every time the unrevised set `Ē_l` hits size `R`.

**Switchable-constraint background (§5.3.3):** each loop closure gets a real-valued **switch
variable `s_ab ∈ ℝ`** mapped through `Ψ(·): ℝ → [0,1]` (linear `Ψ(s_ab) = s_ab`, clamped
`0 ≤ s_ab ≤ 1`). The augmented loop error is `ê_ab = Ψ(s_ab)·e(x_a,x_b)` (Eq. 5.7). A prior penalty
`p_ab = ||γ_ab − s_ab||` (Eq. 5.8) with prior `γ_ab ∈ {0,1}` (1 = believed inlier, 0 = outlier)
prevents the optimizer from zeroing all switches. Full SC cost = Eq. 5.9.

**Trusted / unrevised sets:**
- `Ē_l` (**unrevised set**): all loops tested by kBL-IPC but not yet revised by SR-SC.
- `ᵗE_l = ᶦE_l \ Ē_l` (**trusted set**): inliers already validated AND revised in prior SR-SC runs;
  their inlier status is held fixed during revision.

**SR-SC procedure (Algorithm 7), per trigger:**

1. `ᵗE_l ← ᶦE_l \ Ē_l` (trusted = confirmed inliers minus the unrevised batch).
2. **Find the minimal trusted subgraph `G^T`** (`FindMinTrustedGraph`): the smallest subgraph that
   (a) contains all edges of `Ē_l`, (b) contains all vertices touched by `Ē_l`, and (c) for every
   pair of those vertices has a connecting path through trusted (`ᵗE_l`) or odometry (`E_o`) edges
   only. Minimal version found via **Dijkstra shortest paths** (Fig. 5.5). This reduces the problem
   to ≈ **half** its original size (Fig. 5.10).
3. **Initial guess:** solve Eq. 5.12 on `G^T` *without* the unrevised constraints (trusted + odometry
   only) — gives an initial pose estimate.
4. Init voting accumulator `acc_ab ← 0` for all `(a,b) ∈ Ē_l`.
5. **Voting loop** (`for maxIter times OR until convergence`):
   - Set switch priors `γ_ab ← 1` if `(a,b) ∈ ᶦE_l` else `0` (kBL-IPC labels).
   - **Randomly flip** a fraction `ε` of those priors (`randomFlip(ε)`); some switch variables
     `s_ab` are also randomly fixed to bias the optimizer.
   - **Solve the reduced SC problem Eq. 5.12** on the minimal trusted subgraph:

     `Σ_{E_o∩E^T} e^TΩe  +  Σ_{ᵗE_l∩E^T} e^TΩe  +  Σ_{Ē_l} (ê_ab^T Ω_ab ê_ab + p_ab Ω_p p_ab)`

   - For each `(a,b) ∈ Ē_l`: if resulting `s_ab > 0.5` → `acc_ab ← acc_ab + 1` (counted as inlier
     this round).
6. **Final vote:** for each `(a,b) ∈ Ē_l`, `avgS ← getAverageState(acc_ab)`; if `avgS > inlierTh`
   → promote to inlier (`E_tmp ← E_tmp ∪ {(a,b)}`).
7. `ᶦE_l ← ᶦE_l ∪ E_tmp`; **clear the unrevised set** `Ē_l ← ∅`.

Intuition: loops that come out as inliers across many random perturbations of the priors are robustly
promoted; spurious ones are filtered. SR-SC can run in parallel with the online kBL-IPC stream.

---

## TACO — hyperparameters & values

All from §5.4 (p. 79) unless noted.

| Parameter | Symbol / code name | Value used in thesis | Notes |
|---|---|---|---|
| Odometry confidence / information scaling | `s` (Eq. 5.4) | **`s = 10`** for 2D datasets; **`s = 30`** for 3D Visual SLAM | g2o, Dog-Leg steps. (In Ch.4 IPC used s=3; TACO raised it.) |
| k-best loops cap | `k` (kBL-IPC) | **`k = 2`** (the headline `2BL-IPC`/`2BL-TACO`); also swept k∈{2,3,5,10} and k=∞≡IPC | Fig. 5.8 |
| SR-SC trigger / unrevised batch size | `R` (= max `\|Ē_l\|`) | **`R = 10`** (SR-SC runs every 10 loop-closure detections) | Alg. 7 |
| SR-SC max voting iterations | `maxIter` | **`maxIter = 20`** | Alg. 7 |
| Random prior-flip fraction | `ε` | **`ε = 0.3`** (30% of inlier/outlier labels randomly flipped per round) | Alg. 7 line 7 |
| Voting inlier threshold | `inlierTh` | **`inlierTh = 0.7`** | Alg. 7 line 15 |
| Switch decision threshold | (s_ab cutoff) | **`s_ab > 0.5`** counts as inlier in a round | Alg. 7 line 10 |
| χ² confidence | `α_th` | **`α_th = 0.95`** (inlier ⇔ χ² error < χ²_th) | for methods w/o explicit classification |
| Success-Rate ATE threshold | `th_ATE` | **`th_ATE = 0.75`** | run "successful" if ATE < 0.75 |
| Switch prior weight | `γ_ab` | `1` (inlier) / `0` (outlier), from kBL-IPC labels | Eq. 5.8 |

Solver: C++, **g2o** framework, **Powell's Dog-Leg** optimization steps. CPU: single Intel Xeon Gold
5220. Parameters kept constant across datasets of the same dimensionality (2D vs Visual SLAM).

---

## TACO — numerical results (faithful tables)

### Outlier-injection protocol (§5.4.3, p. 82)
- Outliers added with the **Vertigo package**, as a **percentage of total inliers** (so 100% ⇒
  #outliers = #inliers). Sweep **10% → 100% in steps of 10%**.
- **Ten distinct corrupted datasets generated per outlier percentage** (results averaged).
- Datasets are **outlier-free originally**; original loop closures = inlier ground truth.
- **2D datasets** (from openslam / [116][117]): **Intel, MIT, CSAIL, Freiburg Building (FRH/FR079),
  Freiburg University Hospital**. (Note: FR079 and FRH both appear; M3500 used in Ch.4 only.)
- **Visual SLAM (3D) datasets:** **KITTI_00, KITTI_05** (from KITTI visual-odometry seq. 00 & 05),
  and **TUM_FR1_DESK** (TUM RGB-D). Odometry + loop closures generated with **ORB-SLAM2**; ground
  truth trajectory via **SE-Sync** on the outlier-free version.

### Metrics (§5.4.1–5.4.2)
- **SR (Success Rate)** = N_suc / N_tot, where a run is successful iff ATE < `th_ATE` (=0.75). ↑
- **Precision (PR)** = TP/(TP+FP), **Recall (Rec)** = TP/(TP+FN), **F1** = 2·PR·Rec/(PR+Rec). ↑
- **ATE / RPE** (RMSE) ↓ — but noted as *uninformative* under heavy distortion (hence SR is primary).

### Table 5.1 — 2D datasets, SR / F1 / Precision vs outlier % (p. 86)

| Method | 20% SR | 20% F1 | 20% PR | 40% SR | 40% F1 | 40% PR | 60% SR | 60% F1 | 60% PR | 80% SR | 80% F1 | 80% PR | 100% SR | 100% F1 | 100% PR |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| HUBER  | 0.00 | 0.84 | 0.94 | 0.00 | 0.74 | 0.86 | 0.00 | 0.67 | 0.82 | 0.00 | 0.61 | 0.75 | 0.00 | 0.58 | 0.72 |
| DCS    | 0.24 | 0.90 | 0.83 | 0.20 | 0.83 | 0.71 | 0.22 | 0.77 | 0.62 | 0.20 | 0.71 | 0.55 | 0.22 | 0.67 | 0.50 |
| PCM    | 0.14 | 0.69 | 0.95 | 0.08 | 0.59 | 0.92 | 0.10 | 0.54 | 0.89 | 0.02 | 0.50 | 0.90 | 0.02 | 0.40 | 0.91 |
| GNC    | 0.00 | 0.90 | 0.89 | 0.00 | 0.80 | 0.76 | 0.00 | 0.71 | 0.65 | 0.00 | 0.66 | 0.59 | 0.00 | 0.63 | 0.56 |
| MAXMIX | 0.08 | 0.82 | 1.00 | 0.04 | 0.77 | 1.00 | 0.00 | 0.69 | 0.99 | 0.00 | 0.64 | **0.99** | 0.00 | 0.64 | **0.99** |
| IPC    | 0.50 | 0.94 | **1.00** | 0.50 | 0.92 | 0.99 | 0.32 | 0.90 | **0.99** | 0.30 | 0.90 | **0.99** | 0.32 | 0.92 | 0.98 |
| 2BL-IPC| 0.30 | 0.94 | 0.99 | 0.32 | 0.95 | 0.99 | 0.28 | 0.94 | 0.98 | 0.22 | 0.95 | 0.98 | 0.22 | 0.92 | 0.96 |
| 2BL-TACO| 0.32 | **0.97** | 0.99 | 0.32 | **0.97** | 0.99 | 0.30 | **0.97** | 0.98 | 0.22 | **0.96** | 0.98 | 0.22 | **0.94** | 0.97 |
| **TACO** | **0.52** | **0.97** | **1.00** | **0.44** | **0.97** | 0.99 | **0.36** | 0.93 | **0.99** | **0.36** | 0.94 | **0.99** | **0.38** | 0.93 | 0.98 |

### Table 5.2 — Visual SLAM datasets, SR / F1 / Precision vs outlier % (p. 86)

| Method | 20% SR | 20% F1 | 20% PR | 40% SR | 40% F1 | 40% PR | 60% SR | 60% F1 | 60% PR | 80% SR | 80% F1 | 80% PR | 100% SR | 100% F1 | 100% PR |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| HUBER  | 0.10 | 0.92 | 0.86 | 0.03 | 0.86 | 0.77 | 0.00 | 0.82 | 0.72 | 0.00 | 0.79 | 0.69 | 0.00 | 0.78 | 0.68 |
| DCS    | 0.57 | 0.97 | 0.95 | 0.23 | 0.94 | 0.92 | 0.13 | 0.90 | 0.90 | 0.07 | 0.86 | 0.88 | 0.07 | 0.83 | 0.86 |
| PCM    | 0.47 | 0.99 | 0.97 | 0.27 | 0.98 | 0.96 | 0.33 | 0.96 | 0.95 | 0.27 | 0.97 | 0.94 | 0.13 | 0.93 | 0.92 |
| GNC    | 0.13 | 0.92 | 0.86 | 0.00 | 0.86 | 0.76 | 0.00 | 0.80 | 0.67 | 0.00 | 0.76 | 0.61 | 0.00 | 0.71 | 0.56 |
| MAXMIX | 0.27 | 0.96 | 0.95 | 0.27 | 0.95 | 0.95 | 0.13 | 0.89 | 0.96 | 0.27 | 0.92 | 0.97 | 0.17 | 0.94 | 0.96 |
| IPC    | **0.90** | 0.98 | **0.99** | **0.93** | 0.99 | **0.99** | **0.93** | 0.99 | **0.99** | 0.87 | 0.98 | **0.99** | **0.93** | 0.98 | **0.99** |
| 2BL-IPC| 0.47 | 0.91 | 0.98 | 0.43 | 0.90 | 0.98 | 0.43 | 0.89 | 0.97 | 0.40 | 0.81 | 0.96 | 0.33 | 0.80 | 0.97 |
| 2BL-TACO| 0.50 | 0.97 | 0.98 | 0.40 | 0.94 | 0.98 | 0.43 | 0.96 | 0.98 | 0.40 | 0.95 | 0.98 | 0.30 | 0.93 | 0.97 |
| **TACO** | **0.90** | **0.99** | **0.99** | **0.92** | **0.99** | **0.99** | **0.93** | **0.99** | **0.99** | **0.89** | 0.98 | **0.99** | **0.90** | **0.99** | **0.99** |

### Table 5.3 — 2D datasets, ATE [m] / RPE [m] at 50% & 100% outliers (p. 86)

| Method | 50% ATE↓ | 50% RPE↓ | 100% ATE↓ | 100% RPE↓ |
|---|---|---|---|---|
| HUBER   | 52.19 | 24.18 | 53.55 | 27.67 |
| DCS     | 43.48 | 29.03 | 55.10 | 42.64 |
| PCM     | 43.50 | 27.32 | 56.15 | 31.32 |
| GNC     | 43.86 | 34.36 | 48.56 | 37.63 |
| MAXMIX  | 41.31 | 0.35  | 43.95 | 0.47  |
| IPC     | **12.49** | 0.03 | 22.85 | 0.03 |
| 2BL-IPC | 21.27 | 0.08 | 29.06 | 0.10 |
| 2BL-TACO| 15.49 | 0.11 | 26.70 | 0.11 |
| **TACO**| 16.97 | **0.06** | **21.47** | **0.04** |

### Table 5.4 — Visual SLAM datasets, ATE [m] / RPE [m] at 50% & 100% outliers (p. 87)

| Method | 50% ATE↓ | 50% RPE↓ | 100% ATE↓ | 100% RPE↓ |
|---|---|---|---|---|
| HUBER   | 119.10 | 2.56 | 121.39 | 2.68 |
| DCS     | 59.61  | 1.93 | 61.39  | 1.72 |
| PCM     | 80.68  | 1.94 | 83.45  | 2.42 |
| GNC     | 118.93 | 10.95 | 120.92 | 11.27 |
| MAXMIX  | 119.16 | **0.25** | 121.49 | **0.61** |
| IPC     | 17.28 | 2.11 | 9.05 | 2.04 |
| 2BL-IPC | 63.75 | 2.49 | 92.01 | 2.61 |
| 2BL-TACO| 50.75 | 2.46 | 70.00 | 2.46 |
| **TACO**| **11.66** | 2.07 | **8.99** | 2.05 |

### Table 5.5 — Runtime [s] per dataset (averaged over outlier %) (p. 89)

| Method | MIT | INTEL | CSAIL | FRH | FR079 | FR1_DESK | KITTI_00 | KITTI_05 |
|---|---|---|---|---|---|---|---|---|
| HUBER   | 0.17 | 0.64 | 1.22 | 7.48 | 1.10 | 0.46 | 11.34 | 5.27 |
| DCS     | 0.24 | 5.61 | 2.07 | 3.39 | 3.02 | 0.34 | 1.89 | 1.14 |
| PCM     | 9.19 | 667.56 | 22.51 | 381.64 | 33.82 | 8.98 | 296.72 | 105.51 |
| GNC     | 17.43 | 268.80 | 51.19 | 129.88 | 25.63 | 11.00 | 302.79 | 159.56 |
| MAXMIX  | 2.91 | 43.72 | 2.52 | 23.78 | 1.96 | 0.58 | 11.09 | 4.89 |
| IPC     | 0.67 | 20.14 | 6.25 | 255.84 | 8.57 | 1.12 | 33.59 | 9.17 |
| 2BL-IPC | 0.49 | 18.22 | 5.56 | 45.28 | 5.19 | 0.83 | 28.44 | 6.57 |
| 2BL-TACO| 1.27 | 19.59 | 6.68 | 64.08 | 7.25 | 1.12 | 33.39 | 9.97 |
| **TACO**| 1.47 | 20.39 | 8.20 | 253.32 | 10.51 | 1.33 | 52.92 | 12.99 |

### Headline narrative numbers (§5.4.4–5.4.6)
- **SR averaged over all datasets:** TACO valid solution **> 55%** of the time; 2BL-TACO **> 35%**;
  **all other (non-TACO) methods fail to exceed 20% SR** (closest is DCS).
- **kBL-IPC k-sweep:** 2BL-IPC is on avg **1.73× faster** than IPC, **+1.9% F1**, but **−10.4% SR**.
- **Adding SR-SC (TACO vs IPC):** TACO gains **+1.24% SR** and **+3.03% Recall** over IPC, ≈ equal
  Precision; SR-SC overhead only **+1.14%** time. For 2BL-TACO vs 2BL-IPC: same SR, **+4.1% F1**,
  **+5.96% Recall**, +0.47% Precision; overhead **+1.31%** time.
- Minimal-trusted-subgraph reduction cuts problem size to **≈ half** (Fig. 5.10).

---

## TACO — reproduction targets (what our S2 implementation must match)

Primary go/no-go numbers, in priority order:

1. **Per-edge / loop-level behaviour first:** kBL-IPC with `k=2`, `s=10` (2D), χ² with `α=0.95`
   should reproduce **Table 5.1 `2BL-IPC` row** (e.g. 2D 100%: F1≈0.92, PR≈0.96) before SR-SC is
   added.
2. **TACO (full) on 2D, headline:** Table 5.1 — at 100% outliers **SR≈0.38, F1≈0.93, PR≈0.98**;
   at 20% **SR≈0.52, F1≈0.97, PR≈1.00**. Average **SR > 0.55** across all datasets.
3. **TACO ATE/RPE sanity (2D):** Table 5.3 — at 100% outliers **ATE≈21.47 m, RPE≈0.04**;
   at 50% **ATE≈16.97, RPE≈0.06**. (RPE near-zero is the tell-tale of a correctly de-outliered graph;
   the non-robust baselines sit at RPE 24–42.)
4. **TACO Visual SLAM:** Table 5.2 — TACO ≈ IPC, F1≈0.99, PR≈0.99 across the board, SR≈0.90+;
   Table 5.4 — at 100% **ATE≈8.99, RPE≈2.05**.
5. **Effect-of-SR-SC deltas:** TACO over IPC ≈ **+1.24% SR, +3% Recall**, ≈neutral Precision,
   ~1% time overhead. 2BL-TACO over 2BL-IPC ≈ **+4% F1, +6% Recall**.
6. **Runtime ballpark (Table 5.5):** TACO total runtime comparable to IPC; on FRH, 2BL variants are
   far cheaper (45–64 s vs 253 s) thanks to the k-cap.

Note on metric reliability: ATE/RPE are explicitly called *unreliable* under distortion (Fig. 5.7
shows visually-worse trajectories scoring *better* ATE/RPE). **Use SR + Precision/Recall/F1 as the
verification metrics**, not raw ATE.

---

## IPC parameter values (for our code: `s_factor`, `k_buddies`, χ² thresholds)

### Scaling factor `S` → our `s_factor`
- **IPC (Ch. 4):** `s = 3` for **all** experiments. Quote: *"…exploiting the Dog-Leg optimization
  steps and setting **s = 3** for all experiments."* (§4.4, **p. 59**). `s > 1` is applied to the
  **odometry** information matrices in Eq. 4.6 to keep odometry trusted; it adds no extra variables.
- **TACO (Ch. 5):** the same odometry confidence `s` is set **`s = 10` (2D)** and **`s = 30`
  (3D / Visual SLAM)** — *"Dog-Leg optimization steps were utilized, with **s set to 10 and 30** for
  the 2D and 3D Visual SLAM experiments, respectively."* (§5.4, **p. 79**).
- **Takeaway for `s_factor`:** use **3** to match Chapter-4 IPC; use **10 (2D) / 30 (3D)** to match
  the Chapter-5 TACO experiments.

### "k best buddies" / number of voters `k` → our `use_best_k_buddies` / `k_buddies`
- This corresponds to **kBL-IPC's `k` = number of best (highest-IoU) accepted loop closures** kept
  in the test subgraph. **`k = 2`** is the headline value (`2BL-IPC` / `2BL-TACO`), with the sweep
  **k ∈ {2, 3, 5, 10}** and **k = ∞ (all intersected loops) ≡ plain IPC** (§5.3.2 p. 72–73;
  §5.4.4 p. 84; Fig. 5.8). Quote: *"…minimal independent subgraph `G^k` … with **k = 2** (highlighted
  … in Fig. 5.2)."* (p. 73) and Fig. 5.2 caption: *"…the subgraph chosen by kBL-IPC, where in this
  case **k = 2**."*
- The selection rule for the `k` buddies is **highest Intersection-over-Union (IoU)** of node spans
  (Eq. 5.6, Alg. 6) — *not* arbitrary nearest neighbours. **`k = 2` is the recommended default**
  (best speed/robustness trade-off in the thesis).

### χ² fast / slow rejection thresholds
- IPC/TACO use a **single χ² test** form, `e^T Ω e < χ²_{α,δ}` (IPC Eq. 4.4 p. 58; TACO Eq. 5.5
  p. 71), with:
  - **confidence `α`** and **DoF `δ`** of the error term (for 2D loop closures δ = 3; 3D δ = 6).
  - **`α_th = 0.95`** is the confidence used to define an inlier for methods that don't explicitly
    classify: *"an inlier is defined as a measurement having a χ² error less than χ²_th with
    **α_th = 0.95**."* (IPC §4.4.3 **p. 60**; restated TACO §5.4 **p. 79**).
- The thesis does **not** expose two separate "fast" vs "slow" χ² thresholds as named constants — it
  uses one χ² acceptance threshold (`χ²_{α,δ}` at α=0.95). In our `consensus.cpp`, the "fast/slow
  rejection thresholds" most likely correspond to (a) a per-edge candidate pre-screen vs (b) the
  full-subgraph all-edges test — both share the same `χ²_{α=0.95, δ}` value here. If our code wants
  distinct fast/slow gates, **0.95** is the only confidence value the thesis specifies; any
  asymmetry (e.g. a looser fast gate) is an implementation choice **not** sourced from the thesis.
  SR-SC additionally uses a separate **switch threshold `s_ab > 0.5`** per round and a voting
  promotion threshold **`inlierTh = 0.7`** (these are decision thresholds, not χ² thresholds).

---

## One-line provenance map (printed page → content)
- p.59 — IPC `s = 3`; g2o + Dog-Leg.
- p.58 — IPC Algorithm 5; χ² test Eq. 4.4.
- p.60 — IPC datasets (Intel/MIT/CSAIL/FRH/FR079/M3500), Vertigo outliers, `α_th = 0.95`.
- p.61 — IPC Table 4.1 (F1/RPE/Time, 50%/100%).
- p.63 — IPC Table 4.2 (ACTxC).
- p.72–73 — kBL-IPC IoU (Eq. 5.6), Alg. 6 findKBestLoops, k=2.
- p.74–75 — Switchable Constraints (Eq. 5.7–5.9).
- p.75–78 — SR-SC (Eq. 5.10–5.12, Alg. 7); trusted/unrevised sets.
- p.79 — TACO hyperparameters (s=10/30, R=10, maxIter=20, ε=0.3, inlierTh=0.7, α=0.95, th_ATE=0.75).
- p.82 — TACO datasets (2D + KITTI_00/05 + TUM_FR1_DESK), ORB-SLAM2 + SE-Sync.
- p.86–89 — Tables 5.1–5.5 (results).
