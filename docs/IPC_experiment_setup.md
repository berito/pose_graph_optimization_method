# IPC — experiment setup (the author's protocol, for S1 replication)

**Why this file:** the exact experimental conditions the author used for IPC, in one place, so our S1
run replicates it faithfully. **Sources: IPC paper (ICRA 2024) + thesis Ch.4.** PDF:
`docs/2024_Olivastri_IPC_Incremental_Probabilistic_Consensus.pdf`.
Unlike TACO, **IPC code WAS released** (`github.com/EmilioOlivastri/IPC` → our `ipc/` fork), so the
setup is paper-stated **and** code-checkable. Result targets (Table I/II) live in [`IPC.md`](IPC.md);
this file is the **how-to-run**. (Mirrors [`TACO_experiment_setup.md`](TACO_experiment_setup.md).)

> **Status: S1 already run — IPC did NOT reproduce the paper** (root cause open; see §8). Keep this
> doc as the protocol-of-record while we chase the divergence.

---

## 1. Datasets (Sec. IV-A)

Standard 2D PGO benchmarks, all originally **outlier-free** (their existing loop closures = inlier
ground truth):
- **Real:** INTEL, MIT, CSAIL, **FRH** (Freiburg Building), **FR079** (Freiburg University Hospital)
- **Synthetic:** **M3500** (our start dataset, `experiments/datasets/2D/M3500/`)

(IPC uses M3500; TACO Ch.5 drops it. FR079 + FRH both appear.) Fig. 1 trajectory comparison is FR079
at 70% outliers.

---

## 2. Outlier-injection protocol (Sec. IV-C) — identical to TACO's

- Outliers added with the **Vertigo** package [26] (grossly-wrong random loop closures).
- Count = a **percentage of the number of true inliers**; **100% ⇒ #outliers = #inliers**.
- Sweep **10% → 100%, in steps of 10%**.
- **Ten distinct corrupted trajectories** per outlier percentage → results averaged.

> Same generator/sweep/10-per-rate as TACO. Our `SPOILED_DATA/{rate}/{00..09}.g2o` already matches.

---

## 3. Compared methods (Sec. IV) — the 7 baselines

GNC [8] (gtsam), ADAPT [5] (in g2o), DCS [6] (gtsam), GM [7] / Geman-McClure (gtsam), HUBER [7]
(gtsam), MAXMIX [10] (OpenSLAM), PCM [9] (Kimera [23]).
- For each, the **best fixed parameter set** was used.
- **GNC and PCM** results taken after a **fixed number of iterations** (long convergence times) — i.e.
  the author **under-ran** them; this is the likely reason their reported ATE/RPE look poor vs ours.

---

## 4. Metrics (Sec. IV-B)

- **Precision / Recall / F1** on loop accept/reject (TP/FP/FN over loop closures).
- **ATE** (Absolute Trajectory Error) and **RPE** (Relative Pose Error) [28] vs the outlier-free GT;
  ground truth via **SE-Sync** [27] on the clean dataset.
- **ACTxC** = Average Convergence Time per Constraint (seconds).
- **Inlier definition for evaluation:** χ² error `< χ²_th` with **α_th = 0.95** (use the same to label
  TP/FP/TN/FN).
- Precision/Recall/F1 (Fig. 3) and ATE/RPE/time (Fig. 4) are **averaged over all datasets**.

---

## 5. Hyperparameters — set these in `ipc/cfg/2D/*.yaml`

| Parameter | Our field | Value | Source |
|---|---|---|---|
| Scaling factor **S** (odom up-weight) | `s_factor` | **3** (fixed, **NOT swept** — IPC's *only* control param) | Sec. IV: "s = 3 for all experiments" |
| χ² acceptance test | — | `eᵀ Ω e < χ²_{α,δ}` (Eq. 4) | Sec. III-C — **form only, no number printed** |
| Inlier-definition confidence | — | **α_th = 0.95** | Sec. IV-C |
| Optimizer | — | **Powell's Dog-Leg** on **g2o** | Sec. IV |
| χ² reject thresholds | `fast_reject_th` / `slow_reject_th` | **6.251 / 11.345** (M3500) | **from code, NOT the paper** (≈ χ²₀.₉₀,₃ / χ²₀.₉₉,₃) |
| per-subproblem iters | `fast/slow_reject_iter_base` | **50 / 100** | **from code, NOT the paper** (assumption — see §8) |
| k / "best buddies" | `use_best_k_buddies`, `k_buddies` | **false / 0** | **no paper counterpart** — acceptance needs *all* edges in G′ to agree (this is the inert TACO leftover; IPC ⇒ off) |

Contrast TACO: TACO raises `s` to 10 (2D) / 30 (3D) and turns the k / recovery knobs on.

---

## 6. Solver & hardware

- **Framework:** C++, **g2o**, **Powell's Dog-Leg** steps (Sec. IV).
- IPC stresses `s` is its *only* control parameter and "fairly simple to configure" (Sec. V).

---

## 7. Aggregation for our figures

The paper prints only **all-dataset averages** (Fig. 3/4 line plots; Table I gives the 50%/100%
slices, Table II gives IPC's per-dataset ACTxC). Our two-level averaging (mean over the 10 corrupted
datasets per rate, then mean over datasets) is **our plotting method** to reconstruct those curves —
the author's figures are single-level all-dataset averages.

---

## 8. Known gaps / our deviations (the S1 divergence file)

**S1 verdict: IPC did NOT reproduce the paper** (our averaged recall/accuracy fell short). Verified
NOT the cause: code byte-identical to upstream, Dog-Leg solver, faithful Vertigo generator + metric,
validated `canonic_inliers`, χ² threshold ruled out (6.251/11.345 and 10.64 both ~0.59 recall).

Open leads / assumptions baked into our setup:
- **Per-subproblem iteration budgets unpublished.** `fast/slow_reject_iter_base = 50/100` are
  **from the code, assumed** — the paper never states them. Under-converged subproblems → inliers
  wrongly rejected → high-precision/low-recall, which is exactly our symptom. **Top suspect.**
- **χ² thresholds are code-sourced**, not paper-stated (paper gives only α=0.95 + Eq. 4 form).
- **FR079 low standalone recall (~0.33 on clean) is expected, not a bug** — its `EDGE_SE2`
  information matrices use an unusual (TORO-like) layout, but the file is **byte-identical to the
  author's release → do NOT convert it** (HANDOFF.md fact #6). If averaged recall is short, FR079 is
  the first thing to check — but the dataset stays untouched.
- **Baseline fairness:** the paper under-ran GNC/PCM (fixed-iteration cap), so their reported
  ATE/RPE are pessimistic vs a full solve — relevant when comparing our baseline numbers to theirs.

---

## 9. Reproduction targets (go/no-go) → see [`IPC.md`](IPC.md) Table I/II
Paper numbers are averaged over **6 datasets × 10 trajectories**, so single-dataset M3500 runs won't
match exactly — these are shape/bound targets:
- `s = 3` (the runtime "S = …" print), fixed.
- **IPC F1** ≈ **0.91 @50%**, **0.89 @100%** — should be the **top-F1** method.
- **IPC RPE (lowest)** ≈ **0.05 @50%**, **0.068 @100%**.
- **Recall consistently > 80%** across 10–100% (Fig. 3); precision near-top (with MAXMIX).
- **ACTxC on M3500** ≈ **0.54–0.72 s/constraint** (Table II); other datasets ~0.04–0.11.
