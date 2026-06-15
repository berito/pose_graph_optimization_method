# Parameters: what the AUTHOR used vs what WE supply (verification-critical)

**Purpose:** for every parameter and experimental choice, state exactly what the author published,
what came from his released code, and what **we** had to choose ourselves — so divergence between our
results and his can be attributed to the right source. **This is the most important reference for the
S1/S2 verification.**

Verified against: thesis PDF (Ch.4 IPC, Ch.5 TACO), the released IPC repo (`../IPC_upstream`, code +
configs diffed), and our `ipc/` fork + `taco/` reimplementation.

## The three tiers (read this first)
- **🟢 Tier A — in the PAPER/THESIS.** Author stated it explicitly (quote + location given). Authoritative.
- **🟡 Tier B — in the author's RELEASED CODE/CONFIG, not in the paper.** We inherited it verbatim via
  the IPC fork. Author's value, just unpublished. **(IPC only — TACO released NO code, so it has no Tier B.)**
- **🔴 Tier C — NOT available from the author at all.** We chose it ourselves. **Every Tier-C item is a
  candidate source of divergence and must be flagged in any write-up.**

---

# PART 1 — IPC (Chapter 4). Code WAS released → most gaps are Tier B.

## 1.1 Algorithm / solver parameters

| Parameter | Tier | Author value | Source | Our value | Match? |
|---|---|---|---|---|---|
| Scaling factor **s** (odom info up-weight, Eq 4.6) | 🟢 A (value) | **3**, fixed, all experiments | §4.4 p.59: *"…setting s = 3 for all experiments."* | 3 | ✅ |
| └ s as a **yaml field** | 🟡→🔴 | released code READS `s_factor` from yaml, but released M3500 cfg **omits the line** | code `utils.cpp:325`; upstream cfg has no `s_factor` | we added `s_factor: 3` | ✅ (value=paper) |
| Optimizer | 🟢 A | **Powell's Dog-Leg** steps, on **g2o** | §4.4 p.59 | g2o Dog-Leg (`dl_var`) | ✅ |
| `fast_reject_th` (χ² gate, no-intersection path) | 🟡 B | **6.251** | upstream `cfg/2D/M3500_params.yaml` (byte-identical) | 6.251 | ✅ |
| `slow_reject_th` (χ² gate, intersection path) | 🟡 B | **11.345** (M3500) | upstream cfg (byte-identical) | 11.345 | ✅ |
| `fast_reject_iter_base` | 🟡 B | **50** | upstream cfg (byte-identical) | 50 | ✅ |
| `slow_reject_iter_base` | 🟡 B | **100** | upstream cfg (byte-identical) | 100 | ✅ |
| Subproblem iter ×5 boost when `\|eset\|>100` | 🟡 B | present | upstream `consensus_utils.cpp:13` (byte-identical) | identical | ✅ |
| Final global optimize iters | 🟡 B | **1000** | upstream `simulation.cpp` | 1000 | ✅ |
| `canonic_inliers` (M3500) | 🟡 B | **1954** | upstream cfg | 1954 | ✅ |
| `use_best_k_buddies` / `k_buddies` / `use_recovery` | 🔴 C (forced) | **not in IPC paper at all** (TACO leftovers; acceptance needs *all* edges in G′) | — | false / 0 / false (= IPC mode) | n/a — correctly OFF |

> **Important nuance on the χ² gates:** the *paper* only states the **form** `eᵀΩe < χ²_{α,δ}` and the
> **evaluation** confidence `α_th = 0.95` (§4.4.3) — used to *label* TP/FP after the run. The numeric
> **decision** gates 6.251 / 11.345 are **not in the paper**; they are the author's **released-code**
> values (6.251 ≈ χ²₀.₉₀,₃, 11.345 ≈ χ²₀.₉₉,₃ — neither equals the α=0.95 evaluation value 7.815). So
> "the threshold" is two different things in two roles. We use the author's code values for the decision
> and α=0.95 for labelling — both faithful.

## 1.2 Experiment protocol (all 🟢 A — stated)
- Datasets: INTEL, MIT, CSAIL, FRH, FR079 (real) + M3500 (synthetic); originally outlier-free (§4.4.1).
- Outliers: **Vertigo** package, as **% of inliers**, **10→100 step 10**, 100% ⇒ #out=#in, **10 trajectories/rate** (§4.4.3).
- GT trajectory: **SE-Sync** on the outlier-free graph (§4.4.2).
- Metrics: Precision, Recall, F1, ATE, RPE; inlier-label confidence **α_th = 0.95** (§4.4.3).
- Hardware: single **Intel Xeon Gold 5220** (§4.4).

## 1.3 IPC — what is genuinely OURS / unconfirmed (🔴 C — divergence candidates)
1. **🔴 SPOILED DATASETS (top suspect).** The author's released cfg points at his *own pre-generated*
   files (`/home/slam-emix/Datasets/BACK_END/2D/M3500/DATA/100/02.g2o`). We **regenerated** spoiled
   data with our generator. Same protocol ≠ same random outliers → recall can differ. We do **not** have
   his exact spoiled files.
2. **🔴 Source M3500 `graph.g2o` + `GT.txt` provenance** — ours vs his exact files unverified.
3. **🔴 Baseline "best parameters"** — paper says "best set of parameters" for each comparator but does
   **not list them** (DCS Φ, GNC, Huber δ, MaxMix, PCM threshold). Our `baselines/.../cfg/` values are ours.
4. **🔴 GNC/PCM "fixed number of iterations"** — paper under-ran them but the iteration count is **not given**.

---

# PART 2 — TACO (Chapter 5). NO code released → everything beyond the text is Tier C.

## 2.1 Stated in the thesis (🟢 A) — §5.4 p.79 unless noted
| Parameter | Author value | Source |
|---|---|---|
| Scaling factor **s** | **10** (2D), **30** (3D) | *"Dog-Leg … with s set to 10 and 30 for the 2D and 3D … experiments."* |
| **k** (best-loops cap) | **2** headline; sweep {2,3,5,10}; k=∞≡IPC | §5.3.2, §5.4.4, Fig 5.8 |
| **R** (SR-SC trigger / unrevised batch) | **10** | §5.4: *"SR-SC … every 10 loop closure detections … max size R = 10."* |
| **maxIter** (SR-SC voting rounds) | **20** | §5.4 / Alg 7 |
| **ε** (random prior-flip fraction) | **0.3** | §5.4 |
| **inlierTh** (vote promotion) | **0.7** | §5.4 / Alg 7 |
| per-round switch cutoff | **s_ab > 0.5** | Alg 7 line 10 |
| χ² eval confidence **α_th** | **0.95** | §5.4 |
| Success-Rate ATE threshold **th_ATE** | **0.75** | §5.4 |
| Ψ(s) mapping | linear `Ψ(s)=s`, clamped 0≤s≤1 | §5.3.3 |
| switch prior γ | 1 = inlier / 0 = outlier (from kBL-IPC labels) | Eq 5.8, Alg 7 |
| Optimizer / framework / HW | g2o, **Dog-Leg**, Xeon Gold 5220 | §5.4 |
| Datasets (2D) | Intel, MIT, CSAIL, FRH, FR079 | §5.4.3 |

## 2.2 TACO — NOT stated anywhere → WE supply (🔴 C, all divergence candidates)
| Item | Thesis? | Our choice | Note / risk |
|---|---|---|---|
| **kBL-IPC χ² decision threshold (numeric)** | ❌ only α=0.95 given | mirror IPC 6.251 / 11.345 | TACO uses s=10 (not 3); the matching thresholds are **not published** — assumption. |
| **kBL-IPC per-subproblem iteration counts** | ❌ | reuse IPC 50 / 100 (+×5 boost) | same unknown as IPC §1.1; convergence-sensitive. |
| **Switch-prior weight Ω_p** (Eq 5.8/5.9/5.12) | ❌ | **1.0** (SC-paper default) | author never gives the prior weight. |
| **SC subproblem iterations per voting round** | ❌ | **50** (our `srsc.cpp`) | not specified. |
| **Fraction of switches randomly *fixed*** each round | ❌ ("detail not reported in Alg 7") | currently none/implicit | author explicitly omits it. |
| **Outer-loop convergence tolerance** ("until convergence") | ❌ | run to maxIter | criterion unspecified. |
| **randomFlip mechanism** | ❌ (just "flip a fraction ε") | per-switch Bernoulli(ε) | could also be fixed count ⌈ε·\|batch\|⌉. |
| **Minimal trusted subgraph** | 🟢 method = Dijkstra | we use transitive **node-span superset** | correct but larger → speed only, not correctness. |
| **SR-SC inner solver** | 🟢 Dog-Leg (main solve) | we use **Levenberg-Marquardt** (BlockSolverX) | deviation; LM chosen for the mixed pose+switch block sizes. |
| **RNG seed** | ❌ (randomness inherent) | fixed per-span seed | our choice for reproducibility. |
| spoiled data / baseline params / fixed-iter counts | ❌ | same as IPC §1.3 | same Tier-C set carries over. |

---

# PART 3 — Divergence attribution (updated by this verification)

**S1 (IPC) verdict: did NOT reproduce the paper.** Given the checks above, the suspect ranking is now:

1. **🔴🔴 Spoiled-data mismatch (raised to #1).** Thresholds, iteration budgets, the ×5 boost, the
   solver, and the final-optimize count are now **confirmed byte-identical to the author's release** —
   so they are *no longer* prime suspects. The largest remaining uncontrolled variable is that we
   **regenerated** the Vertigo outliers instead of using his exact spoiled files. → action: obtain/inspect
   his released spoiled data, or verify our generator reproduces his TP/FP distribution.
2. **🔴 Source-graph / GT provenance** — confirm `M3500/graph.g2o` + `GT.txt` byte-match his inputs.
3. **🟡 iter_base sufficiency on *our* data** — 50/100 are his values, but whether they converge on our
   regenerated graphs is still worth a sensitivity check (the high-precision/low-recall symptom fits
   under-convergence). Demoted from "wrong value" to "possibly-insufficient-for-our-data."
4. **🔴 Baseline best-params / under-run counts** — unpublished; affects baseline comparison, not IPC itself.

**S2 (TACO):** since TACO has **no Tier B at all**, *every* numeric choice in Part 2.2 is a potential
divergence source. Verify the **2BL-IPC** row of Table 5.1 first (it exercises only kBL-IPC = the
χ²/iteration assumptions) before enabling SR-SC (which adds the prior-weight / solver / voting Tier-C set).

---

## One-line summary
- **IPC:** we run the author's **exact released parameters** (s=3, 6.251/11.345, 50/100, Dog-Leg, 1000) —
  confirmed via upstream diff. The only things that are *ours* are the **regenerated spoiled data**,
  the **baseline params**, and the inert k/recovery flags (correctly off). Divergence ⇒ look at the data, not the params.
- **TACO:** we match every **published** value (s=10/30, k=2, R=10, maxIter=20, ε=0.3, inlierTh=0.7,
  α=0.95, th_ATE=0.75) but must **invent** the χ² threshold, iteration counts, prior weight, inner solver,
  flip/seed details — all flagged 🔴 above.
