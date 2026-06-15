# Remote-agent brief — correlation-aware online robust PGO
*Self-contained handoff for the coding agent: understand the direction, then the step-by-step build mechanism. Full analysis: [`correlation_aware_solution.md`](correlation_aware_solution.md); gap/evidence: [`correlation_aware_problem.md`](correlation_aware_problem.md).*
*Approach: build the code base, run experiments, **let the experiments answer whether the direction works**. No learning. SE(2) first.*

---

## 1. The direction (one paragraph)
Take **IPC's online property** (per-arrival consensus on a **bounded sub-independent subgraph**) and add **DC-GM's correlation-awareness** (its two terms: outlier penalty + correlation reward) — made **online** by confining those terms to IPC's small subgraph and solving the resulting tiny **discrete-continuous** problem with **DC-SAM**. Net effect: IPC, but the accept/reject of correlated loop closures is decided **jointly** instead of per-edge.

**Why (the gap):** per-edge online methods (IPC, SC, DCS, GNC, riSAM, TACO) are **correlation-blind** — they judge each loop by its own residual, so a *group* of mutually-consistent wrong loops (perceptual aliasing) passes. Correlation-awareness exists only in **batch** methods (DC-GM, PCM), which can't run online. The **online + correlation-aware** combination is unoccupied — that's what this builds.

---

## 2. What you build on (dependencies — read this first)

| Piece | What it is | What you do with it | Get it from |
|---|---|---|---|
| **IPC** | the online baseline (bounded subgraph + per-edge χ²) | **extend it** — this is the code you modify | your fork: `code_base/thesis/robust_pgo/ipc/` |
| **DC-GM** | a **model on paper** (MATLAB/SDP, no usable code) | **reimplement its two terms** (the math only — penalty + reward) | paper: `pdfs/multi_edge/M-C_probabilistic_graphical_model/2019_Lajoie_DCGM_*.pdf` + study notes |
| **DC-SAM** | the **online discrete-continuous solver library** (GTSAM/iSAM2-based) | **clone + use** as the solver framework | `git clone https://github.com/MarineRoboticsGroup/dcsam` (not yet in the repo) |

> So: **IPC = extend, DC-GM = reimplement 2 terms (no code), DC-SAM = the solver you clone.** The only thing with code to fetch is DC-SAM.

---

## 3. The math (IPC notation)

**Symbols:** $\mathcal{X}=\{x_i\}$ poses · $e_{a,b},\Omega_{a,b}$ residual/info of loop $(a,b)$ · $\ell_{a,b}\in\{0,1\}$ inlier label (NEW) · $E_o^I,E_l^I$ odom/loop edges in the independent subgraph $G^I$ · $\mathcal{C}$ correlated loop pairs (input) · $s$ odom up-weight · $\kappa$ outlier penalty (= χ² threshold) · $\lambda$ correlation-reward strength ($\ge0$).

**IPC baseline** — purely continuous; accept/reject by per-edge χ² *after* the solve (no place for correlation):
$$\min_{\mathcal{X}}\ \sum_{E_o^I} e^{\mathsf T} s\Omega e\ +\ \sum_{E_l^I} e_{a,b}^{\mathsf T}\Omega_{a,b} e_{a,b}
\qquad\text{accept iff } e_{a,b}^{\mathsf T}\Omega_{a,b}e_{a,b}<\chi^2_{\alpha,\delta}$$

**Our objective** — 4 terms, discrete-continuous:
$$\min_{\mathcal{X},\,\boldsymbol{\ell}}\
\underbrace{\sum_{E_o^I} e^{\mathsf T} s\Omega e}_{\text{1. odometry}}
+\underbrace{\sum_{E_l^I} \ell_{a,b}\, e_{a,b}^{\mathsf T}\Omega_{a,b} e_{a,b}}_{\text{2. gated loop}}
+\underbrace{\sum_{E_l^I} (1-\ell_{a,b})\,\kappa}_{\text{3. outlier penalty (NEW)}}
+\underbrace{\sum_{((a,b),(c,d))\in\mathcal{C}} \lambda\,\mathbb{1}[\ell_{a,b}\neq\ell_{c,d}]}_{\text{4. correlation reward (NEW)}}$$

- **Term 3** = pay $\kappa$ to reject a loop (blocks "reject everything"; $\kappa$ is the threshold).
- **Term 4** = pay $\lambda$ when two **correlated** loops *disagree* → the only term coupling loops → the correlation-awareness.

**Reduces to IPC at $\lambda=0$:** with no reward, labels decouple and each picks $\ell_{a,b}=1$ iff $e^{\mathsf T}\Omega e<\kappa$ — exactly IPC's χ² test. So **IPC ≡ this method with $\lambda=0$**; correlation is the single added ingredient (the clean on/off comparison).

---

## 4. Step-by-step mechanism (modified IPC `agreementCheck`)
```
ModifiedAgreementCheck(candidate loop e_cand):
  1. G_I ← computeIndependentSubgraph(e_cand)        # SAME as IPC (bounded subgraph)
  2. G_I ← G_I ∪ correlatedLoops(e_cand, C)          # NEW: pull in Term-4 partners
  3. L   ← { loop edges in G_I }                     # these get discrete labels ℓ
  4. fixComplementary(poses ∉ G_I)                   # SAME as IPC (freeze outside)
  5. (X*, ℓ*) ← DC-SAM solve on G_I:                 # REPLACES (poses-solve + per-edge χ²)
        alternate until converged:
          (a) CONTINUOUS: optimize poses X | ℓ   → weighted LS, odom ×s, ℓ=1 active, ℓ=0 off
          (b) DISCRETE:   optimize ℓ | X         → min Σ ℓ·r + Σ(1-ℓ)·κ + Σ_C λ·1[ℓ≠ℓ]
  6. if ℓ*_cand == 1: accept e_cand → CnS; propagateCurrentGuess(...)   # SAME propagation
     else: restore()
```
**Same as IPC:** subgraph construction, $s$-weighting, `fixComplementary`, propagation, the continuous pose solve. **New:** the discrete labels $\ell$ on the subgraph + the joint discrete-continuous solve (steps 2, 5b). The subgraph is **small**, so the discrete solve is cheap (DC-SAM's alternation, or exact enumeration of $2^{|L|}$ if ever needed — submodular for $\lambda\ge0$).

---

## 5. How the model maps onto DC-SAM (≈1:1)
DC-SAM's `update()` takes three graphs; the 4-term model decomposes exactly along them:

| model element | DC-SAM slot | how |
|---|---|---|
| poses + odometry (term 1) | `gtsam::NonlinearFactorGraph` (continuous) | standard GTSAM between-factors |
| **gated loop** (term 2) | `DCFactorGraph` (hybrid) | a custom **`DCFactor`**: error = loop residual when $\ell$=inlier, ≈0/penalty when $\ell$=outlier |
| **outlier penalty** (term 3) | `gtsam::DiscreteFactorGraph` | `DiscretePriorFactor` (unary) per loop label |
| **correlation reward** (term 4) | `gtsam::DiscreteFactorGraph` | a **pairwise discrete factor** coupling labels of correlated loops |

`DCSAM::update()` runs the continuous/discrete alternation incrementally (iSAM2 → online). **No SDP, no solver written from scratch** — you supply the factors.

---

## 6. Data — where the correlation set $\mathcal{C}$ comes from
DC-GM used **ground-truth grouping** (it doesn't detect correlation); do the same:
- **Correlated-outlier generator** (fork IPC's `generateDataset.py`): inject outlier loop **groups** that are mutually consistent (share a common wrong transform), and write a **`.groups` sidecar** (edge → group = $\mathcal{C}$) + per-loop inlier/outlier ground truth. Keep a standard **random-outlier** set as the control.
- Real front-end grouping (deployment) = a later problem; defer.

---

## 7. What you actually code (the only bespoke pieces)
1. **Custom `DCFactor`** for the gated loop closure (error depends on poses + the discrete label).
2. **Discrete factors** for the outlier penalty (unary) + correlation reward (pairwise within a group).
3. **The modified `agreementCheck`** wiring (steps 2 + 5 above): include correlated neighbors, call the DC-SAM solve, decide from $\ell^*$.
4. **Correlated-outlier generator** (+ `.groups` sidecar + ground-truth labels).

Everything else (subgraph, $s$-weighting, propagation, the continuous LM/GN solve) is reused.

---

## 8. Then experiment — let the results answer the direction
**Question the experiment answers:** does the joint, correlation-aware decision recover more inliers / give a better trajectory than per-edge IPC **on correlated outliers**?

- **Key comparison:** this method vs **$\lambda=0$ (= IPC)** on correlated data (correlation on/off — same as DC-GM's "DC-GM vs DC-GMd" ablation). On *random* outliers, expect parity (that's the control).
- **Baselines:** IPC's Table I set (IPC, ADAPT, MAXMIX, DCS, GM, GNC, HUBER, PCM) + DC-GM/RRR; the `RobustOptimizationSLAM` sibling repo already implements most.
- **Metrics:** loop precision/recall (vs GT labels), ATE/RPE (vs GT trajectory), runtime.
- **Ablations:** $\lambda$ on/off; perfect vs noisy grouping; group size.

No predetermined verdict — build it, run it, read what the numbers say.

---

## 9. Constraints
- **No learning** (classical hybrid).
- **SE(2) first**; SE(3) deferred.
- **Keep IPC's continuous machinery intact** — only add the discrete labels + joint decision on the bounded subgraph.
- Keep the correlated subgraph **small** (DC-SAM §VI-A: alternation blows up on dense discrete coupling unless the subset is small — this is *why* IPC's locality matters; verify empirically).
- Build on the **exactly-replicated IPC** — don't extend a baseline that doesn't yet reproduce.

---

## 10. Pointers
- Full direction + DC-SAM feasibility/API detail: [`correlation_aware_solution.md`](correlation_aware_solution.md) (§3 math, §4 feasibility)
- Gap + primary-source evidence: [`correlation_aware_problem.md`](correlation_aware_problem.md)
- DC-GM model to port: [`../deep_analysis/multi_edge/M-C_probabilistic_graphical_model/2019_DCGM_Discrete_Continuous_Graphical_Model.md`](../deep_analysis/multi_edge/M-C_probabilistic_graphical_model/2019_DCGM_Discrete_Continuous_Graphical_Model.md)
- DC-SAM: github.com/MarineRoboticsGroup/dcsam · arXiv:2204.11936
- Code repo: `code_base/thesis/robust_pgo/` — `ipc/src/consensus.cpp` (the χ² decision to modify), `ipc/scripts/generateDataset.py` (generator to fork).
