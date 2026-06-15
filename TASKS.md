# TASKS — correlation-aware robust PGO (experiment-first)

Detailed, command-level plan. High-level phases + gates also live in the literature plan
(`<studies>/thesis/robust_pgo/literature/TASKS.md` — literature/theory tasks only; the experiment tasks live **here**).
**This repo is a superbuild** (see `docs/ATTRIBUTION.md` + `CLAUDE.md`): `ipc/` = our IPC fork (the contribution we patch);
`baselines/` = vendored RobustOptimizationSLAM (the comparators); root `CMakeLists.txt` builds them together.

**Falsification ladder (kill fast):** G1 = baselines fail on correlated outliers? · G2 = oracle-correlation wins? · G3 = realistic signal holds?

## ⏰ Near-term deadlines (advisor)
- **Thu Jun 11 / Fri Jun 12 2026 (online)** — present the **solution proposal: the math**. The correlation-aware / group-joint formulation (DC-GM's two terms on IPC's subgraph) — theory, not code. Source: the solution doc in the literature monorepo.
- **Sat Jun 13 2026** — present **experiment results**. Deliverable = the **S1 IPC-reproduction** numbers (build green + IPC reproduces the paper on the authors' setup), compared against `docs/IPC.md`. ⇒ this week prioritize **S1** so there are results to show.

## Milestones
One spine: each **stage S0–S6** ends in a **milestone** (its deliverable = "done when…").
Two kinds of pass/fail test gate the stages:
- **Verify** (S1, S2) — reproduce a *published* method's results before trusting it. Establishes the trusted foundation.
- **Gate G1–G3** (S3–S5) — the falsification ladder testing *our* hypothesis.

(Milestone = the finish line of a stage; not a separate numbering. Build the trusted base — IPC then TACO — *before* our part.)

| Stage | Milestone (done when…) | Gate test | Original target |
|---|---|---|---|
| **S0** | Superbuild compiles; `ipc_tester_2D` runs | — (setup) | Day 1 |
| **S1** | IPC **reproduces its paper's results** on the authors' setup (datasets, spoiling, metrics) | **Verify**: matches IPC paper within tolerance? No → fix build/setup | Day 1–2 |
| **S2** | TACO **implemented** (authors didn't release code) + **reproduces the TACO paper** | **Verify**: matches TACO paper? No → fix impl | Day 3–4 |
| **S3** | Correlated gap proven — methods hold on random, **break** on correlated | **GATE 1**: break on correlated? No → STOP | Day 5 |
| **S4** ⭐ | Oracle group-joint **beats** baseline IPC on correlated | **GATE 2**: beats baseline? No → idea dead, pivot | Day 6–7 |
| **S5** | Win **survives** a realistic (non-oracle) signal | **GATE 3**: realistic signal still wins? | Week 2 |
| **S6** | Final results doc complete | — | Jul 7 |

## Remaining tasks — by activity type
Status (2026-06-12): **S0** ✅ build · **S1** ✅ ran — *IPC does NOT reproduce the paper* (root cause open) ·
**baselines** ✅ ran + compared (`experiments/figures/`). The contribution (**S2–S6**) is **not started**.

### 🔨 Implementation (write new code)
- [ ] **TACO** module from the paper → new `taco/` wired into the superbuild. [S2]
- [ ] **Correlated-outlier generator** (+ `.groups` sidecar) → `experiments/scripts/generateCorrelatedDataset.py`. [S3]
- [ ] **Oracle group-joint decision** patch in `ipc/src/consensus.cpp`. ⭐ core [S4]
- [ ] **Realistic grouping + DC-SAM** integration. [S5]

### ▶️ Running (execute experiments)
- [ ] Run **TACO** on the S1 setup. [S2]
- [ ] Run **IPC + baselines on correlated vs random** data. [S3]
- [ ] Run **oracle patch vs IPC** on correlated. [S4]
- [ ] Run **realistic-signal** experiment. [S5]
- [ ] **Re-run IPC** once correct iteration values are confirmed.

### 🔍 Testing & Confirmation (check against the source/paper)
- [ ] **Confirm the real iteration values** (IPC `fast/slow_reject_iter_base`; baseline `max_iters`) from author code/papers.
- [ ] **Read the original baseline papers** (PCM, GNC, DCS, MaxMix, GM, ADAPT) → confirm their intended setup/params.
- [ ] **Confirm dataset provenance** (CSAIL/INTEL/MIT vs the author's versions; FR079 already flagged).
- [ ] **Verify TACO** reproduces the TACO paper. [S2 gate]
- [ ] **Verify ADAPT** (our reimplementation) against its paper.

### 🐞 Debugging (find/fix)
- [ ] **Why IPC didn't reproduce** — the convergence / `iter_base` lead (high-precision/low-recall symptom).
- [ ] **Fair treatment** — same iteration/time budget for baselines as IPC (the paper under-ran PCM/GNC).
- [ ] (FR079 gtsam crashes — known; leave or handle.)

### 📝 Admin / Documentation
- [ ] **Commit** the current work to `main`.
- [ ] **Final results doc** (S6, by Jul 7).

## Paths
- **Repo root (superbuild):** `/home/lab_desktop/Documents/code_base/thesis/robust_pgo`
- **Our IPC fork (S4 patch target):** `ipc/src/consensus.cpp` (174 lines — χ² accept/reject) · tester `ipc/examples/ipc_tester_2D.cpp`
- **TACO (S2 implement here):** new `taco/` module (authors released no code)
- **Vendored baselines:** `baselines/` — SC, MaxMix, DCS, GNC, Huber, RRR in `robust_g2o/`; **PCM** + GTSAM tier in `robust_gtsam/`; metrics in `evaluator/`. **PCM = headline comparator for G1/G2.**
- **M3500 (graph + GT):** `experiments/datasets/2D/M3500/{graph.g2o, GT.txt}` ← start here
- **Generator to fork (S3):** `ipc/scripts/generateDataset.py` (Vertigo) → new `experiments/scripts/generateCorrelatedDataset.py` (keep `ipc/` pristine)
- **Configs:** `ipc/cfg/2D/*.yaml` · baseline cfgs under `baselines/*/cfg/`
- **IPC experiment driver (S1):** `ipc/bash/ipc_experiments_2D.sh`
- **Upstream refs (untouched):** `../../learning/papers_code/IPC` · github.com/EmilioOlivastri/{IPC, RobustOptimizationSLAM}

> **Honest notes:** (1) build via `.devcontainer/` — compiles g2o@`20201223_git` automatically. (2) DC-GM is **model-only** (Matlab/SDP) — reimplement its two terms, not its code. (3) DC-SAM not needed until S5 — S4 uses plain enumeration. (4) TACO (S2) is our own implementation — verify against the TACO paper before trusting as comparator.

---

## Reference — Stage detail (S0–S6)

### S0 — Build · Day 1 (DONE ✅)
```bash
cd /home/lab_desktop/Documents/code_base/thesis/robust_pgo
# VS Code: "Reopen in Container" (provisions g2o + GTSAM/Kimera-RPGO). Then:
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)
#   add -DBUILD_GTSAM_BASELINES=ON to also build PCM tier
./build/ipc/ipc_tester_2D -c ipc/cfg/2D/INTEL_params.yaml      # smoke test
```

### S1 — Reproduce IPC · Day 1–2 → Verify (DONE ✅ with partial match)
Mirror authors' setup (datasets · Vertigo random spoiling · paper's outlier rates · `canonic_inliers`) · compute precision/recall + ATE → `experiments/results/S1_ipc_repro.csv` · verdict in `RUN_LOG.md`. **Result: precision reproduces, recall/F1/ATE/RPE diverge.**

### S2 — Implement + verify TACO · Week 2 → Verify (NEXT 🎯)
**2.1** Implement TACO from paper as new `taco/` module (kBL + SR-SC variants per D-DIR-5) · reads same g2o/GT · emits same metrics.
**2.2** Run on S1 setup → `experiments/results/S2_taco_repro.csv`.
**2.3** Verify against TACO paper · fix impl until matches · record verdict in `RUN_LOG.md`.

### S3 — Correlated generator + Gate 1 · Week 2 → GATE 1 (NEXT 🎯)
**3.1** New `experiments/scripts/generateCorrelatedDataset.py` — clusters of non-adjacent poses sharing a common wrong transform + `.groups` sidecar (edge-id → cluster-id) + random control set.
**3.2** Methods × regimes — IPC + TACO + PCM + SC + DCS + GNC + MaxMix + RRR + Huber on random vs correlated → `experiments/results/G1_methods.csv`.
**3.3** ⛔ GATE 1 — hold on random, break on correlated? PASS → S4. FAIL → STOP.

## S4 — Oracle correlation-aware patch · Days 6–7 → **GATE 2** ⭐
**4.1 — Find the decision.** Read `ipc/src/consensus.cpp` → the χ² accept/reject (`fast_reject_th`).

**4.2 — Patch group-joint decision (oracle 𝒞).** On a branch (`git checkout -b ca-oracle`): when an edge belongs to a known group (from `.groups` = the **oracle**), decide the **whole group jointly** — enumerate `2^|group|` accept/reject labelings on that small group, score by joint consistency vs the trusted backbone, pick min cost. (DC-GM's correlation reward, on IPC's subgraph, solved by enumeration — no SDP.) **Done when:** patched tester runs on a correlated dataset using the labels.

**4.3 — ⛔ GATE 2 (make-or-break).** Patched (oracle) vs baseline IPC (+ TACO) on correlated M3500 → `experiments/results/G2_oracle.csv`. Beats baseline → idea has legs, S5. No improvement *even with the oracle* → idea dead → pivot to adaptive-time-bound fallback or the `ipc_revision_limitation_problem` direction. Record verdict in DECISIONS.

### S5 — Realistic signal + DC-SAM · Week 3+ → GATE 3
**5.1** Replace oracle `.groups` with derived grouping (geometric clustering / shared-source heuristic).
**5.2** Clone DC-SAM (https://github.com/MarineRoboticsGroup/dcsam); swap enumeration → DC-SAM online engine; verify §VI-A small-subset condition.
**5.3** Full panel (INTEL · M3500 · CSAIL · FR079 · MIT) × rates × all methods incl. TACO. ⛔ GATE 3.

### S6 — Formalize + final · Week 4 (contract close)
**6.1** Provable formulation (advisor A2): local group-joint solve preserves IPC's online property + adds correlation handling · resolve Q6.
**6.2** Iterate winner / run fallback.
**6.3** Final results doc by Jul 7.

---

**Daily log:** append one line per session to `RUN_LOG.md` (date · step · key number · next).
