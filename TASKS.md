# TASKS — correlation-aware robust PGO (experiment-first)

> **Companion literature tasks:** [`<studies>/thesis/robust_pgo/literature/TASKS.md`](../../../studies/thesis/robust_pgo/literature/TASKS.md) (L1–L5).
> **Repo = superbuild:** `ipc/` (our fork, the contribution we patch) + `baselines/` (vendored comparators) + root `CMakeLists.txt`.
> **Falsification ladder (kill fast):** G1 (baselines break on correlated?) · G2 (oracle wins?) · G3 (realistic signal holds?).

---

## ⏰ Advisor — Sat 2026-06-13 meeting (done)

Presented Week 1: IPC + baseline replication results (precision reproduces; recall / F1 / ATE / RPE do **not**) + the robust-PGO math + TACO understanding.

**Decisions locked:**
- **D-EVAL-3 — Replicate IPC exactly.** Follow the paper's exact steps + techniques; **ask the author directly** for the experimental procedure (iteration budget, dataset provenance, per-method setup).
- **D-DIR-5 — Next deliverable** = experiment **the solution direction** (correlation-aware group-joint) **AND** **TACO options** (kBL + SR-SC), side by side, on the replicated pipeline. Maps to **S2** (TACO) + **S3/S4** (solution direction).

> **Deliverable vs parallel.** Advisor expects the *experiments* in D-DIR-5 as the deliverable. Personal parallel work (`literature/DISCUSSION_*` notes) is sanctioned for confidence-building but is NOT the deliverable — keep separated.

---

## ✅ WEEK 1 — COMPLETED (Mon Jun 9 → Sat Jun 13)

| Stage | What landed | Evidence |
|---|---|---|
| **S0** ✅ Build | Superbuild compiles · `ipc_tester_2D` runs · `.devcontainer` provisions deps · `docs/ATTRIBUTION.md` + `CLAUDE.md` written | commits `f88d5e5` + `c9e189e` (Day 1) |
| **S1** ✅ Ran | IPC ran on all 6 2D datasets via server. **Honest finding: did NOT fully reproduce paper** — precision matches; recall / F1 / ATE / RPE diverge (root cause = `iter_base` / convergence; high-precision / low-recall symptom) | commit `b796437` + figures `comparison_ourIPC_vs_paper_*.png` |
| **Baselines** ✅ Ran + compared | PCM · GNC · DCS · MaxMix · RRR · Huber on same setup · figures + analysis committed | `experiments/figures/` + `experiments/results/` |
| **Advisor deck** ✅ Presented | Beamer/metropolis progress-report Sat Jun 13 (Theory thread + Empirical thread) | `<studies>/.../literature/advisor/progress_report_2026-06-13.{tex,pdf}` |

**Week 1 verdict:** Foundation phase delivered. Replication infra ✅ closed. The honest "didn't fully reproduce" finding becomes Week 2's first investigation (D-EVAL-3).

---

## 🎯 WEEK 2 — ACTIVE (Mon Jun 15 → Sun Jun 21)

**Headline goal:** S2 closed (TACO impl + verify) + G1 gate verdict (S3) + parallel exact-replication investigation per D-EVAL-3.

### Must-ship (Week 2 deliverable to advisor)

- [ ] **D-EVAL-3 — Contact IPC author directly** for exact experimental procedure (iteration budget · dataset provenance · per-method setup) ⭐
- [ ] **S2-impl — Implement TACO from paper** → new `taco/` module in superbuild (kBL + SR-SC variants per D-DIR-5)
- [ ] **S2-verify — Run TACO on S1 setup** → `experiments/results/S2_taco_repro.csv` · matches TACO paper within tolerance?
- [ ] **S3-impl — Correlated-outlier generator** → `experiments/scripts/generateCorrelatedDataset.py` (clusters of non-adjacent poses sharing a wrong transform + `.groups` sidecar)
- [ ] **S3-run — IPC + TACO + baselines on correlated vs random** → `experiments/results/G1_methods.csv`
- [ ] **⛔ G1 GATE** — hold on random, break on correlated? PASS → S4. FAIL → STOP + record verdict in `<lit>/paper_studies/DECISIONS.md`

### Stretch (if G1 passes early)

- [ ] **S4-prep — Read `ipc/src/consensus.cpp`** (174-line χ² accept/reject) — locate the decision point for the patch

### Background investigation (parallel to S2)

- [ ] **Why IPC didn't fully reproduce** — confirm `fast/slow_reject_iter_base` values · fair iteration/time budget for baselines
- [ ] FR079 gtsam crashes — known; leave for now

### Coordination with literature side (`<studies>/.../literature/TASKS.md`)

- **L1** — Problem statement final (advisor A1) — quick parallel write (~Day 6) · informs S2 framing
- **L3** — JIT read of TACO §5.4.3 + DC-GM 2 terms · done together with S2-impl
- **L5** — Personal deepening (DISCUSSION_*) sanctioned as parallel · NOT this week's deliverable

**Week 2 deliverable target (Sun Jun 21 EOD):** S2 verified + G1 verdict logged + L1 problem-statement final + author-contact outcome captured.

---

## 🔮 WEEK 3+ — Queued (do not start yet)

- **S4** ⭐ Oracle group-joint patch in `consensus.cpp` (the make-or-break) → **G2 gate**
- **S5** Realistic grouping + DC-SAM integration → **G3 gate**
- **S6** Final results doc by Jul 7
- **L2** (literature) — Provable formulation (after S2/S3 empirical signal · resolves advisor Q6)
- Additional verification: baseline-paper reads (PCM · GNC · DCS · MaxMix · GM) · dataset provenance check (CSAIL/INTEL/MIT vs author versions) · ADAPT against its paper

---

## Reference — Milestones spine

One spine: each **stage S0–S6** ends in a **milestone** (its deliverable = "done when…"). Two kinds of pass/fail test gate the stages:
- **Verify** (S1, S2) — reproduce a published method before trusting it
- **Gate G1–G3** (S3–S5) — falsification ladder testing *our* hypothesis

| Stage | Milestone (done when…) | Gate test | Original target |
|---|---|---|---|
| **S0** ✅ | Superbuild compiles; `ipc_tester_2D` runs | — | Day 1 |
| **S1** ✅ | IPC reproduces paper within tolerance | **Verify** | Day 1–2 |
| **S2** 🎯 | TACO impl reproduces TACO paper | **Verify** | Week 2 |
| **S3** 🎯 | Methods hold on random, break on correlated | **GATE 1** | Week 2 |
| **S4** ⭐ | Oracle group-joint beats baseline IPC on correlated | **GATE 2** | Week 3 |
| **S5** | Win survives realistic (non-oracle) signal | **GATE 3** | Week 3+ |
| **S6** | Final results doc | — | Jul 7 |

---

## Reference — Paths

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

### S4 — Oracle correlation-aware patch · Week 3 → GATE 2 ⭐
**4.1** Find decision in `ipc/src/consensus.cpp` (χ² `fast_reject_th`).
**4.2** Branch `ca-oracle`: group-joint decision using `.groups` oracle — enumerate `2^|group|` labelings, score by joint consistency, pick min cost (DC-GM correlation reward on IPC subgraph; no SDP).
**4.3** ⛔ GATE 2 (make-or-break) — patched vs baseline IPC + TACO on correlated M3500 → `experiments/results/G2_oracle.csv`. Beats baseline → S5. Doesn't beat even with oracle → idea dead → pivot.

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
