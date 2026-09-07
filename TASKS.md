# TASKS — correlation-aware robust PGO (experiment-first)

Detailed, command-level plan. High-level phases + gates also live in the literature plan
(`<studies>/thesis/robust_pgo/literature/TASKS.md` — literature/theory tasks only; the experiment tasks live **here**).
**This repo is a superbuild** (see `docs/ATTRIBUTION.md` + `CLAUDE.md`): `ipc/` = our IPC fork (the contribution we patch);
`baselines/` = vendored RobustOptimizationSLAM (the comparators); root `CMakeLists.txt` builds them together.

**Experiment ladder (modular exploration):** the work is exploratory — each new approach joins the codebase in its own folder/module (see `baselines/` · `ipc/` · upcoming `taco/`). Stages capture **what the experiment reveals**, not kill conditions. Solutions aren't abandoned on a single negative result — the replication setup is built to test alternatives. E1 = how do baselines behave on correlated vs random outliers? · E2 = does the oracle group-joint approach help? · E3 = does it survive a realistic (non-oracle) signal?

## ⏰ Near-term deadlines (advisor)
- **Sat Jun 13 2026** ✅ DONE — Week 1 progress-report deck presented (IPC + baselines + figures · advisor decisions D-EVAL-3 + D-DIR-5 locked).
- **Sat Jun 20 2026 ⭐ THIS WEEK BINDING DEADLINE** — report findings on the **experimental triad**:
  1. **IPC replication — deep investigation.** Why does it not reproduce? Investigate hard (not shallow claims). Concrete root cause(s) + supporting evidence. Per D-EVAL-3, also contact the IPC author for the exact procedure.
  2. **TACO experiment** — implemented (kBL + SR-SC per D-DIR-5) + ran on S1 setup + verified vs TACO paper + comparison vs IPC + baselines.
  3. **Own solution experiment** — correlation-aware / group-joint approach implemented (new module folder) + ran + compared vs IPC + TACO + baselines on correlated vs random.

  Deliverable = experimental results + findings document covering all three. Supporting literature (L1 problem statement, L3 family JIT reads, adaptive-kernel understanding) feeds the report but is SECONDARY to the experiments running.

### Week 2 day shape — Tue Jun 16 → Sat Jun 20

> User routes; this is a default rhythm. Modular folder structure lets parallel tracks proceed; each new approach lands in its own module so progress on one doesn't block the others.

| Day | Primary | Secondary | Day-end checkpoint |
|---|---|---|---|
| **Mon Jun 15** ✅ | DONE: TACO + DC-IPC modules scaffolded + correlated generator + experiment harness + initial baseline figures (commit `4f3001a`) | tasks.md reorganized (`fa43f89`) | — |
| **Tue Jun 16** (today, partial day done) ✅+🟡 | DONE morning: refactor experiment (`a99a9f9` 14:13). IN PROGRESS: TACO + DC-IPC compiling + first runs on M3500; IPC investigation start; documentation (`docs/DC_IPC_experiment_setup.md`) | IPC author email (D-EVAL-3) if not sent · L1 problem statement draft | Three modules running on at least one dataset by EOD? |
| **Wed Jun 17** ✅ checkpoint passed | DONE: DC-IPC E2 first run on correlated M3500. **Result: DC-IPC came below IPC** (didn't beat baseline). Multi-dataset sweep STARTED (running on additional datasets) to verify the failure is consistent across datasets — confirming this isn't a M3500-specific artifact. | Continue IPC investigation | ✅ All three (IPC · TACO · DC-IPC) producing comparable numbers on M3500. |
| **Thu Jun 18** (Wed already started this) | **Sweep across datasets** — confirm the DC-IPC < IPC result is consistent (or surfaces interesting dataset-specific behavior) · capture E1/E2 multi-dataset tables · interpret the negative result honestly (what's the actual finding?) | Adaptive-kernel reading (L3-Adaptive) — informs any iteration on DC-IPC | Final E1/E2 tables ready · negative-result interpretation drafted? |
| **Fri Jun 19** | Findings document draft (advisor deliverable for Sat) — covers all three threads | E3 if time allows (realistic non-oracle signal) | Findings doc draft complete? |
| **Sat Jun 20 AM** | Findings document polish + advisor deliverable | (nothing — close-out only) | Deliverable shipped to advisor |

**Slip-detection rules (updated 2026-06-16):**
- ✅ TACO + DC-IPC scaffolding done (was the major risk; now compiling + running is the next risk)
- Wed EOD: if all three not producing comparable numbers on M3500 → focus Thu on getting them to comparable state · cut multi-dataset sweep
- Thu EOD: if E1/E2 tables not captured → run only headline metrics for Fri's writeup
- Fri EOD: anything not captured by Fri night → Sat AM becomes panic mode; surface this Fri afternoon, not Fri night

**Parallel work that runs without blocking:**
- IPC investigation can run on its own thread all week (no compile dependency)
- L1 problem statement final can be drafted in any spare 1-2h (no compute dependency)
- Author-contact response (if it lands) feeds IPC investigation

## Milestones
One spine: each **stage S0–S6** ends in a **milestone** (its deliverable = "done when…").
Two kinds of stage:
- **Verify** (S1, S2) — reproduce a *published* method's results before trusting it. Establishes the trusted foundation.
- **Experiment** (S3–S5) — run our hypothesis on the replicable setup; capture what the data shows; iterate on the solution direction.

(Milestone = the finish line of a stage; not a separate numbering. Build the trusted base — IPC then TACO — *before* our part. Each new approach joins as its own folder/module.)

| Stage | Milestone (done when…) | Result captured | Original target |
|---|---|---|---|
| **S0** | Superbuild compiles; `ipc_tester_2D` runs | — (setup) | Day 1 |
| **S1** | IPC **reproduces its paper's results** on the authors' setup (datasets, spoiling, metrics) | **Verify**: matches IPC paper within tolerance? Mismatch → investigate (not abandon) | Day 1–2 |
| **S2** | TACO **implemented** (authors didn't release code) + **reproduces the TACO paper** | **Verify**: matches TACO paper? Mismatch → refine impl | Day 3–4 |
| **S3** | Methods × regimes ran on random vs correlated | **E1**: which methods hold, which break, on correlated? Result feeds direction. | Day 5 |
| **S4** ⭐ | Oracle group-joint patch ran vs baseline IPC + TACO | **E2**: does the group-joint approach help? Result feeds direction. | Day 6–7 |
| **S5** | Realistic (non-oracle) signal pipeline ran across full panel | **E3**: does the win survive realistic signal? Result feeds direction. | Week 2 |
| **S6** | Final results doc complete | — | Jul 7 |

## Remaining tasks — by activity type
Status (2026-06-16 14:13 PM): **S0** ✅ build · **S1** ✅ ran — *IPC does NOT reproduce the paper* (root cause open) ·
**baselines** ✅ ran + compared (`experiments/figures/`) ·
**S2 TACO module** ✅ **scaffolded** (`taco/` exists · commit `4f3001a` Mon evening) ·
**S4-equivalent DC-IPC module** ✅ **scaffolded** (`dc_ipc/` — our correlation-aware hybrid · commit `4f3001a`) ·
**S3 correlated generator** ✅ **created** (`experiments/datagen/generateCorrelatedDataset.py`) ·
**Experiment harness** ✅ done (`experiments/scripts/run_baseline.sh` · `run_dcipc.sh` · `run_ipc.sh` · `run_taco.sh` · analysis registry + report) ·
**Initial figures** ✅ DCS / GNC / PCM comparison plots in `experiments/figures/report/`. **Remaining = verify runs + sweep + E1/E2/E3 capture + IPC investigation + findings doc.**

### 🔨 Implementation (write new code)
- [x] ✅ **TACO** module from the paper → new `taco/` wired into the superbuild. [S2] (committed Mon `4f3001a`)
- [x] ✅ **Correlated-outlier generator** → `experiments/datagen/generateCorrelatedDataset.py` (committed Mon `4f3001a`)
- [x] ✅ **DC-IPC module** (own solution — correlation-aware hybrid) → new `dc_ipc/` parallel to `ipc/` (committed Mon `4f3001a`). NOTE: structured as its own module (not a patch into `ipc/`) — preserves modularity per the experiment-ladder framing.
- [ ] **Realistic grouping + DC-SAM** integration. [S5]
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
- [ ] **Verify TACO** reproduces the TACO paper. [S2 verify]
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
- **Vendored baselines:** `baselines/` — SC, MaxMix, DCS, GNC, Huber, RRR in `robust_g2o/`; **PCM** + GTSAM tier in `robust_gtsam/`; metrics in `evaluator/`. **PCM = headline comparator** (closest prior art that reasons about consistency).
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

### S3 — Correlated generator + experiment 1 · Week 2 (NEXT 🎯)
**3.1** New `experiments/scripts/generateCorrelatedDataset.py` — clusters of non-adjacent poses sharing a common wrong transform + `.groups` sidecar (edge-id → cluster-id) + random control set.
**3.2** Methods × regimes — IPC + TACO + PCM + SC + DCS + GNC + MaxMix + RRR + Huber on random vs correlated → `experiments/results/E1_methods.csv`.
**3.3** **E1 result captured** — which methods hold on random / which break on correlated? Result feeds direction (informs whether group-joint formulation is promising or whether to try a different approach). Log in `paper_studies/DECISIONS.md`.

## S4 — Oracle correlation-aware experiment · Days 6–7 ⭐
**4.1 — Find the decision.** Read `ipc/src/consensus.cpp` → the χ² accept/reject (`fast_reject_th`).

**4.2 — Patch group-joint decision (oracle 𝒞).** On a branch (`git checkout -b ca-oracle`): when an edge belongs to a known group (from `.groups` = the **oracle**), decide the **whole group jointly** — enumerate `2^|group|` accept/reject labelings on that small group, score by joint consistency vs the trusted backbone, pick min cost. (DC-GM's correlation reward, on IPC's subgraph, solved by enumeration — no SDP.) **Done when:** patched tester runs on a correlated dataset using the labels.

**4.3 — E2 result captured.** Patched (oracle) vs baseline IPC (+ TACO) on correlated M3500 → `experiments/results/E2_oracle.csv`. Capture: does the group-joint approach help? By how much? Where does it not help? Result feeds direction. If oracle doesn't help: try a different formulation (the modular structure makes alternatives easy — add new approach as its own module). Log in `paper_studies/DECISIONS.md`.

### S5 — Realistic signal + DC-SAM · Week 3+
**5.1** Replace oracle `.groups` with derived grouping (geometric clustering / shared-source heuristic).
**5.2** Clone DC-SAM (https://github.com/MarineRoboticsGroup/dcsam); swap enumeration → DC-SAM online engine; verify §VI-A small-subset condition.
**5.3** Full panel (INTEL · M3500 · CSAIL · FR079 · MIT) × rates × all methods incl. TACO. **E3 result captured** — does the win survive realistic signal? Or does the realistic-grouping introduce new failure modes? Result feeds direction.

### S6 — Formalize + final · Week 4 (contract close)
**6.1** Provable formulation (advisor A2): local group-joint solve preserves IPC's online property + adds correlation handling · resolve Q6.
**6.2** Iterate winner / run fallback.
**6.3** Final results doc by Jul 7.

---

**Daily log:** append one line per session to `RUN_LOG.md` (date · step · key number · next).
