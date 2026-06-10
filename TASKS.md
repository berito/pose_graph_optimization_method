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

| Stage | Milestone (done when…) | Gate test | Target |
|---|---|---|---|
| **S0** | Superbuild compiles; `ipc_tester_2D` runs | — (setup) | Day 1 |
| **S1** | IPC **reproduces its paper's results** on the authors' setup (datasets, spoiling, metrics) | **Verify**: matches IPC paper within tolerance? No → fix build/setup | Day 1–2 |
| **S2** | TACO **implemented** (authors didn't release code) + **reproduces the TACO paper** | **Verify**: matches TACO paper? No → fix impl | Day 3–4 |
| **S3** | Correlated gap proven — methods hold on random, **break** on correlated | **GATE 1**: break on correlated? No → STOP | Day 5 |
| **S4** ⭐ | Oracle group-joint **beats** baseline IPC on correlated | **GATE 2**: beats baseline? No → idea dead, pivot | Day 6–7 |
| **S5** | Win **survives** a realistic (non-oracle) signal | **GATE 3**: realistic signal still wins? | Week 2 |
| **S6** | Final results doc complete | — | Jul 7 |

## Paths
- **Repo root (superbuild):** `/home/lab_desktop/Documents/code_base/thesis/robust_pgo`
- **Our IPC fork (patch here, S4):** `ipc/src/consensus.cpp` (174 lines — the χ² accept/reject) · tester `ipc/examples/ipc_tester_2D.cpp`
- **TACO (implement, S2):** authors did **not** release code → implement from the paper into a new `taco/` module wired into the superbuild. *(paper ref TBD)*
- **Vendored baselines (comparators):** `baselines/` — SC, MaxMix, DCS, GNC, Huber, RRR in `robust_g2o/`; **PCM** + GTSAM tier in `robust_gtsam/`; metrics in `evaluator/`. **PCM = headline comparator for G1/G2.**
- **M3500 (graph + ground truth):** `experiments/datasets/2D/M3500/{graph.g2o, GT.txt}`  ← start here
- **Generator to fork (S3):** `ipc/scripts/generateDataset.py` (Vertigo) → new `experiments/scripts/generateCorrelatedDataset.py` (keep `ipc/` pristine)
- **Configs:** `ipc/cfg/2D/*.yaml` · baseline cfgs under `baselines/*/cfg/`
- **IPC experiment driver (author's, for S1):** `ipc/bash/ipc_experiments_2D.sh`
- **Upstream refs (untouched):** `../../learning/papers_code/IPC` · github.com/EmilioOlivastri/{IPC, RobustOptimizationSLAM}

> **Honest notes:** (1) build via **`.devcontainer/`** — it compiles g2o@`20201223_git` automatically, no `G2O_ROOT` edits. (2) **DC-GM is model-only** (Matlab/SDP) — reimplement its two terms, not its code. (3) **DC-SAM not needed until S5** — S4 uses plain enumeration on the small group. (4) **TACO (S2) is our own implementation** — the authors released no code, so it must be verified against the TACO paper before being trusted as a comparator.

---

## S0 — Build · Day 1
**0.1 — Build (devcontainer, easiest).**
```bash
cd /home/lab_desktop/Documents/code_base/thesis/robust_pgo
# VS Code: "Reopen in Container" (provisions g2o + GTSAM/Kimera-RPGO). Then the SUPERBUILD:
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)   # ipc + g2o baselines + evaluator
#   add -DBUILD_GTSAM_BASELINES=ON to also build the GTSAM tier + PCM
./build/ipc/ipc_tester_2D -c ipc/cfg/2D/INTEL_params.yaml      # smoke test (run from repo root)
```
*Native fallback:* follow the **Dependencies** section in `README.md` (apt packages + build g2o tag `20201223_git` to `/usr/local`; no `G2O_ROOT` edit needed).
**Done when:** `build/ipc_tester_2D` runs without error.

**0.2 — Experiment scaffold.** `mkdir -p experiments/{configs,datasets,results,scripts}` + a `run.sh` harness that runs a method on a dataset and emits metrics (reused by S1/S2/S3).

## S1 — Reproduce IPC (prove the claim) · Day 1–2 → **Verify**
*Before extending IPC, prove our build reproduces the published IPC result on the authors' own setup. This validates the toolchain AND becomes the random-outlier control that S3 contrasts against.*

**1.1 — Mirror the authors' experiment.** Identify the IPC paper's datasets + protocol (the standard 2D/3D sets, the Vertigo `generateDataset.py` random spoiling at the paper's outlier rates, `canonic_inliers` per dataset). Run **unmodified IPC** across them; compute precision/recall (+ ATE via `evaluator`/evo).

**1.2 — Compare to the paper.** Tabulate our numbers vs the IPC paper's reported numbers → `experiments/results/S1_ipc_repro.csv`.

**1.3 — ✅ Verify.** Match within tolerance? **No → stop and fix the build/setup** before any further stage (a wrong build invalidates everything downstream). Record verdict + key numbers in `RUN_LOG.md`.

## S2 — Implement + verify TACO · Day 3–4 → **Verify**
*The TACO authors did not release code. Implement it ourselves from the paper and prove our implementation reproduces TACO's published results — so it's a trustworthy comparator for our part.*

**2.1 — Implement TACO from the paper.** Build TACO as a new module wired into the superbuild (e.g. `taco/` alongside `ipc/` + `baselines/`), reading the same g2o/GT format and emitting the same metrics. ⚠️ **NEEDS the TACO paper reference** to detail the algorithm sub-steps — fill in once provided.

**2.2 — Run on the S1 setup.** Same datasets / spoiling / metrics as S1 → `experiments/results/S2_taco_repro.csv`.

**2.3 — ✅ Verify.** Reproduces TACO's claimed results within tolerance? **No → fix the implementation** until it matches the paper. Record verdict in `RUN_LOG.md`.

## S3 — Correlated generator + prove the gap · Day 5 → **GATE 1**
**3.1 — `experiments/scripts/generateCorrelatedDataset.py`.** Read `ipc/scripts/generateDataset.py` (adds *independent random* false loops). New script injects **clusters**: pick non-adjacent poses, add false loops among them sharing a **common wrong transform** (mutually consistent), write a **`.groups` sidecar** (edge-id → cluster-id) + a random control set. **Done when:** emits a spoiled `.g2o` + `.groups`.

**3.2 — Methods × regimes.** IPC + **TACO** + the **vendored baselines** (`baselines/` — **PCM** as headline; plus SC, DCS, GNC, MaxMix, RRR, Huber) on **random** vs **correlated**, several rates → `experiments/results/G1_methods.csv` (ATE + outlier precision/recall via ground-truth labels). **PCM is the key comparator** — closest prior art that *does* reason about consistency; if even PCM (and TACO) break on correlated, the gap is sharp.

**3.3 — ⛔ GATE 1.** Hold on random, **break on correlated**? Pass → S4. Don't break → STOP, record why in `<lit>/paper_studies/DECISIONS.md`.

## S4 — Oracle correlation-aware patch · Days 6–7 → **GATE 2** ⭐
**4.1 — Find the decision.** Read `ipc/src/consensus.cpp` → the χ² accept/reject (`fast_reject_th`).

**4.2 — Patch group-joint decision (oracle 𝒞).** On a branch (`git checkout -b ca-oracle`): when an edge belongs to a known group (from `.groups` = the **oracle**), decide the **whole group jointly** — enumerate `2^|group|` accept/reject labelings on that small group, score by joint consistency vs the trusted backbone, pick min cost. (DC-GM's correlation reward, on IPC's subgraph, solved by enumeration — no SDP.) **Done when:** patched tester runs on a correlated dataset using the labels.

**4.3 — ⛔ GATE 2 (make-or-break).** Patched (oracle) vs baseline IPC (+ TACO) on correlated M3500 → `experiments/results/G2_oracle.csv`. Beats baseline → idea has legs, S5. No improvement *even with the oracle* → idea dead → pivot to adaptive-time-bound fallback or the `ipc_revision_limitation_problem` direction. Record verdict in DECISIONS.

## S5 — Realistic signal + DC-SAM + breadth · Week 2 → **GATE 3**
- **5.1** Replace oracle `.groups` with a *derived* grouping (geometric clustering / shared-source heuristic).
- **5.2** `git clone https://github.com/MarineRoboticsGroup/dcsam` (GTSAM-based); swap enumeration → DC-SAM's online engine for the group solve; log subgraph sizes (verify §VI-A small-subset condition).
- **5.3** Full panel (INTEL, M3500, CSAIL, FR079, MIT) × rates, all methods incl. TACO. **⛔ GATE 3:** realistic signal still wins?

## S6 — Formalize + iterate + final · Weeks 3–4 *(contract finish)*
- **6.1** Provable formulation (advisor A2): local group-joint solve preserves IPC's online property AND adds correlation handling. Resolve Q6.
- **6.2** Iterate winner / run fallback.
- **6.3** Final results doc by Jul 7.

---

## ✅ Done (this repo)
- ✅ **S0-0 — Fork + superbuild scaffold** (2026-06-09). IPC forked into `ipc/`; RobustOptimizationSLAM vendored into `baselines/` (commit `62b1c9c`); root superbuild CMake wires IPC + g2o baselines + evaluator (GTSAM/PCM tier opt-in); `.devcontainer/` provisions all deps; `docs/ATTRIBUTION.md` + `CLAUDE.md` written. Substrate ready. → next: **S0.1 build**.

---
**Daily log:** append one line per session to `RUN_LOG.md` (date · step · key number · next).
