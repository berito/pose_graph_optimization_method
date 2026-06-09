# TASKS — correlation-aware robust PGO (experiment-first)

Detailed, command-level plan. High-level phases + gates also live in the literature plan
(`<studies>/thesis/robust_pgo/literature/TASKS.md` — literature/theory tasks only; the experiment tasks live **here**).
**This repo is a superbuild** (see `ATTRIBUTION.md` + `CLAUDE.md`): `ipc/` = our IPC fork (the contribution we patch);
`baselines/` = vendored RobustOptimizationSLAM (the comparators); root `CMakeLists.txt` builds them together.

**Falsification ladder (kill fast):** G1 = baselines fail on correlated outliers? · G2 = oracle-correlation wins? · G3 = realistic signal holds?

## Paths
- **Repo root (superbuild):** `/home/lab_desktop/Documents/code_base/thesis/robust_pgo`
- **Our IPC fork (patch here, S2):** `ipc/src/consensus.cpp` (174 lines — the χ² accept/reject) · tester `ipc/examples/ipc_tester_2D.cpp`
- **Vendored baselines (comparators):** `baselines/` — SC, MaxMix, DCS, GNC, Huber, RRR in `robust_g2o/`; **PCM** + GTSAM tier in `robust_gtsam/`; metrics in `evaluator/`. **PCM = headline comparator for G1/G2.**
- **M3500 (graph + ground truth):** `datasets/2D/M3500/{graph.g2o, GT.txt}`  ← start here
- **Generator to fork (S1):** `scripts/generateDataset.py` (Vertigo) → new `scripts/generateCorrelatedDataset.py`
- **Configs:** `cfg/2D/*.yaml` · baseline cfgs under `baselines/*/cfg/`
- **Upstream refs (untouched):** `../../learning/papers_code/IPC` · github.com/EmilioOlivastri/{IPC, RobustOptimizationSLAM}

> **Honest notes:** (1) build via **`.devcontainer/`** — it compiles g2o@`20201223_git` automatically, no `G2O_ROOT` edits. (2) **DC-GM is model-only** (Matlab/SDP) — reimplement its two terms, not its code. (3) **DC-SAM not needed until S3** — S2 uses plain enumeration on the small group.

---

## S0 — Build + baseline run · Day 1
**0.1 — Build (devcontainer, easiest).**
```bash
cd /home/lab_desktop/Documents/code_base/thesis/robust_pgo
# VS Code: "Reopen in Container" (provisions g2o + GTSAM/Kimera-RPGO). Then the SUPERBUILD:
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)   # ipc + g2o baselines + evaluator
#   add -DBUILD_GTSAM_BASELINES=ON to also build the GTSAM tier + PCM
./ipc_tester_2D -c ../cfg/2D/INTEL_params.yaml      # smoke test (binary under build/ — check build/ or build/ipc/)
```
*Native fallback:* apt-install `DEPENDENCIES.md`, build g2o tag `20201223_git`, set `G2O_ROOT` at `CMakeLists.txt:5`.
**Done when:** `build/ipc_tester_2D` runs without error.

**0.2 — Baseline on M3500 (clean → spoil → run → evaluate).**
```bash
python3 scripts/generateDataset.py -i datasets/2D/M3500/graph.g2o -n 100   # random outliers
# edit cfg/2D/M3500_params.yaml → point at the spoiled file (input_file, canonic_inliers, chi2 thresholds)
./build/ipc_tester_2D -c cfg/2D/M3500_params.yaml                          # → estimate
pip install --user evo
evo_ape tum datasets/2D/M3500/GT.txt <estimate>.txt -a                     # ATE vs ground truth
```
**Done when:** you have IPC's ATE + which loops it accepted/rejected on a spoiled M3500.

**0.3 — Experiment scaffold.** `mkdir -p experiments/{configs,datasets,results,scripts}` + a `run_baseline.sh` reproducing 0.2 in one command.

## S1 — Correlated generator + prove the gap · Day 2 → **GATE 1**
**1.1 — `scripts/generateCorrelatedDataset.py`.** Read `scripts/generateDataset.py` (adds *independent random* false loops). New script injects **clusters**: pick non-adjacent poses, add false loops among them sharing a **common wrong transform** (mutually consistent), write a **`.groups` sidecar** (edge-id → cluster-id) + a random control set. **Done when:** emits a spoiled `.g2o` + `.groups`.

**1.2 — Baselines × regimes.** IPC + the **vendored baselines** (`baselines/` — **PCM** as headline; plus SC, DCS, GNC, MaxMix, RRR, Huber) on **random** vs **correlated**, several rates → `experiments/results/G1_baselines.csv` (ATE + outlier precision/recall via ground-truth labels). **PCM is the key comparator** — it's the closest prior art that *does* reason about consistency, so if even PCM breaks on correlated, the gap is sharp.

**1.3 — ⛔ GATE 1.** Hold on random, **break on correlated**? Pass → S2. Don't break → STOP, record why in `<lit>/paper_studies/DECISIONS.md`.

## S2 — Oracle correlation-aware patch · Days 3–4 → **GATE 2** ⭐
**2.1 — Find the decision.** Read `ipc/src/consensus.cpp` → the χ² accept/reject (`fast_reject_th`).

**2.2 — Patch group-joint decision (oracle 𝒞).** On a branch (`git checkout -b ca-oracle`): when an edge belongs to a known group (from `.groups` = the **oracle**), decide the **whole group jointly** — enumerate `2^|group|` accept/reject labelings on that small group, score by joint consistency vs the trusted backbone, pick min cost. (DC-GM's correlation reward, on IPC's subgraph, solved by enumeration — no SDP.) **Done when:** patched tester runs on a correlated dataset using the labels.

**2.3 — ⛔ GATE 2 (make-or-break).** Patched (oracle) vs baseline IPC on correlated M3500 → `experiments/results/G2_oracle.csv`. Beats baseline → idea has legs, S3. No improvement *even with the oracle* → idea dead → pivot to adaptive-time-bound fallback or the `ipc_revision_limitation_problem` direction. Record verdict in DECISIONS.

## S3 — Realistic signal + DC-SAM + breadth · Week 2 → **GATE 3**
- **3.1** Replace oracle `.groups` with a *derived* grouping (geometric clustering / shared-source heuristic).
- **3.2** `git clone https://github.com/MarineRoboticsGroup/dcsam` (GTSAM-based); swap enumeration → DC-SAM's online engine for the group solve; log subgraph sizes (verify §VI-A small-subset condition).
- **3.3** Full panel (INTEL, M3500, CSAIL, FR079, MIT) × rates. **⛔ GATE 3:** realistic signal still wins?

## S4 — Formalize + iterate + final · Weeks 3–4 *(contract finish)*
- **4.1** Provable formulation (advisor A2): local group-joint solve preserves IPC's online property AND adds correlation handling. Resolve Q6.
- **4.2** Iterate winner / run fallback.
- **4.3** Final results doc by Jul 7.

---

## ✅ Done (this repo)
- ✅ **S0-0 — Fork + superbuild scaffold** (2026-06-09). IPC forked into `ipc/`; RobustOptimizationSLAM vendored into `baselines/` (commit `62b1c9c`); root superbuild CMake wires IPC + g2o baselines + evaluator (GTSAM/PCM tier opt-in); `.devcontainer/` provisions all deps; `ATTRIBUTION.md` + `CLAUDE.md` written. Substrate ready. → next: **S0.1 build**.

---
**Daily log:** append one line per session to `docs/RUN_LOG.md` (date · step · key number · next).
