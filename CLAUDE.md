# robust_pgo — thesis code (correlation-aware robust PGO)

**What this is:** a **thesis fork of IPC** (see `docs/ATTRIBUTION.md`), extended to test whether **correlation-aware**
decisions beat per-edge/correlation-blind robust PGO on **correlated (grouped) outliers**. Experiment-first.

**NOT** the abandoned `../gna_pgo` (old graph-neural-network attempt — dead, ignore it).

## On session start
1. **Read `HANDOFF.md` FIRST** — the verification campaign: what we're doing, the cardinal rule
   (replicate the authors EXACTLY; verify IPC paper → verify TACO → then our contribution; never divert),
   the critical facts (s=3 vs s=10, TACO params inert in IPC, deterministic spoiled data), and current state.
2. Read `TASKS.md` — the command-level plan (S0→S6) + the milestones/gates (S1 verify IPC, S2 verify TACO, then G1–G3).
3. Check build: `ls build/ipc_tester_2D`. If missing, S0.1 (build via `.devcontainer/`).
4. Check `RUN_LOG.md` for the last step reached.

## The bet (falsification ladder — kill fast)
- **G1:** do IPC/baselines fail on correlated outliers? (else no gap)
- **G2 ⭐:** with *oracle* group labels, does group-joint decision beat per-edge? (else idea is dead — pivot)
- **G3:** does it hold with a *realistic* (non-oracle) correlation signal?

## Repo layout (monorepo superbuild)
- `ipc/` — our IPC fork (the contribution; the decision we patch). Has its own `CMakeLists.txt`.
- `baselines/` — **vendored** RobustOptimizationSLAM (Olivastri): the comparators —
  SC, MaxMix, DCS, GNC, Huber, RRR in `robust_g2o/` (g2o-only); GTSAM tier + **PCM** in
  `robust_gtsam/` (opt-in); metrics in `evaluator/`. Provenance + local patches in `baselines/VENDOR.md`.
- Root `CMakeLists.txt` is a superbuild → `mkdir build && cd build && cmake .. && make` builds
  IPC + g2o baselines + evaluator into one `build/`. Add `-DBUILD_GTSAM_BASELINES=ON` for the
  GTSAM/PCM tier (needs GTSAM + Kimera-RPGO + TBB; installed by `.devcontainer/`).
- **PCM** (`baselines/robust_gtsam`) is the closest prior art — the headline comparator for G1/G2.

## Key files
- `ipc/src/consensus.cpp` (174 lines) — IPC's χ² accept/reject; **the decision we patch** (S4).
- `ipc/scripts/generateDataset.py` — Vertigo random-outlier generator; **fork → our `experiments/scripts/generateCorrelatedDataset.py`** (S3). Keep `ipc/` pristine — our generator lives outside it.
  (Baselines also ship their own copy at `baselines/scripts/`.)
- `experiments/datasets/2D/M3500/{graph.g2o, GT.txt}` — start dataset (clean source graph + ground truth).
- `ipc/cfg/2D/*.yaml` — IPC run configs. `ipc/examples/ipc_tester_2D.cpp` — the runnable. `ipc/bash/ipc_experiments_2D.sh` — the author's experiment driver.
  Baseline configs live under `baselines/robust_g2o/cfg/` and `baselines/robust_gtsam/cfg/`.
- `experiments/` — our datasets/configs/results.

## The thesis docs (the why)
Problem + solution + decisions live in the literature monorepo:
- Problem: `<studies>/thesis/robust_pgo/literature/problem_solution/correlation_aware_problem.md`
- Solution: `…/correlation_aware_solution.md`
- Decisions log: `…/paper_studies/DECISIONS.md`

## Discipline (THESIS_CONTRACT — Jun 9 → Jul 7)
Total thesis exclusivity; results over prose; writing deferred. Work the gates in order; record each gate's
verdict + key number in `RUN_LOG.md` and the decisions log. Don't drift into the GNN approach or side work.

## Do NOT
- Touch `../gna_pgo` (abandoned) or the upstream reference clone `../../learning/papers_code/IPC`.
- Commit `build/` or run outputs under `experiments/results/` (see `.gitignore`). The small clean source datasets under `experiments/datasets/` ARE tracked.
