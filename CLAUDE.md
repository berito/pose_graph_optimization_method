# pose_graph_optimization_method — the thesis contribution

⚠ **Renamed 2026-09-07, and the old name was the problem.** It was `robust_pgo`, which
asserts a solution family before `DIRECTION.md` has committed to one — the same mistake that
killed `gna_pgo` (it encoded *Graph-Neural-Attention*, a method later ruled out). **Name the
subject and whose it is; never the method.** Siblings under `code_base/thesis/`:
`pose_graph_optimization_experiments/` (other people's code) and
`pose_graph_optimization_math/` (the concepts, visualised).

⚠ The GitHub remote is still `berito/robust_pgo` until it is renamed there.

**What this is:** a **thesis fork of IPC** (see `docs/ATTRIBUTION.md`), extended to test whether **correlation-aware**
decisions beat per-edge/correlation-blind robust PGO on **correlated (grouped) outliers**. Experiment-first.

**NOT** the abandoned **GNN attempt** (old graph-neural-network line — dead; it lives in `code_base/archive/gna_pgo`, ignore it).

## On session start
1. **Read `HANDOFF.md` FIRST** — the verification campaign: what we're doing, the cardinal rule
   (replicate the authors EXACTLY; verify IPC paper → verify TACO → then our contribution; never divert),
   the critical facts (s=3 vs s=10, TACO params inert in IPC, deterministic spoiled data), and current state.
2. ⭐ **Read [`DIRECTIONS.md`](DIRECTIONS.md)** — every attempt tried and what became of it.
   **d1 (correlation-aware IPC) was tried and did not beat the original.** Do not restart it
   without reading which gate stopped it.
3. Read `TASKS.md` — the command-level plan (S0→S6) + the milestones/gates (S1 verify IPC, S2 verify TACO, then G1–G3).
4. Check build: `ls build/ipc_tester_2D`. If missing, S0.1 (build via `.devcontainer/`).
5. Check `RUN_LOG.md` for the last step reached.

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

## ⭐ Vendor code is never edited. A change lives in OUR code, based on it.

**Set 2026-09-07.** Three kinds of thing live here and they are governed differently:

| | what it is | may we edit it? |
|---|---|---|
| `baselines/` | **vendored upstream** — the comparators | ⛔ **never.** A comparator that is edited stops being a comparator: its numbers are no longer the published method's |
| `ipc/` | **our fork** of IPC — the base we patch | ⚠ patch only where the contribution requires it, and record it in `docs/ATTRIBUTION.md` |
| `dc_ipc/`, and future attempts | **a direction** — ours | ✅ freely |

⭐ **The rule in one line: build ON the vendor, never IN it.** When a vendored file needs to
behave differently, the changed version lives in our own directory and the original stays
untouched — so *what we took* and *what we changed* are always separable, by anyone, later.

*(This was already being done in one place and not stated as a rule: `ipc/scripts/generateDataset.py`
is forked to `experiments/scripts/generateCorrelatedDataset.py` rather than edited in place — "keep
`ipc/` pristine". That instinct is now the rule for everything.)*

⚠ **Why it matters beyond tidiness.** A thesis has to say which parts are its own. If the
edit history of a comparator is mixed into ours, that sentence cannot be written honestly,
and no reviewer can check it.

## ⭐ A direction is a directory, never a branch

Attempts are compared, so they need the **same harness on the same data**. Across long-lived
branches the harness drifts and the numbers stop being comparable — which is the only reason
to run more than one attempt. Full rules, and the record of every attempt: [`DIRECTIONS.md`](DIRECTIONS.md).

- A killed attempt **stays on disk** with a `❌` header naming the gate that killed it.
- `experiments/` — datagen, configs, analysis, metrics — belongs to **all** attempts.
  A direction that edits the measurement has destroyed the comparison.

## Key files
- `ipc/src/consensus.cpp` (174 lines) — IPC's χ² accept/reject; **the decision we patch** (S4).
- `ipc/scripts/generateDataset.py` — Vertigo random-outlier generator; **fork → our `experiments/scripts/generateCorrelatedDataset.py`** (S3). Keep `ipc/` pristine — our generator lives outside it.
  (Baselines also ship their own copy at `baselines/scripts/`.)
- `experiments/datasets/2D/M3500/{graph.g2o, GT.txt}` — start dataset (clean source graph + ground truth).
- `ipc/cfg/2D/*.yaml` — IPC run configs. `ipc/examples/ipc_tester_2D.cpp` — the runnable. `ipc/bash/ipc_experiments_2D.sh` — the author's experiment driver.
  Baseline configs live under `baselines/robust_g2o/cfg/` and `baselines/robust_gtsam/cfg/`.
- `experiments/` — our datasets/configs/results.

## The thesis docs (the why) — in the research repo *(single home; don't restate here)*
- Problem · candidates · decision log → `research/pose_graph_optimization/literature/thinking/DIRECTION.md`
- Scope (in / out) → `research/pose_graph_optimization/planning/SPEC.md`

## Discipline (thesis sprint — live deadline in `research/…/planning/TRACKER.md`, not restated here)
Total thesis exclusivity; results over prose; writing deferred. Work the gates in order; record each gate's
verdict + key number in `RUN_LOG.md` and the decisions log. Don't drift into the GNN approach or side work.

## Do NOT
- Touch the abandoned GNN attempt (`code_base/archive/gna_pgo`) or the upstream reference clone `../../learning/papers_code/IPC`.
- Commit `build/` or run outputs under `experiments/results/` (see `.gitignore`). ⚠ `experiments/datasets` is now a **symlink** to
  `../../pose_graph_optimization_experiments/datasets` — the one shared dataset folder. It was
  **broken** until 2026-09-07 (it pointed at a path that does not exist), so git reported twelve
  tracked dataset files as deleted and the repo had no data at all. **If those files show as
  deleted again, check the symlink first.**
