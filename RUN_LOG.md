# RUN_LOG — one line per session (date · step · key number · next)

See `TASKS.md` for the plan (stages S0–S6, gates G1–G3).

---

## 2026-06-09

- **S0-0 — Fork + superbuild scaffold.** IPC → `ipc/`; RobustOptimizationSLAM vendored → `baselines/` (`62b1c9c`); root superbuild CMake; `.devcontainer/` (g2o + GTSAM 4.2.0 + Kimera-RPGO). Next: S0.1.
- **S0.1 — Build GREEN + IPC runs. ✅** Devcontainer image built; full superbuild compiles & links: IPC (3) + 24 g2o baselines + 8 GTSAM baselines incl. **gtsam_PCM_2D/3D** + evaluator. IPC smoke-run on clean MIT → **Precision=1, Recall=1**, max consistent set=20, ~7 ms/test, `res.{txt,PR}` written.
  - Build fixes found by building (all build-portability, no behavior change): g2o → `/usr/local` (dropped hard-coded `slam-emix` path); yaml-cpp `::` alias (0.7); SuiteSparse `cs.h` include dir (IPC + RRR); TBB CONFIG mode (oneTBB vs vendored FindTBB); RRR `std::make_unique` qualify. All baseline patches logged in `baselines/VENDOR.md`.
  - Config: IPC code needs 4 keys absent from the (inconsistent) upstream cfgs — added `s_factor`/`use_best_k_buddies`/`k_buddies`/`use_recovery` to all 12 cfgs. `s_factor=1.0` (neutral) is a **placeholder** — confirm the paper/author value for exact reproduction.
- **S0.2 — Experiment scaffold. ✅** `experiments/{datasets,configs,results,scripts}` + `run_method.sh` (per-tool CLI dispatch) + README. `canonic_inliers` verified per dataset (CSAIL 128, INTEL 256, M3500 1954, MIT 20, FRH 1505). Moved clean **source** datasets root `datasets/` → `experiments/datasets/` (cfgs + docs repointed; generated spoiled data will go to `experiments/results/`). IPC re-verified on new path (P=R=1).
- **Paper references extracted. ✅** Read IPC paper + Olivastri PhD thesis → `docs/IPC.md` and `docs/TACO.md` (numbers kept in **separate** files per method, IPC vs TACO). Confirmed params: **IPC `s_factor` = 3** (all experiments; was placeholder 1.0 — now fixed in all cfgs, re-verified P=R=1); IPC has **no k** (all edges must agree ⇒ `use_best_k_buddies: false` ✓); χ² uses α=0.95 (DoF 3/2D, 6/3D), fast/slow 6.251/11.345 are code-level. TACO (thesis Ch.5, "Test And Check Optimization") uses s=10(2D)/30(3D), k default 2; **TACO is correlation-BLIND** (per-loop decision) — so it should also fail on correlated outliers (supports G1/our gap).
- **S1 in progress — IPC reproduction pipeline built + validated on M3500.**
  - Tooling (all author-faithful, paths-only changes): `experiments/scripts/generate_spoiled.py` (Vertigo, n=rate×inliers, seeds 0–9, deterministic/reusable), `experiments/scripts/ipc_experiments_2D.sh` (minimal-edit copy of the author's driver; `yq` overrides), `experiments/scripts/aggregate_pr.py`. Installed `yq` (in Dockerfile).
  - **Found:** `k_buddies`/`use_recovery` are TACO params but **INERT in the IPC binary** → see `docs/parameters_ipc_vs_taco.md`. The author's committed script uses **s=10** ("IPC_S10", thesis), **not** the 2024 paper's **s=3**.
  - **Ran full M3500 sweep at s=10** (10 rates ×10 runs, ×10 parallel): precision **0.98–0.996**, recall **0.61–0.77**, F1 0.75–0.87 (`experiments/results/IPC/M3500/.../metrics_summary.csv`). Machinery validated. But recall <80% — this is the **thesis IPC_S10 operating point**, NOT the paper.
  - **NEXT:** rerun M3500 at **s=3** to verify the **2024 paper** (recall should exceed 80%, F1 → ~0.86+), then extend to all 6 datasets, then S2 (TACO). Migrating to a 52-core/64GB server for the full campaign — regenerate spoiled data there (deterministic), raise parallelism (result-neutral). See `HANDOFF.md`.
