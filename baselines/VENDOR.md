# Vendored: RobustOptimizationSLAM

This directory is a **vendored snapshot** of Emilio Olivastri's robust-PGO baseline suite,
kept in-tree so the whole thesis (IPC fork + comparators) builds from one clone.

- **Source:** https://github.com/EmilioOlivastri/RobustOptimizationSLAM
- **Commit:** `62b1c9ccab71ac30fad98f5c5df28087eca0c1ae`
- **Vendored on:** 2026-06-09
- **Stripped on import:** `.git/`, `CMakeFiles/`, `build_command.txt`

## What it gives us (the comparators)

| Tier | Binaries | Methods | Extra deps |
|------|----------|---------|------------|
| `robust_g2o/` | `IN_*` / offline `*` | Switchable Constraints, MaxMix, DCS, GNC, Huber, **RRR** | none beyond IPC (g2o, glog, yaml-cpp) |
| `robust_gtsam/` | `gtsam_*` | GTSAM Huber/DCS/GNC + **PCM** (Pairwise Consistency Maximization) | GTSAM + Kimera-RPGO + TBB |
| `evaluator/` | `evaluator`, `test_newmetrics` | precision / recall / ATE metrics | g2o only |

`PCM` (in the GTSAM tier) is the closest prior art to the thesis's correlation-aware
idea and the headline comparator.

## Local modifications vs upstream (keep this list current)

These are the only edits made to the vendored code; everything else is upstream-verbatim:

1. `CMakeLists.txt` — the GTSAM tier (`robust_gtsam`) is guarded behind the
   `BUILD_GTSAM_BASELINES` option (default OFF) so the g2o baselines + evaluator
   build with nothing more than the IPC fork already needs.
2. `robust_gtsam/CMakeLists.txt` — Intel **MKL** made optional (`find_package(MKL QUIET)`);
   GTSAM is linked against Eigen+TBB instead.

To refresh from upstream: re-clone, re-strip, and re-apply the two edits above.

## License / citation

Upstream terms apply to this directory. Cite the IPC paper (see repo root
`ATTRIBUTION.md`) and Olivastri's robust-optimization work for any use of these baselines.
