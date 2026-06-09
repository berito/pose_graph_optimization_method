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
3. `robust_g2o/CMakeLists.txt` — yaml-cpp **target alias** shim: Ubuntu 22.04 ships yaml-cpp 0.7
   which exports `yaml-cpp` (un-namespaced); add `yaml-cpp::yaml-cpp` alias so the existing link
   lines resolve on both 0.7 and 0.8.
4. `CMakeLists.txt` (top) — add the **SuiteSparse include dir** (`find_path(CSPARSE_INCLUDE_DIR cs.h …)`):
   the RRR baseline includes g2o's csparse solver header, which does `#include <cs.h>`.
5. `robust_gtsam/CMakeLists.txt` — TBB found via **CONFIG mode** (`find_package(TBB CONFIG REQUIRED)`)
   instead of the vendored `cmake/FindTBB.cmake`, which reads the removed header `tbb/tbb_stddef.h`
   and errors on Ubuntu 22.04's oneTBB.
6. `robust_g2o/src/incr/rrr_2D.cpp`, `rrr_3D.cpp` — qualify `make_unique` → `std::make_unique`
   (this g2o ships `g2o::make_unique`; with `using namespace std;` + `using namespace g2o;` the bare
   call was ambiguous). The offline `src/off/rrr_*.cpp` already used `std::`.

All six fixes are build-portability only — no algorithm/behavior change. To refresh from upstream:
re-clone, re-strip, and re-apply the edits above.

## License / citation

Upstream terms apply to this directory. Cite the IPC paper (see repo root
`docs/ATTRIBUTION.md`) and Olivastri's robust-optimization work for any use of these baselines.
