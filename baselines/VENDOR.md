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

Fixes 1–6 are build-portability only — no algorithm/behavior change.

### Added comparator (NOT in upstream): GM (Geman-McClure)
7. `robust_gtsam/src/gm_2D.cpp`, `gm_3D.cpp` (+ `gtsam_GM_2D/3D` targets in `robust_gtsam/CMakeLists.txt`).
   The IPC paper compares against **GM** (comparator [7], "provided by the gtsam library") but the author
   **did not release a GM tester**. These files are **verbatim clones of `dcs_2D/3D.cpp`** with the SINGLE
   algorithmic change being the m-estimator: `mEstimator::DCS::Create(th)` → `mEstimator::GemanMcClure::Create(th)`
   (GTSAM ships `GemanMcClure` in `LossFunctions.h`; we instantiate the library kernel, we do NOT reimplement
   the algorithm). Verified: GM ≡ DCS on clean data (no outliers ⇒ identical), as expected.
   ⚠️ The author's GM "best fixed parameter" was unpublished — `th=Chi2inv(alpha,dof)` mirrors DCS; **tune/confirm for S3/G1.**

### Added comparator (NOT in upstream): ADAPT (Barron adaptive robust loss)
8. `robust_gtsam/include/barron.hpp`, `robust_gtsam/src/adapt_2D.cpp`, `adapt_3D.cpp` (+ `gtsam_ADAPT_2D/3D` targets).
   IPC paper comparator [5] (Barron, CVPR'19). The author "implemented [it] inside g2o" but **did not release it**,
   and it is **not in any standard library** (checked GTSAM, g2o, Ceres — Ceres isn't even installed and ships no
   Barron loss; AEROS/Chebrolu code unreleased). It is **not a fixed kernel** — it needs IRLS. We reimplement:
   `barron.hpp` = Barron loss/weight as a GTSAM mEstimator + MAD scale + alpha grid-MLE (truncated partition);
   the testers run **inner IRLS (GTSAM robust noise, fixed alpha) inside an outer loop that adapts alpha & scale c**
   from the residual distribution (Barron CVPR'19 + Chebrolu RA-L'21 2004.14938 + AEROS Frontiers'22 2110.02018).
   Verified functional: M3500@30% → alpha→-2, **P=R=1** (all 1954 inliers kept, all 586 outliers rejected).
   ⚠️ **S3/G1 verification still required:** the alpha grid + truncated partition + MAD scale are pragmatic
   choices; ADAPT must be checked against the paper's reported ADAPT numbers before being trusted as a comparator
   (treat like TACO — a reimplemented method pending verification).

To refresh from upstream: re-clone, re-strip, re-apply edits 1–6, re-add GM (7) and ADAPT (8) files.

## Known issues
- **gtsam HUBER (`src/huber_2D.cpp`) crashes on MIT** (`IndeterminantLinearSystemException`): it uses the raw
  readG2o initial guess (odom-init is commented out) **and** applies Huber to ALL edges (weakening odometry), so
  GaussNewton diverges on MIT's poor init. Fine on INTEL/CSAIL/M3500 (P=R=1). The **g2o-tier `HUBER_2D`** handles
  MIT (P=1,R=.95) because it odom-initializes and robustifies loop edges only. For the paper we use the **gtsam**
  comparators (GNC/DCS/HUBER/GM); fix gtsam HUBER at S3 (odom-init + loop-only kernel, like DCS/GM) or use g2o
  HUBER for MIT. Do NOT silently change the vendored optimizer first.

## License / citation

Upstream terms apply to this directory. Cite the IPC paper (see repo root
`docs/ATTRIBUTION.md`) and Olivastri's robust-optimization work for any use of these baselines.
