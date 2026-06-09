# robust_pgo — correlation-aware robust PGO (thesis)

A thesis monorepo testing whether **correlation-aware** decisions beat per-edge /
correlation-blind robust pose-graph optimization on **correlated (grouped) outliers**.
It bundles our IPC fork *and* the standard robust-PGO baselines so the whole comparison
builds and runs from a single clone.

See `ATTRIBUTION.md` for what is upstream vs. our contribution, and `TASKS.md`
for the experiment plan. The original upstream IPC README (prerequisites, usage,
citation) is preserved verbatim at [`ipc/README.md`](ipc/README.md).

## Layout

```
robust_pgo/
├── CMakeLists.txt      superbuild — stitches the two halves into one build/
├── ipc/                thesis fork of IPC (our contribution; the decision we patch)
│   ├── src/  examples/  include/  cmake/  CMakeLists.txt
├── baselines/          vendored RobustOptimizationSLAM (Olivastri) — the comparators
│   ├── robust_g2o/     SC · MaxMix · DCS · GNC · Huber · RRR   (g2o only)
│   ├── robust_gtsam/   GTSAM Huber/DCS/GNC + PCM (Kimera-RPGO) (opt-in)
│   ├── evaluator/      precision / recall / ATE metrics
│   └── VENDOR.md       provenance + local patches
├── scripts/            our generators (generateCorrelatedDataset.py, …)
├── cfg/                IPC run configs (2D/3D)
├── datasets/           graphs + ground truth (gitignored)
└── experiments/        our configs, runs, results
```

## Build

Everything builds into one `build/`. Dependencies are provided by `.devcontainer/`
(see `DEPENDENCIES.md` to build on the host instead).

```bash
mkdir -p build && cd build

# IPC + g2o baselines + evaluator (needs only g2o, glog, yaml-cpp):
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# …also build the GTSAM/PCM tier (needs GTSAM + Kimera-RPGO + TBB):
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_GTSAM_BASELINES=ON
make -j$(nproc)
```

Useful options: `-DBUILD_IPC=OFF`, `-DBUILD_BASELINES=OFF`, `-DBUILD_GTSAM_BASELINES=ON`.

Resulting binaries (all in `build/`):

| Binary | From | Method |
|--------|------|--------|
| `ipc_tester_2D` / `ipc_tester_3D`, `graph_fixer` | `ipc/` | IPC χ² consensus (ours to patch) |
| `IN_SC_2D`, `IN_MAXMIX_2D`, `IN_DCS_2D`, `IN_GNC_2D`, `IN_HUBER_2D`, `IN_RRR_2D` (+ `_3D`, + offline) | `baselines/robust_g2o` | per-edge / consistency baselines |
| `gtsam_PCM_2D`, `gtsam_{HUBER,DCS,GNC}_2D` (+ `_3D`) | `baselines/robust_gtsam` | GTSAM tier incl. **PCM** |
| `evaluator` | `baselines/evaluator` | precision/recall/ATE |

## Why the baselines are here

The thesis question is whether a **group-joint** decision beats **per-edge** robust kernels
on grouped outliers. The vendored suite supplies the per-edge comparators (SC, MaxMix, DCS,
GNC, Huber, RRR) and the consistency-set comparator **PCM** — the closest prior art — all on
the same g2o dataset format our `generateCorrelatedDataset.py` produces, so the comparison is
apples-to-apples.

## Cite

Built on IPC (Olivastri & Pretto, ICRA 2024) and Olivastri's robust-optimization baselines —
see `ATTRIBUTION.md` and `baselines/VENDOR.md`.

```bibtex
@INPROCEEDINGS{olivastri2024ipc,
  title={{IPC}: Incremental Probabilistic Consensus-based Consistent Set Maximization for SLAM Backends},
  author={Olivastri, Emilio and Pretto, Alberto},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2024}
}
```
