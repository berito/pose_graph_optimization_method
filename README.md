# robust_pgo — correlation-aware robust PGO (thesis)

A thesis monorepo testing whether **correlation-aware** decisions beat per-edge /
correlation-blind robust pose-graph optimization on **correlated (grouped) outliers**.
It bundles our IPC fork *and* the standard robust-PGO baselines so the whole comparison
builds and runs from a single clone.

See [`docs/ATTRIBUTION.md`](docs/ATTRIBUTION.md) for what is upstream vs. our contribution, and `TASKS.md`
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
└── experiments/        our experiment workspace
    ├── datasets/       clean source graphs + ground truth (gitignored)
    ├── configs/        experiment-specific run configs
    ├── results/        generated spoiled data + metrics (gitignored)
    └── scripts/        run harness (run_method.sh)
```

## Build

Everything builds into one `build/`. Dependencies are provided automatically by
`.devcontainer/` — see **Dependencies** below only if you build on the host.

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

## Dependencies

The `.devcontainer/` installs all of this automatically (recommended). Only follow this if you
build on the host. Two tiers:
- **Default** (`ipc/` + g2o baselines + evaluator) needs **g2o + glog + yaml-cpp + Boost + Eigen**.
- **GTSAM tier** (`-DBUILD_GTSAM_BASELINES=ON`, incl. PCM) additionally needs **GTSAM + Kimera-RPGO + TBB**.

```bash
# system packages (Ubuntu 20.04/22.04/24.04)
sudo apt update && sudo apt install -y \
    build-essential cmake git pkg-config \
    libeigen3-dev libboost-all-dev libgoogle-glog-dev libgflags-dev libyaml-cpp-dev \
    libsuitesparse-dev libcholmod3 libtbb-dev \
    python3 python3-pip python3-numpy python3-matplotlib

# g2o (pinned tag; install to /usr/local so find_package(G2O) locates it)
git clone https://github.com/RainerKuemmerle/g2o.git && cd g2o && git checkout 20201223_git
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc) && sudo make install && sudo ldconfig && cd ../..

# GTSAM tier only (skip if not building -DBUILD_GTSAM_BASELINES=ON):
git clone https://github.com/borglab/gtsam.git && cd gtsam && git checkout 4.2.0
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_WITH_TBB=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_BUILD_TESTS=OFF
make -j$(nproc) && sudo make install && sudo ldconfig && cd ../..
git clone https://github.com/MIT-SPARK/Kimera-RPGO.git && cd Kimera-RPGO
mkdir build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc) && sudo make install && sudo ldconfig && cd ../..
```

Notes: Intel MKL is intentionally not required (GTSAM links Eigen+TBB). `evo` (`pip install evo`) is
optional, for ATE/RPE plots. ROS is only needed if a config sets `visualize: 1`.

## Cite

Built on IPC (Olivastri & Pretto, ICRA 2024) and Olivastri's robust-optimization baselines —
see [`docs/ATTRIBUTION.md`](docs/ATTRIBUTION.md) and `baselines/VENDOR.md`.

```bibtex
@INPROCEEDINGS{olivastri2024ipc,
  title={{IPC}: Incremental Probabilistic Consensus-based Consistent Set Maximization for SLAM Backends},
  author={Olivastri, Emilio and Pretto, Alberto},
  booktitle={2024 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2024}
}
```
