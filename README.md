# pose_graph_optimization_method — thesis code

The code side of the thesis on **robust pose-graph optimization**. It holds the reference
implementations we verify against (`ipc/`, `taco/`), the vendored comparators (`baselines/`),
one shared experiment harness (`experiments/`), and one directory per **attempted direction**.

> ⚠ **No direction is currently selected.** This repo deliberately does **not** name a method
> in its title. The previous name `robust_pgo` asserted a solution family before one had been
> committed to — the same mistake that killed the earlier `gna_pgo` (which encoded
> *Graph-Neural-Attention*, later ruled out). **Name the subject, never the method.**
> `d1` (correlation-aware / group-joint, in `dc_ipc/`) **was tried and did not beat the
> original**; `d2` has not started. See [`DIRECTIONS.md`](DIRECTIONS.md) before assuming any
> direction is live — including anything a subdirectory's own README still claims for itself.

> **On the name:** renamed from `robust_pgo`. The CMake project id (`project(robust_pgo)`) and
> the devcontainer workspace (`/workspaces/robust_pgo`) still use the old name — deliberately,
> since changing them invalidates existing build and container caches. Old commit messages
> referencing `robust_pgo` are historical and cannot change.

## Read in this order

| file | what it is |
|---|---|
| `HANDOFF.md` | ⭐ start here — the verification campaign and its cardinal rule (replicate the authors exactly) |
| [`DIRECTIONS.md`](DIRECTIONS.md) | every direction tried and what became of it — **failures stay on the record** |
| `TASKS.md` | the command-level plan (S0→S6) and its stages |
| `RUN_LOG.md` | the last step actually reached |
| [`docs/ATTRIBUTION.md`](docs/ATTRIBUTION.md) | what is upstream vs. ours |

The original upstream IPC README (prerequisites, usage, citation) is preserved verbatim at
[`ipc/README.md`](ipc/README.md).

## Layout

```
pose_graph_optimization_method/
├── CMakeLists.txt      superbuild — stitches the four halves into one build/
├── HANDOFF.md          ⭐ read first — the campaign, the cardinal rule, the critical facts
├── TASKS.md            command-level plan (S0→S6) + the gates
├── DIRECTIONS.md       every direction tried and its outcome (failures stay on the record)
├── RUN_LOG.md          per-step log — the last step reached
├── ipc/                thesis fork of IPC — self-contained, mirrors the original IPC repo
│   ├── src/ examples/ include/ cmake/ CMakeLists.txt README.md
│   ├── bash/           author's experiment drivers (ipc_experiments_2D/3D.sh)
│   ├── cfg/            IPC run configs (2D/3D)
│   └── scripts/        Vertigo generator + plot (generateDataset.py, plotATERPET.py)
├── taco/               TACO comparator (kBL-IPC + SR-SC) — built ON TOP of ipc/, reuses its sources
├── dc_ipc/             attempt d1 — tried, did not beat the original (see DIRECTIONS.md)
├── baselines/          vendored RobustOptimizationSLAM (Olivastri) — the comparators
│   ├── robust_g2o/     SC · MaxMix · DCS · GNC · Huber · RRR   (g2o only)
│   ├── robust_gtsam/   GTSAM Huber/DCS/GNC + PCM (Kimera-RPGO) (opt-in)
│   ├── evaluator/      precision / recall / ATE metrics
│   └── VENDOR.md       provenance + local patches
├── vendor_patched/     fixed copies of vendored sources — vendor code is never edited in place
├── scripts/            check_vendor.sh (vendor-drift guard) + vendor_patches.txt (the ledger)
├── docs/               IPC.md · TACO.md · ATTRIBUTION.md · experiment setups · parameter tables
└── experiments/        our workspace
    ├── datasets/       clean source graphs + ground truth — TRACKED (~1.9 MB)
    ├── datagen/        our generators (generateCorrelatedDataset.py, generate_spoiled.py)
    ├── configs/        experiment-specific run configs
    ├── scripts/        run harness (run_ipc.sh, run_taco.sh, run_dcipc.sh, run_baseline.sh)
    ├── analysis/       evaluate.sh, registry.py, report.py
    ├── figures/        generated plots
    └── results/        run outputs — only the small .PR/.TE metric files are tracked
```

### What is and isn't in the repo

`experiments/datasets/` holds the **clean source graphs and ground truth** (~1.9 MB, 12 files) and
they **are tracked** — byte-identical to the authors' release, which the FR079 rule in `HANDOFF.md`
depends on. Two heavy things are deliberately **not** tracked and appear as symlinks into the
sibling `pose_graph_optimization_experiments/datasets/`:

| path | size | why not tracked |
|------|------|-----------------|
| `experiments/datasets/2D/M3500/SPOILED_DATA` | 84 MB | generated, deterministic — regenerate with `experiments/datagen/` |
| `experiments/datasets/tum` | 3.9 GB | re-downloadable; exceeds GitHub's 100 MB file limit |

A fresh clone therefore gets the clean sources but **dangling symlinks** for those two — regenerate
or re-download them before a run that needs them.

### The vendoring rule

Build **on** the vendor, never **in** it. Vendored sources under `baselines/` stay pristine; a fixed
copy goes in `vendor_patched/` and is recorded in `scripts/vendor_patches.txt`. `scripts/check_vendor.sh`
enforces this against the pinned upstream clone (exit `0` clean · `1` drift · `2` cannot check).
The same discipline applies to `ipc/` — `taco/` and `dc_ipc/` link its sources rather than copying them.

## Build

Everything builds into one `build/`. Dependencies are provided automatically by
`.devcontainer/` — see **Dependencies** below only if you build on the host.

```bash
mkdir -p build && cd build

# IPC + TACO + DC-IPC + g2o baselines + evaluator (needs only g2o, glog, yaml-cpp):
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# …also build the GTSAM/PCM tier (needs GTSAM + Kimera-RPGO + TBB):
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_GTSAM_BASELINES=ON
make -j$(nproc)
```

Options (all `ON` by default except the GTSAM tier):
`-DBUILD_IPC`, `-DBUILD_TACO`, `-DBUILD_DC_IPC`, `-DBUILD_BASELINES`, `-DBUILD_GTSAM_BASELINES` (default `OFF`).
The GTSAM tier defaults off so the build succeeds before those heavy deps are installed; the
devcontainer installs them and flips it on.

Resulting binaries (all in `build/`):

| Binary | From | Method |
|--------|------|--------|
| `ipc_tester_2D` / `ipc_tester_3D`, `graph_fixer` | `ipc/` | IPC χ² consensus — the reference we verify against |
| `taco_tester_2D` / `taco_tester_3D` | `taco/` | TACO comparator (kBL-IPC + SR-SC) |
| `dc_ipc_tester_2D` / `dc_ipc_tester_3D` | `dc_ipc/` | attempt **d1** (group-joint) — kept for the record, not a live direction |
| `IN_SC_2D`, `IN_MAXMIX_2D`, `IN_DCS_2D`, `IN_GNC_2D`, `IN_HUBER_2D`, `IN_RRR_2D` (+ `_3D`, + offline) | `baselines/robust_g2o` | per-edge / consistency baselines |
| `gtsam_PCM_2D`, `gtsam_{HUBER,DCS,GNC}_2D` (+ `_3D`) | `baselines/robust_gtsam` | GTSAM tier incl. **PCM** |
| `evaluator` | `baselines/evaluator` | precision/recall/ATE |

## Why the baselines are here

Any direction this thesis eventually takes has to be measured against the same comparators on the
same data, so the suite is vendored once and shared: the per-edge robust kernels (SC, MaxMix, DCS,
GNC, Huber, RRR) and the consistency-set method **PCM** — the closest prior art. They all read the
same g2o dataset format the harness in `experiments/datagen/` produces, which is what makes any
comparison apples-to-apples. `baselines/` is **read-only**: a comparator that gets edited stops
being a comparator.

## Dependencies

The `.devcontainer/` installs all of this automatically (recommended). Only follow this if you
build on the host. Two tiers:
- **Default** (`ipc/` + `taco/` + `dc_ipc/` + g2o baselines + evaluator) needs **g2o + glog + yaml-cpp + Boost + Eigen**.
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

## What is ours in here

This repo is mostly **other people's code kept deliberately unmodified**, so the boundary matters.
`docs/ATTRIBUTION.md` and `baselines/VENDOR.md` are the authoritative record; the short version:

| | status |
|---|---|
| `ipc/` | **not ours, and not patched.** Source verified byte-identical to pinned upstream (2026-09-07). Only `cfg/*.yaml` (the author's absolute paths, unrunnable as shipped) and `CMakeLists.txt` differ. Run `scripts/check_vendor.sh` before assuming otherwise. |
| `baselines/` | **vendored, read-only.** Six build-portability fixes only — no algorithm change. |
| `taco/` | **ours** — a reimplementation of TACO (Olivastri PhD, Ch. 5) from the spec, built on `ipc/` sources rather than copying them. Pending verification. |
| `baselines/` GM + ADAPT | **ours** — two comparators the IPC paper uses but the author never released. GM instantiates GTSAM's library kernel; ADAPT is a genuine reimplementation (Barron IRLS). Both pending verification. |
| `vendor_patched/` | **ours** — fixed copies of vendored sources, because vendor code is never edited in place. |
| `experiments/` | **ours** — the shared harness: datagen, configs, run scripts, analysis, metrics. |
| `dc_ipc/` | **ours** — attempt `d1`. Tried; did not beat the original. Kept on the record, not a live direction. |
| `scripts/` | **ours** — `check_vendor.sh`, the drift guard that keeps the rows above honest. |

⚠ Anything in this repo marked "reimplemented" (`taco/`, GM, ADAPT) is **not yet verified against the
authors' published numbers**. `HANDOFF.md` gives the order that verification has to happen in.
