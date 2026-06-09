# Attribution

This repository is a **thesis fork of IPC**, extended for correlation-aware robust pose-graph optimization.

## Upstream
- **IPC — Incremental Probabilistic Consensus-based Consistent Set Maximization for SLAM Backends**
  Emilio Olivastri & Alberto Pretto, 2024.
  Source: https://github.com/EmilioOlivastri/IPC
  Local reference clone (untouched): `../../learning/papers_code/IPC`
  → lives under `ipc/` in this repo.
- **RobustOptimizationSLAM** — Emilio Olivastri's robust-PGO baseline suite
  (Switchable Constraints, MaxMix, DCS, GNC, Huber, RRR, and PCM via Kimera-RPGO).
  Source: https://github.com/EmilioOlivastri/RobustOptimizationSLAM
  **Vendored** in-tree under `baselines/` (commit `62b1c9c`) so the comparators build from one
  clone. Snapshot provenance + the two local CMake patches are listed in `baselines/VENDOR.md`.

The original IPC code (the χ² consensus backend, g2o I/O, `scripts/generateDataset.py` — itself derived from the
[Vertigo](https://github.com/OpenSLAM-org/openslam_vertigo) dataset generator, the example testers, and the bundled
SE(2) datasets in `datasets/2D/`) is the work of the IPC authors. All of it is retained here as the experimental base.

## What is *this thesis's* contribution (the diff)
Everything we add lives clearly on top of upstream:
- `scripts/generateCorrelatedDataset.py` — **new**: injects *correlated / grouped* outliers (clusters of mutually-consistent false loops) + group-label sidecars.
- A **group-joint decision** patched into `src/consensus.cpp` — the correlation-aware extension (DC-GM-style joint scoring on IPC's bounded subgraph).
- `experiments/` — our configs, generated datasets, run scripts, and results.
- `TASKS.md` — the thesis experiment plan.

If/when the IPC repository states a license, the corresponding terms apply to the upstream portions; please cite the
IPC paper for any use of the backend. Our additions are for academic thesis work.
