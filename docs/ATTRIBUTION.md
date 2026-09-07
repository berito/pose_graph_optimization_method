# Attribution

This repository is **not a fork of IPC.** It is thesis code that keeps IPC and the robust-PGO
baseline suite **vendored and unmodified**, and builds everything of ours *alongside* them rather
than inside them. No method direction is currently selected — see [`../DIRECTIONS.md`](../DIRECTIONS.md).

## Upstream (not ours — kept verbatim)

- **IPC — Incremental Probabilistic Consensus-based Consistent Set Maximization for SLAM Backends**
  Emilio Olivastri & Alberto Pretto, ICRA 2024.
  Source: https://github.com/EmilioOlivastri/IPC
  → lives under `ipc/` in this repo.
  **Verified byte-identical to the pinned upstream clone (2026-09-07)**, which is kept next door at
  `../../pose_graph_optimization_experiments/code/IPC`. Only `cfg/*.yaml` (the author's absolute
  paths, unrunnable as shipped) and `CMakeLists.txt` differ. `scripts/check_vendor.sh` enforces this.

- **RobustOptimizationSLAM** — Emilio Olivastri's robust-PGO baseline suite
  (Switchable Constraints, MaxMix, DCS, GNC, Huber, RRR, and PCM via Kimera-RPGO).
  Source: https://github.com/EmilioOlivastri/RobustOptimizationSLAM
  **Vendored** in-tree under `baselines/` at commit `62b1c9ccab71ac30fad98f5c5df28087eca0c1ae`.
  Six local edits, **all build-portability only, no algorithm or behaviour change**; they are
  itemised in `baselines/VENDOR.md`, which is the authoritative list.

The original IPC code — the χ² consensus backend, g2o I/O, `ipc/scripts/generateDataset.py` (itself
derived from the [Vertigo](https://github.com/OpenSLAM-org/openslam_vertigo) generator), the example
testers, and the SE(2) source datasets under `experiments/datasets/2D/` — is the work of the IPC
authors. All of it is retained here as the experimental base, unaltered.

## What is *this thesis's* contribution

Everything of ours lives in its own directory, on top of upstream — nothing is patched into it.

| ours | what it is | status |
|---|---|---|
| `taco/` | reimplementation of TACO (kBL-IPC + SR-SC, Olivastri PhD Ch. 5) from the spec, linking `ipc/` sources rather than copying them | ⚠ **not yet verified** against the author's numbers |
| `baselines/` → GM | comparator the IPC paper uses ([7]) but the author never released. Instantiates GTSAM's own `GemanMcClure` kernel — the algorithm is the library's, not ours | ⚠ threshold unpublished; **to confirm** |
| `baselines/` → ADAPT | comparator [5] (Barron, CVPR'19), never released and in no standard library. A genuine reimplementation: Barron loss as a GTSAM mEstimator + MAD scale + alpha grid-MLE, outer loop adapting alpha & scale | ⚠ **not yet verified** |
| `vendor_patched/` | fixed copies of vendored sources — vendor code is never edited in place | — |
| `scripts/` | `check_vendor.sh` (vendor-drift guard) + `vendor_patches.txt` (the ledger) | — |
| `experiments/` | the shared harness: `datagen/` (incl. `generateCorrelatedDataset.py`, which injects correlated/grouped outliers with group-label sidecars), `configs/`, `scripts/`, `analysis/`, `results/` | — |
| `dc_ipc/` | **attempt `d1`** — a group-joint consensus decision, built as its own module. **Tried; did not beat the original.** Kept on the record, not a live direction | ❌ see `DIRECTIONS.md` |
| `TASKS.md`, `HANDOFF.md`, `DIRECTIONS.md`, `RUN_LOG.md` | the experiment plan and its record | — |

⚠ **`ipc/src/consensus.cpp` is not patched.** An earlier version of this file claimed a group-joint
decision had been patched into it; that was never done — `d1` was built as the separate `dc_ipc/`
module instead, and the file is upstream-verbatim. Do not cite that claim.

## License

If/when the IPC repository states a license, the corresponding terms apply to the upstream portions;
upstream terms likewise apply to `baselines/` (see `baselines/VENDOR.md`). Please cite the IPC paper
(BibTeX in `ipc/README.md`, preserved verbatim from upstream) for any use of the backend, and
Olivastri's robust-optimization work for the baselines. Our additions are for academic thesis work.
