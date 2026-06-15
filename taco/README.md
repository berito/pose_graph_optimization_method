# taco/ — TACO comparator (kBL-IPC + SR-SC)

Thesis reimplementation of TACO (Olivastri PhD, Ch. 5), built **on top of** `ipc/`. See
`docs/TACO.md` (spec) and `docs/TACO_vs_IPC_discussion.md` (the math delta).

## Design: reuse IPC, add only what's new
TACO **is** IPC + two modules, so this project links IPC's helper sources rather than duplicating them:
- **Reused from `ipc/`** (linked, not copied): `src/utils.cpp` (g2o I/O, store/restore, config) and
  `src/consensus_utils.cpp` (χ² test, fixComplementary, propagate, robustify) + all of `ipc/include`.
- **New here** — the only TACO-specific code:
  - `src/kbl_consensus.cpp` — **kBL-IPC** (Module A): test subgraph bounded to the `k` highest-IoU
    accepted loops (Eq. 5.6) instead of the full minimal independent subgraph.
  - `src/srsc.cpp` — **SR-SC** (Module B): randomized switchable-constraint recovery pass
    (reuses the vertigo switchable types in `baselines/robust_g2o/thirdParty/switchableConstraints`).
  - `src/taco_simulation.cpp` — incremental driver (kBL-IPC stream + SR-SC trigger every R=10 loops).
  - `examples/taco_tester_{2D,3D}.cpp` — runnables.

`ipc/` is left **pristine** (the contribution patches it only at S4); TACO lives alongside it.

## Variants via config (`cfg/2D/*.yaml`)
| Variant | `use_best_k_buddies` | `k_buddies` | `use_recovery` |
|---|---|---|---|
| IPC (full subgraph) | false | 0 | false |
| 2BL-IPC | true | 2 | false |
| TACO (full + SR-SC) | false | 0 | true |
| 2BL-TACO | true | 2 | true |

`s_factor` = 10 (2D) / 30 (3D) for TACO; thresholds mirror the IPC configs.

## Build & run
Built by the root superbuild (`-DBUILD_TACO=ON`, default). Executables land in `build/taco/`.
```
./build/taco/taco_tester_2D -c taco/cfg/2D/M3500_params.yaml
```

## Status
- **kBL-IPC (Module A)**: complete — IoU top-k subgraph; falls back to the full IPC subgraph when
  `use_best_k_buddies=false` (so `k=∞ ≡ IPC`).
- **SR-SC (Module B)**: complete — Algorithm 7. Per trigger it builds a self-contained sub-optimizer
  over the trusted subgraph span, attaches vertigo switchable constraints (reused from the SC
  baseline) to the unrevised batch, runs `maxIter=20` rounds with `ε=0.3` randomized prior flips
  (fixed per-span RNG seed → reproducible), and promotes loops whose average vote exceeds
  `inlierTh=0.7`. Promotion-only (additive recovery), matching the thesis.
  - Approximation vs thesis: the trusted subgraph is the transitive node-span superset (IPC-style),
    not the strictly-minimal Dijkstra subgraph (Fig. 5.5) — correct, possibly larger; a future
    speed optimization. Switch-prior weight = 1.0 (SC-paper default).

Both `taco_tester_2D` and `taco_tester_3D` compile and link clean. Not yet run/verified against the
thesis tables (S2 verification is the next step).
