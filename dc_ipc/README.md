# dc_ipc/ — DC-IPC (our correlation-aware contribution)

Our solution algorithm: **correlation-aware / group-joint** robust PGO. Where IPC/TACO decide one
loop closure at a time (correlation-blind), DC-IPC decides a **correlated group of loops jointly** —
the thesis bet (G2: with oracle group labels, does group-joint beat per-edge?).

Built **on top of** `ipc/` and `taco/`; both stay pristine.

## Design: reuse ipc + taco, add only the new decision
- **Reused (linked, not copied):**
  - from `ipc/`: `src/utils.cpp`, `src/consensus_utils.cpp` (g2o I/O, χ² test, propagate, robustify) + headers.
  - from `taco/`: `src/kbl_consensus.cpp` (kBL subgraph) and `src/srsc.cpp` (SR-SC recovery) + headers.
  - vertigo switchable types from `baselines/robust_g2o/thirdParty/switchableConstraints` (for SR-SC).
- **New here (the contribution):**
  - `src/dc_consensus.cpp` — `DC_IPC` class with `agreementCheckGroup(...)`, the **group-joint decision**.
  - `src/dc_simulation.cpp` — driver + `loadGroups()` (reads the correlation sidecar).
  - `examples/dc_ipc_tester_{2D,3D}.cpp` — runnables.

## Correlation groups
The driver reads `<dataset>.groups` (oracle group labels, produced by the S3 correlated-outlier
generator): one group per non-empty line, space-separated vertex-id pairs `a1 b1 a2 b2 …` naming the
loop edges in that group. **No sidecar → every loop is its own singleton group ⇒ behaves exactly like
correlation-blind IPC** (the baseline to beat).

## Config (`cfg/2D/*.yaml`)
Same fields as IPC/TACO. `use_best_k_buddies`/`k_buddies` enable the kBL subgraph (borrowed from TACO);
`use_recovery` enables SR-SC. `s_factor` = 3 (IPC baseline) or 10 (TACO 2D).

## Build & run
Built by the root superbuild (`-DBUILD_DC_IPC=ON`, default). Executables land in `build/dc_ipc/`.
```
./build/dc_ipc/dc_ipc_tester_2D -c dc_ipc/cfg/2D/M3500_params.yaml
```

## Algorithm (implemented) — the 4-term discrete-continuous model
Per `docs/REMOTE_BRIEF_correlation_aware.md` §3, on IPC's bounded subgraph we minimize over poses `X`
and binary labels `ℓ`:
`Σ_odom eᵀ(sΩ)e  +  Σ_loop ℓ·eᵀΩe  +  Σ_loop (1−ℓ)·κ  +  Σ_{(ab,cd)∈C} λ·𝟙[ℓ_ab≠ℓ_cd]`
- `κ` = the χ² reject threshold (`fast/slow_reject_th`); `λ` = correlation-reward (config key `lambda`, default 0).
- `DC_IPC::agreementCheckGroup()` solves it by **alternation**: (a) continuous g2o solve with `ℓ=1` loops
  active, (b) discrete **exact enumeration** of `2^|L|` labellings (submodular ⇒ exact; per-edge-threshold
  fallback + log when `|L|>16`). Decide each member by `ℓ*`.
- **`λ=0` ⇒ IPC** (guaranteed: fast-path delegates to the per-edge `agreementCheck`).

### Solver choice (vs the brief's DC-SAM)
The brief maps the model onto **DC-SAM** (GTSAM/iSAM2 hybrid). We implement the *math* directly in
**g2o** instead — the discrete step is exactly solvable by enumeration on IPC's small subgraph, so DC-SAM
(a heavy GTSAM dependency + a rewrite of our g2o stack) is **not needed for SE(2) + small subgraphs**.
DC-SAM remains the documented fallback if dense discrete coupling ever needs iSAM2-scale incrementality.

## Status
- Algorithm + driver (group-atomic decisions) + group sidecar + λ config: **implemented, compiles (2D+3D)**.
- **Not yet run/verified.** Needs the S3 correlated-outlier generator (`.groups` sidecar) to feed it.
- **Gate verdict is RELATIVE to our replicated IPC, not the paper** (see `TASKS.md` "Baseline of record"):
  the test is the correlation **on/off ablation on the same binary** — DC-IPC **λ>0 vs λ=0 (≡ IPC)** on the
  same correlated data. Internally valid **regardless of whether IPC reproduces the paper (S1)** — both arms
  share every assumption, so the delta isolates the correlation term. Pass = λ>0 beats λ=0 on correlated.
- Known modelling note: `λ=0` matches IPC's per-edge *thresholding*, not IPC's exact accepted-inlier *veto*
  (the joint model drops a now-inconsistent edge rather than vetoing the candidate) — intended for the
  correlation on/off comparison.
