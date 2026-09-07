# HANDOFF — verification campaign (READ THIS FIRST)

For any agent/person continuing this work (esp. on the server). This travels with the repo;
the project memory files do NOT. Also read: `TASKS.md`, `RUN_LOG.md`, `docs/IPC.md`,
`docs/TACO.md`, `docs/parameters_ipc_vs_taco.md`.

## What we are doing — and the strict order
We are **VERIFYING published robust-PGO results before building anything of our own**:
1. **Verify IPC (2024 ICRA paper)** — reproduce its numbers from our build.
2. **Verify TACO** — implement it from the thesis (the authors released no TACO code) + reproduce its numbers.
3. **Only then** our correlation-aware contribution (S3+ in `TASKS.md`).

Do **not** skip to step 3. Do **not** modify the methods. We confirm the authors' claims first.

## The cardinal rule — do not violate
**Replicate the author EXACTLY.** During verification:
- Mirror the author's own scripts/params verbatim (`ipc/bash/ipc_experiments_2D.sh`, `ipc/scripts/generateDataset.py`).
- The ONLY things allowed to differ are **result-neutral**: file paths, output location, and parallelism/scheduling.
- **NEVER** change a result-affecting parameter: `s_factor`, χ² thresholds, outlier counts, # Monte-Carlo runs, datasets, the generator.
- Don't reinvent generators/metrics. If something is ambiguous, **read the author's scripts/papers — do not guess.**

## Critical facts (already learned — don't relearn the hard way)
1. **`s=3` (paper) vs `s=10` (thesis).** The 2024 IPC paper uses **`s_factor=3`**. The author's *committed*
   script `ipc_experiments_2D.sh` uses **`s_factor=10`** ("IPC_S10", the thesis comparator). For **paper
   verification use `s=3`.** (Confirmed: `s=10` → recall ~0.61–0.77 on M3500; the paper wants recall **>80%**.)
2. **TACO params are INERT in the IPC binary.** `k_buddies`, `use_best_k_buddies`, `use_recovery` are read
   from config but **never used** by IPC (they are TACO's kBL/SR-SC, implemented only in the unreleased TACO
   code). So the IPC run is plain IPC regardless of these. Detail: `docs/parameters_ipc_vs_taco.md`.
3. **Spoiled data is deterministic** (seeds = run index 0–9). **Regenerate on the server** with
   `experiments/scripts/generate_spoiled.py` — do NOT transfer it; same seeds → byte-identical files.
4. **Metrics** come from each run's `.PR` (IPC's own precision/recall). Aggregate per rate with
   `experiments/scripts/aggregate_pr.py`. Paper numbers are **averaged over 6 datasets**, so a single-dataset
   run matches the **trend/bounds**, not exact figures (per-dataset numbers aren't published).
5. **Datasets** (IPC paper): CSAIL, FR079, FRH, MIT, INTEL, M3500 — in `experiments/datasets/2D/`.
   `canonic_inliers`: CSAIL 128, INTEL 256, M3500 1954, MIT 20, FRH 1505, **FR079 229**.
   Outlier rates **10–100%** (`n = rate × inliers`), **10 runs** (00–09).
   (Inlier count = loop-closure edges `b≠a+1` in the clean graph; method validated against all 5 known values.)
6. **FR079 — DO NOT "FIX" IT.** Its `EDGE_SE2` info matrices look malformed to g2o (`q_θθ=0`, `q_xθ` huge
   on all 1217 edges — the TORO-vs-g2o column-order signature). **It is NOT ours to convert:** the file is
   **byte-identical to the author's release** (verified vertices+edges against the IPC Google-Drive copy on
   2026-06-10). The author ran this exact file, so we must too (cardinal rule). Consequence: FR079's **standalone
   clean recall is only ~0.33** (accepts 76/229) — **this is expected**, not a bug, and is absorbed into the
   paper's **6-dataset-averaged** recall (per-dataset recall is never published; only Table II *times* are).
   **IF the S1 averaged recall diverges from the paper (>80%), FR079 is the first suspect — revisit it then,
   but NEVER silently edit the author's dataset.** (Converting it would corrupt the replication.)
7. **Hardware vs results.** Author ran on an **Intel Xeon Gold 5220** (thesis; IPC paper doesn't state it).
   - **Accuracy (Precision/Recall/F1/ATE/RPE) is hardware-INDEPENDENT** — deterministic from input+params.
     So the verification verdict is valid on any machine (your 16-core box, the 52-core server, the Xeon).
   - **Timing (convergence time, ACTxC, runtime) IS hardware-dependent.** Do NOT expect to match the
     paper's absolute time numbers (IPC ≈321/443 s; M3500 ACTxC ≈0.54–0.72 s/constraint) — compare only
     RELATIVE timing/trends. **Never fail verification because times differ across machines.**

## Current state (at handoff)
- Build **GREEN** in the devcontainer (IPC + 24 g2o baselines + GTSAM/PCM + evaluator).
- Machinery **VALIDATED**: full M3500 sweep ran faithfully at **s=10** → clean P/R/F1 curve (precision
  0.98–0.996, recall 0.61–0.77). Saved under `experiments/results/IPC/M3500/`. This reproduces the
  *thesis* "IPC_S10" operating point, NOT the 2024 paper.
- **NEXT:** run M3500 with **s=3** to verify the 2024 paper (recall should exceed 80%), then extend to all
  6 datasets, then **S2 (implement + verify TACO)**.

## How to run the verification (exact procedure)
In the devcontainer (or native build to `/usr/local`):
```bash
# 1. build
mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)   # + -DBUILD_GTSAM_BASELINES=ON for PCM
cd ..
# 2. generate spoiled data per dataset (deterministic)
python3 experiments/scripts/generate_spoiled.py --clean experiments/datasets/2D/M3500/graph.g2o --inliers 1954 --out experiments/datasets/2D/M3500/SPOILED_DATA
# 3. run IPC, author-faithful. For the PAPER set s_factor=3.0 in the script's yq overrides (NOT 10).
bash experiments/scripts/ipc_experiments_2D.sh          # parallelism may be raised on the server (result-neutral)
# 4. aggregate + compare
python3 experiments/scripts/aggregate_pr.py --root experiments/results/IPC/M3500/110125/G2O_IPC_REC_20
# 5. compare to docs/IPC.md ; record verdict + key numbers in RUN_LOG.md
```

## Do NOT
- Modify `ipc/` algorithm code or change the author scripts' parameter VALUES (only paths/scheduling).
- Reinvent the generator or metrics.
- Start the correlation-aware contribution before IPC **and** TACO are verified.
- **Convert / "repair" FR079.graph** — it is the author's exact release file (see critical fact #6); fixing
  its info-matrix layout would diverge from the author. Leave it as-is; if S1 averaged numbers diverge, THEN revisit.
