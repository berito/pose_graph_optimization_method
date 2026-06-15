# Author-reported results — single comparison reference (IPC Ch.4 + TACO Ch.5)

**One place** for every result the author published, so our experiments can be diffed against them.
**Authoritative source:** the PhD thesis (Olivastri 2025), transcribed verbatim from
`docs/2025_Olivastri_PhD_Thesis_Robust_PGO.pdf` (Ch.4 = IPC pp.49–63, Ch.5 = TACO pp.65–90).
IPC Ch.4 numbers also match the ICRA-2024 paper (`docs/2024_Olivastri_IPC_*.pdf`).

**On graphs vs tables:** all *tabulated* values are reproduced exactly below. The papers also show
**line-plot figures** (IPC Figs 4.4/4.5; TACO Figs 5.8/5.9/5.11) whose per-point values are **not
printed** — only the discrete slices that the author tabulated (e.g. 50%/100%, or 20/40/60/80/100%)
are recoverable. Those tabulated slices ARE all captured here; the un-tabulated curve points cannot
be digitized from the text and are flagged where relevant (no values are invented). Higher = better
(↑) / lower = better (↓) as marked.

---

# PART A — IPC (Chapter 4 / ICRA 2024)

## A1. Table 4.1 — averaged over all 6 datasets × 10 trajectories, at 50% & 100% outliers
(F1 ↑, RPE [m] ↓, TIME [s] ↓.) *Note: the author tabulates only F1/RPE/TIME here; ATE appears only
in the Fig. 4.5 plot and is NOT given numerically.*

| Method | F1@50 ↑ | RPE@50 ↓ | TIME@50 ↓ | F1@100 ↑ | RPE@100 ↓ | TIME@100 ↓ |
|---|---|---|---|---|---|---|
| GNC    | 0.747 | 32.20  | 155.86   | 0.604 | 35.53  | 349.04    |
| ADAPT  | 0.873 | 1.867  | 5457.07  | 0.738 | 1.856  | 12576.27  |
| MAXMIX | 0.77  | 0.324  | 70.99    | 0.66  | 0.43   | 137.29    |
| DCS    | 0.796 | 26.12  | **4.58** | 0.66  | 37.684 | **8.28**  |
| GM     | 0.795 | 23.64  | 7.07     | 0.65  | 38.33  | 9.59      |
| HUBER  | 0.706 | 21.44  | 10.04    | 0.55  | 27.32  | 8.29      |
| PCM    | 0.53  | 25.01  | 8289.04  | 0.371 | 28.54  | 12655.41  |
| **IPC**| **0.91** | **0.05** | 321.37 | **0.89** | **0.068** | 442.733 |

## A2. Table 4.2 — IPC ACTxC (avg convergence time per constraint, s), per dataset vs outlier %
| Dataset | 20% | 40% | 60% | 80% | 100% |
|---|---|---|---|---|---|
| MIT   | 0.053 | 0.050 | 0.050 | 0.050 | 0.044 |
| INTEL | 0.056 | 0.051 | 0.058 | 0.054 | 0.056 |
| M3500 | 0.577 | 0.542 | 0.717 | 0.601 | 0.623 |
| CSAIL | 0.043 | 0.042 | 0.043 | 0.042 | 0.043 |
| FRH   | 0.098 | 0.095 | 0.100 | 0.098 | 0.109 |
| FR079 | 0.042 | 0.042 | 0.043 | 0.041 | 0.043 |

## A3. Figs 4.4 / 4.5 — qualitative trends only (per-point values NOT printed)
- Fig 4.4 (Precision/Recall/F1 vs 10–100%): MAXMIX & IPC highest precision (≈top), then PCM; GNC/DCS/GM
  precision drops <0.5 past 50% outliers. Recall ~flat for all; GM & DCS highest recall, then ADAPT & IPC
  (IPC recall consistently >80%). Only the 50%/100% F1 slices are tabulated (→ A1).
- Fig 4.5 (ATE/RPE/Time vs 10–100%): IPC clearly lowest ATE (~20–35 m vs ~50–80 m pack); IPC/MAXMIX/ADAPT
  best RPE; DCS/GM/HUBER cheapest, PCM/ADAPT most expensive. Only 50%/100% RPE & TIME slices tabulated (→ A1);
  ATE values not numerically reported.

---

# PART B — TACO (Chapter 5, T-RO submitted)

Methods: HUBER, DCS, PCM, GNC, MAXMIX (comparators) + IPC, 2BL-IPC, 2BL-TACO, TACO (author family).
Primary metric = **SR** (Success Rate ↑, run "successful" iff ATE < th_ATE = 0.75 m). PR = Precision ↑.

## B1. Table 5.1 — 2D datasets: SR / F1 / Precision vs outlier %
| Method | SR20 | F1·20 | PR20 | SR40 | F1·40 | PR40 | SR60 | F1·60 | PR60 | SR80 | F1·80 | PR80 | SR100 | F1·100 | PR100 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| HUBER    | 0.0  | 0.84 | 0.94 | 0.0  | 0.74 | 0.86 | 0.0  | 0.67 | 0.82 | 0.0  | 0.61 | 0.75 | 0.0  | 0.58 | 0.72 |
| DCS      | 0.24 | 0.90 | 0.83 | 0.2  | 0.83 | 0.71 | 0.22 | 0.77 | 0.62 | 0.2  | 0.71 | 0.55 | 0.22 | 0.67 | 0.50 |
| PCM      | 0.14 | 0.69 | 0.95 | 0.08 | 0.59 | 0.92 | 0.1  | 0.54 | 0.89 | 0.02 | 0.50 | 0.90 | 0.02 | 0.40 | 0.91 |
| GNC      | 0.0  | 0.90 | 0.89 | 0.0  | 0.80 | 0.76 | 0.0  | 0.71 | 0.65 | 0.0  | 0.66 | 0.59 | 0.0  | 0.63 | 0.56 |
| MAXMIX   | 0.08 | 0.82 | 1.00 | 0.04 | 0.77 | 1.00 | 0.0  | 0.69 | 0.99 | 0.0  | 0.64 | 0.99 | 0.0  | 0.64 | 0.99 |
| IPC      | 0.5  | 0.94 | 1.00 | 0.5  | 0.92 | 0.99 | 0.32 | 0.90 | 0.99 | 0.3  | 0.90 | 0.99 | 0.32 | 0.92 | 0.98 |
| 2BL-IPC  | 0.3  | 0.94 | 0.99 | 0.32 | 0.95 | 0.99 | 0.28 | 0.94 | 0.98 | 0.22 | 0.95 | 0.98 | 0.22 | 0.92 | 0.96 |
| 2BL-TACO | 0.32 | 0.97 | 0.99 | 0.32 | 0.97 | 0.99 | 0.3  | 0.97 | 0.98 | 0.22 | 0.96 | 0.98 | 0.22 | 0.94 | 0.97 |
| **TACO** | 0.52 | 0.97 | 1.00 | 0.44 | 0.97 | 0.99 | 0.36 | 0.93 | 0.99 | 0.36 | 0.94 | 0.99 | 0.38 | 0.93 | 0.98 |

## B2. Table 5.2 — Visual SLAM datasets: SR / F1 / Precision vs outlier %
*(corrected: PCM @60% SR = 0.33, was mis-transcribed 0.13 earlier.)*

| Method | SR20 | F1·20 | PR20 | SR40 | F1·40 | PR40 | SR60 | F1·60 | PR60 | SR80 | F1·80 | PR80 | SR100 | F1·100 | PR100 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| HUBER    | 0.10 | 0.92 | 0.86 | 0.03 | 0.86 | 0.77 | 0.00 | 0.82 | 0.72 | 0.00 | 0.79 | 0.69 | 0.00 | 0.78 | 0.68 |
| DCS      | 0.57 | 0.97 | 0.95 | 0.23 | 0.94 | 0.92 | 0.13 | 0.90 | 0.90 | 0.07 | 0.86 | 0.88 | 0.07 | 0.83 | 0.86 |
| PCM      | 0.47 | 0.99 | 0.97 | 0.27 | 0.98 | 0.96 | 0.33 | 0.96 | 0.95 | 0.27 | 0.97 | 0.94 | 0.13 | 0.93 | 0.92 |
| GNC      | 0.13 | 0.92 | 0.86 | 0.00 | 0.86 | 0.76 | 0.0  | 0.80 | 0.67 | 0.00 | 0.76 | 0.61 | 0.0  | 0.71 | 0.56 |
| MAXMIX   | 0.27 | 0.96 | 0.95 | 0.27 | 0.95 | 0.95 | 0.13 | 0.89 | 0.96 | 0.27 | 0.92 | 0.97 | 0.17 | 0.94 | 0.96 |
| IPC      | 0.90 | 0.98 | 0.99 | 0.93 | 0.99 | 0.99 | 0.93 | 0.99 | 0.99 | 0.87 | 0.98 | 0.99 | 0.93 | 0.98 | 0.99 |
| 2BL-IPC  | 0.47 | 0.91 | 0.98 | 0.43 | 0.90 | 0.98 | 0.43 | 0.89 | 0.97 | 0.40 | 0.81 | 0.96 | 0.33 | 0.80 | 0.97 |
| 2BL-TACO | 0.50 | 0.97 | 0.98 | 0.40 | 0.94 | 0.98 | 0.43 | 0.96 | 0.98 | 0.40 | 0.95 | 0.98 | 0.30 | 0.93 | 0.97 |
| **TACO** | 0.90 | 0.99 | 0.99 | 0.92 | 0.99 | 0.99 | 0.93 | 0.99 | 0.99 | 0.89 | 0.98 | 0.99 | 0.90 | 0.99 | 0.99 |

## B3. Table 5.3 — 2D datasets: ATE [m] ↓ / RPE [m] ↓ at 50% & 100%
| Method | ATE@50 | RPE@50 | ATE@100 | RPE@100 |
|---|---|---|---|---|
| HUBER    | 52.19 | 24.18 | 53.55 | 27.67 |
| DCS      | 43.48 | 29.03 | 55.10 | 42.64 |
| PCM      | 43.50 | 27.32 | 56.15 | 31.32 |
| GNC      | 43.86 | 34.36 | 48.56 | 37.63 |
| MAXMIX   | 41.31 | 0.35  | 43.95 | 0.47  |
| IPC      | **12.49** | 0.03 | 22.85 | 0.03 |
| 2BL-IPC  | 21.27 | 0.08  | 29.06 | 0.10  |
| 2BL-TACO | 15.49 | 0.11  | 26.70 | 0.11  |
| **TACO** | 16.97 | **0.06** | **21.47** | **0.04** |

## B4. Table 5.4 — Visual SLAM datasets: ATE [m] ↓ / RPE [m] ↓ at 50% & 100%
| Method | ATE@50 | RPE@50 | ATE@100 | RPE@100 |
|---|---|---|---|---|
| HUBER    | 119.10 | 2.56  | 121.39 | 2.68  |
| DCS      | 59.61  | 1.93  | 61.39  | 1.72  |
| PCM      | 80.68  | 1.94  | 83.45  | 2.42  |
| GNC      | 118.93 | 10.95 | 120.92 | 11.27 |
| MAXMIX   | 119.16 | **0.25** | 121.49 | **0.61** |
| IPC      | 17.28  | 2.11  | 9.05   | 2.04  |
| 2BL-IPC  | 63.75  | 2.49  | 92.01  | 2.61  |
| 2BL-TACO | 50.75  | 2.46  | 70.00  | 2.46  |
| **TACO** | **11.66** | 2.07 | **8.99** | 2.05 |

## B5. Table 5.5 — Runtime [s] per dataset (averaged over all outlier %)
*(corrected: GNC FR079 = 25.63, was mis-transcribed 23.63 earlier.)*

| Method | MIT | INTEL | CSAIL | FRH | FR079 | FR1_DESK | KITTI_00 | KITTI_05 |
|---|---|---|---|---|---|---|---|---|
| HUBER    | 0.17 | 0.64   | 1.22  | 7.48   | 1.10  | 0.46 | 11.34  | 5.27   |
| DCS      | 0.24 | 5.61   | 2.07  | 3.39   | 3.02  | 0.34 | 1.89   | 1.14   |
| PCM      | 9.19 | 667.56 | 22.51 | 381.64 | 33.82 | 8.98 | 296.72 | 105.51 |
| GNC      | 17.43| 268.80 | 51.19 | 129.88 | 25.63 | 11.00| 302.79 | 159.56 |
| MAXMIX   | 2.91 | 43.72  | 2.52  | 23.78  | 1.96  | 0.58 | 11.09  | 4.89   |
| IPC      | 0.67 | 20.14  | 6.25  | 255.84 | 8.57  | 1.12 | 33.59  | 9.17   |
| 2BL-IPC  | 0.49 | 18.22  | 5.56  | 45.28  | 5.19  | 0.83 | 28.44  | 6.57   |
| 2BL-TACO | 1.27 | 19.59  | 6.68  | 64.08  | 7.25  | 1.12 | 33.39  | 9.97   |
| **TACO** | 1.47 | 20.39  | 8.20  | 253.32 | 10.51 | 1.33 | 52.92  | 12.99  |

## B6. Narrative delta numbers (§5.4.4–5.4.6) — not in any table
- **2BL-IPC vs IPC:** ~**1.73× faster**, **+1.9% F1**, **−10.4% SR**.
- **TACO vs IPC:** **+1.24% SR**, **+3.03% Recall**, ≈ equal Precision, **+1.14%** time overhead.
- **2BL-TACO vs 2BL-IPC:** same SR, **+4.1% F1**, **+5.96% Recall**, **+0.47% Precision**, **+1.31%** time.
- **Avg SR over all datasets:** TACO **>55%**, 2BL-TACO **>35%**, all non-TACO methods **<20%** (closest = DCS).
- Minimal-trusted-subgraph reduces problem size to **≈ half** (Fig 5.10).

## B7. Fig 5.7 — ATE/RPE under high distortion (illustrative; shows why ATE is unreliable)
| Trajectory | variant | ATE [m] | RPE [m] |
|---|---|---|---|
| KITTI_00 | (a2) less distorted | 191.05 | 3.48 |
| KITTI_00 | (a3) collapsed | 140.65 | 0.39 |
| KITTI_05 | (b2) less distorted | 151.1 | 2.66 |
| KITTI_05 | (b3) collapsed | 110.8 | 0.25 |

(The qualitatively-better (a2)/(b2) score *worse* ATE/RPE than the collapsed (a3)/(b3) — the author's
argument for using SR over raw ATE.)

---

# PART C — How to use this for our comparison

- **Our S1 IPC** (random outliers) → diff against **A1** (F1/RPE@50,100) + **A2** (ACTxC) + **A3** trends.
- **Our S2 TACO/2BL-IPC** → diff against **B1/B2** (SR/F1/PR), **B3/B4** (ATE/RPE), **B5** (runtime), **B6** deltas.
  Priority order (per `TACO_experiment_setup.md` §9): reproduce **2BL-IPC** row of B1 with SR-SC off, then **TACO**.
- Verdict metrics: **SR + P/R/F1** (B1/B2). ATE/RPE (B3/B4) are secondary — author shows they're unreliable (B7).
- For the per-outlier-% *curve* points not in any table, the only author values are the tabulated columns above;
  the figures themselves are in the thesis PDF (Figs 4.4/4.5, 5.8/5.9/5.11) for visual comparison.

Hyperparameters that produced these (Ch.5 §5.4 p.79): s = 10 (2D) / 30 (3D), R = 10, maxIter = 20,
ε = 0.3, inlierTh = 0.7, switch-cutoff 0.5, α = 0.95, th_ATE = 0.75; g2o + Dog-Leg; single Xeon Gold 5220.
