# Parameters: which belong to IPC vs TACO

Reference for the config keys in `ipc/cfg/*.yaml` — which method each belongs to, and
(critically) which ones the **IPC binary actually uses**. Sources: `docs/IPC.md`, `docs/TACO.md`,
and the IPC source in `ipc/`.

## IPC parameters (used by the IPC algorithm)
| key | meaning | used in code |
|---|---|---|
| `s_factor` (S) | odometry information scaling | ✅ `consensus.cpp` (`robustifyVoters`) |
| `fast_reject_th` | χ² threshold, first check | ✅ `consensus.cpp` |
| `slow_reject_th` | χ² threshold, second check | ✅ `consensus.cpp` |
| `fast_reject_iter_base` | optimizer iters, first check | ✅ |
| `slow_reject_iter_base` | optimizer iters, second check | ✅ |
| `canonic_inliers` | # true loop closures (for P/R) | ✅ |
| `ground_truth`, `dataset`, `output`, `name`, `visualize` | I/O | ✅ |

IPC paper values: **S = 3**, χ² confidence **α = 0.95**. IPC accepts a loop only if it is
consistent with **all** edges in the bounded subgraph (no `k`, no recovery).

## TACO parameters (TACO functionality — thesis Ch.5)
TACO = **kBL-IPC** (cap the consistency test to the `k` best-IoU previously-accepted loops)
**+ SR-SC** (a randomized-voting Switchable-Constraints error-recovery pass).
| key | meaning | TACO module |
|---|---|---|
| `k_buddies` (k) | # best-IoU loops kept in the test subgraph | kBL-IPC |
| `use_best_k_buddies` | switch for the kBL subset | kBL-IPC |
| `use_recovery` | run the recovery pass | SR-SC |

TACO values: **s = 10 (2D) / 30 (3D)**, **k = 2** default (swept {2,3,5,10}); `k = ∞` ≡ plain IPC.

## ⚠️ Critical: the TACO params are INERT in the IPC binary
`ipc/src/utils.cpp:332-334` **reads** `use_best_k_buddies`, `k_buddies`, `use_recovery` into the
config struct (`ipc/include/ipc/utils.hpp:35-37`), but **nothing in the IPC algorithm ever uses
them** — `consensus.cpp` only consumes `s_factor` + the χ² thresholds + iter bases. There is no
`_k_buddies` / `_use_recovery` member. They are placeholder fields.

**Consequence:** running the IPC binary with `k_buddies=2, use_recovery=true` (as the author's
`ipc/bash/ipc_experiments_2D.sh` does) changes nothing — it runs **plain IPC**. TACO's real
kBL/SR-SC logic is in the **unreleased TACO code**, NOT in this binary. So our S1 run with those
flags = plain IPC at the script's `s=10, th=10.64` (= the author's **"IPC_S10"** comparator),
**not** TACO.

## Verification implication
- Author's committed `ipc_experiments_2D.sh` → `s=10, th=10.64` → reproduces the **thesis "IPC_S10"**
  (the IPC baseline in the TACO chapter).
- The **2024 ICRA paper** IPC used **s=3** (no recovery/k). Different target — choose deliberately.
- When we implement TACO ourselves (S2), `k_buddies`/`use_recovery` become live (in OUR TACO code),
  and these values matter.
