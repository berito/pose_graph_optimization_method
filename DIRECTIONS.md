# DIRECTIONS — every attempt, and what became of it

**One row per direction tried. A direction that fails stays here.** The record is the point:
without it the next attempt has no way to know what was already ruled out, and a folder of
dead code looks like unfinished work rather than an answered question.

⚠ **This file is the code side of a rule the research repo already follows.**
`literature/thinking/DIRECTION.md` keeps refuted candidates in place — `C3 ❌ REFUTED` is
still there with its reason. Same discipline here, for the same reason: **the reason
outlives the idea.**

## The attempts

| # | direction | changed | tested against | outcome |
|---|---|---|---|---|
| **d1** | **correlation-aware IPC** (`dc_ipc/`) | the consensus decision: judge a *group* of correlated edges jointly instead of edge-by-edge | IPC, TACO, the `baselines/` robust back-ends, on correlated (grouped) outliers | ⚠ **tried — did not beat the original.** Which gate it failed and on what data is **not recorded**; see below |
| d2 | — | — | — | not started |

⛔ **d1's outcome is recorded here as the user reported it, and that is not good enough.**
`CLAUDE.md` defines a falsification ladder — **G1** do the baselines actually fail on
correlated outliers · **G2** with *oracle* group labels, does a group-joint decision beat
per-edge · **G3** does it survive a realistic, non-oracle correlation signal — and the
useful fact is **which gate stopped it**. G2 failing kills the idea outright; G3 failing
kills only this *signal* and leaves the idea open.

**Before d2 starts, fill that in** from `RUN_LOG.md` and `experiments/results/`. A direction
recorded as "did not work" teaches nothing; one recorded as "G2 held, G3 failed on
`<signal>`" is a starting point.

## How a direction is added, and how one dies

**A direction is a directory, never a branch.** Comparing two attempts needs the *same*
harness on the *same* data; across long-lived branches the harness drifts and the numbers
stop being comparable — which is the only reason to run more than one attempt.

1. **New attempt** → a new directory, built on the same base, with its own falsification
   ladder written down *before* it runs.
2. **Killed attempt** → it stays on disk. Add a `❌` header to its README naming the gate
   that killed it, and add its row here. **Do not delete it, do not branch it away.**
3. **The harness is never edited by a direction.** `experiments/` — datagen, configs,
   analysis, metrics — belongs to all of them. The moment one attempt changes the
   measurement, its numbers stop being comparable with its rivals'.
4. **`baselines/` is read-only.** It is vendored upstream code; a comparator that gets
   edited stops being a comparator.

**Branches keep the job they are good at:** a throwaway spike, and a tag at submission so
the state that produced the reported numbers is recoverable.

## ⚠ The layout question, still open

The four top-level directories are three different kinds of thing, and the names do not say
which:

| directory | what it actually is |
|---|---|
| `ipc/` | a **fork we patch** — ours, modified |
| `taco/` | a **comparator**, reimplemented on top of `ipc/` |
| `baselines/` | **vendored upstream**, never edited |
| `dc_ipc/` | **an attempt** — d1 |

A move to `directions/d1_.../` and a read-only `vendor/` was proposed on 2026-09-07 and
**not carried out**: it rewrites four `add_subdirectory` lines, two nested `CMakeLists.txt`,
two run scripts and the docs, and the build could not be verified from outside the
devcontainer at the time. **Do it in one commit, with a build immediately after** — not
piecemeal.

## Where the data lives

`experiments/datasets` is a **symlink** to
`../../pose_graph_optimization_experiments/datasets` — the one shared dataset folder, so
nothing is duplicated between repos.

⚠ It was **broken** until 2026-09-07: it pointed at `../../../datasets`
(`code_base/datasets`), which does not exist, so git reported twelve tracked dataset files
as deleted and the repo had no data at all. If those files show as deleted again, check the
symlink before checking anything else.
