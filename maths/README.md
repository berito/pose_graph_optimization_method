# pose_graph_optimization_math

**The maths of the papers, made visual. The objective is understanding, not results.**

A concept from a paper is read once and half-held. The same concept with a slider on its
parameter is held properly: you see *what the parameter does*, not what the sentence said
it does. That is the whole purpose of this repo.

⚠ **This is not an experiment.** Nothing here produces a number that goes in the thesis.
Benchmarks, baselines and results live in
[`pose_graph_optimization_experiments/`](../pose_graph_optimization_experiments); the
method being built lives in
[`pose_graph_optimization_method/`](../pose_graph_optimization_method). This repo answers
"what is this thing actually doing", and its output is a figure and an intuition.

## The three siblings

| repo | what it is |
|---|---|
| `pose_graph_optimization_experiments/` | other people's code — cloned, pinned, built, run |
| `pose_graph_optimization_math/` | ⭐ **here** — the concepts, visualised |
| `pose_graph_optimization_method/` | the contribution being built (was `robust_pgo`) |

The reading and the planning for all three live in the research repo, under
`research/pose_graph_optimization/`.

## What to build — the plan is [`PLAN.md`](PLAN.md)

Four subjects, each with the question it settles and what counts as understood. Summary:

Taken from the families already written up. Each is one parameter and one picture.

| concept | the paper | what changing the parameter should show |
|---|---|---|
| **Barron's general loss** | `P095` | one α sweeping through L2 → Charbonnier → Cauchy → Geman-McClure → Welsch. ⭐ The families in the corpus are **points on this curve**, and seeing them as one surface is the point |
| **Black–Rangarajan duality** | the foundation of `m_estimators` | a robust cost `Σρ(r)` beside `Σ[ω r² + Ψ(ω)]` — the same minimum reached two ways, and where the weight comes from |
| **GNC / annealing schedules** | `annealing` family | a control parameter deforming a convex surrogate into the true non-convex cost, and what a bad schedule does to the minimum it lands in |
| **switch vs kernel** | `gated_weight` vs `m_estimators` | a weight **solved for** against a weight **derived** — the distinction the two notes turn on, shown rather than argued |


## Running

Not yet written. Python, and every figure should be reproducible from a single command.

⭐ **Order: M1 → M2 → M4 → M3**, one at a time. Four half-built figures teach nothing.
