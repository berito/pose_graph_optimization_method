# TASKS — the work of this repo

⭐ **This file holds the STEPS. The board holds the task.**
`research/pose_graph_optimization/planning/WEEK.md` says *that* this work exists and whether
it is moving; **what to build lives here**, and *why each thing is worth building* lives in
[`PLAN.md`](PLAN.md).

| file | answers |
|---|---|
| the board, in `research/` | is this happening at all, and against what else |
| **`TASKS.md`** (here) | what to do next |
| [`PLAN.md`](PLAN.md) | what question each figure settles, and what counts as understood |

⚠ **One home each.** Do not restate the board here; do not restate `PLAN.md`'s reasoning here.

---

## M1 · Barron's α — the corpus as one surface

- [ ] `ρ(r; α, c)` with an α control, three panels changing together: **loss · influence
      `ψ = ρ′` · weight `ω = ψ/r`**
- [ ] mark the named members on the α axis — L2 · Charbonnier · Cauchy · Geman-McClure · Welsch
- [ ] ⭐ show that **maximising correntropy is minimising Welsch** (α = −∞) — the note records
      this, and it is the thing that makes the "one curve" claim more than tidy
- [ ] the check from `PLAN.md`: can you say what each α gives, and why the **weight** panel is
      the one that matters, without looking it up?

## M2 · Black–Rangarajan — where the weight comes from

- [ ] `Σρ(r)` and `Σ[ω r² + Ψ(ω)]` on the same residuals, outer-loop iterates plotted on both
- [ ] `Ψ(ω)` on its own axis — the term that stops every weight going to zero, invisible in
      the `ρ` form
- [ ] ⚠ `P144` states this **with preconditions**. Draw a case where they hold; if cheap, one
      where they do not

## M4 · Solved-for versus derived — the axis between two families

- [ ] one graph, one outlier: a kernel (weight **derived** each iteration) beside a switch
      (weight **solved for** with the poses)
- [ ] both weight trajectories against iteration, with the trajectory error beside them
- [ ] ⭐ the point to make visible: a derived weight **cannot disagree with its residual**; a
      solved one can be pushed by the rest of the graph

## M3 · The schedule — annealing's only open question

- [ ] a cost with several valleys; animate the width from enormous to final, tracking the
      minimiser from the previous step
- [ ] three schedules on the same problem — too fast · the paper's · too slow — marking which
      minimum each reaches
- [ ] ⭐ the claim the note calls *"the single most misread thing about the family"*: the
      **final cost is identical** in all three; only the arrival point differs

---

## Order

**M1 → M2 → M4 → M3.** M1 gives the vocabulary; M2 gives the weight that M4 needs; M3 is
independent. ⚠ **One at a time** — four half-built figures teach nothing.

⭐ **M4 is the one that may feed `DIRECTION.md`.** The others explain the field; M4 draws the
boundary the thesis sits on, so it is the figure most likely to become an argument rather
than an illustration.
