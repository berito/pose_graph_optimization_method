# PLAN — what to build here, and what each figure has to settle

**A figure earns its place by settling a question the note could only assert.** Every item
below names the question first, then the picture. If a picture would not change what you
believe, it does not get built.

⚠ **The maths comes from the notes, not from memory.** Each item cites the note it is
drawn from — `research/pose_graph_optimization/literature/reading/DISCUSSION_<family>.md`.
A wrong picture must be traceable to a wrong reading, not to an invention.

⭐ **These are not experiments.** No number here goes in the thesis. The output is an
intuition and a figure, and the figure may end up in the thesis *as an illustration* —
never as evidence.

---

## M1 · Barron's α — the corpus as one surface

**From** `DISCUSSION_m_estimators.md` · `P095` Barron 2019.

**The question.** The kernel family is written up as a list — Huber, Cauchy, Geman-McClure,
Welsch — and the note claims they are **points on one curve** with a single shape
parameter. Is that a genuine unification, or a re-parameterisation that hides the choice?

**Build.** One α slider over the loss `ρ(r; α, c)`, and beside it three panels that change
together: the **loss**, its **influence function** `ψ = ρ′`, and the **weight** `ω = ψ/r`.

**Understood when** you can say, without looking it up, what α = 2, 1, 0, −2 and −∞ each
give, and *why the weight panel is the one that matters* — the note's whole argument is
about weights, not losses.

⭐ **The specific thing to look for.** The note records that maximising correntropy
`Σ exp(−e²/2σ²)` **is** minimising the Welsch loss — Barron's **α = −∞**. So a method
published as an information-theoretic idea lands on this same curve. The figure should make
that coincidence visible rather than reported.

---

## M2 · Black–Rangarajan — where the weight comes from

**From** the foundation of `DISCUSSION_m_estimators.md`, and the axis `DISCUSSION_gated_weight.md` turns on.

**The question.** `Σρ(r)` and `Σ[ω r² + Ψ(ω)]` are said to have the same minimiser. If that
is true, **every** robust method has per-measurement weights — and the interesting question
stops being *"does it weight?"* and becomes *"is the weight derived or solved for?"*

**Build.** The robust cost and the augmented cost side by side on the same residuals, with
the outer-loop iterates plotted on both. Add the penalty `Ψ(ω)` on its own axis: it is the
term that stops every weight going to zero, and it is invisible in the `ρ` form.

**Understood when** you can point at where the weight *appears* in the second form and
show that it was implicit in the first.

⚠ `P144` states this as a theorem **with preconditions**. The figure must show a case where
the preconditions hold — and, if cheap, one where they do not.

---

## M3 · The schedule — annealing's only open question

**From** `DISCUSSION_annealing.md`, Topics 2 and 4.

**The question.** The note is unusually precise here, and the picture has to carry two
claims that sound contradictory:

> *"Don't solve the hard problem. Solve an easy one, then deform it into the hard one,
> tracking the solution."*

> *"Annealing adds no robustness. It removes the dependence on where you started."*

**Build.** A 1-D or 2-D cost with several valleys. Animate the width from enormous to final,
tracking the minimiser from the previous step. Then run three schedules on the same problem:
too fast · the paper's · too slow — and mark which minimum each lands in.

**Understood when** the figure shows that the **final** cost is identical in all three runs,
and only the *arrival point* differs. That is the claim the note calls *"the single most
misread thing about the family"*, and prose has not made it stick.

⭐ **And it is where the field disagrees:** *"how far do you shrink at each step, and when do
you stop?"* — five years of papers are answers to that one question. The figure is the place
to feel why it is hard.

---

## M4 · Solved-for versus derived — the axis between two families

**From** `DISCUSSION_gated_weight.md`'s membership test.

**The question.** The two notes are separated by one test: *is the weight an unknown the
solver returns, and is there one per measurement?* Both produce a number per edge. **What
actually differs?**

**Build.** One graph, one outlier. Run a kernel (weight **derived** from the residual each
iteration) and a switch (weight **solved for** alongside the poses). Plot both weight
trajectories against iteration, and the trajectory error beside them.

**Understood when** you can see the difference the notes argue: a derived weight is a
function of the current residual and cannot disagree with it, while a solved weight is a
free variable that the rest of the graph can push around — *"the influence of outliers is
merely reduced, but not removed."*

⭐ **This is the one that may feed `DIRECTION.md`.** M1–M3 explain the field; this one draws
the boundary the thesis sits on, so it is the figure most likely to become an argument
rather than an illustration.

---

## Order, and why

**M1 → M2 → M4 → M3.** M1 gives the vocabulary. M2 gives the weight, which M4 then needs.
M3 is independent and can move earlier if the annealing note is being revised.

⚠ **One at a time, and stop when it is understood.** Four half-built visualisations teach
nothing; the deliverable is the intuition, and it is reached one figure at a time.

## What this repo is not for

- **Benchmarks, baselines, comparisons** → `../pose_graph_optimization_experiments/`
- **The method being built** → `../pose_graph_optimization_method/`
- **The reading, the notes, the register, the planning for all three** →
  `research/pose_graph_optimization/`
