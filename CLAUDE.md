# pose_graph_optimization_math — the concepts, made visual

**What this is:** the mathematics of the papers, drawn. A concept read once is half-held; the
same concept with a slider on its parameter is held properly — you see *what the parameter
does*, not what the sentence said it does.

⚠ **This is not an experiment, and nothing here produces a thesis number.** Benchmarks and
baselines live in `../pose_graph_optimization_experiments/`; the contribution lives in
`../pose_graph_optimization_method/`. A figure from here may appear in the thesis **as an
illustration, never as evidence.**

## On session start

1. **Read [`PLAN.md`](PLAN.md)** — four subjects, each with *the question it settles* and
   what counts as understood. **Build one at a time**; four half-finished figures teach
   nothing.
2. Check which are done: `ls src/`.

## ⭐ The maths comes from the notes, not from memory

Every figure names the paper and the equation it draws, and the note it came from —
`research/pose_graph_optimization/literature/reading/DISCUSSION_<family>.md`. **A wrong
picture must be traceable to a wrong reading, not to an invention.**

⚠ This is the same provenance rule as `research/system/CONDUCT.md` § 1, and it bites harder
here: a plot is persuasive whether or not it is right, and an invented constant produces a
curve that looks exactly as convincing as a correct one.

## ⭐ A figure earns its place by settling a question

**If a picture would not change what you believe, it does not get built.** Each item in
`PLAN.md` states its question first and the drawing second — deliberately, so a figure
cannot be justified after the fact by being pretty.

The test for "done" is not that the plot renders. It is written per subject in `PLAN.md`,
and it is always a sentence you can say without looking anything up.

## Layout

| | |
|---|---|
| `src/` | one module per subject — `m1_barron_alpha`, `m2_black_rangarajan`, … |
| `out/` | figures. **Generated, gitignored** — regenerate, never commit |
| `PLAN.md` | what to build, and what each has to settle |

**Every figure must be reproducible from one command.** A plot that exists only in a
notebook someone ran once is not a result; it is a memory.

## Do NOT

- **Do not commit figures.** `out/` is gitignored — they are regenerated.
- **Do not put benchmark numbers here**, or compare methods for performance. That is
  `../pose_graph_optimization_experiments/`. This repo answers *what is this thing doing*.
- **Do not commit** without an explicit yes — `research/system/CONDUCT.md` § 3.
- **Do not invent a constant, a parameter range or an equation to make a curve look right.**
  Take it from the note, or stop and ask.
