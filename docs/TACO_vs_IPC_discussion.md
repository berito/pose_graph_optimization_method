# TACO over IPC — the math delta only (temp discussion)

> You already know IPC (subgraph objective + $\chi^2$ consensus test) and Switchable Constraints (SC).
> This file explains **only what TACO adds on top of IPC**, in math, step by step. Nothing else.
>
> *Math renders in VS Code's Markdown preview (KaTeX) and on GitHub. If symbols still look like raw
> text, open the preview pane: `Ctrl/Cmd + Shift + V`.*

---

## 0. The whole delta in one picture

IPC is a **single online test**: for each incoming loop closure, build a consistency subgraph,
optimize it, run one $\chi^2$ test, accept/reject — **decision is final**.

TACO keeps that loop but bolts on **two modules**, at two different time horizons:

```
          ONLINE (per loop)                 OFFLINE (every R loops)
  ┌──────────────────────────┐        ┌────────────────────────────┐
  │  Module A: kBL-IPC        │  ──▶   │  Module B: SR-SC            │
  │  = IPC, but the test      │  Ē_l   │  = SC + randomized voting,  │
  │    subgraph is BOUNDED    │ (batch)│    can UNDO earlier labels  │
  │    to k loops (IoU)       │        │    on a trusted subgraph    │
  └──────────────────────────┘        └────────────────────────────┘
        cheaper / faster                   recovers from mistakes
```

- **Module A (kBL-IPC)** changes *one thing* in IPC math: **how the test subgraph is chosen.**
- **Module B (SR-SC)** is *genuinely new* math: a retrospective pass that can **flip** decisions.

Everything below is just these two.

---

## 1. Module A — kBL-IPC: bound the test subgraph

### What IPC does (the only line we need)
IPC tests a candidate loop $e_{ab}$ by building the **minimal independent subgraph** $G(a,b)$ — every
previously accepted loop that *intersects* the candidate's node span, plus the spanning odometry.
Then it optimizes the subgraph and runs the $\chi^2$ test over **all** its edges.

**Problem (purely computational):** the intersecting set can grow toward the *whole graph* → the
"local" test becomes a global solve → slow (this is exactly the FRH 253 s blow-up).

### What "gather the accepted loops relevant to the candidate" actually means + how it's done

**What "relevant" means.** A loop closure $e_{ab}$ doesn't just connect poses $a$ and $b$ — it
**constrains the whole stretch of trajectory between them**, because nodes $a, a{+}1, \dots, b$ are
chained by odometry, so optimizing the loop **moves all of them**. Two loops are therefore *relevant
to each other* (geometrically coupled) when their node spans **share poses**: if loop $A$ covers
$\{10..20\}$ and loop $B$ covers $\{15..25\}$, both constrain $15..20$ — you cannot optimize one
without affecting the other. A loop over $\{200..210\}$ shares nothing → irrelevant.

> "Relevant accepted loops" = accepted loops whose node spans **overlap** the candidate's span, so
> the candidate's optimization actually interacts with them. The smallest self-contained cluster of
> such loops + their poses + connecting odometry is the **minimal independent subgraph** (*independent*
> = nothing outside it shares a constrained pose, so it can be optimized in isolation).

**How it's done — one cheap test, applied repeatedly.** Every loop is just a pair of node indices, so
"share poses" is an **integer interval-overlap test**:

$$
\text{overlap}(a,b,c,d) \;=\; \big(\max(a,c)\;\le\;\min(b,d)\big)
$$

e.g. $[10,20]$ vs $[15,25]$: $\max(10,15)=15\le\min(20,25)=20$ → **true**; vs $[200,210]$:
$\max(10,200)=200\le 20$ → **false**.

The gather procedure is *grow-and-rescan* (this builds IPC's minimal independent subgraph):

```
gatherRelevant(candidate [a,b], acceptedLoops):
    cluster_span ← [a, b]              # interval the subgraph currently covers
    relevant     ← []
    changed      ← true
    while changed:                     # keep going until a full pass adds nothing
        changed ← false
        for each loop [c,d] in acceptedLoops not already in 'relevant':
            if overlap(cluster_span, [c,d]):
                relevant.append([c,d])                       # pull it in
                cluster_span ← [min(span.start,c), max(span.end,d)]   # GROW the interval
                changed ← true                               # grew → rescan
    return relevant     # + odometry edges over cluster_span = minimal independent subgraph
```

The `while changed` rescan is the **transitive** part: loop A joins by overlapping the candidate, A
stretches the interval, now loop B overlaps the *stretched* interval and joins even though it never
touched the original candidate. That chaining is why the set has **no size bound**.

**Trace** — candidate $[10,20]$; accepted $[15,25],[23,40],[8,12],[200,210]$:

| Pass | cluster_span | test | result |
|---|---|---|---|
| 1 | $[10,20]$ | $[15,25]$? $15\le20$ ✓ | pull in → grow to $[10,25]$ |
| 1 | $[10,25]$ | $[23,40]$? $23\le25$ ✓ | pull in → grow to $[10,40]$ |
| 1 | $[10,40]$ | $[8,12]$? $10\le12$ ✓ | pull in → grow to $[8,40]$ |
| 1 | $[8,40]$ | $[200,210]$? $200\le40$ ✗ | skip |
| 2 | $[8,40]$ | rescan — nothing new | **stop** |

Gathered $\{[15,25],[23,40],[8,12]\}$, span $[8,40]$ ($[23,40]$ joined *transitively* — it never
overlapped the original $[10,20]$). Add odometry for nodes $8\ldots40$ → minimal independent subgraph.

**How kBL changes this gather step:** it drops the `while changed` growth loop entirely — **one pass,
no growing**: test each accepted loop's overlap against the **original** candidate only, score by
IoU, keep the top $k$, stop. Same overlap test; no transitive chaining; bounded by $k$. (So in the
trace, kBL with $k=2$ keeps the two highest-IoU loops and never pulls in $[23,40]$ transitively.)

### Why the author added kBL-IPC (the reasoning)
IPC's guarantee comes from testing against the **full** minimal independent subgraph — but that set
has **no upper bound on size**. On long trajectories with dense loop closures it keeps absorbing
intersecting loops until each "local" test is effectively a **global optimization**. The author's
goal was a method that stays **online / incremental** (test each loop as it arrives, in bounded
time). So the question he posed was: *do I really need all intersecting loops to decide, or do a few
most-relevant ones carry almost all the discriminative power?* IoU answers that — the loops that
overlap the candidate most are the ones whose geometry actually constrains it; the rest contribute
little to the $\chi^2$ verdict. Capping at $k$ makes the per-loop cost **constant** instead of
graph-dependent, trading a little robustness for a bounded, real-time-able test. (Table 5.5: on FRH
the $k$-cap drops runtime from 253 s to 45–64 s — that gap *is* the motivation.)

### The one change: pick $k$ loops instead of all of them

Define the **node span** of a loop $e_{ab}$ (with $a<b$):

$$
V(a,b) = \{\, x_i : a \le i \le b \,\}
$$

Score the candidate $e_{ab}$ against every accepted inlier loop $e_{cd} \in {}^{i}E_l$ by
**Intersection-over-Union**:

$$
\mathrm{IoU}(e_{ab}, e_{cd}) \;=\; \frac{\lvert V(a,b)\cap V(c,d)\rvert}{\lvert V(a,b)\cup V(c,d)\rvert}
\qquad\text{(Eq. 5.6)}
$$

Keep the **$k$ loops with the highest IoU** → set $E_l^{k}$. Build the *approximate* subgraph
$G^{k}(a,b)$ from those $k$ loops + the candidate + spanning odometry. **That's the entire modification.**

### Where kBL enters the IPC loop (the change is in ONE step only)

Recall the plain-IPC per-candidate cycle:

1. Candidate loop $e_{ab}$ arrives.
2. **Build the minimal independent subgraph** — gather the accepted loops relevant to the candidate.
3. Optimize that subgraph (PGO solve).
4. $\chi^2$ consistency check of the new solution against the subgraph's edges.
5. All pass → **accept**, add to ${}^{i}E_l$; any fail → **reject**, add to ${}^{o}E_l$, revert.
6. Go on to the next candidate.

kBL-IPC changes **only step 2**. Everything else is untouched:

| Step | Plain IPC | kBL-IPC |
|---|---|---|
| 1. candidate arrives | same | same |
| **2. build subgraph** | **ALL intersecting accepted loops** (no size bound) | **top-$k$ IoU intersecting loops only** |
| 3. optimize subgraph | same | same (smaller → faster) |
| 4. $\chi^2$ check | same | same (over the smaller edge set) |
| 5. accept / reject | same | same |
| 6. go on | same | same |

Why step 2 is the one worth touching: it's the part with **no size limit**. On a dense graph the set
of intersecting accepted loops keeps growing, so the subgraph optimized in step 3 swells toward the
whole graph and the "local" test becomes a near-global solve (the FRH 253 s blow-up). The $k$
highest-IoU loops are the ones actually constraining the same poses as the candidate; the rest barely
move the $\chi^2$ verdict, so dropping them caps the cost at little accuracy loss. Setting
$k=\infty$ ("keep all") reverts step 2 to plain IPC — so kBL-IPC **is** IPC at $k=\infty$, and finite
$k$ is a bounded approximation of the same subgraph. (The spanning **odometry** backbone is always
kept regardless of $k$, so $G^{k}$ stays connected and optimizable; only *which loop closures* go in
is trimmed.)

### Algorithm

```
# --- Alg. 6: findKBestLoops(candidate e_ab, accepted inliers ᶦE_l, k) ---
findKBestLoops(e_ab, ᶦE_l, k):
    scored ← []
    for each accepted loop e_cd in ᶦE_l:
        if V(a,b) ∩ V(c,d) ≠ ∅:                 # only loops that intersect the candidate
            scored.append( (IoU(e_ab, e_cd), e_cd) )
    sort scored by IoU descending
    return first k entries of scored            # E_l^k  (the k best "buddies")


# --- kBL-IPC per-candidate cycle (step 2 = the one new line) ---
on candidate loop e_ab:
    E_l^k ← findKBestLoops(e_ab, ᶦE_l, k)                       # STEP 2 — the ONLY change
    G^k   ← buildSubgraph(E_l^k, e_ab, spanning odometry E_o)   # k loops + candidate + odom backbone

    x*    ← optimize(G^k)            using Eq. 5.4              # STEP 3
    pass  ← χ²-test(x*, G^k)         using Eq. 5.5 (α=0.95)     # STEP 4  (all edges < χ²_{α,δ} ?)

    if pass:                                                    # STEP 5
        ᶦE_l ← ᶦE_l ∪ {e_ab}          # accept: add to inliers
        propagate x* to the rest of the graph
    else:
        ᵒE_l ← ᵒE_l ∪ {e_ab}          # reject: add to outliers
        revert subgraph nodes to pre-optimization state

    Ē_l ← Ē_l ∪ {e_ab}               # also queue into the unrevised batch (feeds SR-SC, §2)
    if |Ē_l| == R:  trigger SR-SC(Ē_l)                         # → Module B
```

The only line that differs from plain IPC is the `findKBestLoops` call in step 2: IPC would instead
take **every** intersecting loop in $G(a,b)$. Steps 3–5 are the identical optimize → $\chi^2$ →
accept/reject you already know.

### Everything after is byte-identical IPC math, just over $E^{k}$

Same weighted objective (odometry up-weighted by $s>1$, **no extra variables**):

$$
\sum_{(a,a+1)\in E_o} e_{a,a+1}^{\top}\,\big(s\,\Omega_{a,a+1}\big)\,e_{a,a+1}
\;+\;
\sum_{(a,b)\in E_l^{k}} e_{a,b}^{\top}\,\Omega_{a,b}\,e_{a,b}
\qquad\text{(Eq. 5.4)}
$$

Same single $\chi^2$ consensus test, just the edge set is $E^{k}$ instead of the full subgraph:

$$
e_{a,b}^{\top}\,\Omega_{a,b}\,e_{a,b} \;<\; \chi^2_{\alpha,\delta}
\qquad \forall\, e_{a,b}\in (E_o \cup {}^{i}E_l)\cap G^{k}
\qquad\text{(Eq. 5.5)}
$$

with confidence $\alpha = 0.95$ and error DoF $\delta$ ($\delta=3$ in 2D, $\delta=6$ in 3D) — both
unchanged from IPC.

### Why this is the *right* knob (the math property that matters)
- $k=\infty$ (keep all intersecting loops) $\iff$ **exactly plain IPC**. So kBL-IPC is a **strict
  generalization**: IPC is the $k=\infty$ corner.
- Smaller $k$ → fewer constraints in the test → faster, but the consensus check sees less evidence
  → slightly less robust.
- IoU is the principled selector: it ranks accepted loops by **how much trajectory they share** with
  the candidate, i.e. by how informative they are for *this* candidate's test. Top-$k$ by IoU keeps
  the most constraining edges and drops the near-irrelevant ones.
- Thesis default $k=2$ ("2BL-IPC"): $\approx 1.73\times$ faster than IPC, $+1.9\%$ F1, $-10.4\%$ SR.

> **One-line takeaway (Module A):** *same IPC test, but the subgraph is the top-$k$ IoU loops instead
> of all intersecting loops.* Pure approximation/speed knob, $k=\infty \equiv$ IPC.

---

## 2. Module B — SR-SC: a retrospective check that can flip decisions

This is the real addition. You know SC, so I only cover **what SR-SC adds on top of plain SC**.

### Why the author added SR-SC (the reasoning)
IPC — and kBL-IPC even more so — is **greedy and irreversible**: each loop is judged once, against
only the evidence available *at that moment*, and the accept/reject is **final**. Two failure modes
follow directly:
1. **Order dependence / premature decisions.** A correct loop arriving early, before enough
   supporting loops exist, can fail the test and be discarded forever; a bad loop accepted early
   poisons later tests. There is no mechanism in IPC to ever revisit it.
2. **The robustness the $k$-cap gave away.** kBL-IPC's bounded test sees less evidence, so it makes
   *more* of these early mistakes — measured as the $-10.4\%$ SR drop. Something has to **buy that
   back**, or bounding the subgraph isn't worth it.

So the author's reasoning was: keep the fast online test, but add a **second, slower pass that is
allowed to change its mind** once more evidence has accumulated — the "**check**" half of *Test And
Check*. Two design choices fall out of that goal:
- *Why SC and not just re-run IPC?* Re-running the hard accept/reject would just repeat the same
  brittle decision. SC makes the inlier/outlier status a **continuous, optimizable** variable
  $s_{ab}$, so a borderline loop can be **softly** pulled toward inlier or outlier by the whole batch
  jointly — a joint, reversible decision instead of a per-loop final one.
- *Why randomize + vote, not solve SC once?* A single SC solve is still anchored to kBL-IPC's
  possibly-wrong priors $\gamma$. Perturbing the priors and voting tests whether a loop's status is
  **stable** or an **artifact of one particular labelling** — that stability is precisely the signal
  that justifies *overturning* the original decision.
- *Why only on a trusted subgraph?* The recovery must run repeatedly and offline without stalling the
  online stream, so it is restricted to the minimal subgraph around the unrevised batch ($\approx$
  half size) — making "change your mind" cheap enough to do every $R$ loops.

### Recap anchor — plain SC (so the deltas are visible)
Each loop gets a switch $s_{ab}\in\mathbb{R}$, mapped $\Psi(s_{ab})\in[0,1]$ (linear, clamped).
Augmented error and prior penalty:

$$
\hat{e}_{ab} = \Psi(s_{ab})\cdot e(x_a,x_b) \qquad\text{(Eq. 5.7)}
$$

$$
p_{ab} = \lVert \gamma_{ab} - s_{ab}\rVert, \qquad \gamma_{ab}\in\{0,1\} \qquad\text{(Eq. 5.8)}
$$

$$
\text{SC cost} \;=\; \sum_{\text{odom}} e^{\top}\Omega e
\;+\; \sum \hat{e}_{ab}^{\top}\Omega\,\hat{e}_{ab}
\;+\; \sum p_{ab}\,\Omega_p\,p_{ab} \qquad\text{(Eq. 5.9)}
$$

Plain SC = optimize all poses **and** all switches **jointly, once, deterministically**, over the
**whole graph**. SR-SC keeps this cost but changes *three* things around it.

### Delta (a) — **Selective**: solve on a *minimal trusted subgraph*, not the whole graph

Split the accepted inliers into two sets:
- $\bar{E}_l$ — **unrevised**: loops kBL-IPC labelled but SR-SC hasn't checked yet (the current batch).
- ${}^{t}E_l = {}^{i}E_l \setminus \bar{E}_l$ — **trusted**: inliers already validated *and* revised in
  prior SR-SC runs; their labels are **held fixed** (not switchable here).

Build the **minimal trusted subgraph** $G^{T}$ = smallest subgraph that (i) contains all $\bar{E}_l$
edges + their vertices, and (ii) connects every such vertex pair using **only trusted or odometry
edges**. Found by **Dijkstra shortest paths**. Effect: the SC problem shrinks to $\approx$ **half**
the full size, and only the *batch* $\bar{E}_l$ switches are free variables — the trusted backbone
anchors the geometry.

Reduced cost actually optimized (only $\bar{E}_l$ carries switch + prior terms):

$$
\underbrace{\sum_{E_o\cap G^{T}} e^{\top}\Omega e \;+\; \sum_{{}^{t}E_l\cap G^{T}} e^{\top}\Omega e}_{\text{trusted backbone, switches FIXED}}
\;+\;
\underbrace{\sum_{\bar{E}_l}\big(\hat{e}_{ab}^{\top}\Omega_{ab}\hat{e}_{ab} + p_{ab}\Omega_p p_{ab}\big)}_{\text{batch, switches FREE}}
\qquad\text{(Eq. 5.12)}
$$

### Delta (b) — **Randomized**: perturb the priors and re-solve many times

Plain SC trusts the priors $\gamma_{ab}$ once. SR-SC runs the solve **$\texttt{maxIter}$ times**, and
each round:
- set priors from kBL-IPC labels: $\gamma_{ab}\leftarrow 1$ if $(a,b)\in {}^{i}E_l$ else $0$;
- **randomly flip a fraction $\varepsilon$** of those priors ($\texttt{randomFlip}(\varepsilon)$,
  $\varepsilon = 0.3$), and randomly fix some switches.

So each round optimizes Eq. 5.12 under a **different perturbed prior** $\gamma^{(r)}$.

### Delta (c) — **Voting**: aggregate the rounds into a robust label

Per round $r$, read out each batch switch and cast a binary vote:

$$
\text{vote}_{ab}^{(r)} = \mathbb{1}\!\big(s_{ab}^{(r)} > 0.5\big),
\qquad
\text{acc}_{ab} = \sum_{r=1}^{\texttt{maxIter}} \text{vote}_{ab}^{(r)}
$$

Final promotion by the **average** vote vs a threshold:

$$
\overline{s}_{ab} = \frac{\text{acc}_{ab}}{\texttt{maxIter}},
\qquad
(a,b)\ \text{promoted to inlier} \iff \overline{s}_{ab} > \texttt{inlierTh}\;(=0.7)
$$

Then ${}^{i}E_l \leftarrow {}^{i}E_l \cup \{\text{promoted}\}$, and **clear** $\bar{E}_l \leftarrow \varnothing$.

### Why randomized voting is mathematically better than one SC solve
A single SC solve inherits whatever (possibly wrong) prior $\gamma$ kBL-IPC handed it; a bad label can
drag a switch the wrong way. Averaging the binary outcome over **random perturbations of the prior**
is a Monte-Carlo estimate of *how stable* a loop's inlier status is under prior uncertainty:

$$
\overline{s}_{ab} \;\approx\; \mathbb{E}_{\gamma\,\sim\,\text{flip}(\varepsilon)}\Big[\,\mathbb{1}\big(s_{ab}(\gamma) > 0.5\big)\,\Big]
\;\approx\; P\big(\text{loop } ab \text{ is inlier} \mid \text{perturbed priors}\big)
$$

A true inlier comes out $s_{ab}>0.5$ **regardless** of which neighbours got flipped → high
$\overline{s}$ → promoted. A spurious one is sensitive to the prior → its votes scatter →
$\overline{s} < 0.7$ → not promoted. The threshold $\texttt{inlierTh}=0.7$ is "promote only if it
survives $\ge 70\%$ of perturbations." This is what lets SR-SC **undo** a wrong kBL-IPC/IPC decision
that a deterministic, final test never could.

> **One-line takeaway (Module B):** *plain SC, but solved repeatedly on a half-size trusted subgraph
> with the priors randomly flipped each round, then majority-voted* — turning a one-shot deterministic
> switch into a perturbation-averaged confidence.

---

## 3. How A and B connect (the only coupling)

- kBL-IPC tests each loop → appends it to $\bar{E}_l$ (whatever the label).
- When $\lvert\bar{E}_l\rvert$ reaches $R\,(=10)$: run SR-SC on $\bar{E}_l$ → promote/flip →
  ${}^{i}E_l$ updated → $\bar{E}_l \leftarrow \varnothing$.

That's it: kBL-IPC is the fast forward pass, SR-SC is the periodic correction pass. They can run in
parallel (SR-SC offline while the online stream continues).

---

## 4. What the additions actually buy (so the math has a price tag)

| Addition | Mathematical change vs IPC | Measured effect (thesis) |
|---|---|---|
| kBL-IPC ($k=2$) | subgraph = top-$k$ IoU loops, not all intersecting | **1.73× faster**, $+1.9\%$ F1, **$-10.4\%$ SR** |
| SR-SC (→ full TACO) | + randomized-voted SC recovery on trusted subgraph | **$+1.24\%$ SR, $+3\%$ Recall**, $\approx$ Precision, $+1.1\%$ time |
| both (2BL-TACO) | $k$-cap **and** recovery | same SR, **$+4\%$ F1, $+6\%$ Recall** vs 2BL-IPC |

Key reading: the **$k$-cap trades SR for speed**; **SR-SC buys SR/recall back** by un-doing mistakes.
Full TACO = "recover the robustness you'd lose from bounding, and then some."

---

## 5. The three equations to actually remember

1. **IoU subgraph selector** (Module A, the *only* new IPC-side math):
   $\mathrm{IoU} = \dfrac{\lvert V(a,b)\cap V(c,d)\rvert}{\lvert V(a,b)\cup V(c,d)\rvert}$ → keep top-$k$
   → $G^{k}$.  (with $k=\infty \equiv$ IPC)
2. **Reduced SC cost on the trusted subgraph** (Module B, Eq. 5.12): switches free only on $\bar{E}_l$.
3. **Perturbation-averaged voting rule:**
   $\overline{s}_{ab} = \frac{1}{\texttt{maxIter}}\sum_r \mathbb{1}(s_{ab}^{(r)}>0.5) > \texttt{inlierTh}
   \Rightarrow$ inlier.

Everything else in TACO is bookkeeping (the trusted/unrevised set split, the Dijkstra minimal graph,
the $R$-trigger). The *ideas* are exactly these three.
