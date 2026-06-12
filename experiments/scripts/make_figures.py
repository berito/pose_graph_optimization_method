#!/usr/bin/env python3
# Plot IPC P/R/F1 and ATE/RPE vs outlier % into experiments/figures/.
# Usage: python3 experiments/scripts/make_figures.py [TAG]   (TAG default IPC_S3)
import glob, statistics as st, sys, os
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

TAG = sys.argv[1] if len(sys.argv) > 1 else "IPC_S3"
DS = ["CSAIL","FR079","FRH","MIT","INTEL","M3500"]; RATES = list(range(10,101,10))
OUT = "experiments/figures/ipc"; os.makedirs(OUT, exist_ok=True)
LBL_OURS="our replication"; LBL_PAPER="paper (reported)"
def f1(p,r): return 0.0 if p+r==0 else 2*p*r/(p+r)

PR = {d:{} for d in DS}; TE = {d:{} for d in DS}
for d in DS:
    for r in RATES:
        pr=[tuple(float(x) for x in open(f).read().split("\n")[0].split()[:2])
            for f in glob.glob(f"experiments/results/IPC/{d}/*/{TAG}/{r}/*.PR")]
        if pr: PR[d][r]=(st.mean(p for p,_ in pr), st.mean(rr for _,rr in pr))
        te=[]
        for f in glob.glob(f"experiments/results/IPC/{d}/*/{TAG}/{r}/*.TE"):
            p=open(f).read().split()
            if len(p)>=6: te.append((float(p[1]),float(p[5])))
        if te: TE[d][r]=(st.mean(a for a,_ in te), st.mean(rr for _,rr in te))

def col(D,d,i,isf1=False): return [(f1(*D[d][r]) if isf1 else D[d][r][i]) if r in D[d] else float('nan') for r in RATES]
def avg(D,i,isf1=False): return [st.mean([(f1(*D[d][r]) if isf1 else D[d][r][i]) for d in DS if r in D[d]]) for r in RATES]

# Paper IPC reference, approx (50/100 = exact Table I)
PCURVE = {"Precision":[1.00,0.99,0.99,0.98,0.98,0.98,0.97,0.97,0.97,0.97],
          "Recall":  [0.90,0.89,0.88,0.87,0.86,0.86,0.85,0.85,0.84,0.84],
          "F1":      [0.93,0.93,0.92,0.92,0.91,0.91,0.90,0.90,0.89,0.89],
          "ATE [m]": [22,23,24,25,26,28,30,31,32,34],
          "RPE [m]": [0.04,0.04,0.045,0.048,0.05,0.055,0.058,0.062,0.065,0.068]}

def panels(metrics, getD, ylim, fname, title, per_dataset, show_diff=False):
    fig, ax = plt.subplots(1, len(metrics), figsize=(5.3*len(metrics),4.4))
    if len(metrics)==1: ax=[ax]
    for k,(t,i,isf1) in enumerate(metrics):
        a=ax[k]; ours=avg(getD,i,isf1); paper=PCURVE[t]
        if per_dataset:
            for d in DS: a.plot(RATES, col(getD,d,i,isf1), marker='.',lw=1,alpha=0.5,label=d)
        a.plot(RATES, ours, 'k-o', lw=2.5, label=LBL_OURS)
        a.plot(RATES, paper, 'r--s', lw=2, label=LBL_PAPER)
        if show_diff:
            fmt="{:+.0f}" if t.startswith("ATE") else "{:+.3f}" if t.startswith("RPE") else "{:+.2f}"
            a.fill_between(RATES, ours, paper, color='gray', alpha=0.12)
            for xr,o,p in zip(RATES, ours, paper):
                a.annotate(fmt.format(o-p), (xr,(o+p)/2), fontsize=7, color='dimgray', ha='center', va='center')
        a.set_title(f"{t} vs outlier %"); a.set_xlabel("outliers [%]"); a.grid(alpha=0.3)
        if ylim: a.set_ylim(*ylim)
        a.legend(fontsize=8, loc='upper left')
    fig.suptitle(title); fig.tight_layout(); fig.savefig(f"{OUT}/{fname}.png", dpi=110); plt.close(fig)

PRF1=[("Precision",0,False),("Recall",1,False),("F1",0,True)]
ATERPE=[("ATE [m]",0,False),("RPE [m]",1,False)]
panels(PRF1, PR, (0,1.02), f"{TAG}_per_dataset_PRF1", f"{TAG} — P/R/F1 per dataset", True)
panels(ATERPE, TE, None, f"{TAG}_per_dataset_ATE_RPE", f"{TAG} — ATE/RPE per dataset", True)
panels(PRF1, PR, (0,1.02), f"{TAG}_avg_vs_paper_PRF1", f"{TAG} — our replication vs paper (P/R/F1; numbers = replication − paper)", False, show_diff=True)
panels(ATERPE, TE, None, f"{TAG}_avg_vs_paper_ATE_RPE", f"{TAG} — our replication vs paper (ATE/RPE; numbers = replication − paper)", False, show_diff=True)

# Head-to-head bars: ONLY exact Table-I metrics (F1, RPE) at 50% & 100%
def ar(D,rate,i,isf1=False): return st.mean([(f1(*D[d][rate]) if isf1 else D[d][rate][i]) for d in DS if rate in D[d]])
fige, axe = plt.subplots(1,2, figsize=(10,4.5)); x=np.arange(2); w=0.34
for j,(name,ours,paper,fmt) in enumerate([
    ("F1  (higher = better)",  [ar(PR,50,0,True),ar(PR,100,0,True)], [0.91,0.89],  "{:.2f}"),
    ("RPE [m]  (lower = better)", [ar(TE,50,1),ar(TE,100,1)],        [0.05,0.068], "{:.3f}")]):
    a=axe[j]
    a.bar(x-w/2, ours, w, color="black", label=LBL_OURS)
    a.bar(x+w/2, paper, w, color="crimson", label=LBL_PAPER)
    for i,(o,p) in enumerate(zip(ours,paper)):
        a.text(i-w/2,o,fmt.format(o),ha="center",va="bottom",fontsize=11)
        a.text(i+w/2,p,fmt.format(p),ha="center",va="bottom",fontsize=11)
    a.set_xticks(x); a.set_xticklabels(["50% outliers","100% outliers"]); a.set_title(name)
    a.set_ylim(0, max(max(ours),max(paper))*1.3); a.legend(loc="upper left")
fige.suptitle(f"{TAG} — HEAD-TO-HEAD: our replication vs paper (Table I exact, 50%/100%)")
fige.tight_layout(); fige.savefig(f"{OUT}/{TAG}_headtohead_50_100.png", dpi=120)
print(f"wrote 5 figures to {OUT}/ for tag {TAG}")
