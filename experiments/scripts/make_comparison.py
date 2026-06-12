#!/usr/bin/env python3
# Author-reported methods + our replicated IPC. F1/RPE exact (Table I); Precision/ATE digitized (Fig 3/4, approx).
import glob, statistics as st, os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

DS=["CSAIL","FRH","MIT","INTEL","M3500"]; RATES=list(range(10,101,10))
METHODS=["IPC","PCM","GNC","DCS","HUBER","GM","ADAPT","MAXMIX"]
OUT="experiments/figures/baseline"; os.makedirs(OUT,exist_ok=True)
def f1(p,r): return 0.0 if p+r==0 else 2*p*r/(p+r)
def src(m): return ("IPC","IPC_S3") if m=="IPC" else (m,"BASE")

def load(m):
    folder,tag=src(m); PR={}; TE={}
    for d in DS:
        for r in RATES:
            prs=glob.glob(f"experiments/results/{folder}/{d}/*/{tag}/{r}/*.PR")
            if prs:
                v=[tuple(float(x) for x in open(f).read().split("\n")[0].split()[:2]) for f in prs]
                PR[(d,r)]=(st.mean(p for p,_ in v),st.mean(rr for _,rr in v))
            tes=glob.glob(f"experiments/results/{folder}/{d}/*/{tag}/{r}/*.TE")
            if tes:
                t=[(lambda p:(float(p[1]),float(p[5])))(open(f).read().split()) for f in tes]
                TE[(d,r)]=(st.mean(a for a,_ in t),st.mean(rr for _,rr in t))
    return PR,TE
DATA={m:load(m) for m in METHODS}
def avg(m,rate,metric):
    PR,TE=DATA[m]; vals=[]
    for d in DS:
        if metric in('prec','rec','f1') and (d,rate) in PR:
            p,r=PR[(d,rate)]; vals.append({'prec':p,'rec':r,'f1':f1(p,r)}[metric])
        elif metric in('ate','rpe') and (d,rate) in TE:
            a,rp=TE[(d,rate)]; vals.append(a if metric=='ate' else rp)
    return st.mean(vals) if vals else float('nan')

# author-reported: F1/RPE exact (Table I); precision/ATE digitized from Fig 3/4 (approximate)
PF1={'50':{'GNC':0.747,'ADAPT':0.873,'MAXMIX':0.77,'DCS':0.796,'GM':0.795,'HUBER':0.706,'PCM':0.53,'IPC':0.91},
     '100':{'GNC':0.604,'ADAPT':0.738,'MAXMIX':0.66,'DCS':0.66,'GM':0.65,'HUBER':0.55,'PCM':0.371,'IPC':0.89}}
PRPE={'50':{'GNC':32.20,'ADAPT':1.867,'MAXMIX':0.324,'DCS':26.12,'GM':23.64,'HUBER':21.44,'PCM':25.01,'IPC':0.05},
      '100':{'GNC':35.53,'ADAPT':1.856,'MAXMIX':0.43,'DCS':37.684,'GM':38.33,'HUBER':27.32,'PCM':28.54,'IPC':0.068}}
PPREC={'50':{'IPC':0.97,'MAXMIX':0.96,'PCM':0.90,'ADAPT':0.83,'HUBER':0.78,'GNC':0.58,'DCS':0.54,'GM':0.54},
       '100':{'IPC':0.95,'MAXMIX':0.93,'PCM':0.82,'ADAPT':0.68,'HUBER':0.58,'GNC':0.42,'DCS':0.42,'GM':0.42}}
PATE={'50':{'IPC':24,'MAXMIX':55,'ADAPT':55,'PCM':60,'HUBER':62,'GNC':68,'DCS':70,'GM':72},
      '100':{'IPC':33,'MAXMIX':65,'ADAPT':62,'PCM':70,'HUBER':72,'GNC':78,'DCS':80,'GM':80}}
OURS_IPC={(met,rate):avg('IPC',rate,met) for met in('prec','f1','ate','rpe') for rate in(50,100)}

print(f"{'method':<7} | {'F1@50 o/p':>12} | {'F1@100 o/p':>12} | {'RPE@50 o/p':>14} | {'RPE@100 o/p':>14}")
for m in METHODS:
    print(f"{m:<7} | {avg(m,50,'f1'):5.2f}/{PF1['50'][m]:<4.2f} | {avg(m,100,'f1'):5.2f}/{PF1['100'][m]:<4.2f} | "
          f"{avg(m,50,'rpe'):6.2f}/{PRPE['50'][m]:<5.2f} | {avg(m,100,'rpe'):6.2f}/{PRPE['100'][m]:<5.2f}")

def vs_paper(grid, fname, suptitle):
    fig,ax=plt.subplots(2,2,figsize=(15,9.2))
    for idx,(title,P,metric,rate,logy) in enumerate(grid):
        a=ax.flat[idx]
        entries=[(m,P[str(rate)][m],'crimson' if m=='IPC' else 'lightgray') for m in METHODS]
        entries.append(("IPC*",OURS_IPC[(metric,rate)],'tab:blue'))   # IPC* = our replication
        entries.sort(key=lambda t:t[1] if logy else -t[1])
        names=[e[0] for e in entries]; ys=[e[1] for e in entries]
        a.bar(range(9),ys,color=[e[2] for e in entries],edgecolor='black',linewidth=0.4)
        a.set_xticks(range(9)); a.set_xticklabels(names,rotation=90,fontsize=8)
        for lab in a.get_xticklabels():
            if lab.get_text()=="IPC*": lab.set_color('tab:blue'); lab.set_fontweight('bold')
            elif lab.get_text()=="IPC": lab.set_color('crimson')
        if logy: a.set_yscale('log'); a.set_ylim(top=max(ys)*1.6)
        else: a.set_ylim(0,1.10)
        fmt="{:.0f}" if metric=='ate' else "{:.2f}" if metric in('prec','f1') else "{:.3f}"
        oi=names.index("IPC*"); a.text(oi,ys[oi],fmt.format(ys[oi]),ha='center',va='bottom',color='tab:blue',fontsize=8,fontweight='bold')
        a.set_title(f"{title}  (our IPC rank {names.index('IPC*')+1}/9)",fontsize=10); a.grid(axis='y',alpha=0.3,which='both')
    fig.suptitle(suptitle); fig.tight_layout(); fig.savefig(f"{OUT}/{fname}",dpi=100)

vs_paper([("F1 @50%",PF1,'f1',50,False),("F1 @100%",PF1,'f1',100,False),
          ("RPE @50%",PRPE,'rpe',50,True),("RPE @100%",PRPE,'rpe',100,True)],
         "comparison_ourIPC_vs_paper_exact.png",
         "Author-reported methods (gray) + author IPC (crimson) + our IPC (IPC*, blue) — F1 & RPE (Table I, EXACT)")
vs_paper([("Precision @50%",PPREC,'prec',50,False),("Precision @100%",PPREC,'prec',100,False),
          ("ATE @50%",PATE,'ate',50,True),("ATE @100%",PATE,'ate',100,True)],
         "comparison_ourIPC_vs_paper_digitized.png",
         "Author-reported methods + our IPC (IPC*) — Precision & ATE (DIGITIZED from Fig 3/4, APPROX)")
print("\nwrote 2 figures")
