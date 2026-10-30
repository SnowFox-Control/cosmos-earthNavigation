"""Build paper figures, tables, numerical macros and a complete trial CSV."""
from pathlib import Path
import csv
import hashlib
import json
import platform
import numpy as np
from scipy.io import loadmat
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT=Path(__file__).resolve().parents[1]
DATA=ROOT/'results'/'comparison';PAPER=ROOT/'paper';FIG=PAPER/'figures'
raw=json.loads((DATA/'campaign.json').read_text())
verified=json.loads((DATA/'verification.json').read_text())['entries']
control=json.loads((DATA/'mpc_metrics.json').read_text())
bykey={(r['method'],r['seed']):r for r in raw['entries']}
rows=[]
for v in verified:
    row=dict(v);r=bykey[(v['method'],v['seed'])]
    row.update(searchFitness=r['searchFitness'],seconds=r['seconds'],evaluations=r['evaluations'])
    rows.append(row)
methods=['PSO','GA','SA'];summary={}
for method in methods:
    subset=[r for r in rows if r['method']==method]
    score=np.array([r['verifiedFitness'] for r in subset])
    summary[method]=dict(median=float(np.median(score)),q25=float(np.quantile(score,.25)),
        q75=float(np.quantile(score,.75)),passed=sum(r['success'] for r in subset),
        seconds=float(np.median([r['seconds'] for r in subset])),
        medianAbsResidual=np.median(np.abs([r['residual'] for r in subset]),axis=0).tolist())
with (DATA/'trials.csv').open('w',newline='',encoding='utf-8-sig') as f:
    writer=csv.writer(f)
    writer.writerow(['method','seed','evaluations','search_fitness','verified_fitness',
        'height_error_m','radial_m_s','tangential_error_m_s','pass','search_seconds',
        'fuel_kg','time_s','radius_step_difference_m'])
    for r in rows:
        writer.writerow([r['method'],r['seed'],r['evaluations'],r['searchFitness'],
            r['verifiedFitness'],*r['residual'],int(r['success']),r['seconds'],r['fuel'],
            r['endTime'],r['difference'][0]])

plt.rcParams.update({'font.family':'Microsoft YaHei','font.size':9,
    'axes.unicode_minus':False,'pdf.fonttype':42,'axes.spines.top':False,
    'axes.spines.right':False,'legend.frameon':False,'lines.linewidth':1.4})
colors={'PSO':'#1769aa','GA':'#d27526','SA':'#32865b'}
def save(fig,name):
    fig.savefig(FIG/(name+'.pdf'),bbox_inches='tight',pad_inches=.03)
    fig.savefig(FIG/(name+'.png'),bbox_inches='tight',dpi=170)
    plt.close(fig)
def grid(ax):ax.grid(True,color='.88',lw=.5)
fig,ax=plt.subplots(1,2,figsize=(7,3),layout='constrained')
for i,method in enumerate(methods):
    h=np.array([r['history'] for r in raw['entries'] if r['method']==method])
    x=h[0,:,1];y=h[:,:,2]
    ax[0].semilogy(x,np.median(y,axis=0),color=colors[method],label=method)
    ax[0].fill_between(x,np.quantile(y,.25,axis=0),np.quantile(y,.75,axis=0),color=colors[method],alpha=.16)
    subset=[r for r in rows if r['method']==method]
    ax[1].scatter(i+np.linspace(-.11,.11,5),[r['verifiedFitness'] for r in subset],color=colors[method],s=25)
    ax[1].plot([i-.16,i+.16],[summary[method]['median']]*2,color=colors[method],lw=2)
ax[0].set(xlabel='累计轨迹评价次数',ylabel='搜索最优目标',title='(a) 收敛中位数与四分位区间')
ax[0].legend();ax[1].set(xticks=range(3),xticklabels=methods,yscale='log',
    ylabel='高精度复核目标',title='(b) 全部五次复核结果')
for a in ax:grid(a)
save(fig,'fig_planner_comparison')
fig,ax=plt.subplots(1,3,figsize=(7,2.6),layout='constrained')
for j,title in enumerate(['高度误差 / m','径向速度 / (m/s)','切向速度误差 / (m/s)']):
    for i,method in enumerate(methods):
        values=np.abs([r['residual'][j] for r in rows if r['method']==method])
        ax[j].scatter(i+np.linspace(-.1,.1,5),np.maximum(values,1e-6),s=23,color=colors[method])
    ax[j].axhline(500 if j==0 else 10,color='.3',lw=.9,ls='--')
    ax[j].set(xticks=range(3),xticklabels=methods,yscale='log',ylabel='绝对残差',title=title);grid(ax[j])
save(fig,'fig_planner_residuals')

ref=loadmat(DATA/'best_pso_reference.mat',squeeze_me=True,struct_as_record=False)['result']
out=ref.out;Re=ref.p.Re
fig,ax=plt.subplots(2,2,figsize=(7,4.5),layout='constrained')
ax[0,0].plot(out.downrange/1000,out.h/1000,color=colors['PSO'])
ax[0,0].set(xlabel='地表弧长 / km',ylabel='高度 / km')
ax[0,1].plot(out.t,out.h/1000,color=colors['PSO']);ax[0,1].axhline(600,color='.4',ls='--',lw=.8)
ax[0,1].set(xlabel='时间 / s',ylabel='高度 / km')
ax[1,0].plot(ref.control.t,ref.control.q,color=colors['PSO']);ax[1,0].set(xlabel='时间 / s',ylabel='推进剂流率 / (kg/s)')
ax[1,1].plot(out.t,out.y[:,2],color=colors['GA'],label='径向')
ax[1,1].plot(out.t,out.y[:,3],color=colors['PSO'],label='切向');ax[1,1].legend()
ax[1,1].set(xlabel='时间 / s',ylabel='速度 / (m/s)')
for a in ax.flat:grid(a)
save(fig,'fig_pure_reference')

mpc=loadmat(DATA/'mpc_pure_pso.mat',squeeze_me=True,struct_as_record=False)['mpc']
t=mpc.time;reference=mpc.referenceState
fig,ax=plt.subplots(2,2,figsize=(7,4.7),layout='constrained')
ax[0,0].plot(Re*(reference[:,1]-reference[0,1])/1000,(reference[:,0]-Re)/1000,'--',color='.3',label='参考')
for y,label,col in [(mpc.openState,'开环',colors['GA']),(mpc.closedState,'MPC',colors['PSO'])]:
    ax[0,0].plot(Re*(y[:,1]-reference[0,1])/1000,(y[:,0]-Re)/1000,label=label,color=col)
    pos=np.hypot(y[:,0]-reference[:,0],Re*(y[:,1]-reference[:,1]))
    vel=np.linalg.norm(y[:,2:4]-reference[:,2:4],axis=1)
    ax[0,1].semilogy(t,np.maximum(pos,1e-4),color=col,label=label)
    ax[1,0].semilogy(t,np.maximum(vel,1e-5),color=col,label=label)
ax[0,0].set(xlabel='地表弧长 / km',ylabel='高度 / km');ax[0,0].legend()
ax[0,1].set(xlabel='时间 / s',ylabel='位置跟踪误差 / m');ax[0,1].legend()
ax[1,0].set(xlabel='时间 / s',ylabel='速度跟踪误差 / (m/s)');ax[1,0].legend()
valid=np.isfinite(mpc.QPExitflags)
ax[1,1].semilogy(t[:-1][valid],np.maximum(mpc.QPSolveSeconds[valid]*1000,1e-3),'.',ms=3,color=colors['PSO'])
ax[1,1].set(xlabel='时间 / s',ylabel='QP求解时间 / ms')
for a in ax.flat:grid(a)
save(fig,'fig_pure_mpc')

def sci(v):
    m,e=f'{v:.2e}'.split('e')
    return rf'\ensuremath{{{m}\times10^{{{int(e)}}}}}'
macros={}
for method in methods:
    macros[method+'Pass']=str(summary[method]['passed'])
    macros[method+'Median']=sci(summary[method]['median'])
metrics=control['metrics']
macros.update(ClosedPos=f"{metrics['closedPositionRMS']:.3f}",OpenPos=f"{metrics['openPositionRMS']:.1f}",
    ClosedVel=f"{metrics['closedVelocityRMS']:.4f}",OpenVel=f"{metrics['openVelocityRMS']:.3f}",
    MaxFineRadius=f"{max(abs(r['difference'][0]) for r in rows):.3f}",
    ReferenceSeed=str(control['referenceSeed']),ReferenceTime=f'{ref.p.tf:.3f}',
    ReferenceFuel=f'{out.propellantUsed:.1f}',ReferenceHeight=f'{ref.residual[0]:.3f}',
    ReferenceRadial=f'{ref.residual[1]:.4f}',ReferenceTangential=f'{ref.residual[2]:.4f}',
    QPCount=str(metrics['QPCalls']),QPFail=str(metrics['QPFailures']),
    QPMean=f"{metrics['meanQPSeconds']*1000:.3f}",QPMax=f"{metrics['maxQPSeconds']*1000:.1f}")
(PAPER/'comparison_values.tex').write_text('\n'.join(r'\newcommand{\\'+k+'}{'+v+'}' for k,v in macros.items()).replace('{\\\\','{\\')+'\n',encoding='utf8')
lines=[r'\begin{table}[htbp]\centering\small',r'\caption{等预算无精修比较（每种算法五次）。IQR为25\%--75\%区间，耗时不含编译与复核。}\label{tab:algorithm}',
    r'\begin{tabular}{lcccc}\toprule',r'算法 & 复核目标中位数 & IQR & 通过数 & 耗时中位数/s\\\midrule']
for method in methods:
    s=summary[method]
    lines.append(f"{method} & {sci(s['median'])} & [{sci(s['q25'])}, {sci(s['q75'])}] & {s['passed']}/5 & {s['seconds']:.2f}"+r'\\')
lines.extend([r'\bottomrule\end{tabular}\end{table}'])
(PAPER/'comparison_table.tex').write_text('\n'.join(lines),encoding='utf8')
lines=[r'\begin{table}[htbp]\centering\small',
    r'\caption{纯PSO参考的开闭环对照。}\label{tab:mpc}',r'\begin{tabular}{lrr}\toprule',r'指标 & 开环 & MPC\\\midrule']
data=[('位置跟踪RMS / m',metrics['openPositionRMS'],metrics['closedPositionRMS']),
      ('速度跟踪RMS / (m/s)',metrics['openVelocityRMS'],metrics['closedVelocityRMS']),
      ('关机高度误差 / m',metrics['openOrbitResidual'][0],metrics['closedOrbitResidual'][0]),
      ('关机径向速度 / (m/s)',metrics['openOrbitResidual'][1],metrics['closedOrbitResidual'][1]),
      ('关机切向速度误差 / (m/s)',metrics['openOrbitResidual'][2],metrics['closedOrbitResidual'][2])]
for label,a,b in data:lines.append(f'{label} & {a:.4f} & {b:.4f}'+r'\\')
lines.append(r'\bottomrule\end{tabular}\end{table}')
(PAPER/'mpc_table.tex').write_text('\n'.join(lines),encoding='utf8')
hashes={p.relative_to(ROOT).as_posix():hashlib.sha256(
    p.read_bytes().replace(b'\r\n',b'\n') if p.suffix in ['.m','.py'] else p.read_bytes()
    ).hexdigest() for p in [
    DATA/'inputs.mat',ROOT/'experiments'/'compare_planners.py',ROOT/'experiments'/'planner_cpu.py',
    ROOT/'model'/'rocket_dynamics.m',ROOT/'experiments'/'verify_comparison.m']}
metadata=dict(algorithms=summary,referenceSeed=control['referenceSeed'],metrics=metrics,
    trials=15,localRefinement=False,python=platform.python_version(),numpy=np.__version__,
    hashConvention='Raw bytes for binary inputs; LF-normalized bytes for .m/.py sources',
    integrator='double RK4, max step .05 s; MATLAB ode45 independent verification',
    hashes=hashes)
(DATA/'summary.json').write_text(json.dumps(metadata,indent=2),encoding='utf8')
(PAPER/'figures'/'comparison_summary.json').write_text(json.dumps(metadata,indent=2),encoding='utf8')
print(json.dumps(summary,indent=2))
print('Generated 4 vector figures, 2 tables, numerical macros, all-trial CSV and provenance.')
