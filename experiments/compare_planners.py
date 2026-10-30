"""Equal-budget pure PSO, real-coded GA and independent-chain SA.

MATLAB exports physical settings, exact random forcing and paired initial points.
The paper uses Numba CPU integration; a cached CuPy CUDA backend is optional.
MATLAB verifies every fixed best control with ode45. No local minimizer is called.
"""
from pathlib import Path
import argparse
import hashlib
import json
import os
import time
import numpy as np
from scipy.io import loadmat, savemat

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT/'results'/'comparison'

def input_fingerprint(data):
    """Hash numeric content, not the timestamp in a MAT-file header."""
    digest=hashlib.sha256()
    p=data['p'];o=data['opt']
    for name in ['initial','samples','seeds']:
        a=np.ascontiguousarray(data[name],dtype=np.float64)
        digest.update(name.encode());digest.update(str(a.shape).encode());digest.update(a.tobytes())
    values=[getattr(p,name) for name in ['mu','Re','rho0','H','Cd','A','c','m0','mdry','h0',
        'theta0','u0','w0','turnStart','qMax','deltaMax']]
    values+=list(o.lb)+list(o.ub)+list(o.objectiveScale)+[o.targetAltitude,o.rampSeconds,
        float(p.noise.enabled),p.noise.sampleInterval,p.noise.referenceDensity]+list(p.noise.sigma0)
    digest.update(np.asarray(values,dtype=np.float64).tobytes())
    return digest.hexdigest()


class Evaluator:
    def __init__(self, data, step=.05):
        import cupy as cp
        self.cp=cp;self.step=step
        p=data['p'];opt=data['opt']
        assert p.h0==1 and p.u0==0 and p.w0==0 and p.turnStart==5
        assert p.noise.sampleInterval==1 and opt.rampSeconds==.5 and opt.targetAltitude==600000
        self.lb=opt.lb;self.ub=opt.ub
        sigma=p.noise.sigma0 if p.noise.enabled else np.zeros(2)
        par=np.array([p.mu,p.Re,p.rho0,p.H,p.Cd,p.A,p.c,p.m0,
                      sigma[0],sigma[1],p.noise.referenceDensity,p.mdry])
        self.parameters=cp.asarray(par);self.noise=cp.asarray(np.ascontiguousarray(data['samples']))
        self.noise_n=data['samples'].shape[1]
        self.kernel=cp.RawKernel((ROOT/'experiments'/'planner_kernel.cu').read_text(),
                                'trajectories',options=('--std=c++11',))
        self.gpu=cp.cuda.runtime.getDeviceProperties(0)['name'].decode()

    def __call__(self, unit, terminal=False):
        cp=self.cp
        unit=np.atleast_2d(unit)
        z=cp.asarray(np.ascontiguousarray(self.lb+unit*(self.ub-self.lb)))
        scores=cp.empty(len(unit));states=cp.full((len(unit),4),np.nan)
        self.kernel(((len(unit)+63)//64,),(64,),
                    (z,np.int32(len(unit)),self.noise,np.int32(self.noise_n),
                     self.parameters,np.float64(self.step),scores,states))
        values=cp.asnumpy(scores)
        assert np.isfinite(values).all()
        return (values,cp.asnumpy(states)) if terminal else values


def reflect(x):
    y=np.mod(x,2)
    return np.minimum(y,2-y)


def search(method, initial, evaluate, seed, iterations):
    # Same initial points, independent reproducible algorithm random streams.
    rng=np.random.default_rng(seed+1000*sum(map(ord,method)))
    x=initial.copy();M,dim=x.shape
    started=time.perf_counter();f=evaluate(x)
    idx=f.argmin();best=float(f[idx]);bestx=x[idx].copy()
    personal=x.copy();pf=f.copy();velocity=np.zeros_like(x);history=[]
    history.append([0,M,best,float(f.mean())]);accepted=0
    def tournament():
        candidates=rng.integers(M,size=(M,3))
        return candidates[np.arange(M),f[candidates].argmin(axis=1)]
    for k in range(1,iterations+1):
        progress=(k-1)/max(1,iterations-1)
        if method=='PSO':
            inertia=1.1-progress
            velocity=inertia*velocity+1.49*rng.random(x.shape)*(personal-x)+1.49*rng.random(x.shape)*(bestx-x)
            velocity=np.clip(velocity,-.2,.2)
            candidate=reflect(x+velocity)
        elif method=='GA':
            a,b=tournament(),tournament()
            blend=rng.uniform(-.25,1.25,x.shape)
            candidate=blend*x[a]+(1-blend)*x[b]
            candidate=reflect(candidate+(rng.random(x.shape)<1/dim)*rng.normal(0,.1,x.shape))
            candidate[0]=bestx
        else:
            candidate=x.copy();j=rng.integers(dim,size=M)
            width=.2*(1-progress)+.005*progress
            candidate[np.arange(M),j]+=rng.normal(0,width,M)
            candidate=reflect(candidate)
        scores=evaluate(candidate)
        if method=='SA':
            temperature=1e-6**progress
            take=(scores<=f)|(np.log(np.maximum(np.finfo(float).tiny,rng.random(M)))<-(scores-f)/temperature)
            accepted+=int(take.sum());x[take]=candidate[take];f[take]=scores[take]
        else:
            x=candidate;f=scores
        if method=='PSO':
            take=f<pf;personal[take]=x[take];pf[take]=f[take]
        idx=f.argmin()
        if f[idx]<best:
            best=float(f[idx]);bestx=x[idx].copy()
        history.append([k,M*(k+1),best,float(f.mean())])
        if k%30==0 or k==iterations:
            print(f'{method} seed={seed} iter={k} eval={M*(k+1)} J={best:.8g} seconds={time.perf_counter()-started:.1f}',flush=True)
    return dict(method=method,seed=int(seed),x=bestx.tolist(),searchFitness=best,
                history=history,evaluations=M*(iterations+1),seconds=time.perf_counter()-started,
                accepted=accepted)


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--iterations',type=int,default=180)
    parser.add_argument('--seeds',type=int,nargs='+',default=[161,162,163,164,165])
    parser.add_argument('--step',type=float,default=.05)
    parser.add_argument('--probe',action='store_true')
    parser.add_argument('--backend',choices=['numba','cuda'],default='numba')
    args=parser.parse_args()
    data=loadmat(OUT/'inputs.mat',squeeze_me=True,struct_as_record=False)
    input_hash=input_fingerprint(data)
    if args.backend=='numba':
        from planner_cpu import CPUEvaluator
        evaluate=CPUEvaluator(data,args.step)
    else:
        evaluate=Evaluator(data,args.step)
    initial=data['initial'][:,:,0]
    started=time.perf_counter();v,terminal=evaluate(initial,True)
    print('Evaluator warmup',time.perf_counter()-started,'backend',evaluate.gpu,flush=True)
    started=time.perf_counter();evaluate(initial)
    print('Cached batch seconds',time.perf_counter()-started,flush=True)
    savemat(OUT/f'{args.backend}_probe.mat',dict(initial=initial,scores=v,terminal=terminal,step=args.step))
    if args.probe:return
    results=[]
    for seed in args.seeds:
        col=np.flatnonzero(data['seeds']==seed)[0];initial=data['initial'][:,:,col]
        for method in ['PSO','GA','SA']:
            path=OUT/f'{method.lower()}_seed{seed}_pure.json'
            if path.exists():
                entry=json.loads(path.read_text())
                assert entry['iterations']==args.iterations and entry['step']==args.step
                assert entry['inputHash']==input_hash and entry['protocolVersion']==1
                assert entry['backend']==args.backend, 'Keep different backends in separate archives.'
            else:
                entry=search(method,initial,evaluate,seed,args.iterations)
                entry.update(iterations=args.iterations,step=args.step,gpu=evaluate.gpu,
                             localRefinement=False,initial=initial.tolist(),
                             inputHash=input_hash,protocolVersion=1,backend=args.backend)
                path.write_text(json.dumps(entry,indent=2),encoding='utf8')
            results.append(entry)
    savemat(OUT/'pure_candidates.mat',
            dict(unit=np.array([r['x'] for r in results]),methods=np.array([r['method'] for r in results],dtype=object),
                 seeds=np.array([r['seed'] for r in results])))
    (OUT/'campaign.json').write_text(json.dumps(dict(entries=results,settings=vars(args),
        noiseSeed=int(data['p'].noise.seed),localRefinement=False),indent=2),encoding='utf8')


if __name__=='__main__':
    main()
