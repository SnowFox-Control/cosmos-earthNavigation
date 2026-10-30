"""Compiled parallel RK4, same equations and forcing as planner_kernel.cu."""
import math
import numpy as np
from numba import njit, prange, set_num_threads, get_num_threads


@njit(inline='always')
def rhs(t,r,u,w,m,z,noise,p):
    t1=z[7];restart=t1+z[8];t2=z[9];finish=restart+t2
    if t<5:angle=0.
    elif t<.2*t1:angle=z[0]*(t-5)/(.2*t1-5)
    elif t<.6*t1:angle=z[0]+(z[1]-z[0])*(t-.2*t1)/(.4*t1)
    elif t<t1-.5:angle=z[1]+(z[2]-z[1])*(t-.6*t1)/(.4*t1-.5)
    elif t<t1:angle=z[2]
    elif t<restart:angle=z[2]+(z[3]-z[2])*(t-t1)/(restart-t1)
    elif t<restart+.5:angle=z[3]
    elif t<restart+.5*t2:angle=z[3]+.5*(z[4]-z[3])*(t-restart-.5)/(.5*t2-.5)
    elif t<finish-.5:angle=.5*(z[3]+z[4])+.5*(z[4]-z[3])*(t-restart-.5*t2)/(.5*t2-.5)
    else:angle=z[4]
    if t<t1-.5:q=z[5]
    elif t<t1:q=z[5]*(t1-t)/.5
    elif t<restart:q=0.
    elif t<restart+.5:q=z[6]*(t-restart)/.5
    elif t<finish-.5:q=z[6]
    else:q=z[6]*max(0.,(finish-t)/.5)
    V=math.sqrt(u*u+w*w);beta=math.atan2(w,u) if V>1e-9 else 0.
    alpha=0. if t<5 else beta+angle*math.pi/180
    rho=p[2]*math.exp(-max(r-p[1],0.)/p[3]);K=.5*p[4]*p[5]*rho
    i=int(math.floor(t));f=t-i;norm=math.sqrt((1-f)**2+f*f)
    ar=p[8]*rho/p[10]*((1-f)*noise[0,i]+f*noise[0,i+1])/norm
    aw=p[9]*rho/p[10]*((1-f)*noise[1,i]+f*noise[1,i+1])/norm
    return (u,w*w/r+p[6]*q*math.cos(alpha)/m-p[0]/(r*r)-K*V*u/m+ar,
            -u*w/r+p[6]*q*math.sin(alpha)/m-K*V*w/m+aw,-q)


@njit(parallel=True,cache=True)
def cpu_batch(parameters,noise,p,step):
    count=len(parameters);scores=np.empty(count);terminal=np.full((count,4),np.nan)
    for row in prange(count):
        z=parameters[row]
        used=z[5]*(z[7]-.25)+z[6]*(z[9]-.5)
        if used>p[7]-p[11]:
            scores[row]=1000+((used-(p[7]-p[11]))/1000)**2
            continue
        finish=z[7]+z[8]+z[9];r=p[1]+1;u=0.;w=0.;m=p[7]
        n=int(math.ceil(finish/step));dt=finish/n;impact=0.
        for k in range(n):
            t=k*dt
            a=rhs(t,r,u,w,m,z,noise,p)
            b=rhs(t+dt/2,r+dt*a[0]/2,u+dt*a[1]/2,w+dt*a[2]/2,m+dt*a[3]/2,z,noise,p)
            c=rhs(t+dt/2,r+dt*b[0]/2,u+dt*b[1]/2,w+dt*b[2]/2,m+dt*b[3]/2,z,noise,p)
            d=rhs(t+dt,r+dt*c[0],u+dt*c[1],w+dt*c[2],m+dt*c[3],z,noise,p)
            r+=dt*(a[0]+2*b[0]+2*c[0]+d[0])/6
            u+=dt*(a[1]+2*b[1]+2*c[1]+d[1])/6
            w+=dt*(a[2]+2*b[2]+2*c[2]+d[2])/6
            m+=dt*(a[3]+2*b[3]+2*c[3]+d[3])/6
            if r<p[1]:
                impact=100+finish-(t+dt);break
        vc=math.sqrt(p[0]/(p[1]+600000))
        value=((r-p[1]-600000)/10000)**2+(u/1000)**2+((w-vc)/1000)**2+impact
        scores[row]=value if math.isfinite(value) else 1e12
        terminal[row]=np.array([r,u,w,m])
    return scores,terminal


class CPUEvaluator:
    def __init__(self,data,step=.05):
        p=data['p'];opt=data['opt'];self.step=step
        assert p.h0==1 and p.u0==0 and p.w0==0 and p.turnStart==5
        assert p.noise.sampleInterval==1 and opt.rampSeconds==.5 and opt.targetAltitude==600000
        self.lb=opt.lb;self.ub=opt.ub
        sigma=p.noise.sigma0 if p.noise.enabled else np.zeros(2)
        self.parameters=np.array([p.mu,p.Re,p.rho0,p.H,p.Cd,p.A,p.c,p.m0,
                         sigma[0],sigma[1],p.noise.referenceDensity,p.mdry])
        self.noise=np.ascontiguousarray(data['samples']);threads=min(16,get_num_threads())
        self.gpu=f'CPU/Numba {threads} threads';set_num_threads(threads)
    def __call__(self,unit,terminal=False):
        z=np.ascontiguousarray(self.lb+np.atleast_2d(unit)*(self.ub-self.lb))
        values,states=cpu_batch(z,self.noise,self.parameters,self.step)
        assert np.isfinite(values).all()
        return (values,states) if terminal else values
