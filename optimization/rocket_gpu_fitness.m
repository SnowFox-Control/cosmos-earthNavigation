function fitness=rocket_gpu_fitness(unit,p,opt)
% Batched scalar CUDA kernels: one fixed-step RK4 rocket trajectory per particle.
% Shared GPU search evaluator; fixed returned controls are verified with ode45.
z=opt.lb+unit.*(opt.ub-opt.lb);
mu=p.mu;Re=p.Re;rho0=p.rho0;H=p.H;drag=.5*p.Cd*p.A;
m0=p.m0;dry=p.mdry;c=p.c;holdTime=p.turnStart;r0=Re+p.h0;
u0=p.u0;w0=p.w0;theta0=p.theta0;
target=opt.targetAltitude;vc=sqrt(mu/(Re+target));
scaleH=opt.objectiveScale(1);scaleU=opt.objectiveScale(2);scaleW=opt.objectiveScale(3);
ramp=opt.rampSeconds;step=opt.gpuStep;
field=rocket_gaussian_disturbance(p,sum(opt.ub(8:10))+2);
ns=field.settings;noiseDt=ns.sampleInterval;noiseR=0;noiseW=0;
if ns.enabled,noiseR=ns.sigma0(1);noiseW=ns.sigma0(2);end
noiseDensity=ns.referenceDensity;
stream=RandStream('mt19937ar','Seed',ns.seed);
samples=randn(stream,2,ceil((sum(opt.ub(8:10))+2)/noiseDt)+3);
nr=gpuArray(samples(1,:));nw=gpuArray(samples(2,:));
data=gpuArray(z);cols=cell(1,10);for j=1:10,cols{j}=data(:,j);end
fitness=gather(arrayfun(@trajectory,cols{:}));
fitness=fitness(:);
    function value=trajectory(a1,a2,a3,a4,a5,q1,q2,t1,coast,t2)
        finish=t1+coast+t2;restart=t1+coast;
        used=q1*(t1-ramp/2)+q2*(t2-ramp);
        if used>m0-dry,value=1000+((used-(m0-dry))/1000)^2;return;end
        r=r0;theta=theta0;u=u0;w=w0;m=m0;
        steps=ceil(finish/step);dt=finish/steps;impact=false;impactTime=finish;
        for k=1:steps
            t=(k-1)*dt;
            [r1,th1,u1,w1,m1]=rhs(t,r,u,w,m,a1,a2,a3,a4,a5,q1,q2,t1,restart,finish);
            [r2,th2,u2,w2,m2]=rhs(t+dt/2,r+dt*r1/2,u+dt*u1/2,w+dt*w1/2,m+dt*m1/2,a1,a2,a3,a4,a5,q1,q2,t1,restart,finish);
            [r3,th3,u3,w3,m3]=rhs(t+dt/2,r+dt*r2/2,u+dt*u2/2,w+dt*w2/2,m+dt*m2/2,a1,a2,a3,a4,a5,q1,q2,t1,restart,finish);
            [r4,th4,u4,w4,m4]=rhs(t+dt,r+dt*r3,u+dt*u3,w+dt*w3,m+dt*m3,a1,a2,a3,a4,a5,q1,q2,t1,restart,finish);
            r=r+dt*(r1+2*r2+2*r3+r4)/6;
            theta=theta+dt*(th1+2*th2+2*th3+th4)/6;
            u=u+dt*(u1+2*u2+2*u3+u4)/6;w=w+dt*(w1+2*w2+2*w3+w4)/6;
            m=m+dt*(m1+2*m2+2*m3+m4)/6;
            if r<Re,impact=true;impactTime=t+dt;break;end
        end
        value=((r-Re-target)/scaleH)^2+(u/scaleU)^2+((w-vc)/scaleW)^2;
        if impact,value=value+100+finish-impactTime;end
        if ~isfinite(value),value=1e12;end
    end
    function [dr,dth,du,dw,dm]=rhs(t,r,u,w,m,a1,a2,a3,a4,a5,q1,q2,t1,restart,finish)
        % Piecewise-linear controls exactly match the two-burn decoder.
        if t<holdTime,angle=0;
        elseif t<.2*t1,angle=a1*(t-holdTime)/(.2*t1-holdTime);
        elseif t<.6*t1,angle=a1+(a2-a1)*(t-.2*t1)/(.4*t1);
        elseif t<t1-ramp,angle=a2+(a3-a2)*(t-.6*t1)/(.4*t1-ramp);
        elseif t<t1,angle=a3;
        elseif t<restart,angle=a3+(a4-a3)*(t-t1)/(restart-t1);
        elseif t<restart+ramp,angle=a4;
        elseif t<restart+.5*(finish-restart)
            angle=a4+((a5-a4)/2)*(t-restart-ramp)/(.5*(finish-restart)-ramp);
        elseif t<finish-ramp
            angle=(a4+a5)/2+((a5-a4)/2)*(t-restart-.5*(finish-restart))/(.5*(finish-restart)-ramp);
        else,angle=a5;end
        if t<t1-ramp,q=q1;
        elseif t<t1,q=q1*(t1-t)/ramp;
        elseif t<restart,q=0;
        elseif t<restart+ramp,q=q2*(t-restart)/ramp;
        elseif t<finish-ramp,q=q2;
        else,q=q2*max(0,(finish-t)/ramp);end
        speed=sqrt(u*u+w*w);beta=0;if speed>1e-9,beta=atan2(w,u);end
        alpha=beta+angle*pi/180;if t<holdTime,alpha=0;end
        rho=rho0*exp(-max(r-Re,0)/H);k=drag*rho;
        index=floor(t/noiseDt)+1;lambda=t/noiseDt-floor(t/noiseDt);
        normFactor=sqrt((1-lambda)^2+lambda^2);
        ar=noiseR*rho/noiseDensity*((1-lambda)*nr(index)+lambda*nr(index+1))/normFactor;
        aw=noiseW*rho/noiseDensity*((1-lambda)*nw(index)+lambda*nw(index+1))/normFactor;
        dr=u;dth=w/r;dm=-q;
        du=w*w/r+c*q*cos(alpha)/m-mu/(r*r)-k*speed*u/m+ar;
        dw=-u*w/r+c*q*sin(alpha)/m-k*speed*w/m+aw;
    end
end
