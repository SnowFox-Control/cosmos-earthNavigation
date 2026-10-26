function model=rocket_mpc_linearize(time,p,ref,cfg)
% Linearized discrete nonlinear map in dimensionless tracking coordinates.
S=cfg.stateScale;S(2)=S(2)/p.Re;
x=ref.state(time);next=ref.state(time+cfg.sampleTime);
f=rocket_mpc_step(time,x,[0;0],p,ref,cfg,cfg.sampleTime);
A=zeros(5);B=zeros(5,2);dx=[.5;1e-7;.01;.01;.1];du=[1e-5;.01];
for j=1:5
    plus=x;minus=x;plus(j)=plus(j)+dx(j);minus(j)=minus(j)-dx(j);
    A(:,j)=(rocket_mpc_step(time,plus,[0;0],p,ref,cfg,cfg.sampleTime)- ...
        rocket_mpc_step(time,minus,[0;0],p,ref,cfg,cfg.sampleTime))/(2*dx(j));
end
for j=1:2
    c=zeros(2,1);c(j)=du(j);
    B(:,j)=(rocket_mpc_step(time,x,c,p,ref,cfg,cfg.sampleTime)- ...
        rocket_mpc_step(time,x,-c,p,ref,cfg,cfg.sampleTime))/(2*du(j));
end
model.A=(A.*S.')./S;
model.B=(B.*cfg.correctionScale.')./S;
model.d=(f-next)./S;
% Bounds guarantee correction validity at interpolation nodes within a step.
times=unique([time,time+cfg.sampleTime,ref.control.t(ref.control.t>=time & ref.control.t<=time+cfg.sampleTime)]);
u=ref.command(times);
model.lb=max(-cfg.maxCorrection,[-p.deltaMax-min(u(1,:));-min(u(2,:))])./cfg.correctionScale;
model.ub=min(cfg.maxCorrection,[p.deltaMax-max(u(1,:));p.qMax-max(u(2,:))])./cfg.correctionScale;
if (time>=ref.cutoff) || (cfg.preserveBurnSchedule && ...
        time<ref.restart && time+cfg.sampleTime>ref.firstCutoff)
    model.lb=zeros(2,1);model.ub=zeros(2,1);
end
model.next=next;
end
