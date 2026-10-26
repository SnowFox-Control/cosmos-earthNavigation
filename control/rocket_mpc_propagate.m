function [next,impact]=rocket_mpc_propagate(time,state,step,correction,p,ref,cfg,noise)
% Actual nonlinear plant, same dynamics as planning, different disturbance seed.
opts=odeset('RelTol',1e-9,'AbsTol',[1e-5 1e-12 1e-7 1e-7 1e-7 1e-3], ...
    'MaxStep',min([.25,step,noise.maxStep]),'Events',@events);
locked=state(5)<=p.mdry+1e-8;
[t,y,te,ye,ie]=ode45(@rhs,[time,time+step],state,opts);
impact=any(ie==2);next=y(end,:).';
if any(ie==1) && ~impact
    next=ye(find(ie==1,1),:).';next(5)=p.mdry;locked=true;
    if t(end)<time+step
        [~,y,~,~,ie]=ode45(@rhs,[t(end),time+step],next,opts);
        next=y(end,:).';impact=any(ie==2);
    end
end
    function dy=rhs(t,x)
        u=rocket_mpc_command(t,correction,p,ref,cfg);
        if locked,u=[0;0];end
        a=noise.evaluate(t,x(1)-p.Re);
        dy=rocket_dynamics(t,x,p,u(1),u(2),a);
    end
    function [value,terminal,direction]=events(~,x)
        if locked,value=[1;x(1)-p.Re];else,value=[x(5)-p.mdry;x(1)-p.Re];end
        terminal=[1;1];direction=[-1;-1];
    end
end
