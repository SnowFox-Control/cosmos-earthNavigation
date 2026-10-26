function out = rocket_simulate(p, control)
%ROCKET_SIMULATE Planar, nonrotating Earth; state [r theta u w m I].
% delta: thrust angle relative to velocity, positive toward +theta.
% Controls are piecewise linear, held at endpoint values outside knot span.
validateattributes(control.t, {'numeric'}, {'vector','real','finite','increasing'});
assert(numel(control.t)>=2 && control.t(1)==0, 'Control times must start at zero.');
assert(numel(control.q)==numel(control.t) && numel(control.delta)==numel(control.t), ...
    'Control vectors must have equal lengths.');
validateattributes(control.q, {'numeric'}, {'vector','real','finite','nonnegative','<=',p.qMax});
validateattributes(control.delta, {'numeric'}, {'vector','real','finite','>=',-p.deltaMax,'<=',p.deltaMax});
assert(p.m0>p.mdry && p.mdry>0 && p.h0>0 && p.tf>0 && p.c>0, 'Invalid physical parameters.');
y0 = [p.Re+p.h0; p.theta0; p.u0; p.w0; p.m0; 0];
qInterpolant=griddedInterpolant(control.t(:),control.q(:),'linear','nearest');
deltaInterpolant=griddedInterpolant(control.t(:),control.delta(:),'linear','nearest');
disturbance=rocket_gaussian_disturbance(p,p.tf);
opts = odeset('RelTol',p.relTol,'AbsTol',p.absTol,'MaxStep',min(p.maxStep,disturbance.maxStep));
poweredOpts = odeset(opts,'Events',@poweredEvents);
[t,y,te,~,ie] = ode45(@(t,y) rhs(t,y,true),[0 p.tf],y0,poweredOpts);
out.burnoutTime = NaN;
out.burnoutState = [];
out.impactTime = NaN;
if any(ie==2)
    out.impactTime = te(find(ie==2,1));
elseif any(ie==1)
    out.burnoutTime = te(find(ie==1,1));
    y(end,5) = p.mdry;
    out.burnoutState = y(end,:);
    if t(end)<p.tf
        coastOpts = odeset(opts,'Events',@groundEvent);
        [tc,yc,tec] = ode45(@(t,y) rhs(t,y,false),[t(end) p.tf],y(end,:).',coastOpts);
        t = [t; tc(2:end)];
        y = [y; yc(2:end,:)];
        if ~isempty(tec), out.impactTime = tec(end); end
    end
end
out.t = t;
out.y = y;
out.h = y(:,1)-p.Re;
out.downrange = p.Re*(y(:,2)-p.theta0);
out.speed = hypot(y(:,3),y(:,4));
out.gamma = atan2(y(:,3),y(:,4)); % Flight path angle above local horizontal
out.gamma(out.speed<1e-9) = NaN; % Direction is undefined at rest
out.q = qInterpolant(t);
out.delta = deltaInterpolant(t);
out.delta(t<p.turnStart)=0;
if ~isnan(out.burnoutTime)
    out.q(t>=out.burnoutTime)=0;
    out.delta(t>=out.burnoutTime)=0;
end
out.thrust = p.c*out.q;
out.alpha = atan2(y(:,4),y(:,3))+out.delta; % From outward radial direction
out.alpha(t<p.turnStart)=0; % Prescribed vertical thrust during launch hold
out.dynamicPressure = 0.5*p.rho0*exp(-max(out.h,0)/p.H).*out.speed.^2;
out.drag = p.Cd*p.A*out.dynamicPressure;
out.totalImpulse = y(end,6);
out.propellantUsed = p.m0-y(end,5);
out.p = p;
out.control = control;
out.disturbanceAcceleration=disturbance.evaluate(t,out.h);
out.noiseConfig=disturbance.settings;

    function [q,delta] = command(time)
        time = min(max(time,control.t(1)),control.t(end));
        q = qInterpolant(time);
        delta = deltaInterpolant(time);
        if time<p.turnStart, delta=0; end
    end

    function dy = rhs(time,state,powered)
        q=0; delta=0;
        if powered, [q,delta]=command(time); end
        noise=disturbance.evaluate(time,state(1)-p.Re);
        dy=rocket_dynamics(time,state,p,delta,q,noise);
    end

    function [value,isterminal,direction] = poweredEvents(~,state)
        value=[state(5)-p.mdry; state(1)-p.Re];
        isterminal=[1;1]; direction=[-1;-1];
    end

    function [value,isterminal,direction] = groundEvent(~,state)
        value=state(1)-p.Re; isterminal=1; direction=-1;
    end
end
