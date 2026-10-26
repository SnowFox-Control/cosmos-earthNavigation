function [metrics,out] = rocket_design(z,p,knotTimes)
%ROCKET_DESIGN Forward interface for later optimization; not an optimizer.
% z=[delta_1 ... delta_N q_1 ... q_N], angles in radians, rates in kg/s.
% Bounds: [-p.deltaMax,+p.deltaMax] and [0,p.qMax].
n=numel(knotTimes);
assert(numel(z)==2*n,'Design vector length must be twice the knot count.');
control.t=knotTimes;
control.delta=reshape(z(1:n),1,[]);
control.q=reshape(z(n+1:end),1,[]);
out=rocket_simulate(p,control);
metrics.terminalTime=out.t(end);
metrics.terminalAltitude=out.h(end);
metrics.terminalSpeed=out.speed(end);
metrics.terminalRadialSpeed=out.y(end,3);
metrics.terminalTangentialSpeed=out.y(end,4);
metrics.propellantUsed=out.propellantUsed;
metrics.totalImpulse=out.totalImpulse;
metrics.peakDynamicPressure=max(out.dynamicPressure); % Sampled maximum
metrics.burnoutTime=out.burnoutTime;
metrics.impacted=~isnan(out.impactTime);
metrics.reachedFinalTime=abs(out.t(end)-p.tf)<1e-6;
end
