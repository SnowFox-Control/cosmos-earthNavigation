function [score,out,residual]=rocket_two_burn_objective(unit,p,opt)
% Shared CPU fitness, identical terminal scales/penalties to the GPU evaluator.
[p,control]=rocket_two_burn_decode(unit,p,opt);out=[];residual=nan(1,3);
used=trapz(control.t,control.q);
if used>p.m0-p.mdry
    score=1e3+((used-(p.m0-p.mdry))/1000)^2;return;
end
out=rocket_simulate(p,control);
vc=sqrt(p.mu/(p.Re+opt.targetAltitude));
residual=[out.h(end)-opt.targetAltitude,out.y(end,3),out.y(end,4)-vc];
score=sum((residual./opt.objectiveScale).^2);
if ~isnan(out.impactTime),score=score+100+p.tf-out.t(end);end
if ~isfinite(score),score=1e12;end
end
