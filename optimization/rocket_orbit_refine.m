function result=rocket_orbit_refine()
% Local terminal correction of the GA candidate; reported separately from GA.
loaded=load(rocket_file('rocket_orbit_ga_result.mat'),'result');
result=loaded.result;
base=result.p; opt=result.opt;
base.maxStep=1; base.relTol=1e-9;
base.absTol=[1e-5 1e-12 1e-7 1e-7 1e-7 1e-3];
span=opt.ub-opt.lb;
x0=(result.z-opt.lb)./span;
options=optimoptions('fmincon','Algorithm','sqp','Display','iter', ...
    'MaxIterations',150,'MaxFunctionEvaluations',4000, ...
    'FiniteDifferenceType','central','FiniteDifferenceStepSize',1e-4, ...
    'OptimalityTolerance',1e-8,'StepTolerance',1e-10);
[x,fval,flag,output]=fmincon(@objective,x0,[],[],[],[],zeros(size(x0)),ones(size(x0)),[],options);
result.gaZ=result.z; result.gaResidual=result.residual;
result.localOutput=output; result.localExitflag=flag; result.localFitness=fval;
result.z=opt.lb+x.*span;
[p,c]=rocket_orbit_decode(result.z,base,opt);
result.p=p; result.control=c;
result.out=rocket_simulate(p,c);
o=result.out;
result.residual=[o.h(end)-opt.targetAltitude,o.y(end,3),o.y(end,4)-result.targetSpeed];
result.success=abs(result.residual(1))<=opt.altitudeTolerance && ...
    all(abs(result.residual(2:3))<=opt.velocityTolerance) && isnan(o.impactTime);
r=o.y(end,1); u=o.y(end,3); w=o.y(end,4);
energy=(u^2+w^2)/2-p.mu/r; semimajor=-p.mu/(2*energy);
ecc=hypot(r*w^2/p.mu-1,r*u*w/p.mu);
result.osculatingEccentricity=ecc;
result.osculatingPerigee=semimajor*(1-ecc)-p.Re;
result.osculatingApogee=semimajor*(1+ecc)-p.Re;
fine=p; fine.maxStep=.25; fine.relTol=1e-10; fine.absTol=p.absTol/10;
verified=rocket_simulate(fine,c);
result.refinementDifference=verified.y(end,1:5)-o.y(end,1:5);
save(rocket_file('rocket_orbit_final.mat'),'result');
rocket_orbit_report(result);
fprintf('Osculating perigee/apogee: %.6f / %.6f km; eccentricity %.9g\n', ...
    result.osculatingPerigee/1000,result.osculatingApogee/1000,ecc);
fprintf('Fine-step differences [r theta u w m]:\n'); disp(result.refinementDifference);

    function value=objective(x)
        [pp,cc]=rocket_orbit_decode(opt.lb+x.*span,base,opt);
        oo=rocket_simulate(pp,cc);
        e=[(oo.h(end)-opt.targetAltitude)/10000,oo.y(end,3)/1000, ...
            (oo.y(end,4)-result.targetSpeed)/1000];
        value=sum(e.^2);
        if ~isnan(oo.impactTime), value=value+100+pp.tf-oo.t(end); end
    end
end
