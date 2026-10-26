function result = rocket_orbit_ga()
% Pure genetic search; no local optimizer. Requires Global Optimization Toolbox.
[base,opt]=rocket_orbit_config();
rng(opt.seed,'twister');
fast=base; fast.maxStep=3; fast.relTol=2e-6;
fast.absTol=[.01 1e-9 1e-4 1e-4 1e-4 1];
history=[];
targetRadius=base.Re+opt.targetAltitude;
targetSpeed=sqrt(base.mu/targetRadius);
fprintf('Target radius %.3f km, circular speed %.3f m/s\n',targetRadius/1000,targetSpeed);
fprintf('Mass ratio %.3f, Isp %.3f s, ideal delta-v %.3f m/s\n', ...
    base.m0/base.mdry,base.c/9.80665,base.c*log(base.m0/base.mdry));
initial=opt.lb+rand(opt.population,9).*(opt.ub-opt.lb);
% Include gentle-to-strong gravity-turn seeds, then let GA explore freely.
for k=1:16
    initial(k,:)=[0.5+0.5*k 0 0 0 0 300 300 300 .94];
end
initial(17,:)=[0 4.43 10 10 10 183.27 180 186.23 .96];
options=optimoptions('ga','PopulationSize',opt.population, ...
    'MaxGenerations',opt.generations,'MaxStallGenerations',60, ...
    'FunctionTolerance',1e-10,'EliteCount',4,'CrossoverFraction',0.8, ...
    'InitialPopulationMatrix',(initial-opt.lb)./(opt.ub-opt.lb),'Display','iter','UseParallel',false, ...
    'OutputFcn',@record);
started=tic;
[unitZ,fitness,exitflag,output,unitPopulation,scores]=ga(@objective,9,[],[],[],[], ...
    zeros(1,9),ones(1,9),[],options);
z=opt.lb+unitZ.*(opt.ub-opt.lb);
population=opt.lb+unitPopulation.*(opt.ub-opt.lb);
[p,control]=rocket_orbit_decode(z,base,opt);
out=rocket_simulate(p,control);
residual=[out.h(end)-opt.targetAltitude,out.y(end,3),out.y(end,4)-targetSpeed];
result=struct('z',z,'fitness',fitness,'exitflag',exitflag,'output',output, ...
    'population',population,'scores',scores,'history',history,'out',out, ...
    'p',p,'opt',opt,'control',control,'residual',residual, ...
    'targetSpeed',targetSpeed,'elapsedSeconds',toc(started));
result.success=abs(residual(1))<=opt.altitudeTolerance && ...
    all(abs(residual(2:3))<=opt.velocityTolerance) && isnan(out.impactTime);
save(rocket_file('rocket_orbit_ga_result.mat'),'result');
rocket_orbit_report(result);

    function value=objective(design)
        design=opt.lb+design.*(opt.ub-opt.lb);
        [pp,cc]=rocket_orbit_decode(design,fast,opt);
        oo=rocket_simulate(pp,cc);
        e=[(oo.h(end)-opt.targetAltitude)/10000,oo.y(end,3)/1000, ...
            (oo.y(end,4)-targetSpeed)/1000];
        value=sum(e.^2);
        if ~isnan(oo.impactTime), value=value+100+(pp.tf-oo.t(end)); end
    end

    function [state,options,changed]=record(options,state,flag)
        changed=false;
        if strcmp(flag,'iter') || strcmp(flag,'init')
            history(end+1,:)=[state.Generation,min(state.Score),mean(state.Score)];
            if mod(state.Generation,10)==0
                checkpoint=struct('population',state.Population,'scores',state.Score, ...
                    'history',history,'generation',state.Generation);
                save(rocket_file('rocket_orbit_ga_checkpoint.mat'),'checkpoint');
            end
            % Stop on actual physical tolerances, not just a small fitness change.
            [~,idx]=min(state.Score);
            physical=opt.lb+state.Population(idx,:).*(opt.ub-opt.lb);
            [pp,cc]=rocket_orbit_decode(physical,base,opt);
            oo=rocket_simulate(pp,cc);
            e=[oo.h(end)-opt.targetAltitude,oo.y(end,3),oo.y(end,4)-targetSpeed];
            if abs(e(1))<=opt.altitudeTolerance && all(abs(e(2:3))<=opt.velocityTolerance) ...
                    && isnan(oo.impactTime)
                state.StopFlag='Orbit insertion tolerances satisfied';
            end
        end
    end
end
