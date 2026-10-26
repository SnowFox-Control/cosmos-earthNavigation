function result=rocket_two_burn_pso(overrides)
% Two-burn PSO seeded with suborbital ascent, coast, circularization profiles.
% PSO searches globally; SQP subsequently corrects terminal residuals.
[base,opt]=rocket_two_burn_config();
if nargin>0
    names=fieldnames(overrides);
    for j=1:numel(names)
        assert(isfield(opt,names{j}),'Unknown optimizer option: %s',names{j});
        opt.(names{j})=overrides.(names{j});
    end
end
% z = [pitch1 pitch2 pitch3 circularPitchStart circularPitchEnd,
%      q1 q2 burn1Duration coastDuration burn2Duration].
lb=opt.lb; ub=opt.ub;
assert(any(strcmpi(opt.accelerationBackend,{'auto','gpu','cpu'})),'Unknown acceleration backend.');
validateattributes(opt.gpuStep,{'numeric'},{'scalar','real','finite','positive'});
gpuInfo=rocket_gpu_setup(~strcmpi(opt.accelerationBackend,'cpu'));
parallelInfo=rocket_parallel_setup(opt.useParallel && (~gpuInfo.enabled || ...
    (opt.useLocalRefinement && opt.parallelLocalRefinement)),opt.parallelWorkers);
vc=sqrt(base.mu/(base.Re+opt.targetAltitude));
fast=base; fast.maxStep=2; fast.relTol=5e-7;
fast.absTol=[.001 1e-10 1e-5 1e-5 1e-5 .1];
rng(opt.seed,'twister'); history=[];
initial=rand(opt.swarmSize,10);
% Heuristic: first burn uses roughly 2/3 of fuel, second burn uses most
% of the remainder. Vary turn strength, coast time and final-burn duration.
for k=1:floor(opt.swarmSize*opt.heuristicFraction)
    q1=260+70*rand; q2=200+100*rand;
    budget=base.m0-base.mdry;
    t1=budget*(33000+7000*rand)/54000/q1;
    t2=(budget*50000/54000-q1*t1)/q2;
    z=[1+5*rand -2+4*rand -3+6*rand -2+4*rand -2+4*rand ...
        q1 q2 t1 40+130*rand t2];
    initial(k,:)=min(1,max(0,(z-lb)./(ub-lb)));
end
options=optimoptions('particleswarm','SwarmSize',opt.swarmSize, ...
    'MaxIterations',opt.maxIterations,'MaxStallIterations',opt.maxStallIterations, ...
    'FunctionTolerance',opt.functionTolerance,'InertiaRange',opt.inertiaRange, ...
    'SelfAdjustmentWeight',opt.selfAdjustmentWeight, ...
    'SocialAdjustmentWeight',opt.socialAdjustmentWeight, ...
    'MinNeighborsFraction',opt.minNeighborsFraction, ...
    'InitialPoints',initial,'UseParallel',parallelInfo.enabled && ~gpuInfo.enabled,'Display',opt.display, ...
    'OutputFcn',@record);
if gpuInfo.enabled
    try
        rocket_gpu_fitness(initial(1:2,:),base,opt); % Compile/check before starting PSO
        options.UseVectorized=true;
    catch exception
        warning('rocket:GPUKernelFallback','GPU kernel failed: %s; using CPU.',exception.message);
        gpuInfo.enabled=false;gpuInfo.reason=exception.message;
        parallelInfo=rocket_parallel_setup(opt.useParallel,opt.parallelWorkers);
        options.UseParallel=parallelInfo.enabled;
    end
end
started=tic;
if gpuInfo.enabled,fitnessFunction=@(x)rocket_gpu_fitness(x,base,opt);
else,fitnessFunction=@(x)objective(x,fast);end
[x,fval,flag,psoOutput,points]=particleswarm(fitnessFunction,10, ...
    zeros(1,10),ones(1,10),options);
psoX=x; psoFitness=fval;
[pg,cg]=decode(x,base); og=rocket_simulate(pg,cg);
psoResidual=residual(og);
if opt.saveResults
    save(rocket_file('rocket_two_burn_pso_stage.mat'),'psoX','psoFitness','psoResidual', ...
        'psoOutput','points','history','opt');
end
fprintf('\nPSO residual [height m, radial m/s, tangential m/s]:\n'); disp(psoResidual);
precise=base; precise.maxStep=.75; precise.relTol=1e-9;
precise.absTol=[1e-5 1e-12 1e-7 1e-7 1e-7 1e-3];
if opt.useLocalRefinement
localOptions=optimoptions('fmincon','Algorithm','sqp','MaxIterations',opt.localMaxIterations, ...
    'MaxFunctionEvaluations',opt.localMaxFunctionEvaluations,'Display',opt.display,'FiniteDifferenceType','central', ...
    'FiniteDifferenceStepSize',1e-5,'OptimalityTolerance',1e-9,'StepTolerance',1e-10, ...
    'UseParallel',parallelInfo.enabled && opt.parallelLocalRefinement);
    [x,fval,localFlag,localOutput]=fmincon(@(x)objective(x,precise),x,[],[],[],[], ...
        zeros(1,10),ones(1,10),@fuelConstraint,localOptions);
else
    localFlag=NaN; localOutput=struct('message','Local refinement disabled');
    fval=objective(x,precise);
end
[p,control]=decode(x,precise); out=rocket_simulate(p,control);
e=residual(out);
result=struct('z',lb+x.*(ub-lb),'fitness',fval,'exitflag',flag, ...
    'optimizer','PSO','output',psoOutput,'psoX',psoX,'psoFitness',psoFitness,'psoResidual',psoResidual, ...
    'localExitflag',localFlag,'localOutput',localOutput,'history',history, ...
    'out',out,'p',p,'opt',opt,'control',control,'residual',e, ...
    'targetSpeed',vc,'elapsedSeconds',toc(started));
result.success=abs(e(1))<=opt.altitudeTolerance && ...
    all(abs(e(2:3))<=opt.velocityTolerance) && isnan(out.impactTime);
result.parallelInfo=parallelInfo;
result.gpuInfo=gpuInfo;
fine=p; fine.maxStep=.2; fine.relTol=1e-10; fine.absTol=p.absTol/10;
v=rocket_simulate(fine,control);
result.refinementDifference=v.y(end,1:5)-out.y(end,1:5);
r=out.y(end,1); u=out.y(end,3); w=out.y(end,4);
energy=(u^2+w^2)/2-p.mu/r; a=-p.mu/(2*energy);
ecc=hypot(r*w*w/p.mu-1,r*u*w/p.mu);
result.osculatingEccentricity=ecc;
result.osculatingPerigee=a*(1-ecc)-p.Re;
result.osculatingApogee=a*(1+ecc)-p.Re;
if opt.saveResults
    save(rocket_file('rocket_two_burn_result.mat'),'result');
    save(rocket_file('rocket_orbit_final.mat'),'result');
end
if opt.makeReport,rocket_orbit_report(result);end
fprintf('Osculating perigee/apogee %.6f / %.6f km, eccentricity %.9g\n', ...
    result.osculatingPerigee/1000,result.osculatingApogee/1000,ecc);
fprintf('Fine-step difference [r theta u w m]:\n');disp(result.refinementDifference);

    function [p,c]=decode(unit,p)
        z=lb+unit.*(ub-lb);
        t1=z(8); coast=z(9); t2=z(10); restart=t1+coast; finish=restart+t2;
        % 0.5 s finite shutdown/ignition ramps; exact zero-flow coast.
        ramp=opt.rampSeconds;
        c.t=[0 p.turnStart .20*t1 .60*t1 t1-ramp t1 restart restart+ramp restart+.5*t2 finish-ramp finish];
        c.q=[z(6)*ones(1,5) 0 0 z(7)*ones(1,3) 0];
        c.delta=[0 0 z(1:3) z(3) z(4) z(4) (z(4)+z(5))/2 z(5) z(5)]*pi/180;
        p.tf=finish;
    end
    function e=residual(o)
        e=[o.h(end)-opt.targetAltitude,o.y(end,3),o.y(end,4)-vc];
    end
    function f=objective(unit,p)
        [pp,cc]=decode(unit,p);
        used=trapz(cc.t,cc.q);
        if used>base.m0-base.mdry
            f=1e3+((used-(base.m0-base.mdry))/1000)^2;return;
        end
        o=rocket_simulate(pp,cc);
        e=residual(o)./opt.objectiveScale; f=sum(e.^2);
        if ~isnan(o.impactTime),f=f+100+pp.tf-o.t(end);end
    end
    function [c,ceq]=fuelConstraint(unit)
        [~,cc]=decode(unit,base);
        c=(trapz(cc.t,cc.q)-(base.m0-base.mdry))/1000; ceq=[];
    end
    function stop=record(values,flag)
        stop=false;
        if strcmp(flag,'iter') || strcmp(flag,'init')
            history(end+1,:)=[values.iteration,values.bestfval,values.meanfval];
            if opt.saveResults && mod(values.iteration,20)==0
                checkpoint=struct('optimizer','PSO','history',history, ...
                    'swarm',values.swarm,'scores',values.swarmfvals, ...
                    'bestx',values.bestx,'iteration',values.iteration);
                save(rocket_file('rocket_two_burn_pso_checkpoint.mat'),'checkpoint');
            end
        end
    end
end
