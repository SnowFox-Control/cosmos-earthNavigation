function result=rocket_mpc_run(overrides)
% Closed-loop LTV MPC tracking a saved PSO trajectory; open-loop comparison.
cfg=rocket_mpc_config();
if nargin>0
    names=fieldnames(overrides);
    for j=1:numel(names)
        assert(isfield(cfg,names{j}),'Unknown MPC option: %s',names{j});
        cfg.(names{j})=overrides.(names{j});
    end
end
assert(cfg.sampleTime>0 && cfg.predictionSteps>=1 && cfg.rkSubsteps>=1);
assert(all(cfg.stateScale>0) && all(cfg.correctionScale>0));
saved=load(rocket_file(cfg.referenceFile),'result');planning=saved.result;p=planning.p;
endTime=min(planning.out.t(end),cfg.endTime);N=cfg.predictionSteps;Ts=cfg.sampleTime;
ref=rocket_mpc_reference(planning,endTime+(N+2)*Ts);
times=unique([0:Ts:endTime,endTime]).';count=numel(times)-1;
assert(count>0,'MPC duration must be positive.');
grid=(0:count+N)*Ts;
parallelInfo=rocket_parallel_setup(cfg.useParallel,cfg.parallelWorkers);
models=cell(1,numel(grid));precomputeTimer=tic;
if parallelInfo.enabled
    parfor j=1:numel(grid)
        model=rocket_mpc_linearize(grid(j),p,ref,cfg);model.current=ref.state(grid(j));models{j}=model;
    end
else
    for j=1:numel(grid)
        model=rocket_mpc_linearize(grid(j),p,ref,cfg);model.current=ref.state(grid(j));models{j}=model;
    end
end
precomputeSeconds=toc(precomputeTimer);
plant=p;
plant.noise.enabled=cfg.plantNoiseEnabled;plant.noise.seed=cfg.plantNoiseSeed;
plant.noise.sigma0=cfg.plantNoiseSigma;
noise=rocket_gaussian_disturbance(plant,endTime+1);
initial=planning.out.y(1,:).';initial(1:5)=initial(1:5)+cfg.initialOffset;
assert(initial(1)>p.Re && initial(5)>p.mdry,'Invalid initial state.');
closed=zeros(numel(times),6);open=closed;closed(1,:)=initial.';open(1,:)=initial.';
corrections=zeros(count,2);commands=zeros(count,2);flags=NaN(count,1);solveSeconds=zeros(count,1);
slack=zeros(count,1);previous=[0;0];closedImpact=false;openImpact=false;timer=tic;
for k=1:count
    t=times(k);step=times(k+1)-t;
    if t>=cfg.startTime && ~closedImpact && closed(k,5)>p.mdry+1e-6
        [correction,diagnostic]=rocket_mpc_qp(closed(k,:).',k,models,previous,p,cfg);
        flags(k)=diagnostic.exitflag;solveSeconds(k)=diagnostic.seconds;slack(k)=diagnostic.slack;
    else,correction=[0;0];end
    corrections(k,:)=correction.';commands(k,:)=rocket_mpc_command(t,correction,p,ref,cfg).';
    if ~closedImpact
        [next,closedImpact]=rocket_mpc_propagate(t,closed(k,:).',step,correction,plant,ref,cfg,noise);
        closed(k+1,:)=next.';
    else,closed(k+1,:)=closed(k,:);end
    if ~openImpact
        [next,openImpact]=rocket_mpc_propagate(t,open(k,:).',step,[0;0],plant,ref,cfg,noise);
        open(k+1,:)=next.';
    else,open(k+1,:)=open(k,:);end
    previous=correction;
    if cfg.showProgress && (mod(k,30)==0 || k==count)
        fprintf('MPC %d/%d, t=%.1f s, QP failures=%d\n',k,count,times(k+1),sum(flags<=0));
    end
end
reference=ref.state(times).';scale=cfg.stateScale.';scale(2)=scale(2)/p.Re;
errorClosed=(closed(:,1:5)-reference)./scale;
errorOpen=(open(:,1:5)-reference)./scale;
metric=struct('closedNormalizedRMS',sqrt(mean(sum(errorClosed.^2,2))), ...
    'openNormalizedRMS',sqrt(mean(sum(errorOpen.^2,2))), ...
    'closedTerminalTracking',closed(end,1:5)-reference(end,:), ...
    'openTerminalTracking',open(end,1:5)-reference(end,:), ...
    'QPFailures',sum(flags<=0),'QPCalls',sum(~isnan(flags)), ...
    'meanQPSeconds',mean(solveSeconds(~isnan(flags))), ...
    'maxQPSeconds',max(solveSeconds),'closedImpact',closedImpact,'openImpact',openImpact);
metric.closedPositionRMS=sqrt(mean((closed(:,1)-reference(:,1)).^2+ ...
    (p.Re*(closed(:,2)-reference(:,2))).^2));
metric.openPositionRMS=sqrt(mean((open(:,1)-reference(:,1)).^2+ ...
    (p.Re*(open(:,2)-reference(:,2))).^2));
metric.closedVelocityRMS=sqrt(mean(sum((closed(:,3:4)-reference(:,3:4)).^2,2)));
metric.openVelocityRMS=sqrt(mean(sum((open(:,3:4)-reference(:,3:4)).^2,2)));
if abs(endTime-ref.cutoff)<1e-6
    targetSpeed=sqrt(p.mu/(p.Re+planning.opt.targetAltitude));
    metric.closedOrbitResidual=[closed(end,1)-p.Re-planning.opt.targetAltitude,closed(end,3),closed(end,4)-targetSpeed];
    metric.openOrbitResidual=[open(end,1)-p.Re-planning.opt.targetAltitude,open(end,3),open(end,4)-targetSpeed];
end
result=struct('time',times,'closedState',closed,'openState',open,'referenceState',reference, ...
    'corrections',corrections,'commands',commands,'QPExitflags',flags,'QPSolveSeconds',solveSeconds, ...
    'altitudeSlack',slack,'metrics',metric,'cfg',cfg,'p',p,'plant',plant, ...
    'referenceFile',cfg.referenceFile,'referenceOptimizer',planning.optimizer, ...
    'parallelInfo',parallelInfo,'precomputeSeconds',precomputeSeconds,'elapsedSeconds',toc(timer));
result.referenceControl=planning.control;
result.targetAltitude=planning.opt.targetAltitude;
result.burnSchedule=struct('firstCutoff',planning.z(8),'restart',planning.z(8)+planning.z(9), ...
    'cutoff',planning.out.t(end));
if cfg.saveResults,save(rocket_file('rocket_mpc_result.mat'),'result');end
disp(metric);
if cfg.makeReport,rocket_mpc_report(result);end
end
