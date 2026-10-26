function report=benchmark_acceleration()
% Same 96-particle batch: serial ode45 versus CUDA RK4, after GPU warmup.
[p,opt]=rocket_two_burn_config();s=load(rocket_file('rocket_two_burn_result.mat'),'result');
plan=s.result;x=(plan.z-opt.lb)./(opt.ub-opt.lb);batch=repmat(x,96,1);
gpuInfo=rocket_gpu_setup(true);assert(gpuInfo.enabled);
rocket_gpu_fitness(batch,p,opt);
timer=tic;gpuScores=rocket_gpu_fitness(batch,p,opt);gpuSeconds=toc(timer);
p.tf=plan.out.t(end);timer=tic;scores=zeros(96,1);
for k=1:96
    out=rocket_simulate(p,plan.control);
    residual=[out.h(end)-opt.targetAltitude,out.y(end,3), ...
        out.y(end,4)-sqrt(p.mu/(p.Re+opt.targetAltitude))];
    scores(k)=sum((residual./opt.objectiveScale).^2);
end
cpuSeconds=toc(timer);
report=struct('gpu',gpuInfo.name,'batchSize',96,'gpuSeconds',gpuSeconds, ...
    'serialCPUSeconds',cpuSeconds,'speedup',cpuSeconds/gpuSeconds, ...
    'gpuObjective',gpuScores(1),'cpuObjective',scores(1),'gpuStep',opt.gpuStep, ...
    'relativeObjectiveDifference',abs(gpuScores(1)-scores(1))/max(abs(scores(1)),eps));
assert(all(isfinite(gpuScores)));
assert(report.relativeObjectiveDifference<.1,'GPU approximation differs excessively.');
save(rocket_file('rocket_acceleration_benchmark.mat'),'report');disp(report);
end
