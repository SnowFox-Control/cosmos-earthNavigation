function result=rocket_two_burn_search(method,overrides)
% Pure PSO / real-coded GA / multi-start SA. No local refinement is performed.
[base,opt]=rocket_two_burn_config();
if nargin>1
    names=fieldnames(overrides);
    for j=1:numel(names)
        assert(isfield(opt,names{j}),'Unknown option: %s',names{j});
        opt.(names{j})=overrides.(names{j});
    end
end
assert(~opt.useLocalRefinement,'Local refinement has been removed from pure comparison runs.');
gpuInfo=rocket_gpu_setup(~strcmpi(opt.accelerationBackend,'cpu'));
fast=base;fast.maxStep=2;fast.relTol=5e-7;fast.absTol=[.001 1e-10 1e-5 1e-5 1e-5 .1];
initial=rocket_two_burn_initial(base,opt);
parallelInfo=rocket_parallel_setup(opt.useParallel && ~gpuInfo.enabled,opt.parallelWorkers);
if gpuInfo.enabled
    fitness=@(x)rocket_gpu_fitness(x,base,opt);
    try,fitness(initial(1:2,:));
    catch exception
        if strcmpi(opt.accelerationBackend,'gpu'),rethrow(exception);end
        warning('rocket:GPUFallback','%s',exception.message);gpuInfo.enabled=false;
        parallelInfo=rocket_parallel_setup(opt.useParallel,opt.parallelWorkers);fitness=@cpu;
    end
else,fitness=@cpu;end
% Initialization is identical across methods; subsequent random streams differ.
rng(opt.seed+sum(double(upper(method)))*1000,'twister');
search=rocket_population_search(method,fitness,initial,opt);
precise=base;precise.maxStep=.2;precise.relTol=1e-10;
precise.absTol=[1e-6 1e-13 1e-8 1e-8 1e-8 1e-4];
[p,control,z]=rocket_two_burn_decode(search.x,precise,opt);
[score,out,residual]=rocket_two_burn_objective(search.x,precise,opt);
assert(~isempty(out),'No fuel-feasible candidate was found.');
fine=p;fine.maxStep=.1;fine.relTol=1e-11;fine.absTol=p.absTol/10;
check=rocket_simulate(fine,control);
vc=sqrt(p.mu/(p.Re+opt.targetAltitude));
result=struct('optimizer',upper(method),'z',z,'fitness',score,'searchFitness',search.fitness, ...
    'search',search,'history',search.history(:,[1 3 4]),'residual',residual,'out',out, ...
    'p',p,'opt',opt,'control',control,'targetSpeed',vc,'elapsedSeconds',search.elapsedSeconds, ...
    'gpuInfo',gpuInfo,'parallelInfo',parallelInfo,'localExitflag',NaN, ...
    'localOutput',struct('message','No local refinement'),'psoX',search.x, ...
    'psoFitness',search.fitness,'psoResidual',residual,'output', ...
    struct('iterations',opt.maxIterations,'funccount',search.evaluations), ...
    'refinementDifference',check.y(end,1:5)-out.y(end,1:5));
result.success=all(abs(residual)<=[opt.altitudeTolerance opt.velocityTolerance opt.velocityTolerance]) ...
    && isnan(out.impactTime) && out.propellantUsed<=p.m0-p.mdry+1e-4;
r=out.y(end,1);u=out.y(end,3);w=out.y(end,4);energy=(u*u+w*w)/2-p.mu/r;
a=-p.mu/(2*energy);ecc=hypot(r*w*w/p.mu-1,r*u*w/p.mu);
result.osculatingEccentricity=ecc;result.osculatingPerigee=a*(1-ecc)-p.Re;
result.osculatingApogee=a*(1+ecc)-p.Re;
if opt.saveResults
    name=['rocket_two_burn_pure_' lower(method) '.mat'];save(rocket_file(name),'result');
    if strcmpi(method,'PSO')
        save(rocket_file('rocket_two_burn_result.mat'),'result');
        save(rocket_file('rocket_orbit_final.mat'),'result');
        if result.success,save(rocket_file('rocket_pure_pso_reference.mat'),'result');end
    end
end
if opt.makeReport,rocket_orbit_report(result);end
    function values=cpu(batch)
        values=zeros(size(batch,1),1);
        if parallelInfo.enabled
            parfor row=1:size(batch,1)
                values(row)=rocket_two_burn_objective(batch(row,:),fast,opt);
            end
        else
            for row=1:size(batch,1),values(row)=rocket_two_burn_objective(batch(row,:),fast,opt);end
        end
    end
end
