function [p,opt]=rocket_two_burn_config()
% Main optimization interface. Mass/engine/target are in rocket_orbit_config.
[p,opt]=rocket_orbit_config();
opt.mode='two_burn';
opt=rmfield(opt,{'population','generations'}); % Legacy continuous-burn GA options
opt.seed=161;
opt.swarmSize=96;
opt.maxIterations=180;
opt.maxStallIterations=80;
opt.functionTolerance=1e-10;
opt.inertiaRange=[.1 1.1];
opt.selfAdjustmentWeight=1.49;
opt.socialAdjustmentWeight=1.49;
opt.minNeighborsFraction=.25;
opt.useLocalRefinement=true;       % false: PSO only
opt.localMaxIterations=160;
opt.localMaxFunctionEvaluations=6000;
opt.saveResults=true;
opt.makeReport=true;
opt.display='iter';
opt.useParallel=true;             % Parallel particle objective evaluations
opt.parallelWorkers=4;
opt.parallelLocalRefinement=true; % Parallel SQP finite differences
opt.accelerationBackend='auto';  % auto/gpu/cpu: GPU PSO batch + CPU ode45 refinement
opt.gpuStep=.05;                 % Fixed-step GPU RK4; final answer uses ode45
opt.heuristicFraction=.5;
opt.objectiveScale=[10000 1000 1000]; % [height m, radial m/s, tangential m/s]
opt.rampSeconds=.5;
% Columns: angle1 angle2 angle3 angle4 angle5 q1 q2 burn1 coast burn2
% Units: degrees (1:5), kg/s (6:7), seconds (8:10).
opt.lb=[0 -10 -10 -10 -10 180 120 80 20 25];
opt.ub=[10 10 10 10 10 350 350 180 250 120];
assert(opt.swarmSize>=8 && opt.swarmSize==floor(opt.swarmSize));
assert(opt.heuristicFraction>=0 && opt.heuristicFraction<=1);
assert(all(opt.objectiveScale>0) && opt.maxIterations>=1 && p.turnStart>0);
assert(all(opt.lb<opt.ub) && all(opt.ub(6:7)<=p.qMax));
assert(all(abs([opt.lb(1:5) opt.ub(1:5)])<=p.deltaMax*180/pi+1e-9));
assert(.2*opt.lb(8)>p.turnStart && opt.rampSeconds>0);
assert(opt.rampSeconds<.4*min(opt.lb([8 10])));
end
