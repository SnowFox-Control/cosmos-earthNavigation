function verify_pso()
% Exercise actual rocket PSO and optional SQP without overwriting saved runs.
small=struct('swarmSize',12,'maxIterations',2,'maxStallIterations',2, ...
    'saveResults',false,'makeReport',false,'display','off','useLocalRefinement',false, ...
    'useParallel',false,'accelerationBackend','cpu');
a=rocket_two_burn_pso(small);
assert(strcmp(a.optimizer,'PSO') && a.output.iterations==2);
assert(size(a.history,2)==3 && size(a.history,1)>=3);
assert(all(diff(a.history(:,2))<=1e-10));
assert(all(a.psoX>=0 & a.psoX<=1));
assert(isfinite(a.fitness) && numel(a.residual)==3);
assert(isnan(a.localExitflag));
small.useLocalRefinement=true;
small.localMaxIterations=2;
small.localMaxFunctionEvaluations=100;
b=rocket_two_burn_pso(small);
assert(~isnan(b.localExitflag) && isfinite(b.fitness));
assert(isequal(a.psoX,b.psoX),'Fixed seed should reproduce the PSO stage.');
fprintf('PSO callback, bounds, reproducibility and SQP integration passed.\n');
end
