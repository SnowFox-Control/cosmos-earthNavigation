function verify_pso()
% Pure planner regression; local refinement is intentionally unavailable.
verify_planner_comparison();
s=struct('swarmSize',12,'maxIterations',2,'saveResults',false,'makeReport',false, ...
 'display','off','useLocalRefinement',false,'useParallel',false,'accelerationBackend','cpu');
a=rocket_two_burn_pso(s);b=rocket_two_burn_ga(s);c=rocket_two_burn_sa(s);
assert(strcmp(a.optimizer,'PSO') && strcmp(b.optimizer,'GA') && strcmp(c.optimizer,'SA'));
assert(isnan(a.localExitflag) && isnan(b.localExitflag) && isnan(c.localExitflag));
assert(a.output.funccount==36 && b.output.funccount==36 && c.output.funccount==36);
fprintf('Pure optimizer CPU integration smoke checks passed.\n');
end
