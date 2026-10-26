function verify_mpc()
% Validate the saved full closed-loop run and physical control constraints.
saved=load(rocket_file('rocket_mpc_result.mat'),'result');r=saved.result;
assert(strcmpi(r.referenceOptimizer,'PSO'));
assert(all(isfinite(r.closedState),'all') && ~r.metrics.closedImpact);
assert(min(r.closedState(:,5))>=r.p.mdry-1e-5);
assert(all(abs(r.commands(:,1))<=r.p.deltaMax+1e-10));
assert(all(r.commands(:,2)>=-1e-10 & r.commands(:,2)<=r.p.qMax+1e-10));
assert(r.metrics.QPFailures==0);
assert(r.metrics.closedPositionRMS<r.metrics.openPositionRMS);
assert(r.metrics.closedVelocityRMS<r.metrics.openVelocityRMS);
assert(r.plant.noise.seed==r.cfg.plantNoiseSeed);
assert(r.metrics.QPCalls>0);
fprintf('Saved PSO-reference MPC: constraints, feedback improvement and QP status passed.\n');
end
