% RELAXED_BOUNDS  Re-run the two-burn PSO with widened box constraints.
%
% The baseline problem caps the thrust angle at 10 deg and the coast at 250 s;
% its optimum sits on both caps (delta4 = 10.000 deg, delta5 = 9.9996 deg,
% coast = 250.000 s).  This driver widens the caps to 15 deg / 400 s and
% re-runs the same pipeline (GPU-batched PSO, then SQP), writing the result
% to results/data/rocket_two_burn_relaxed.mat without touching the baseline
% files (saveResults = false, makeReport = false).
%
% Run from the project root:
%     matlab -batch "run('experiments/relaxed_bounds.m')"
%
% Requires: Global Optimization Toolbox, Optimization Toolbox, and a CUDA GPU
% for the fast path (falls back to a CPU process pool automatically).

startup
[p, opt] = rocket_two_burn_config();
fprintf('LB %s\n', mat2str(opt.lb));
fprintf('UB %s\n', mat2str(opt.ub));
fprintf('DELTAMAX %.2f deg\n', p.deltaMax * 180 / pi);

tic
r = rocket_two_burn_pso(struct('saveResults', false, ...
                              'makeReport', false, ...
                              'display', 'iter'));
fprintf('\nELAPSED %.1f s\n', toc);

fprintf('SUCCESS %d\n', r.success);
fprintf('RESIDUAL %s\n', mat2str(r.residual, 8));
fprintf('Z %s\n', mat2str(r.z, 10));
fprintf('PROP %.3f MFINAL %.3f IMPULSE %.3f\n', ...
        r.out.propellantUsed, r.out.y(end, 5), r.out.totalImpulse);
fprintf('GPU %d %s\n', r.gpuInfo.enabled, r.gpuInfo.name);

save(rocket_file('rocket_two_burn_relaxed.mat'), 'r');
fprintf('DONE\n');
