% RELAXED_VERIFY  Coast propagation and MPC run for the relaxed-bounds solution.
%
% The planning driver (experiments/relaxed_bounds.m) only produced the design
% and its trajectory.  The paper's verification and control sections need two
% more artefacts, both written next to the baseline ones:
%   results/data/rocket_orbit_coast_relaxed.mat   unpowered coast, one period
%   results/data/rocket_mpc_relaxed.mat           closed loop vs open loop
%
% Run from the project root:
%     matlab -batch "run('experiments/relaxed_verify.m')"

startup
s = load(rocket_file('rocket_two_burn_relaxed.mat'));
result = s.r;
p = result.p;
state = result.out.y(end, :);

% ---------------------------------------------------------------- 1. coast
period = 2 * pi * sqrt((p.Re + result.opt.targetAltitude)^3 / p.mu);
disturbance = rocket_gaussian_disturbance(p, result.out.t(end) + period);
opts = odeset('RelTol', 1e-10, 'AbsTol', [1e-5 1e-12 1e-7 1e-7], ...
              'MaxStep', min(2, disturbance.maxStep));
[tc, yc] = ode45(@(tt, v) coast(tt, v, p, state(5), disturbance, result.out.t(end)), ...
                 [0 period], state(1:4).', opts);
coastResult = struct('t', tc, 'y', yc, 'period', period, ...
                     'altitude', yc(:, 1) - p.Re);
save(rocket_file('rocket_orbit_coast_relaxed.mat'), 'coastResult');
fprintf('COAST period %.3f s, altitude %.6f .. %.6f km, samples %d\n', ...
        period, min(coastResult.altitude) / 1e3, ...
        max(coastResult.altitude) / 1e3, numel(tc));

% ------------------------------------------------------------------ 2. MPC
save(rocket_file('rocket_two_burn_relaxed_ref.mat'), 'result');
m = rocket_mpc_run(struct('referenceFile', 'rocket_two_burn_relaxed_ref.mat', ...
                          'saveResults', false, 'makeReport', false, ...
                          'showProgress', false, 'useParallel', true));
save(rocket_file('rocket_mpc_relaxed.mat'), 'm');
fprintf('MPC closed pos rms %.3f m, vel rms %.4f m/s\n', ...
        m.metrics.closedPositionRMS, m.metrics.closedVelocityRMS);
fprintf('MPC open   pos rms %.1f m, vel rms %.3f m/s\n', ...
        m.metrics.openPositionRMS, m.metrics.openVelocityRMS);
fprintf('MPC closed orbit residual %s\n', mat2str(m.metrics.closedOrbitResidual, 6));
fprintf('MPC open   orbit residual %s\n', mat2str(m.metrics.openOrbitResidual, 6));
fprintf('MPC QP calls %d, failures %d, mean %.3f ms, max %.2f ms\n', ...
        m.metrics.QPCalls, m.metrics.QPFailures, ...
        m.metrics.meanQPSeconds * 1e3, m.metrics.maxQPSeconds * 1e3);
fprintf('DONE\n');


function dy = coast(tt, v, p, mass, disturbance, t0)
r = v(1); u = v(3); w = v(4);
sp = hypot(u, w);
k = .5 * p.rho0 * exp(-(r - p.Re) / p.H) * p.Cd * p.A;
noise = disturbance.evaluate(t0 + tt, r - p.Re);
dy = [u; w / r; ...
      w * w / r - p.mu / r^2 - k * sp * u / mass + noise(1); ...
      -u * w / r - k * sp * w / mass + noise(2)];
end
