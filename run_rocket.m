function run_rocket(action)
%RUN_ROCKET Project entry point: optimize, animate, verify, report, demo.
run(fullfile(fileparts(mfilename('fullpath')),'startup.m'));
if nargin==0,action='help';end
switch lower(char(action))
    case 'optimize'
        result=rocket_two_burn_pso();
        if result.success,verify_orbit();end
    case 'optimize-ga'
        rocket_two_burn_ga();
    case 'optimize-sa'
        rocket_two_burn_sa();
    case 'compare-planners'
        compare_planners();
    case 'animate'
        rocket_mpc_comet();
    case 'animate-reference'
        rocket_polar_comet();
    case 'mpc'
        rocket_mpc_run();
    case 'benchmark'
        benchmark_acceleration();
    case 'verify'
        verify_rocket();verify_disturbance();verify_orbit();
    case 'report'
        saved=load(rocket_file('rocket_two_burn_result.mat'),'result');
        rocket_orbit_report(saved.result);
    case 'demo'
        demo_rocket;
    otherwise
        fprintf('run_rocket(''optimize'') : pure PSO (no local refinement)\n');
        fprintf('run_rocket(''optimize-ga'') / (''optimize-sa'') : pure GA / multi-start SA\n');
        fprintf('run_rocket(''compare-planners'') : equal-budget MATLAB comparison\n');
        fprintf('run_rocket(''animate'')  : simultaneous MPC/open-loop comet animation\n');
        fprintf('run_rocket(''animate-reference'') : original single-reference animation\n');
        fprintf('run_rocket(''mpc'')      : MPC tracking of saved PSO trajectory\n');
        fprintf('run_rocket(''benchmark''): compare GPU batch and serial CPU evaluation\n');
        fprintf('run_rocket(''verify'')   : physical and orbital checks\n');
        fprintf('run_rocket(''report'')   : redraw saved optimization report\n');
        fprintf('run_rocket(''demo'')     : original 24-ton ascent example\n');
end
end
