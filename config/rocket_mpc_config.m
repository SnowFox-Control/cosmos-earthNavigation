function cfg=rocket_mpc_config()
% LTV constrained MPC tracks a saved successful PSO ascent (SI units).
% Full state feedback is assumed; prediction uses zero-mean future disturbance.
cfg.sampleTime=2;                  % Receding-horizon control interval, s
cfg.predictionSteps=15;            % 30 s prediction horizon by default
cfg.rkSubsteps=4;                  % RK4 substeps in the prediction map
cfg.startTime=5;                   % Apply feedback after initial launch hold
cfg.stateScale=[500;500;30;30;1000]; % [r, Re*theta, u, w, m] error scales
cfg.stateWeights=[4;2;4;4;.1];
cfg.terminalMultiplier=5;
cfg.inputWeights=[.2;.2];
cfg.inputRateWeights=[.5;.5];
cfg.correctionScale=[2*pi/180;50]; % One normalized correction unit: rad, kg/s
cfg.maxCorrection=[5*pi/180;100];
cfg.positionSlackWeight=1e5;       % Soft altitude/ground constraint penalty
cfg.plantNoiseEnabled=true;
cfg.plantNoiseSeed=20261021;       % Different from trajectory-planning seed
cfg.plantNoiseSigma=[.1 .1];
cfg.initialOffset=[0;0;0;0;0];    % [r m, theta rad, u m/s, w m/s, mass kg]
cfg.preserveBurnSchedule=true;     % No new burns during scheduled coast
cfg.useParallel=true;             % Precompute independent LTV models in parallel
cfg.parallelWorkers=4;
cfg.saveResults=true;
cfg.makeReport=true;
cfg.showProgress=true;
cfg.endTime=Inf;                  % Inf = reference cutoff; finite for short checks
cfg.referenceFile='rocket_pure_pso_reference.mat'; % Pure, independently verified PSO
end
