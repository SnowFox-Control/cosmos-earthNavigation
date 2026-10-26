function info=rocket_parallel_setup(requested,workers)
% Reuse a process pool; report fallback explicitly when unavailable.
info=struct('enabled',false,'workers',0,'reason','Disabled by configuration');
if ~requested,return;end
if ~license('test','Distrib_Computing_Toolbox')
    info.reason='Parallel Computing Toolbox unavailable';
    warning('rocket:ParallelUnavailable','%s; using serial evaluation.',info.reason);return;
end
try
    pool=gcp('nocreate');
    if isempty(pool),pool=parpool('Processes',workers);end
    projectRoot=fileparts(mfilename('fullpath'));
    futures=parfevalOnAll(pool,@addpath,0,projectRoot,fullfile(projectRoot,'model'), ...
        fullfile(projectRoot,'config'),fullfile(projectRoot,'optimization'),fullfile(projectRoot,'control'));
    wait(futures);
    info.enabled=true;info.workers=pool.NumWorkers;info.reason='Reusable CPU process pool';
catch exception
    info.reason=exception.message;
    warning('rocket:ParallelUnavailable','Parallel setup failed: %s; using serial evaluation.',exception.message);
end
end
