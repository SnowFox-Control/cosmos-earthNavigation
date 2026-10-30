function campaign=compare_planners(overrides)
% Equal-evaluation pure-optimizer experiment. Saved data are never used as seeds.
root=fileparts(fileparts(mfilename('fullpath')));run(fullfile(root,'startup.m'));
settings=struct('seeds',161:165,'swarmSize',96,'maxIterations',180, ...
    'accelerationBackend','gpu','gpuStep',.05,'resume',true);
if nargin>0
    fields=fieldnames(overrides);
    for j=1:numel(fields)
        assert(isfield(settings,fields{j}),'Unknown experiment setting');settings.(fields{j})=overrides.(fields{j});
    end
end
folder=fullfile(root,'results','comparison_matlab');if ~exist(folder,'dir'),mkdir(folder);end
methods={'PSO','GA','SA'};entries={};bestScore=Inf;bestPlan=[];
for s=1:numel(settings.seeds)
    for a=1:numel(methods)
        method=methods{a};seed=settings.seeds(s);
        path=fullfile(folder,sprintf('%s_seed%d.mat',lower(method),seed));
        expected=struct('seed',seed,'swarmSize',settings.swarmSize,'maxIterations',settings.maxIterations, ...
            'saveResults',false,'makeReport',false,'display','iter','useLocalRefinement',false, ...
            'accelerationBackend',settings.accelerationBackend,'gpuStep',settings.gpuStep);
        if settings.resume && exist(path,'file')
            loaded=load(path,'result');result=loaded.result;
            assert(result.opt.maxIterations==settings.maxIterations && ...
                result.opt.swarmSize==settings.swarmSize && result.opt.gpuStep==settings.gpuStep, ...
                'Cached experiment settings differ: %s',path);
            [nowP,nowOpt]=rocket_two_burn_config();
            assert(isequal(result.opt.lb,nowOpt.lb) && isequal(result.opt.ub,nowOpt.ub) && ...
                result.opt.targetAltitude==nowOpt.targetAltitude && ...
                isequal(result.opt.objectiveScale,nowOpt.objectiveScale),'Cached problem differs.');
            physical={'mu','Re','rho0','H','Cd','A','c','m0','mdry','h0','theta0','u0','w0','turnStart','noise'};
            for key=physical
                assert(isequaln(result.p.(key{1}),nowP.(key{1})),'Cached physical model differs.');
            end
        else
            result=rocket_two_burn_search(method,expected);
            save(path,'result');
        end
        item=struct('method',method,'seed',seed,'evaluations',result.search.evaluations, ...
            'searchFitness',result.searchFitness,'verifiedFitness',result.fitness, ...
            'residual',result.residual,'success',result.success,'seconds',result.elapsedSeconds, ...
            'z',result.z,'propellant',result.out.propellantUsed,'endTime',result.p.tf, ...
            'integrationDifference',result.refinementDifference,'history',result.search.history, ...
            'gpu',result.gpuInfo.name,'initial',result.search.initial);
        entries{end+1}=item; %#ok<AGROW>
        if strcmp(method,'PSO') && result.fitness<bestScore
            bestScore=result.fitness;bestPlan=result;
        end
        text=jsonencode(item,PrettyPrint=true);fid=fopen(strrep(path,'.mat','.json'),'w','n','UTF-8');
        assert(fid>=0);fwrite(fid,text,'char');fclose(fid);
        fprintf('VERIFIED %s seed=%d J=%.8g residual=%s pass=%d\n', ...
            method,seed,result.fitness,mat2str(result.residual,7),result.success);
    end
end
campaign=struct('settings',settings,'entries',{entries},'created',char(datetime('now')), ...
    'localRefinement',false,'noiseSeed',20261020,'SA','96 independent chains (or configured population)', ...
    'note','Verification evaluates fixed controls; it never modifies the candidate.');
save(fullfile(folder,'campaign.mat'),'campaign');
fid=fopen(fullfile(folder,'campaign.json'),'w','n','UTF-8');assert(fid>=0);
fwrite(fid,jsonencode(campaign,PrettyPrint=true),'char');fclose(fid);
result=bestPlan;save(fullfile(folder,'best_pso_reference.mat'),'result');
end
